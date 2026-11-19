/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   FatFs low-level driver: SD card in SPI mode on SPI2
  *          (Team 10 Companion Board v2: PB13 SCK, PB14 MISO, PB15 MOSI, PA8 CS)
  ******************************************************************************
  * Works with SDSC (v1), SDHC and SDXC cards (SDXC must be formatted FAT32).
  * The card is brought up at ~195 kHz as the SD spec requires, then switched to
  * SD_SPI_FAST_PRESCALER (50 MHz APB1 / 4 = 12.5 MHz).
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;
#define SD_SPI                  (&hspi2)
#define SD_CS_PORT              GPIOA
#define SD_CS_PIN               GPIO_PIN_8
#define SD_SPI_SLOW_PRESCALER   SPI_BAUDRATEPRESCALER_256   /* 50 MHz / 256 = 195 kHz for card init (must be <= 400 kHz) */
#define SD_SPI_FAST_PRESCALER   SPI_BAUDRATEPRESCALER_4     /* 50 MHz / 4 = 12.5 MHz after init. _2 (25 MHz) also works on short traces */

#define SD_CS_LOW()    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()   HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET)

/* SD commands (0x80 flag = application command, sent after CMD55) */
#define CMD0    (0)
#define CMD1    (1)
#define ACMD41  (0x80 + 41)
#define CMD8    (8)
#define CMD9    (9)
#define CMD12   (12)
#define CMD16   (16)
#define CMD17   (17)
#define CMD18   (18)
#define ACMD23  (0x80 + 23)
#define CMD24   (24)
#define CMD25   (25)
#define CMD55   (55)
#define CMD58   (58)

/* Card type flags */
#define CT_MMC    0x01
#define CT_SD1    0x02
#define CT_SD2    0x04
#define CT_SDC    (CT_SD1 | CT_SD2)
#define CT_BLOCK  0x08   /* block addressing (SDHC/SDXC) */

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t cardType = 0;

/* Debug-visible: add to Live Expressions if the card will not mount */
volatile uint8_t  sdCardType = 0;        /* 0 = no card answered, 2 = SDv1, 4 = SDv2 byte-addressed, 12 = SDHC/SDXC */
volatile uint32_t sdDiskErrCount = 0;    /* failed sector reads/writes */

/* ---- SPI helpers ---- */
static void sd_set_speed(uint32_t prescaler)
{
  __HAL_SPI_DISABLE(SD_SPI);
  SD_SPI->Instance->CR1 = (SD_SPI->Instance->CR1 & ~SPI_CR1_BR) | prescaler;
  SD_SPI->Init.BaudRatePrescaler = prescaler;
  __HAL_SPI_ENABLE(SD_SPI);
}

static uint8_t sd_xchg(uint8_t d)
{
  uint8_t r = 0xFF;
  HAL_SPI_TransmitReceive(SD_SPI, &d, &r, 1, 50);
  return r;
}

/* Receive n bytes while clocking out 0xFF (the buffer itself is used as the 0xFF source) */
static void sd_rx_multi(BYTE *buff, UINT n)
{
  memset(buff, 0xFF, n);
  HAL_SPI_TransmitReceive(SD_SPI, buff, buff, (uint16_t)n, 100);
}

static void sd_tx_multi(const BYTE *buff, UINT n)
{
  HAL_SPI_Transmit(SD_SPI, (uint8_t *)buff, (uint16_t)n, 100);
}

/* Wait until the card stops signalling busy (MISO held low) */
static int sd_wait_ready(uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  do
  {
    if (sd_xchg(0xFF) == 0xFF) return 1;
  } while ((HAL_GetTick() - t0) < timeout_ms);
  return 0;
}

static void sd_deselect(void)
{
  SD_CS_HIGH();
  sd_xchg(0xFF);   /* one extra clock so the card releases MISO */
}

static int sd_select(void)
{
  SD_CS_LOW();
  sd_xchg(0xFF);
  if (sd_wait_ready(500)) return 1;
  sd_deselect();
  return 0;
}

static int sd_rcvr_datablock(BYTE *buff, UINT btr)
{
  uint8_t token;
  uint32_t t0 = HAL_GetTick();
  do
  {
    token = sd_xchg(0xFF);
  } while (token == 0xFF && (HAL_GetTick() - t0) < 200);
  if (token != 0xFE) return 0;

  sd_rx_multi(buff, btr);
  sd_xchg(0xFF);   /* discard CRC */
  sd_xchg(0xFF);
  return 1;
}

static int sd_xmit_datablock(const BYTE *buff, uint8_t token)
{
  if (!sd_wait_ready(500)) return 0;

  sd_xchg(token);
  if (token != 0xFD)   /* 0xFD = stop-transmission token, no data */
  {
    sd_tx_multi(buff, 512);
    sd_xchg(0xFF);     /* dummy CRC */
    sd_xchg(0xFF);
    uint8_t resp = sd_xchg(0xFF);
    if ((resp & 0x1F) != 0x05) return 0;   /* data not accepted */
  }
  return 1;
}

static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg)
{
  uint8_t n, res;

  if (cmd & 0x80)
  {
    cmd &= 0x7F;
    res = sd_send_cmd(CMD55, 0);
    if (res > 1) return res;
  }

  if (cmd != CMD12)
  {
    sd_deselect();
    if (!sd_select()) return 0xFF;
  }

  sd_xchg((uint8_t)(0x40 | cmd));
  sd_xchg((uint8_t)(arg >> 24));
  sd_xchg((uint8_t)(arg >> 16));
  sd_xchg((uint8_t)(arg >> 8));
  sd_xchg((uint8_t)arg);
  n = 0x01;                        /* dummy CRC + stop bit */
  if (cmd == CMD0) n = 0x95;       /* valid CRC for CMD0(0) */
  if (cmd == CMD8) n = 0x87;       /* valid CRC for CMD8(0x1AA) */
  sd_xchg(n);

  if (cmd == CMD12) sd_xchg(0xFF); /* skip a stuff byte */

  n = 10;
  do
  {
    res = sd_xchg(0xFF);
  } while ((res & 0x80) && --n);

  return res;
}

static void sd_cs_pin_init(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  SD_CS_HIGH();
  g.Pin = SD_CS_PIN;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_PORT, &g);
  SD_CS_HIGH();
}

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
  uint8_t n, cmd, ty, ocr[4];
  uint32_t t0;

  if (pdrv) return STA_NOINIT;

  sd_cs_pin_init();
  sd_set_speed(SD_SPI_SLOW_PRESCALER);

  /* At least 74 clocks with CS high to put the card into SPI mode */
  SD_CS_HIGH();
  for (n = 0; n < 10; n++) sd_xchg(0xFF);

  ty = 0;
  if (sd_send_cmd(CMD0, 0) == 1)                        /* enter idle state */
  {
    t0 = HAL_GetTick();
    if (sd_send_cmd(CMD8, 0x1AA) == 1)                  /* SDv2 */
    {
      for (n = 0; n < 4; n++) ocr[n] = sd_xchg(0xFF);
      if (ocr[2] == 0x01 && ocr[3] == 0xAA)             /* card works at 2.7-3.6 V */
      {
        while ((HAL_GetTick() - t0) < 1000 && sd_send_cmd(ACMD41, 1UL << 30)) { }   /* wait for init end, HCS set */
        if ((HAL_GetTick() - t0) < 1000 && sd_send_cmd(CMD58, 0) == 0)
        {
          for (n = 0; n < 4; n++) ocr[n] = sd_xchg(0xFF);
          ty = (ocr[0] & 0x40) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
        }
      }
    }
    else                                                /* SDv1 or MMCv3 */
    {
      if (sd_send_cmd(ACMD41, 0) <= 1) { ty = CT_SD1; cmd = ACMD41; }
      else                             { ty = CT_MMC; cmd = CMD1; }
      while ((HAL_GetTick() - t0) < 1000 && sd_send_cmd(cmd, 0)) { }
      if ((HAL_GetTick() - t0) >= 1000 || sd_send_cmd(CMD16, 512) != 0) ty = 0;
    }
  }
  cardType = ty;
  sdCardType = ty;
  sd_deselect();

  if (ty)
  {
    sd_set_speed(SD_SPI_FAST_PRESCALER);
    Stat &= (DSTATUS)~STA_NOINIT;
  }
  else
  {
    Stat = STA_NOINIT;
  }
  return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
  if (pdrv) return STA_NOINIT;
  return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
  if (pdrv || !count) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  if (!(cardType & CT_BLOCK)) sector *= 512;   /* byte addressing on SDSC */

  if (count == 1)
  {
    if (sd_send_cmd(CMD17, sector) == 0 && sd_rcvr_datablock(buff, 512)) count = 0;
  }
  else
  {
    if (sd_send_cmd(CMD18, sector) == 0)
    {
      do
      {
        if (!sd_rcvr_datablock(buff, 512)) break;
        buff += 512;
      } while (--count);
      sd_send_cmd(CMD12, 0);   /* stop transmission */
    }
  }
  sd_deselect();

  if (count) { sdDiskErrCount++; return RES_ERROR; }
  return RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  /* USER CODE HERE */
  if (pdrv || !count) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;
  if (Stat & STA_PROTECT) return RES_WRPRT;

  if (!(cardType & CT_BLOCK)) sector *= 512;

  if (count == 1)
  {
    if (sd_send_cmd(CMD24, sector) == 0 && sd_xmit_datablock(buff, 0xFE)) count = 0;
  }
  else
  {
    if (cardType & CT_SDC) sd_send_cmd(ACMD23, count);   /* pre-erase hint */
    if (sd_send_cmd(CMD25, sector) == 0)
    {
      do
      {
        if (!sd_xmit_datablock(buff, 0xFC)) break;
        buff += 512;
      } while (--count);
      if (!sd_xmit_datablock(NULL, 0xFD)) count = 1;      /* stop token */
    }
  }
  sd_deselect();

  if (count) { sdDiskErrCount++; return RES_ERROR; }
  return RES_OK;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
  DRESULT res = RES_ERROR;
  uint8_t csd[16];
  DWORD csize;

  if (pdrv) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
    case CTRL_SYNC:   /* wait for any internal write to finish */
      if (sd_select()) res = RES_OK;
      break;

    case GET_SECTOR_COUNT:
      if (sd_send_cmd(CMD9, 0) == 0 && sd_rcvr_datablock(csd, 16))
      {
        if ((csd[0] >> 6) == 1)   /* CSD v2 (SDHC/SDXC) */
        {
          csize = (DWORD)csd[9] + ((DWORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
          *(DWORD *)buff = csize << 10;
        }
        else                      /* CSD v1 (SDSC / MMC) */
        {
          uint8_t n = (uint8_t)((csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2);
          csize = (DWORD)(csd[8] >> 6) + ((DWORD)csd[7] << 2) + ((DWORD)(csd[6] & 3) << 10) + 1;
          *(DWORD *)buff = csize << (n - 9);
        }
        res = RES_OK;
      }
      break;

    case GET_SECTOR_SIZE:
      *(WORD *)buff = 512;
      res = RES_OK;
      break;

    case GET_BLOCK_SIZE:   /* erase block size in sectors; only used by f_mkfs */
      *(DWORD *)buff = 128;
      res = RES_OK;
      break;

    default:
      res = RES_PARERR;
      break;
  }

  sd_deselect();
  return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */
