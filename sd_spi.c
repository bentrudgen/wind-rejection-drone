/*
 * sd_spi.c - Minimal SD card driver over SPI for STM32 HAL.
 * Based on the standard SD SPI-mode command flow (same structure as ChaN's
 * FatFs MMC/SD examples), written for hspi2 + a GPIO chip select.
 *
 * Place in Core/Src/
 *
 * Assumes CubeMX names:  hspi2, and a GPIO labelled SD_CS (SD_CS_Pin / SD_CS_GPIO_Port).
 * If your label differs, change the two defines below.
 */
#include "sd_spi.h"
#include <string.h>

extern SPI_HandleTypeDef hspi2;

/* ---- Config ----------------------------------------------------------- */
#define SD_CS_PORT        SD_CS_GPIO_Port
#define SD_CS_PIN         SD_CS_Pin

/* SD init must run at <= 400 kHz. With APB1 = 50 MHz, /256 gives ~195 kHz (/128 would be ~390 kHz, too close). */
#define SD_SLOW_PRESCALER SPI_BAUDRATEPRESCALER_256
/* After init: /8 = 6.25 MHz at 50 MHz APB1. Try _4 (12.5 MHz) once everything works. */
#define SD_FAST_PRESCALER SPI_BAUDRATEPRESCALER_8

/* ---- SD commands ------------------------------------------------------ */
#define CMD0    (0)
#define CMD8    (8)
#define CMD9    (9)
#define CMD12   (12)
#define CMD16   (16)
#define CMD17   (17)
#define CMD18   (18)
#define CMD24   (24)
#define CMD25   (25)
#define CMD55   (55)
#define CMD58   (58)
#define ACMD41  (0x80 | 41)   /* high bit = "send CMD55 first" */

#define CS_LOW()   HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET)

/* ---- State ------------------------------------------------------------ */
volatile uint8_t sd_init_error = 0xFF;   /* 0xFF = never run */
static uint8_t   sd_block = 0;           /* 1 = block addressing (SDHC/SDXC) */
static uint8_t   ff_buf[512];            /* all 0xFF, used as dummy TX data for reads */

/* ---- Low-level SPI helpers -------------------------------------------- */
static uint8_t spi_xfer(uint8_t tx)
{
    uint8_t rx = 0xFF;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 100);
    return rx;
}

static void spi_set_prescaler(uint32_t presc)
{
    hspi2.Init.BaudRatePrescaler = presc;
    HAL_SPI_Init(&hspi2);
}

static int wait_ready(uint32_t timeout_ms)
{
    uint32_t t = HAL_GetTick();
    do {
        if (spi_xfer(0xFF) == 0xFF) return 1;
    } while ((HAL_GetTick() - t) < timeout_ms);
    return 0;
}

static void sd_deselect(void)
{
    CS_HIGH();
    spi_xfer(0xFF);          /* extra clocks release DO */
}

static int sd_select(void)
{
    CS_LOW();
    spi_xfer(0xFF);
    if (wait_ready(500)) return 1;
    sd_deselect();
    return 0;
}

/* Send a command, return R1 response (0xFF = no response / select failed).
 * CS is left LOW afterwards so the caller can read data. */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t res, n, crc = 0x01;

    if (cmd & 0x80) {                    /* ACMD: CMD55 first */
        cmd &= 0x7F;
        res = sd_cmd(CMD55, 0);
        if (res > 1) return res;
    }

    if (cmd != CMD12) {
        sd_deselect();
        if (!sd_select()) return 0xFF;
    }

    spi_xfer(0x40 | cmd);
    spi_xfer((uint8_t)(arg >> 24));
    spi_xfer((uint8_t)(arg >> 16));
    spi_xfer((uint8_t)(arg >> 8));
    spi_xfer((uint8_t)arg);
    if (cmd == CMD0) crc = 0x95;         /* CRC only checked for CMD0 and CMD8 */
    if (cmd == CMD8) crc = 0x87;
    spi_xfer(crc);

    if (cmd == CMD12) spi_xfer(0xFF);    /* discard stuff byte after stop */

    n = 10;
    do {
        res = spi_xfer(0xFF);
    } while ((res & 0x80) && --n);

    return res;
}

/* Receive a data block (token 0xFE, data, 2 CRC bytes). */
static int rx_datablock(uint8_t *buf, uint16_t len)
{
    uint8_t token;
    uint32_t t = HAL_GetTick();

    do {
        token = spi_xfer(0xFF);
    } while (token == 0xFF && (HAL_GetTick() - t) < 200);
    if (token != 0xFE) return 0;

    /* TX buffer must be 0xFF: SD data-in line (MOSI) must idle high */
    if (HAL_SPI_TransmitReceive(&hspi2, ff_buf, buf, len, 1000) != HAL_OK) return 0;

    spi_xfer(0xFF);                      /* discard CRC */
    spi_xfer(0xFF);
    return 1;
}

/* Transmit a data block with the given token (0xFE single, 0xFC multi, 0xFD stop). */
static int tx_datablock(const uint8_t *buf, uint8_t token)
{
    if (!wait_ready(500)) return 0;

    spi_xfer(token);
    if (token != 0xFD) {
        if (HAL_SPI_Transmit(&hspi2, (uint8_t *)buf, 512, 1000) != HAL_OK) return 0;
        spi_xfer(0xFF);                  /* dummy CRC */
        spi_xfer(0xFF);
        if ((spi_xfer(0xFF) & 0x1F) != 0x05) return 0;   /* data accepted? */
    }
    return 1;
}

/* ---- Public API ------------------------------------------------------- */
uint8_t SD_Init(void)
{
    uint8_t n, ocr[4];
    uint32_t t;

    memset(ff_buf, 0xFF, sizeof(ff_buf));
    sd_block = 0;

    spi_set_prescaler(SD_SLOW_PRESCALER);
    CS_HIGH();
    HAL_Delay(10);
    for (n = 0; n < 10; n++) spi_xfer(0xFF);     /* >= 74 clocks with CS high */

    sd_init_error = 1;
    if (sd_cmd(CMD0, 0) != 1) goto done;         /* enter idle state */

    sd_init_error = 2;
    if (sd_cmd(CMD8, 0x1AA) != 1) goto done;     /* SDv2 voltage check */
    for (n = 0; n < 4; n++) ocr[n] = spi_xfer(0xFF);
    if (ocr[2] != 0x01 || ocr[3] != 0xAA) goto done;

    sd_init_error = 3;
    t = HAL_GetTick();
    while (sd_cmd(ACMD41, 1UL << 30) != 0) {     /* HCS bit set */
        if ((HAL_GetTick() - t) > 1000) goto done;
    }

    sd_init_error = 4;
    if (sd_cmd(CMD58, 0) != 0) goto done;        /* read OCR */
    for (n = 0; n < 4; n++) ocr[n] = spi_xfer(0xFF);
    sd_block = (ocr[0] & 0x40) ? 1 : 0;          /* CCS bit */

    if (!sd_block) sd_cmd(CMD16, 512);           /* SDSC: force 512-byte blocks */

    sd_init_error = 0;
    spi_set_prescaler(SD_FAST_PRESCALER);

done:
    sd_deselect();
    return sd_init_error;
}

uint8_t SD_ReadBlocks(uint8_t *buf, uint32_t sector, uint32_t count)
{
    if (!count) return 1;
    if (!sd_block) sector *= 512;

    if (count == 1) {
        if (sd_cmd(CMD17, sector) == 0 && rx_datablock(buf, 512)) count = 0;
    } else {
        if (sd_cmd(CMD18, sector) == 0) {
            do {
                if (!rx_datablock(buf, 512)) break;
                buf += 512;
            } while (--count);
            sd_cmd(CMD12, 0);                    /* stop transmission */
        }
    }
    sd_deselect();
    return count ? 1 : 0;
}

uint8_t SD_WriteBlocks(const uint8_t *buf, uint32_t sector, uint32_t count)
{
    if (!count) return 1;
    if (!sd_block) sector *= 512;

    if (count == 1) {
        if (sd_cmd(CMD24, sector) == 0 && tx_datablock(buf, 0xFE)) count = 0;
    } else {
        if (sd_cmd(CMD25, sector) == 0) {
            do {
                if (!tx_datablock(buf, 0xFC)) break;
                buf += 512;
            } while (--count);
            if (!tx_datablock(0, 0xFD)) count = 1;   /* stop token */
        }
    }
    sd_deselect();
    return count ? 1 : 0;
}

uint8_t SD_Sync(void)
{
    if (sd_select()) {
        sd_deselect();
        return 0;
    }
    return 1;
}

uint32_t SD_GetSectorCount(void)
{
    uint8_t csd[16];
    uint32_t n = 0, cs;
    uint8_t nb;

    if (sd_cmd(CMD9, 0) == 0 && rx_datablock(csd, 16)) {
        if ((csd[0] >> 6) == 1) {                    /* CSD v2 (SDHC/SDXC) */
            cs = csd[9] + ((uint32_t)csd[8] << 8) + ((uint32_t)(csd[7] & 63) << 16) + 1;
            n = cs << 10;
        } else {                                     /* CSD v1 */
            nb = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
            cs = (csd[8] >> 6) + ((uint32_t)csd[7] << 2) + ((uint32_t)(csd[6] & 3) << 10) + 1;
            n = cs << (nb - 9);
        }
    }
    sd_deselect();
    return n;
}
