/*
 * sd_spi.h - Minimal SD card driver over SPI (SPI mode) for STM32 HAL.
 * Target: companion board, SPI2 (PB13 SCK / PB14 MISO / PB15 MOSI), CS = PA8.
 * Supports SDv2 cards (SDSC v2, SDHC, SDXC). Not MMC / SDv1.
 *
 * Place in Core/Inc/
 */
#ifndef SD_SPI_H
#define SD_SPI_H

#include "main.h"
#include <stdint.h>

/* Last result of SD_Init(), readable in the debugger (Live Expressions):
 *   0 = OK
 *   1 = CMD0 failed   -> no response: check wiring, power, CS pin, SPI mode, MISO pull-up, card seated
 *   2 = CMD8 failed   -> card is not SDv2 or voltage check mismatch
 *   3 = ACMD41 timeout-> card never left idle state
 *   4 = CMD58 failed  -> could not read OCR
 */
extern volatile uint8_t sd_init_error;

uint8_t  SD_Init(void);                                                     /* 0 = OK */
uint8_t  SD_ReadBlocks(uint8_t *buf, uint32_t sector, uint32_t count);      /* 0 = OK */
uint8_t  SD_WriteBlocks(const uint8_t *buf, uint32_t sector, uint32_t count); /* 0 = OK */
uint8_t  SD_Sync(void);                                                     /* 0 = card ready */
uint32_t SD_GetSectorCount(void);                                           /* 0 on failure */

#endif /* SD_SPI_H */
