/*
 * sd_logger.c - Ring buffer -> CSV on SD. See sd_logger.h for usage.
 * Place in Core/Src/
 *
 * Design:
 *  - Logger_Push formats one CSV row into a RAM ring buffer. It never touches the card.
 *  - Logger_Service moves data from the ring to the file in bounded chunks and calls
 *    f_sync about once a second, so a power cut loses at most ~1 s of data.
 *  - If the card errors, the logger enters LOGGER_ERROR and silently drops rows.
 *    It never blocks or retries forever: the flight code keeps running.
 */
#include "sd_logger.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

/* ---- Config ----------------------------------------------------------- */
#define LOG_RING_SIZE   8192u              /* MUST be a power of two */
#define LOG_RING_MASK   (LOG_RING_SIZE - 1u)
#define LOG_LINE_MAX    256u               /* max formatted row length */
#define LOG_WRITE_MIN   512u               /* don't write until this much is buffered... */
#define LOG_MAX_WRITE   2048u              /* ...and never write more than this per call */
#define LOG_SYNC_MS     1000u              /* f_sync interval */

/* Fallback if sd_spi.h's macro is not defined. Card detect is active-low on PB10. */
#ifndef SD_CARD_PRESENT
#define SD_CARD_PRESENT() (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET)
#endif

#define LOG_HEADER "t_ms,thr_tx,arm_tx,fc_armed,link_ok," \
                   "vbat_deci,amps_centi,mah_drawn," \
                   "cmd_roll,cmd_pitch,cmd_yaw,tx_roll,tx_pitch,tx_yaw,att_roll,att_pitch,att_yaw\n"

/* ---- State ------------------------------------------------------------ */
volatile logger_stats_t logger_stats;

static uint8_t           ring[LOG_RING_SIZE];
static volatile uint32_t head;             /* written only by Logger_Push (free-running) */
static volatile uint32_t tail;             /* written only by Logger_Service (free-running) */
static volatile logger_state_t state = LOGGER_IDLE;

static FIL      log_file;
static char     log_name[16];
static uint32_t last_sync_ms;
static uint8_t  dirty;

/* ---- Helpers ---------------------------------------------------------- */
static void enable_cycle_counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void fail(FRESULT res)
{
    logger_stats.errors++;
    logger_stats.last_res = res;
    state = LOGGER_ERROR;
}

/* Write one bounded chunk from the ring to the file. */
static void write_chunk(void)
{
    uint32_t t = tail;
    uint32_t used = head - t;
    uint32_t contig = LOG_RING_SIZE - (t & LOG_RING_MASK);
    uint32_t n = used;
    UINT bw = 0;
    uint32_t t0, dt;
    FRESULT res;

    if (n > contig)        n = contig;
    if (n > LOG_MAX_WRITE) n = LOG_MAX_WRITE;
    if (n == 0) return;

    t0 = HAL_GetTick();
    res = f_write(&log_file, &ring[t & LOG_RING_MASK], n, &bw);
    dt = HAL_GetTick() - t0;
    if (dt > logger_stats.max_write_ms) logger_stats.max_write_ms = dt;

    if (res != FR_OK || bw != n) {
        fail(res != FR_OK ? res : FR_DISK_ERR);
        return;
    }
    tail = t + n;
    logger_stats.bytes_written += n;
    dirty = 1;
}

/* ---- Public API ------------------------------------------------------- */
logger_status_t Logger_Init(void)
{
    FRESULT res;
    UINT bw;
    uint32_t n;

    enable_cycle_counter();
    memset((void *)&logger_stats, 0, sizeof(logger_stats));
    head = tail = 0;
    dirty = 0;
    state = LOGGER_IDLE;

    if (!SD_CARD_PRESENT()) return LOG_ERR_NO_CARD;

    res = f_mount(&USERFatFS, USERPath, 1);
    if (res != FR_OK) { logger_stats.last_res = res; return LOG_ERR_MOUNT; }

    /* Find the first unused LOGnnnn.CSV (8.3 name) */
    for (n = 1; n <= 9999; n++) {
        snprintf(log_name, sizeof(log_name), "LOG%04lu.CSV", (unsigned long)n);
        if (f_stat(log_name, NULL) == FR_NO_FILE) break;
    }
    if (n > 9999) return LOG_ERR_NAME;

    res = f_open(&log_file, log_name, FA_CREATE_NEW | FA_WRITE);
    if (res != FR_OK) { logger_stats.last_res = res; return LOG_ERR_OPEN; }

    res = f_write(&log_file, LOG_HEADER, strlen(LOG_HEADER), &bw);
    if (res == FR_OK) res = f_sync(&log_file);
    if (res != FR_OK) {
        logger_stats.last_res = res;
        f_close(&log_file);
        return LOG_ERR_OPEN;
    }

    last_sync_ms = HAL_GetTick();
    state = LOGGER_RUNNING;
    return LOG_OK;
}

void Logger_Push(const log_record_t *r)
{
    char line[LOG_LINE_MAX];
    int len;
    uint32_t h, used, i, c0, us;

    if (state != LOGGER_RUNNING) { logger_stats.records_dropped++; return; }

    c0 = DWT->CYCCNT;

    len = snprintf(line, sizeof(line), "%lu,%u,%u,%u,%u,%u,%d,%u,%d,%d,%d,%u,%u,%u,%d,%d,%d\n",
        (unsigned long)r->t_ms, r->thr_tx, r->arm_tx, r->fc_armed, r->link_ok,
        r->vbat_deci, r->amps_centi, r->mah_drawn,
        r->cmd_roll, r->cmd_pitch, r->cmd_yaw,
        r->tx_roll, r->tx_pitch, r->tx_yaw,
        r->att_roll, r->att_pitch, r->att_yaw);

    if (len <= 0 || len >= (int)sizeof(line)) { logger_stats.records_dropped++; return; }

    h = head;
    used = h - tail;
    if ((LOG_RING_SIZE - used) < (uint32_t)len) {      /* ring full: drop whole row */
        logger_stats.records_dropped++;
        return;
    }
    for (i = 0; i < (uint32_t)len; i++) ring[(h + i) & LOG_RING_MASK] = (uint8_t)line[i];
    head = h + (uint32_t)len;                          /* publish after the copy */

    logger_stats.records_pushed++;
    if (used + (uint32_t)len > logger_stats.ring_high_water)
        logger_stats.ring_high_water = used + (uint32_t)len;

    us = (DWT->CYCCNT - c0) / (SystemCoreClock / 1000000u);
    if (us > logger_stats.max_push_us) logger_stats.max_push_us = us;
}

void Logger_Service(void)
{
    uint32_t now, used;
    uint8_t sync_due;

    if (state != LOGGER_RUNNING) return;

    now = HAL_GetTick();
    used = head - tail;
    sync_due = ((now - last_sync_ms) >= LOG_SYNC_MS);

    /* One bounded chunk per call: write when enough is buffered, or flush a partial
     * block when a sync is due so the sync captures everything. */
    if (used >= LOG_WRITE_MIN || (sync_due && used > 0)) {
        write_chunk();
        if (state != LOGGER_RUNNING) return;
        used = head - tail;
    }

    if (sync_due && used == 0) {
        if (dirty) {
            uint32_t t0 = HAL_GetTick();
            FRESULT res = f_sync(&log_file);
            uint32_t dt = HAL_GetTick() - t0;
            if (dt > logger_stats.max_sync_ms) logger_stats.max_sync_ms = dt;
            if (res != FR_OK) { fail(res); return; }
            dirty = 0;
        }
        last_sync_ms = HAL_GetTick();
    }
}

logger_status_t Logger_Close(void)
{
    FRESULT res;
    logger_status_t status = LOG_OK;

    if (state == LOGGER_IDLE) return LOG_OK;

    if (state == LOGGER_RUNNING) {
        while ((head - tail) > 0 && state == LOGGER_RUNNING) write_chunk();
    }
    if (state == LOGGER_ERROR) status = LOG_ERR_WRITE;

    res = f_close(&log_file);                  /* f_close also syncs */
    if (res != FR_OK) { logger_stats.last_res = res; status = LOG_ERR_WRITE; }

    state = LOGGER_IDLE;
    return status;
}

logger_state_t Logger_GetState(void) { return state; }
const char    *Logger_FileName(void) { return log_name; }
