/*
 * sd_logger.h - Flight-data logger: RAM ring buffer -> CSV file on SD (via FatFs).
 * Place in Core/Inc/
 *
 * Intended use in the final firmware:
 *   Logger_Init();                  // once per run (e.g. when correction is toggled on)
 *   ...
 *   Logger_Push(&record);           // from the control loop, every cycle. Never touches the SD card.
 *   Logger_Service();               // from a low-priority context, as often as possible
 *   ...
 *   Logger_Close();                 // end of run (e.g. correction toggled off)
 *
 * Threading: Logger_Push is the single producer, Logger_Service the single consumer.
 * They may run in different contexts (e.g. Push from a timer interrupt, Service from main)
 * but each must only ever be called from one context.
 */
#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "main.h"
#include "ff.h"
#include <stdint.h>

/* One row of the log. To add/remove a column: edit this struct, LOG_HEADER, and the
 * snprintf in Logger_Push (sd_logger.c). Keep the three in the same order. */
typedef struct {
    uint32_t t_ms;
    uint16_t thr_tx;     /* throttle sent to the FC, us */
    uint8_t  arm_tx;     /* 1 = arm signal sent (AUX1 high) */
    uint8_t  fc_armed;   /* the FC's own armed state (MSP_STATUS), refreshed ~every 180 ms */
    uint8_t  link_ok;    /* pilot link alive */
} log_record_t;

typedef enum {
    LOG_OK = 0,
    LOG_ERR_NO_CARD,     /* card-detect says empty */
    LOG_ERR_MOUNT,       /* f_mount failed */
    LOG_ERR_NAME,        /* LOG0001..LOG9999 all exist */
    LOG_ERR_OPEN,        /* f_open / header write failed */
    LOG_ERR_WRITE
} logger_status_t;

typedef enum { LOGGER_IDLE = 0, LOGGER_RUNNING, LOGGER_ERROR } logger_state_t;

/* Watch these in Live Expressions. */
typedef struct {
    uint32_t records_pushed;
    uint32_t records_dropped;   /* ring full, logger in error, or row too long */
    uint32_t bytes_written;
    uint32_t errors;
    uint32_t max_write_ms;      /* longest single f_write */
    uint32_t max_sync_ms;       /* longest single f_sync */
    uint32_t max_push_us;       /* longest Logger_Push (formatting + copy) */
    uint32_t ring_high_water;   /* peak bytes held in ring (size is LOG_RING_SIZE) */
    FRESULT  last_res;          /* last FatFs result that was not FR_OK */
} logger_stats_t;

extern volatile logger_stats_t logger_stats;

logger_status_t Logger_Init(void);
void            Logger_Push(const log_record_t *r);
void            Logger_Service(void);
logger_status_t Logger_Close(void);
logger_state_t  Logger_GetState(void);
const char     *Logger_FileName(void);

#endif /* SD_LOGGER_H */
