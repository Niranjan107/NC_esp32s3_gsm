/**
 * @file store_forward.h
 * @brief Persistent offline buffer for MQTT readings (LittleFS on the
 *        `storage` partition). FIFO, one file per reading, format-agnostic —
 *        it stores the exact publish JSON and replays it byte-for-byte on
 *        reconnect so the server's MD5 dedup matches.
 */
#ifndef STORE_FORWARD_H
#define STORE_FORWARD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Mount LittleFS on the `storage` partition and count existing records. */
void sf_init(void);

/** True if the buffer filesystem mounted OK. */
bool sf_available(void);

/** Number of readings currently buffered on flash. */
uint32_t sf_count(void);

/** Persist one reading (the exact publish JSON) to the FIFO buffer.
 *  Non-blocking-ish (a small flash write). When the partition is nearly full it
 *  EVICTS the oldest records to make room, so the reading happening now is
 *  always recorded (see sf_evicted()). */
void sf_buffer(const char *json, int len);

/** Read the OLDEST buffered record into `buf` (null-terminated) and return its
 *  path in `path_out` (pass to sf_delete after the broker ACKs it).
 *  Pass buf == NULL to fetch only the path (delete without reading).
 *  @return true if a record was returned. */
bool sf_oldest(char *buf, int bufsz, int *out_len, char *path_out, int path_sz);

/** Delete a record file (after its publish is confirmed by the broker). */
void sf_delete(const char *path);

/** Storage usage snapshot. Sizes are bytes of the `storage` partition. */
typedef struct {
    uint32_t total_bytes;   /* partition size */
    uint32_t used_bytes;    /* filesystem in use (includes overhead) */
    uint32_t free_bytes;    /* total - used */
    uint8_t  used_pct;      /* 0..100 */
    uint32_t records;       /* unsent readings waiting on flash */
    uint32_t capacity_left; /* ~how many more readings fit before eviction */
    uint32_t evicted;       /* oldest records dropped because the buffer was full */
} sf_info_t;

/** Fill `out` with the current storage picture. Safe to call any time. */
void sf_get_info(sf_info_t *out);

/** How many records have been evicted (dropped as oldest) since boot. */
uint32_t sf_evicted(void);

/** Delete ALL buffered readings. Destructive: these are unsent payment records.
 *  @return number of records deleted. */
uint32_t sf_clear_all(void);

#ifdef __cplusplus
}
#endif

#endif /* STORE_FORWARD_H */
