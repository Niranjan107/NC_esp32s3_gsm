/**
 * @file store_forward.c
 * @brief Offline reading buffer on LittleFS (see store_forward.h).
 *
 * One file per reading, named by a zero-padded sequence (00000001.json ...) so
 * lexical order == arrival order (FIFO). We store the EXACT publish JSON and
 * replay it unchanged, so the format (WM / MA text / MA binary) is irrelevant
 * and the server's MD5 dedup still matches on a resend.
 */
#include "store_forward.h"
#include "sdkconfig.h"

#if defined(CONFIG_NCLE_MQTT_ENABLE) && defined(CONFIG_NCLE_MQTT_STORE_FORWARD)

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <unistd.h>
#include "esp_log.h"
#include "esp_littlefs.h"

static const char *TAG = "SF";

#define SF_BASE     "/sf"
#define SF_PART     "storage"
#define SF_MAX_REC  1500       /* safety cap per record (real max ~400 B) */
#define SF_HEADROOM 8192       /* keep this much free -> evict oldest when full */
#define SF_MAX_EVICT 8         /* records evicted per write, at most (see sf_buffer) */

static bool     s_mounted   = false;
static uint32_t s_write_seq = 1;          /* next sequence number to write */
static volatile uint32_t s_count = 0;     /* buffered records */
static volatile uint32_t s_evicted = 0;   /* oldest records dropped because full */

/* sf_buffer() evicts the oldest record when the partition is nearly full, so it
 * needs these before they are defined below. */
bool sf_oldest(char *buf, int bufsz, int *out_len, char *path_out, int path_sz);
void sf_delete(const char *path);

/* Scan the directory: set next write seq = max+1 and count existing records. */
static void sf_scan(void)
{
    DIR *d = opendir(SF_BASE);
    if (!d) {
        return;
    }
    struct dirent *e;
    uint32_t maxseq = 0, count = 0;
    while ((e = readdir(d)) != NULL) {
        if (e->d_name[0] == '.') {
            continue;
        }
        uint32_t seq = strtoul(e->d_name, NULL, 10);
        if (seq > maxseq) {
            maxseq = seq;
        }
        count++;
    }
    closedir(d);
    s_write_seq = maxseq + 1;
    s_count = count;
}

void sf_init(void)
{
    esp_vfs_littlefs_conf_t conf = {
        .base_path        = SF_BASE,
        .partition_label  = SF_PART,
        .format_if_mount_failed = true,
        .dont_mount       = false,
    };
    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LittleFS mount failed on '%s': %s", SF_PART, esp_err_to_name(ret));
        return;
    }
    s_mounted = true;
    sf_scan();

    size_t total = 0, used = 0;
    esp_littlefs_info(SF_PART, &total, &used);
    ESP_LOGI(TAG, "Store-and-forward ready: %lu buffered, %u/%u bytes used",
             (unsigned long)s_count, (unsigned)used, (unsigned)total);
}

bool sf_available(void) { return s_mounted; }
uint32_t sf_count(void) { return s_count; }

void sf_buffer(const char *json, int len)
{
    if (!s_mounted || !json || len <= 0) {
        return;
    }
    if (len > SF_MAX_REC) {
        ESP_LOGW(TAG, "record too big (%d B), dropped", len);
        return;
    }
    /* FIFO eviction when nearly full: make room by deleting the OLDEST records
     * rather than refusing the new one.
     *
     * The reading arriving now is a farmer standing at the scale - refusing it
     * means this collection is never recorded anywhere. The oldest stuck
     * records have been waiting longest (days, at 1 MB / ~2800 readings) and are
     * the ones most likely already reconciled from the printed receipts. So when
     * something must go, it is the oldest.
     *
     * Bounded loop: never evict more than a handful per write, so a corrupt
     * filesystem cannot turn one reading into an unbounded delete storm. */
    size_t total = 0, used = 0;
    int evicted = 0;
    while (esp_littlefs_info(SF_PART, &total, &used) == ESP_OK &&
           total > 0 && (total - used) < (size_t)(len + SF_HEADROOM) &&
           s_count > 0 && evicted < SF_MAX_EVICT) {
        char oldest[48];
        int dummy_len = 0;
        if (!sf_oldest(NULL, 0, &dummy_len, oldest, sizeof(oldest))) {
            break;                      /* nothing to evict */
        }
        ESP_LOGW(TAG, "buffer full (%u/%u B) - evicting oldest %s",
                 (unsigned)used, (unsigned)total, oldest);
        sf_delete(oldest);
        s_evicted++;
        evicted++;
    }

    char path[40];
    snprintf(path, sizeof(path), SF_BASE "/%010lu.json", (unsigned long)s_write_seq);
    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "write open failed: %s", path);
        return;
    }
    size_t w = fwrite(json, 1, (size_t)len, f);
    fclose(f);
    if (w != (size_t)len) {
        ESP_LOGE(TAG, "short write, removing %s", path);
        unlink(path);
        return;
    }
    s_write_seq++;
    s_count++;
    ESP_LOGI(TAG, "Buffered reading -> %s (buffered=%lu)", path, (unsigned long)s_count);
}

/* Find the oldest record. Pass buf == NULL to get only its path (used by the
 * eviction path in sf_buffer, which deletes without reading the contents). */
bool sf_oldest(char *buf, int bufsz, int *out_len, char *path_out, int path_sz)
{
    if (!s_mounted || !path_out || path_sz <= 0) {
        return false;
    }
    if (buf && bufsz <= 1) {
        return false;
    }
    DIR *d = opendir(SF_BASE);
    if (!d) {
        return false;
    }
    struct dirent *e;
    uint32_t minseq = 0xFFFFFFFFu;
    char minname[40] = {0};
    while ((e = readdir(d)) != NULL) {
        if (e->d_name[0] == '.') {
            continue;
        }
        uint32_t seq = strtoul(e->d_name, NULL, 10);
        if (seq < minseq) {
            minseq = seq;
            strncpy(minname, e->d_name, sizeof(minname) - 1);
        }
    }
    closedir(d);
    if (minname[0] == '\0') {
        return false;
    }

    snprintf(path_out, path_sz, SF_BASE "/%s", minname);

    if (buf == NULL) {
        return true;        /* path-only: caller just wants to delete it */
    }

    FILE *f = fopen(path_out, "rb");
    if (!f) {
        return false;
    }
    int n = (int)fread(buf, 1, (size_t)(bufsz - 1), f);
    fclose(f);
    if (n <= 0) {
        /* empty/garbage record - discard it so we don't loop on it */
        unlink(path_out);
        if (s_count) s_count--;
        return false;
    }
    buf[n] = '\0';
    *out_len = n;
    return true;
}

void sf_delete(const char *path)
{
    if (!path) {
        return;
    }
    if (unlink(path) == 0 && s_count) {
        s_count--;
    }
}

uint32_t sf_evicted(void) { return s_evicted; }

void sf_get_info(sf_info_t *out)
{
    if (!out) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->records = s_count;
    out->evicted = s_evicted;

    size_t total = 0, used = 0;
    if (!s_mounted || esp_littlefs_info(SF_PART, &total, &used) != ESP_OK || total == 0) {
        return;                          /* not mounted -> zeros, used_pct 0 */
    }

    out->total_bytes = (uint32_t)total;
    out->used_bytes  = (uint32_t)used;
    out->free_bytes  = (uint32_t)(total > used ? total - used : 0);
    out->used_pct    = (uint8_t)((used * 100) / total);

    /* How many more readings fit before eviction starts. Uses a realistic
     * record size rather than the 1500 B safety cap, and honours the headroom
     * the writer keeps free, so the number matches what actually happens. */
    const uint32_t typical_rec = 400;
    uint32_t usable = (out->free_bytes > SF_HEADROOM)
                      ? (out->free_bytes - SF_HEADROOM) : 0;
    out->capacity_left = usable / typical_rec;
}

uint32_t sf_clear_all(void)
{
    if (!s_mounted) {
        return 0;
    }
    DIR *d = opendir(SF_BASE);
    if (!d) {
        return 0;
    }

    /* Collect names first: deleting while walking the directory stream is not
     * safe on LittleFS. Batch-and-restart keeps memory flat on a full buffer. */
    uint32_t deleted = 0;
    for (;;) {
        char names[16][40];
        int n = 0;
        struct dirent *e;
        while (n < 16 && (e = readdir(d)) != NULL) {
            if (e->d_name[0] == '.') {
                continue;
            }
            strncpy(names[n], e->d_name, sizeof(names[0]) - 1);
            names[n][sizeof(names[0]) - 1] = '\0';
            n++;
        }
        closedir(d);

        for (int i = 0; i < n; i++) {
            char path[48];
            snprintf(path, sizeof(path), SF_BASE "/%s", names[i]);
            if (unlink(path) == 0) {
                deleted++;
                if (s_count) s_count--;
            }
        }
        if (n < 16) {
            break;                       /* directory exhausted */
        }
        d = opendir(SF_BASE);            /* restart the walk after deletions */
        if (!d) {
            break;
        }
    }

    ESP_LOGW(TAG, "Buffer cleared on request - %lu unsent readings deleted",
             (unsigned long)deleted);
    return deleted;
}

#else  /* store-and-forward disabled -> no-op stubs */

void sf_init(void) {}
bool sf_available(void) { return false; }
uint32_t sf_count(void) { return 0; }
void sf_buffer(const char *json, int len) { (void)json; (void)len; }
bool sf_oldest(char *b, int bs, int *ol, char *p, int ps)
{ (void)b; (void)bs; (void)ol; (void)p; (void)ps; return false; }
void sf_delete(const char *path) { (void)path; }
uint32_t sf_evicted(void) { return 0; }
void sf_get_info(sf_info_t *out) { if (out) memset(out, 0, sizeof(*out)); }
uint32_t sf_clear_all(void) { return 0; }

#endif
