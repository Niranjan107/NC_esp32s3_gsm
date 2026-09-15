#ifndef WM_CAPTURE_H
#define WM_CAPTURE_H

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Captures the WM weight during one MA transmission window, for merging into
 * the MA message's MQTT copy. Mirrors the mobile app's BLE flow: every valid
 * NON-ZERO value is stored in a temporary list; when the MA arrives we wait a
 * 200 ms grace period (stragglers still collected), select ONE weight with the
 * app's findRepeatingNumber() rule (oldest value repeated >=4 times, else the
 * first value within average +/- 0.5), then clear the list.
 * Thread-safe: start()/result() run on the MA RX task, feed() on the WM RX
 * task. WiFi-side only - never affects console/BLE. */

/* MA first byte: begin a new capture window (clears previous state). */
void wm_capture_start(void);

/* Tell the capture how the analyser frames its readings, so it can pick the
 * right grace period (menuconfig: NCLE_WM_GRACE_MS / _CONTINUOUS_MS).
 *   false - TIMEOUT-framed (1xxx/2xxx/5xxx): the window is already open for
 *           seconds while the receipt transmits, so a short grace is enough.
 *   true  - terminator-framed (3xxx PARENTHESES, 4xxx NEWLINE): the frame ends
 *           in ~40ms, so a longer grace is the only way the window collects
 *           real samples instead of falling back to the last known weight.
 * Called by the MA driver whenever the detection mode is (re)configured. */
void wm_capture_set_continuous(bool continuous);

/* Each WM output (cleaned value string, e.g. "+0008.50Kg") during the window. */
void wm_capture_feed(const char *wm_clean);

/* End the window: select ONE weight from the collected list using the app
 * team's rule (oldest repeated >=4x, else first within avg +/- 0.5, else the
 * last value) and clear the list. Returns true + fills `out` if a weight was
 * measured; false + empty `out` if no non-zero WM data arrived this window.
 * `sz` should be >= 24. */
bool wm_capture_result(char *out, size_t sz);

/* Build the combined cloud message: take the MA JSON and merge in this
 * transaction's settled WM weight. Writes the merged JSON into `out` and returns
 * its length (0 on error).
 *
 * >>> THIS IS THE SINGLE PLACE TO CHANGE THE CLOUD WIRE FORMAT <<<
 * When the app/server team specifies how the combined data should look (field
 * name, nesting, units, etc.), edit ONLY this function. Nothing else changes.
 * Current format: inserts "wm":"<value>" after the {"device":"ma"} prefix - the
 * "wm" name tells the server the quantity came from the weighing machine, not
 * from the milk analyzer. */
int wm_capture_merge_into_ma(const char *ma_json, int ma_len, char *out, size_t out_sz);

#ifdef __cplusplus
}
#endif

#endif /* WM_CAPTURE_H */
