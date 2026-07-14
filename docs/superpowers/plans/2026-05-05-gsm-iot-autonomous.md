# GSM IoT Autonomous HTTP Mode — Implementation Plan

**Date:** 2026-05-05
**Status:** Draft, pending user approval
**Goal:** Connector runs autonomously over GSM/HTTPS. No BLE involvement in the data flow. AWS used as test backend; production server can replace it later by changing URLs only.

---

## 1. Current state (already working — do not break)

- ✅ GSM connectivity: PWRKEY, RESET, AT command engine, PDP context
- ✅ HTTPS POST + GET via Quectel `AT+QHTTP*`
- ✅ BLE-triggered `gsm_send_to_print`: POST data + read response + print
- ✅ Modular `gsm_task` with periodic alive/signal/network polling
- ✅ Status callback to BLE
- ✅ Reset polarity Kconfig flag
- ✅ Race-free UART mutex (read body inside mutex)

**Direction 1 (Connector → AWS, BLE-triggered) stays as-is.** No changes to `gsm_send_to_print` or related code.

---

## 2. What we are adding

```
                  ┌────────────────────────────────────────────────┐
                  │           ESP32 Connector                       │
                  │                                                 │
                  │  Direction 1 (existing, BLE-triggered):         │
                  │     gsm_send_to_print → POST data → AWS         │
                  │                                                 │
                  │  Direction 2 (new, AUTONOMOUS — no BLE):        │
                  │     ┌────────── Autonomous HTTPS loop ─────────┐│
                  │     │  on boot:  heartbeat POST                 ││
                  │     │  every Ns: fetch GET   → if data, print   ││
                  │     │  after print: ack POST                    ││
                  │     └───────────────────────────────────────────┘│
                  └────────────────────────────────────────────────┘
```

The whole flow uses **HTTPS only** — same protocol stack you already have working. URLs are Kconfig-configurable so swapping AWS for the production backend is one config change later.

---

## 3. Goals

1. Connector auto-connects to GSM at boot (no BLE command needed)
2. Connector sends "online" heartbeat POST to AWS on connect
3. Connector polls AWS every N seconds for print jobs
4. If poll returns non-empty body → print it → send ack POST
5. If 3 consecutive heartbeats/polls fail → log "network lost", attempt PDP reconnect
6. URL/auth changes never require firmware code changes — only `menuconfig` and reflash

---

## 4. Kconfig additions (in `main/Kconfig.projbuild`)

```
NCLE_GSM_AUTO_START                bool   default n
    # Start gsm_task on boot (no BLE required)

NCLE_GSM_AUTO_FETCH_ENABLE         bool   default n
    # Enable heartbeat + fetch + ack loop

NCLE_GSM_HEARTBEAT_URL             string default ""
    # POST on connect: "I'm online"

NCLE_GSM_FETCH_URL                 string default ""
    # GET periodically; non-empty body = print job

NCLE_GSM_ACK_URL                   string default ""
    # POST after print: "I printed successfully"

NCLE_GSM_FETCH_INTERVAL_S          int    default 30   range 10 3600
    # How often to poll the fetch URL

NCLE_GSM_AUTH_HEADER               string default ""
    # Optional: e.g. "Authorization: Bearer xxx"
    # Empty = no auth (fine for AWS testing)
```

All five `NCLE_GSM_*_URL`/`*_S`/`*_HEADER` keys depend on `NCLE_GSM_AUTO_FETCH_ENABLE`.

---

## 5. Code changes

### 5.1 `components/gsm/gsm_task.c`

Add new state:
```c
static bool s_heartbeat_sent_this_session = false;
static int  s_consecutive_http_failures   = 0;
static uint32_t s_ms_since_last_fetch     = 0;
```

Inside the existing `gsm_task_body()` poll loop:

1. **Right after PDP comes up:** if `!s_heartbeat_sent_this_session`, POST to `NCLE_GSM_HEARTBEAT_URL` with a small body like `{"deviceId":"<mac>","model":"NCLV4"}`. On success → set flag.

2. **Every iteration:** track elapsed ms. When `>= NCLE_GSM_FETCH_INTERVAL_S * 1000`:
   - Call `gsm_http_get(NCLE_GSM_FETCH_URL, ...)`
   - If HTTP 204 or empty body → nothing to do, reset failure counter
   - If HTTP 200 with body → `printer_print_with_special_chars(body)`, then POST to `NCLE_GSM_ACK_URL`
   - If HTTP error or timeout → increment failure counter

3. **If 3 consecutive failures:**
   - Log `"network lost, resetting PDP"`
   - Set `s_pdp_active = false`, `s_heartbeat_sent_this_session = false`
   - Next poll will re-activate PDP, then re-send heartbeat

### 5.2 `components/gsm/gsm.c`

Add a tiny helper:
```c
esp_err_t gsm_set_auth_header(const char *header_line);
```
Stores a static const char* used by `gsm_http_configure()` to append `"Authorization: ..."` via `AT+QHTTPCFG="customheader",...`.

If `NCLE_GSM_AUTH_HEADER` is empty, no header is added (current behavior).

### 5.3 `main/main.c`

After BLE init block, add (gated by `CONFIG_NCLE_GSM_AUTO_START`):

```c
#ifdef CONFIG_NCLE_GSM_AUTO_START
    gsm_task_set_status_callback(gsm_ble_status_callback, NULL);  // existing
    if (gsm_set_auth_header) {
        gsm_set_auth_header(CONFIG_NCLE_GSM_AUTH_HEADER);          // new
    }
    gsm_task_start();                                              // NEW: auto-start
#endif
```

This way the connector boots → GSM comes up → fetch loop begins. No BLE needed.

---

## 6. AWS Lambda changes

Update Lambda `nclv4-receipt` (or create separate functions) to handle three method+path combos:

```python
def lambda_handler(event, context):
    method = event['requestContext']['http']['method']
    path   = event['rawPath']

    # Heartbeat: device announces it's online
    if method == 'POST' and path.endswith('/heartbeat'):
        # Optional: store device-online timestamp in DynamoDB or env var
        return {'statusCode': 200, 'body': 'OK'}

    # Fetch: ESP32 polls for print jobs
    if method == 'GET'  and path.endswith('/fetch'):
        if os.environ.get('PRINT_NOW', 'no').lower() == 'yes':
            return {
                'statusCode': 200,
                'headers': {'Content-Type': 'text/plain'},
                'body': build_receipt_from_env_vars()
            }
        return {'statusCode': 204, 'body': ''}

    # Ack: ESP32 confirms print done
    if method == 'POST' and path.endswith('/ack'):
        # Optional: clear PRINT_NOW so we don't re-print on next poll
        return {'statusCode': 200, 'body': 'ACK'}

    # Legacy: existing /receipt for gsm_send_to_print
    if method == 'POST':
        return legacy_receipt_handler(event)

    return {'statusCode': 405, 'body': 'Method Not Allowed'}
```

API Gateway routes:
- `POST /heartbeat` → Lambda
- `GET  /fetch`     → Lambda
- `POST /ack`       → Lambda
- `POST /receipt`   → Lambda (existing)

Lambda env vars (operator-editable in AWS console):
- `PRINT_NOW`   = "yes" | "no"
- `WEIGHT`, `FAT`, `SNF`, `RATE` = receipt content

---

## 7. Operator workflow (end-to-end)

```
1. Power on connector
   → ESP32 boots
   → gsm_task starts automatically (CONFIG_NCLE_GSM_AUTO_START=y)
   → GSM connects to Airtel
   → POST to NCLE_GSM_HEARTBEAT_URL ("I'm online")
   → Begin polling NCLE_GSM_FETCH_URL every 30s

2. AWS console (operator):
   → Edit Lambda env vars: WEIGHT=2.5, FAT=4.5, PRINT_NOW=yes
   → Save

3. Within 30s:
   → ESP32 polls fetch URL
   → Receives receipt body
   → Prints on thermal printer
   → POST to NCLE_GSM_ACK_URL ("printed successfully")
   → Lambda /ack handler clears PRINT_NOW

4. Subsequent polls return HTTP 204, no spam printing.

5. Operator wants another receipt:
   → Edit env vars again, set PRINT_NOW=yes, save
   → Within 30s, prints again
```

---

## 8. Acceptance criteria

- [ ] Connector boots, GSM auto-connects without BLE intervention
- [ ] Heartbeat POST visible in AWS Lambda CloudWatch logs
- [ ] Fetch GET returns 204 when `PRINT_NOW=no` — no print, no spam
- [ ] Fetch GET returns receipt when `PRINT_NOW=yes` — printer prints
- [ ] Ack POST visible in CloudWatch after print
- [ ] If GSM connection drops, connector reconnects without reboot
- [ ] No regression in existing `gsm_send_to_print` BLE command
- [ ] Switching `NCLE_GSM_HEARTBEAT_URL` etc. in menuconfig and reflashing works without code changes — proves portable to production server

---

## 9. Effort estimate

| Item | Time |
|---|---|
| Add Kconfig flags | 10 min |
| Modify `gsm_task` (state + heartbeat + fetch + ack + retry) | 1 hour |
| Add `gsm_set_auth_header` + `customheader` integration | 15 min |
| Auto-start hook in `main.c` | 5 min |
| AWS Lambda update + 3 API Gateway routes | 20 min (operator) |
| End-to-end test + debug | 30 min |
| **Total** | **~2.5 hours** |

---

## 10. Out of scope (will NOT do)

- ❌ MQTT (deferred until HTTP polling is proven and approved by backend team)
- ❌ Auto-trigger of Direction 1 (upload) on WM/MA data arrival — keeps BLE-triggered as user requested
- ❌ Production server endpoints — using AWS for testing until backend team finalizes API spec
- ❌ Bearer/JWT token refresh logic — using static optional `NCLE_GSM_AUTH_HEADER` for testing
- ❌ Multipart/form-data POST + .zip wrapping — not needed for testing
- ❌ Per-device unique IDs in heartbeat (using BLE MAC for now is fine)
- ❌ Persistent print queue on server (single env-var slot for testing)
- ❌ OTA firmware update over the autonomous channel

---

## 11. Migration path (when backend team approves)

After demo + backend team architectural decision:

```
Current (testing):                        Migration target (production):
AWS Lambda + API Gateway      ─────►      Their backend server (HTTPS REST)
  - HEARTBEAT_URL = AWS                     - HEARTBEAT_URL = https://your-domain.com/connector/heartbeat
  - FETCH_URL     = AWS                     - FETCH_URL     = https://your-domain.com/connector/fetch
  - ACK_URL       = AWS                     - ACK_URL       = https://your-domain.com/connector/ack
  - AUTH_HEADER   = ""                      - AUTH_HEADER   = "Bearer <prod_token>"

Code changes required: NONE
Config changes required: 4 menuconfig values + reflash
```

---

## 12. Future work (not in this plan)

- **MQTT push** via AWS IoT Core — only if backend team prefers MQTT after seeing HTTP demo
  - Per-device X.509 certificates
  - Quectel `AT+QMTOPEN`/`QMTCONN`/`QMTSUB`/`QMTPUB`
  - True server-initiated push, lower SIM data usage at scale
  - Estimated effort: 6-8 hours, plus AWS IoT Core setup
- **Auto-trigger Direction 1** (upload to AWS automatically when WM+MA data arrives, no BLE)
- **Per-device cert provisioning pipeline** for production
- **Print queue** on server (multiple jobs queued, ESP32 fetches them in order)
- **Heartbeat-based device monitoring dashboard** (AWS CloudWatch dashboard)

---

## 13. Decision needed before starting

User confirms:
- [ ] HTTP-only build approved (no MQTT in this plan)
- [ ] AWS used for testing; production server is later swap-in
- [ ] Direction 1 (BLE-triggered upload) stays as-is, no changes
- [ ] Operator manually toggles AWS Lambda env vars to trigger print jobs (for demo)
- [ ] Acceptance criteria in §8 are correct and complete
- [ ] Out-of-scope items in §10 are correct

If approved → start implementation immediately (Kconfig → gsm_task → main.c → Lambda).

---

## 14. References

- Existing V1 spec: [`2026-04-29-gsm-component-v1-design.md`](../specs/2026-04-29-gsm-component-v1-design.md)
- Existing V1 plan: [`2026-04-29-gsm-component-v1.md`](2026-04-29-gsm-component-v1.md)
- Quectel EC200U HTTP application note (for `AT+QHTTP*` reference)
- AWS API Gateway: https://docs.aws.amazon.com/apigateway/
