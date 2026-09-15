# application/ — what the device does with the internet

Empty until Step 4 of `docs/GSM_PORT_PLAN.md`.

This layer is **copied unchanged** from the WiFi product:

```
D:\ESP32S3-CLV4-WIFI\nitara.connector.esp32s3\components\application\
    fota/              HTTPS firmware update
    mqtt_client_svc/   MQTT publish + store-and-forward to flash
    wm_capture/        weight merge into the cloud message
```

## Why it can be copied without editing

It reaches the network **only** through `base/common/net_link.h`, never through
`wifi_sta` or `gsm` directly. The application asks "can I send right now?" and
does not know, or need to know, which transport answers.

That is also the test: if copying it here requires editing anything inside this
folder, something is wrong in `net_link` or in the `base/` sync (Step 3) — fix
that instead of patching the application layer.

## Before it will run

- `mqtt_secrets.h` created from `mqtt_secrets.h.example`
- a LittleFS partition in `partitions.csv` for store-and-forward (fails at
  runtime, not build time, if missing)
