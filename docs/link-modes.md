# Link Modes — GSM, WiFi, or Neither

How the connector chooses where readings go, and the commands the mobile app
uses to change it.

---

## The three modes

The device runs **exactly one** connectivity stack at a time. This is a memory
constraint rather than a preference — see [Why only one](#why-only-one) below.

| Mode | What runs | Readings go to | Free heap |
|---|---|---|---|
| **gsm** *(default)* | Cellular modem | the server, over GSM | ~90 KB |
| **wifi** | WiFi radio | the server, over WiFi | ~64 KB |
| **off** | Neither | the app, over BLE | ~105 KB |

The operator selects the mode from the app. It is stored on the device and
survives a reboot, because field machines are switched off between the morning
and evening collection sessions — a mode that reset each boot would mean
re-switching twice a day.

**BLE always works, in every mode.** It is how the operator changes the mode,
and the way back from a WiFi whose router has been replaced.

---

## Typical use

A site's SIM stops working. The operator:

1. Connects to the device over BLE
2. Sends the WiFi network name and password
3. Switches the device to wifi mode

The device keeps sending readings to the same server, over the site's WiFi
instead of the SIM. When the SIM is replaced, the operator switches back.

---

## Commands

### Enter WiFi credentials

```json
{"command":"wifi_config","ssid":"<network name>","password":"<password>"}
```

Works in **any** mode. If WiFi is not currently running the credentials are
stored without being tested:

```json
{"response_message":"wifi_config","status_code":0,"data":{"state":"saved"}}
```

If WiFi **is** running they are tested first and saved only on success:

```json
{"response_message":"wifi_config","status_code":0,"data":{"state":"connecting"}}
```

The distinction matters to the app: `"saved"` means stored but unproven — the
password could still be wrong, and that will only show when the mode switches.

### Check WiFi

```json
{"command":"wifi_status"}
```

While WiFi is not running:
```json
{"running":false,"provisioned":true,"ssid":"Site Router"}
```

While it is:
```json
{"connected":true,"bars":4,"rssi":-48,"cloud":true}
```

`cloud` is the field that matters — `connected` only means the device joined
the router, while `cloud` means the server is actually reachable. A router with
no internet gives `connected:true, cloud:false`.

### Forget WiFi credentials

```json
{"command":"wifi_erase"}
```

### Read the current mode

```json
{"command":"get_link_mode"}
```
```json
{"mode":"gsm","stored":"gsm"}
```

`mode` is what is running now; `stored` is what the device will boot into.
They differ only if a switch failed and the device fell back.

### Change the mode

```json
{"command":"set_link_mode","mode":"gsm"}
{"command":"set_link_mode","mode":"wifi"}
{"command":"set_link_mode","mode":"off"}
```

The reply comes back immediately, before the switch happens:

```json
{"response_message":"set_link_mode","status_code":0,"data":{"mode":"wifi","state":"switching"}}
```

**The switch then takes up to 30 seconds** — mostly powering the modem down.
The app should not treat the reply as "done"; poll `get_link_mode` or
`wifi_status` to see the outcome.

Already in that mode:
```json
{"mode":"gsm","state":"unchanged"}
```

A mode this build does not have:
```json
{"response_message":"set_link_mode_failed","status_code":-1,"data":{"reason":"unknown_mode"}}
```

**`set_link_mode` is refused over MQTT** — BLE and USB only. Switching tears
down the link the command arrived on, and a new mode that cannot connect would
leave the device unreachable from the cloud with nobody on site expecting it.
`get_link_mode` is allowed remotely.

### Everything at once

```json
{"command":"diag"}
```

Reports the active link, its details, MQTT state, the reading buffer, and the
full device configuration.

---

## Measured behaviour

All figures from hardware, firmware 2.1.0.1000.

| | GSM | WiFi |
|---|---|---|
| Boot to server connected | ~36 s | **9.7 s** |
| Switch away from | 19 s | 1 s |
| Switch to | 8 s | 26 s |
| Free heap while running | 89.7 KB | 63.9 KB |

WiFi reconnects almost four times faster, because there is no modem to power up
and no APN to negotiate.

---

## What does not change with the mode

**Readings.** Every reading is written to flash first and deleted only once the
server confirms it. That holds on either link, unchanged — the application layer
asks only "can I send?" and never learns which radio answered.

**The offline rule.** With no link up, readings go to the app over BLE and are
*not* buffered — the app owns delivery from there. That applies in off mode and
during any outage, on either link.

**BLE.** Unaffected in all three modes.

---

## Why only one

Both network stacks in memory at once leaves 43 KB of heap free. A firmware
update needs about 45 KB to download, so a device with both running could not
update itself.

Starting WiFi only when it is selected keeps gsm mode at 90 KB and wifi mode at
64 KB, both clear of that floor. This is why there is no automatic failover: it
would require both stacks resident.

---

## Building without WiFi

Set `CONFIG_NCLE_WIFI_ENABLE=n` for a GSM-only product. Verified on hardware:

| | WiFi build | GSM-only build |
|---|---|---|
| Firmware size | 1,761 KB | 1,469 KB |
| Free heap at boot | 89.7 KB | **123.7 KB** |
| WiFi commands registered | 3 | 0 |
| `esp_wifi` symbols linked | yes | **none** |

The GSM-only build behaves exactly as the firmware did before this feature
existed. `set_link_mode` still offers `gsm` and `off` and refuses `wifi` with
`unknown_mode`; `off` is not a WiFi feature.

A device whose stored mode is `wifi`, re-flashed with a GSM-only build, rewrites
its stored mode to `gsm` on the next boot rather than failing to bring anything
up every time.
