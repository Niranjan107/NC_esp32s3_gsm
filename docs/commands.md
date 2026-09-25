# Command Reference

Every command the firmware accepts, as of 2.1.0.1003. Verified against the
dispatch chain in `cmd_parser.c` and the component registrations in
`wifi_sta.c` and `link_mode.c` — 32 commands, nothing omitted.

**Format:** `{"command":"name", ...}#` — the `#` terminates it. Over USB, `\r`
or `\n` also works.

**Reply:** `{"response_message":"<name>","status_code":<n>,"data":<json>}`
where `status_code` is `0` for success and `-1` for failure. Some commands
reply without `data`; two (`diag`, `self_diagnosis`) emit a raw JSON object
instead of a wrapped response.

**Transports:** BLE, USB console, and MQTT (`clv4/<id>/cmd`). Six commands are
refused over MQTT — marked below.

---

## Connectivity

| Command | Syntax | MQTT |
|---|---|---|
| Switch link | `{"command":"set_link_mode","mode":"gsm"}#` | refused |
| Read link | `{"command":"get_link_mode"}#` | ok |
| Set WiFi | `{"command":"wifi_config","ssid":"Net","password":"pass"}#` | refused |
| WiFi state | `{"command":"wifi_status"}#` | ok |
| Forget WiFi | `{"command":"wifi_erase"}#` | refused |
| Everything | `{"command":"diag"}#` | ok |

**`set_link_mode`** — `mode` is `gsm`, `wifi` or `off`. Reply is
`{"mode":"wifi","state":"switching"}` and arrives *before* the switch, which
takes up to 30 seconds. Poll `get_link_mode` for the outcome. Already in that
mode gives `"state":"unchanged"`. An unavailable mode — including `wifi` on a
GSM-only build — gives `set_link_mode_failed` with `{"reason":"unknown_mode"}`.

**`wifi_config`** — `password` is optional for an open network. Works in any
mode: with WiFi not running the credentials are stored untested and the reply
is `{"state":"saved"}`; with WiFi running they are tested first and the reply
is `{"state":"connecting"}`. The app should treat `saved` as unproven.

**`wifi_status`** — running:
`{"connected":true,"bars":4,"rssi":-48,"cloud":true}`. Not running:
`{"running":false,"provisioned":true,"ssid":"Site Router"}`. `cloud` is the
field that matters — `connected` only means the router was joined.

**`diag`** — emits a raw object, not a wrapped response: the active link and
its details, MQTT counters, buffer state, BLE mode, store-forward flag and the
full port configuration.

---

## Port configuration

| Command | Syntax | MQTT |
|---|---|---|
| Milk analyser | `{"command":"ma_port_config","model":1002,"baud_rate":1200,"data_bits":8,"stop_bits":1,"parity":0,"stream":0}#` | ok |
| Weighing machine | `{"command":"wm_port_config","model":9001,"baud_rate":9600,"data_bits":8,"stop_bits":1,"parity":0,"stream":1}#` | ok |
| Printer | `{"command":"printer_port_config","model":8000,"baud_rate":1200,"data_bits":8,"stop_bits":1,"parity":0,"stream":0}#` | ok |
| Read all | `{"command":"get_current_config"}#` | ok |
| Defaults | `{"command":"reset_config"}#` | refused |

`parity`: 0 none, 1 odd, 2 even. `stream`: 0 single-read, 1 continuous.

**All six keys are required**, including `stream` on the printer, which ignores
the value but fails the command if it is absent. Replies are
`<device>_config_success` or `<device>_config_fail`, with no data.

**`get_current_config`** sends four replies in sequence — `get_ma_config`,
`get_wm_config`, `get_printer_config`, then `get_current_config_success`. The
app must expect all four.

**`reset_config`** restores meter and printer defaults. WiFi credentials, link
mode and BLE mode are **not** affected.

---

## Printer

| Command | Syntax |
|---|---|
| Print | `{"command":"print_receipt","data":"Text to print"}#` |
| Status | `{"command":"check_printer_status"}#` |
| Reprint last | `{"command":"reprint_last_receipt"}#` |

`data` must be a string. `check_printer_status` replies with a bare string —
`"ready"` or `"not_ready"` — not an object. `reprint_last_receipt` gives
`reprint_fail_no_receipt` if nothing is cached.

---

## Device information

| Command | Syntax | Reply |
|---|---|---|
| Device ID | `{"command":"ncle_get_unique_id"}#` | `"AABBCCDDEEFF"` |
| Version | `{"command":"get_firmware_version"}#` | `"2.1.0.1003"` |
| Battery | `{"command":"get_battery_status"}#` | `{"pct":85}` |
| Hardware check | `{"command":"self_diagnosis"}#` | see below |

**`get_battery_status`** replies under `response_message:"battery"`, not the
command name.

**`self_diagnosis`** sends an acknowledgement, then a raw object with firmware
version, hardware ID, and the connected state of the meters, printer and
battery. Note the field is `Printer_status` with a capital P.

---

## Reading delivery

| Command | Syntax | MQTT |
|---|---|---|
| BLE mode | `{"command":"set_ble_data","mode":"auto"}#` | refused |
| Read BLE mode | `{"command":"get_ble_data"}#` | ok |
| Buffer on/off | `{"command":"set_store_forward","enable":true}#` | refused |
| Buffer count | `{"command":"get_storage_info"}#` | ok |
| Wipe buffer | `{"command":"clear_buffer","confirm":true}#` | ok |

### Where readings go

Two independent rules.

**Flash** follows the **link** — the internet connection, not the broker:

| Link | Stored? |
|---|---|
| Up | yes, deleted on the broker's acknowledgement |
| Down | no — BLE carries it, the app owns delivery *(except `ble_data=off`: stored)* |

A reading is stored whether or not the broker is reachable. That is the point:
it survives a broker outage and is sent when the broker returns.

**BLE** follows the **mode**:

| Mode | Sent to the phone | Flash |
|---|---|---|
| `auto` *(default)* | only while the broker is unreachable | on |
| `always` | every reading | on |
| `app` | every reading | **off** |
| `off` | never | on |

`auto` keeps BLE quiet while the cloud is carrying readings, so an app that
forwards to the server cannot duplicate what the device already sent.

`app` is for a site with no SIM and no WiFi, ever — the phone is the only
delivery path. It switches the buffer off, because with no network the device
would fill its ~2,800-record store over a few weeks and then evict in a loop
for readings nothing will ever deliver. **Switching to `app` deletes whatever
is already buffered**; the reply reports how many in `"cleared"`.

`off` is for a site that wants readings on the server only, never on a phone.
With the link down the reading is still stored — flash is its only copy — and
sent when the link returns.

### Where readings go — full table

**`set_link_mode`** decides which radio runs:

| link_mode | GSM | WiFi | BLE | Internet |
|---|---|---|---|---|
| `gsm` *(default)* | on | off | on | via SIM |
| `wifi` | off | on | on | via router |
| `off` | off | off | on | none |

**`set_ble_data`** in `gsm` or `wifi` mode:

| ble_data | Internet + broker up | Internet up, broker down | Internet down |
|---|---|---|---|
| `auto` *(default)* | BLE ⛔ · Flash ✅ | BLE ✅ · Flash ✅ | BLE ✅ · Flash ⛔ |
| `always` | BLE ✅ · Flash ✅ | BLE ✅ · Flash ✅ | BLE ✅ · Flash ⛔ |
| `app` | BLE ✅ · Flash ⛔ | BLE ✅ · Flash ⛔ | BLE ✅ · Flash ⛔ |
| `off` | BLE ⛔ · Flash ✅ | BLE ⛔ · Flash ✅ | BLE ⛔ · Flash ✅ |

**`set_ble_data`** in `link_mode=off`:

| ble_data | BLE | Flash |
|---|---|---|
| `auto` / `always` / `app` | ✅ | ⛔ |
| `off` | ⛔ | ⛔ |

The rules behind it:

- **Flash** follows the internet: up → stored, deleted on the broker's
  acknowledgement; down → not stored, the app owns delivery.
- **Exception:** with `ble_data=off` nothing goes to the phone, so the reading
  is stored even with the internet down — flash is its only copy.
- **`link_mode=off`** never stores: no internet, ever, so nothing could deliver it.
- `app` turns storage off permanently by changing `store_forward`.
- `link_mode=off` + `ble_data=off` gives the board no delivery path at all — not
  a configuration to deploy.

| Site | link_mode | ble_data |
|---|---|---|
| Normal, has a SIM | `gsm` | `auto` |
| SIM failed, has WiFi | `wifi` | `auto` |
| Operator needs live readings on the phone | `gsm` | `always` |
| Server only, no phone access | `gsm` | `off` |
| No network ever, phone is the only path | `off` | `auto` |

**`clear_buffer`** needs `"confirm":true` as a JSON boolean — the number `1` is
rejected. It deletes unsent milk readings from LittleFS only; meter, printer and
WiFi settings live in NVS and are untouched. Check `get_storage_info` first.

---

## Firmware update over the network

| Command | Syntax |
|---|---|
| Update | `{"command":"fota_start","url":"https://.../firmware.bin"}#` |
| Revert | `{"command":"fota_rollback"}#` |

Progress is published to `clv4/<id>/resp` and BLE as
`{"fota":"downloading","percent":N}` in 10% steps, then
`{"fota":"success"}` before rebooting.

Errors: `"missing url"`, `"fota_start_fail_busy"` if one is already running,
`"start failed"` otherwise. `fota_rollback` gives `"no_previous_firmware"` if
there is nothing to revert to.

Measured on hardware with a 1.77 MB image: 422 s over GSM, 307 s over WiFi.
Both allowed over MQTT.

---

## Firmware update over BLE

For a device with no network. The app sends the image in chunks.

| Command | Syntax |
|---|---|
| Start | `{"command":"ota_begin","size":1808832}#` |
| Send chunk | `{"command":"ota_write","seq":0,"data":"<base64>"}#` |
| Finish | `{"command":"ota_end"}#` |
| Cancel | `{"command":"ota_abort"}#` |
| Progress | `{"command":"ota_status"}#` |

Each chunk must decode to **512 bytes or fewer** — larger fails with
`"decode failed"`. `ota_write` replies `{"pct":N,"seq":N}`. `ota_end` verifies
the image and reboots after one second.

---

## Development

| Command | Syntax |
|---|---|
| MA frame timing | `{"command":"get_cycle_timing"}#` |
| Timing logs | `{"command":"set_timing_debug","enable":true}#` |

`get_cycle_timing` reports the milk-reading cycle in milliseconds, with `-1`
where a stage has not run.

---

## Refused over MQTT

Six commands, each of which could cut the device off from the cloud:

`wifi_config` · `wifi_erase` · `reset_config` · `set_ble_data` ·
`set_store_forward` · `set_link_mode`

They answer `command_not_allowed_remotely`. Everything else is allowed —
including `fota_start`, `fota_rollback` and `clear_buffer`.

---

## Errors common to every command

| Reply | Cause |
|---|---|
| `invalid_json_string` | the JSON did not parse |
| `command_not_found` | no `"command"` field, or it is not a string |
| `undefined_command` | the name is not recognised |

---

## Known gap: `apn_config` does not exist

GSM fault messages tell the operator to set the APN from the mobile app:

```
-> set it over BLE/USB: {"command":"apn_config",...}
"Set the correct APN from the mobile app (apn_config)"
```

**There is no such command.** Nor `sim_status` or `gsm_status`, which
`docs/GSM_PORT_PLAN.md` also lists. The underlying functions exist
(`gsm_apn_set_and_connect`, `gsm_apn_store`, `gsm_apn_get`) but nothing is
wired to JSON, so sending `apn_config` returns `undefined_command`.

The APN is discovered automatically by trying a list of candidates, so the
command is not needed — but the message is misleading and should either be
corrected or the command added. SIM and signal state are visible through
`diag` meanwhile.
