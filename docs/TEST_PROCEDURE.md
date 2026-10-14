# Test Procedure — NCLite GSM Connector

**For:** the hardware team, testing a board before it goes to a site.

You do not need to know anything about the firmware. Follow the steps in
order, tick each result, and note anything that does not match. Every step
says what you should see and what it means if you see something else.

**Time:** about 2 hours 45 minutes for the full set. Section 1 alone is 15 minutes and
catches most faults.

---

## What you need

- The connector board, powered
- A SIM card with an active data plan, in the holder
- A USB cable to a PC
- A phone with the Nitara app, or any BLE terminal app
- A WiFi network you can connect to — a phone hotspot is fine
- The milk analyser and weighing machine, or their simulators

---

## How to send commands

Commands go in over the USB cable or over BLE from the phone. Both work the
same way.

**Over USB:** open a serial terminal at **115200 baud**, type the command, and
press Enter.

**Over BLE:** connect to the device named **NitaraCLE5G** and send the command
as text.

Every command looks like this and **must end with `#`**:

```
{"command":"diag"}#
```

The device answers with a line of JSON. If you see
`{"response_message":"undefined_command"}` you have mistyped the command name.

---

## Section 1 — First power-up (15 minutes)

### 1.1 Does it start?

Connect the USB cable and watch the terminal. Within 3 seconds you should see:

```
NCLite ESP32-S3 Starting...
Firmware version: 2.1.0.1003
```

**Write down the version number.** If nothing appears, check the cable and
that the terminal is set to 115200 baud.

| | |
|---|---|
| ✅ Pass | Version line appears |
| ❌ Fail | Nothing, or the text is garbled |

### 1.2 Do all the parts start?

Keep watching. You should see each of these:

```
WM Module: Enabled
MA Module: Enabled
Printer Module: Enabled
Battery Module: Enabled
BLE SPP Module: Enabled (Device: NitaraCLE5G)
```

| | |
|---|---|
| ✅ Pass | All five appear |
| ❌ Fail | Any one missing — note which |

### 1.3 How much memory is free?

Look for:

```
Memory: Free heap: 89328 bytes
```

**Write down the number.** It should be **between 85,000 and 95,000**.

| | |
|---|---|
| ✅ Pass | 85,000 – 95,000 |
| ⚠️ Check | Below 70,000 — report it, the board may still work but something is wrong |

### 1.4 Does the SIM work?

Wait up to 60 seconds. You should see the six-stage check:

```
[1/6] MODEM ......... OK
[2/6] SIM ........... OK
[3/6] SIGNAL ........ OK
[4/6] NETWORK ....... OK
[5/6] DATA .......... OK
[6/6] INTERNET ...... OK
```

**If any stage says FAILED, the line underneath tells you what to fix.** For
example:

```
[2/6] SIM ........... FAILED (absent)
  -> insert a SIM card
```

| Stage that fails | What to check |
|---|---|
| 1 MODEM | Module power (3.8–4.2 V) and the TX/RX wiring |
| 2 SIM | SIM is inserted the right way round and seated |
| 3 SIGNAL | Antenna is connected |
| 4 NETWORK | SIM is active and has credit |
| 5 DATA | SIM has a data plan |
| 6 INTERNET | Data plan not exhausted |

**A SIM with no data balance** still connects and gets an IP address — stages
1–5 pass — but nothing gets through. It looks like this:

```
=== PING RESULT: 4 sent, 0 received, 100% loss ===
=== HTTP REDIRECTED by the network (status 302) - carrier page, usually NO DATA BALANCE ===
[6/6] INTERNET ...... FAILED (IP assigned, but no data gets through)
>>> ACTION : Recharge the data pack - the SIM has no active data
GSM_TASK: ... data=1 (DATA mode) FAULT=no_internet (Recharge the data pack ...)
```

The LED shows **5 blinks**. **Recharge the SIM** — the device re-checks every
minute and prints `=== INTERNET RESTORED ===` by itself, with no restart.

### Reading the GSM status line

Every 10 seconds the log prints a line like this:

```
GSM_TASK: alive=1 sim=ready reg=1 rssi=17 bars=3 net=1 data=1 (DATA mode)
```

| Field | Meaning | Good value |
|---|---|---|
| `alive` | Modem is responding | `1` |
| `sim` | SIM card state: `ready`, `absent`, `pin_required` or `error` | `ready` |
| `reg` | Registered on a mobile network | `1` |
| `rssi` | Signal strength, 0–31 (99 = unknown). Below **10** is weak | 10 or more |
| `bars` | The same signal as 0–5 bars, like a phone | 3 or more |
| `net` | Network state: 1 home, 5 roaming, 2 searching, 3 refused, 0 not registered | `1` or `5` |
| `data` | Internet data link is up | `1` |
| `(DATA mode)` | The link is carrying internet traffic | — |

How `rssi` turns into `bars`:

| rssi | 0 or 99 | 1–6 | 7–12 | 13–18 | 19–24 | 25–31 |
|---|---|---|---|---|---|---|
| bars | 0 | 1 | 2 | 3 | 4 | 5 |

**In DATA mode the signal does not update.** While the internet link is up
the modem cannot be asked for its signal, so `rssi` and `bars` show the last
reading taken **just before** the link came up. They refresh the next time
the link drops and reconnects.

If something is wrong, the line ends with the fault and what to do, for
example:

```
... data=0 FAULT=sim_absent (Insert a SIM card into the holder)
```

**`rssi` below 10 (2 bars or fewer)** works, but the link may drop from time
to time. Check the antenna is screwed on firmly, or move it higher or nearer a
window.

### The same status, sent to the app over BLE

If a phone is connected over BLE, the device also sends this status to the
app as one JSON line:

```
{"device":"gsm","alive":1,"sim":"ready","registered":1,"data":1,"rssi":17,"bars":3,"net":1,"iccid":"8991...","fault":"ok","problem":"Everything working","action":"No action needed"}
```

It carries the same values as the log line, plus:

| Field | Meaning |
|---|---|
| `iccid` | The SIM card's serial number (printed on the SIM) — use it to tell SIMs apart |
| `fault` | Short name of the first thing that failed, or `ok` |
| `problem` | What is wrong, in plain words — the app can show it as is |
| `action` | What the user should do about it |

**When it is sent** — the device checks the modem every 10 seconds (the log
line), but only sends to the app when it matters, so the app is not flooded:

| When | Sent |
|---|---|
| Something changed: modem, SIM, registration, data link, fault, or the number of bars | Within **10 seconds** (at the next check) |
| Nothing changed | Every **60 seconds**, so the app knows the device is still there |

So on a working connection the app gets one message a minute; pull the SIM
and it hears within 10 seconds.

Small changes in `rssi` alone do not trigger a message; a change in `bars`
does.

`fault` values and what they mean:

| `fault` | Problem | LED (Section 7) |
|---|---|---|
| `ok` | Everything working | Solid ON |
| `modem_dead` | GSM module is not responding | 1 blink |
| `sim_absent` | No SIM card detected | 2 blinks |
| `sim_locked` | SIM card is PIN locked | 2 blinks |
| `sim_failure` | SIM card is present but faulty | 2 blinks |
| `no_signal` | No mobile signal at all | 3 blinks |
| `weak_signal` | Signal too weak — connection will drop | 3 blinks |
| `sim_barred` | Network refused this SIM | 4 blinks |
| `no_coverage` | No mobile network found here | 4 blinks |
| `no_data_link` | Registered, but could not get an IP address | 5 blinks |
| `no_internet` | Got an IP address, but no data is flowing | 5 blinks |

**To check it:** connect the app (or a BLE terminal such as nRF Connect) and
confirm a `{"device":"gsm",...}` line arrives within a minute, and that it
matches the log line.

### 1.5 Does it reach the server?

```
MQTT_SVC: Connected to broker
```

| | |
|---|---|
| ✅ Pass | The line appears within 60 seconds of power-up |
| ❌ Fail | It does not — note what the six stages said |

### 1.6 Check it all at once

Send:

```
{"command":"diag"}#
```

You get one long line back. Look for these three things:

- `"link":"gsm"` — the SIM is being used
- `"connected":true` inside `"mqtt"` — the server is reachable
- `"buffered":0` — no unsent readings

**Section 1 complete.** If everything passed, the board is working. The rest
tests specific features.

---

## Section 2 — Meters and printer (15 minutes)

### 2.1 Weighing machine

Put a weight on the scale, or run the simulator. In the terminal you should
see a line like:

```
{"device":"wm","data":"0001.25Kg","model":9001}
```

| | |
|---|---|
| ✅ Pass | A weight appears when the scale changes |
| ❌ Fail | Nothing — check the WM cable on **GPIO44** |

### 2.2 Milk analyser

Take a reading, or run the simulator. You should see a block of text:

```
{"device":"ma","data":"Provisional Acknowldgement Slip ... FAT: 4.10% SNF: 8.7% ...","model":1002}
```

Then immediately after:

```
SF: Buffered reading -> /sf/0000000001.json (buffered=1)
MQTT_SVC: Flushed /sf/0000000001.json (published=1, buffered left=0)
```

The first line means the reading was saved. The second means it reached the
server and the saved copy was deleted.

| | |
|---|---|
| ✅ Pass | Both lines appear, `buffered left=0` |
| ❌ Fail | No reading — check the MA cable on **GPIO39/40** |
| ❌ Fail | Buffered but never flushed — the server is not reachable |

### 2.3 Printer

```
{"command":"check_printer_status"}#
```

Expect `"ready"`. Then print something:

```
{"command":"print_receipt","data":"TEST PRINT 123"}#
```

| | |
|---|---|
| ✅ Pass | Paper comes out with the text on it |
| ❌ Fail | `"not_ready"` — check paper and the printer cable |

### 2.4 Battery

```
{"command":"get_battery_status"}#
```

Expect a percentage. On USB power without a battery fitted this reads 0% —
that is normal.

---

## Section 3 — Losing the network (20 minutes)

This is the important section. It proves readings are not lost when the
network fails.

### 3.1 Take the SIM out while it is running

With the device connected and working, **pull the SIM card out**.

Within about 30 seconds you should see:

```
GSM: PPP lost IP - data link is down
GSM_TASK: ... sim=absent ... FAULT=sim_absent (Insert a SIM card into the holder)
```

| | |
|---|---|
| ✅ Pass | The device notices within 60 seconds and says the SIM is absent |
| ❌ Fail | It keeps claiming to be connected — **report this** |

### 3.2 Take readings with no network

Take **three milk analyser readings** while the SIM is out.

You should see, for each one:

```
Link down - reading handed to the app over BLE (app=1)
```

**You should NOT see** `Buffered reading`. That is correct — with the network
down the reading goes to the phone app, which is responsible for it from
there.

| | |
|---|---|
| ✅ Pass | Each reading says "handed to the app" |
| ❌ Fail | Readings are silently dropped with no message |

### 3.3 Put the SIM back

Reinsert the SIM. The device should recover by itself:

```
GSM: Power OFF sequence...
GSM: Power ON sequence...
GSM_TASK: SIM state changed: absent -> ready
GSM: APN 'airtelgprs.com' WORKED
MQTT_SVC: Connected to broker
```

This takes **60 to 90 seconds**. The power cycle is deliberate — the modem
only looks at the SIM slot when it starts up.

| | |
|---|---|
| ✅ Pass | Back to the server within 2 minutes, no intervention |
| ❌ Fail | Still down after 3 minutes — **report this** |

### 3.4 Take a reading again

One more milk reading. It should buffer and flush as in step 2.2.

---

## Section 4 — Switching to WiFi (20 minutes)

Only if the site may use WiFi. Skip if the product is GSM-only.

### 4.1 Enter the WiFi details

While still on GSM, send:

```
{"command":"wifi_config","ssid":"YOUR_NETWORK","password":"YOUR_PASSWORD"}#
```

Expect `{"state":"saved"}`.

**`saved` means stored but not tested.** A wrong password will not show up
until the next step.

### 4.2 Switch to WiFi

```
{"command":"set_link_mode","mode":"wifi"}#
```

Expect `{"mode":"wifi","state":"switching"}` straight away, then over the
next 30 seconds:

```
LINK_MODE: switching gsm -> wifi
LINK_MODE: stopping GSM
GSM: Power OFF complete
WIFI_STA: Connected to AP: YOUR_NETWORK
WIFI_STA: Got IP: 192.168.1.50
MQTT_SVC: Connected to broker
```

| | |
|---|---|
| ✅ Pass | WiFi joins and the server is reached within 60 seconds |
| ❌ Fail | `Disconnected from AP` repeatedly — wrong password, or a 5 GHz network. **The device only works on 2.4 GHz.** |

### 4.3 Check it

```
{"command":"diag"}#
```

Look for `"link":"wifi"` and `"connected":true`.

### 4.4 Take a reading over WiFi

One milk reading. Same as step 2.2 — buffered, then flushed.

### 4.5 Switch off and on again

Power the device down, wait 5 seconds, power it up.

It should come back **on WiFi**, without being told:

```
LINK_MODE: stored mode: wifi
LINK_MODE: mode: wifi
```

| | |
|---|---|
| ✅ Pass | Comes back on WiFi by itself |
| ❌ Fail | Reverts to GSM — **report this** |

### 4.6 Switch back to GSM

```
{"command":"set_link_mode","mode":"gsm"}#
```

Should return to GSM within 60 seconds. **Leave the device on GSM** when you
finish — that is the normal setting for a site.

---

## Section 5 — Remote firmware update (30 minutes)

Only if you have been given a firmware file and a URL for it.

### 5.1 Note the current version

```
{"command":"get_firmware_version"}#
```

Write it down.

### 5.2 Start the update

```
{"command":"fota_start","url":"<the URL you were given>"}#
```

Expect `{"state":"started"}`, then progress every 10%:

```
FOTA: {"fota":"downloading","percent":10}
FOTA: {"fota":"downloading","percent":20}
...
FOTA: {"fota":"success"}
```

**This takes 5 to 8 minutes.** Do not power the device off during it.

| | |
|---|---|
| ✅ Pass | Reaches 100% and says `success` |
| ❌ Fail | Stops partway — note the percentage and any error |

### 5.3 Check the new version

The device reboots by itself. Check:

```
{"command":"get_firmware_version"}#
```

It should be the **new** version.

| | |
|---|---|
| ✅ Pass | Version has changed, device works normally |
| ❌ Fail | Still the old version, or it will not start |

---

## Section 6 — Leave it running (overnight, optional)

Leave the device powered and connected overnight with the terminal logging.

Next morning check:

- Still says `Connected to broker`
- `{"command":"diag"}#` shows `"buffered":0`
- Free heap has not dropped much — compare with step 1.3

| | |
|---|---|
| ✅ Pass | Still connected, memory steady |
| ⚠️ Report | Memory dropped by more than 10,000 bytes |
| ❌ Fail | Disconnected and did not recover |

---

## Section 7 — Status LED (20 minutes)

In GSM mode there is no app, so the LED is how a user knows the device is
online. Check that it tells the truth in every mode.

**What the LED means:**

| LED | Meaning |
|---|---|
| **Solid ON** | Connected — GSM or WiFi up, and the server (MQTT) reachable |
| **Double blink**, pause | Connecting |
| **1–5 blinks**, long pause | Fault — count the blinks (tables below) |
| **Heartbeat** (short flash every second) | BLE mode (`off`) — app only |
| **5 quick blinks** | A reading was received |
| **Fast steady blink** | Firmware update — do not unplug |

A fault code shows only after the fault has lasted **40 seconds**. Before that
the LED shows "connecting". So after each step below, **wait at least 40
seconds** before judging.

| GSM blinks | Problem | WiFi blinks | Problem |
|---|---|---|---|
| 1 | Modem not responding | 1 | No WiFi saved |
| 2 | SIM missing, locked or faulty | 2 | Cannot join the router |
| 3 | No signal / weak signal | 3 | WiFi up, no internet |
| 4 | SIM barred / no coverage | | |
| 5 | No data, or server not reachable | | |

Every change of pattern is also printed in the log, for example:

```
LED: connecting
LED: connected
LED: fault, code 2
```

### 7.1 GSM connected

In GSM mode with a working SIM and antenna, power the device on.

| | |
|---|---|
| ✅ Pass | Double blink, then **solid ON** once `Connected to broker` appears |
| ❌ Fail | Never goes solid, or goes solid before the server is connected |

### 7.2 GSM faults

With the device solid ON, try each fault in turn and put it back afterwards.
Wait for solid ON again before the next one.

| Do this | Expected LED (after 40 s) | Result |
|---|---|---|
| Pull the SIM out | 2 blinks | ☐ |
| Unplug the antenna | 3 blinks (may take longer — the modem holds on to a weak signal) | ☐ |
| Fit a SIM with no data plan (optional — only if you have one) | 5 blinks | ☐ |

Putting the fault right should bring the LED back to double blink, then
solid ON.

### 7.3 A reading while connected

Take a milk analyser reading.

| | |
|---|---|
| ✅ Pass | 5 quick blinks, then back to **solid ON** |

### 7.4 WiFi

Switch to WiFi mode (Section 4).

| Do this | Expected LED |
|---|---|
| Router working | Double blink, then **solid ON** |
| Switch the router off | 2 blinks after 40 s |
| Router back on | Back to solid ON |

### 7.5 BLE mode

Send `{"command":"set_link_mode","mode":"off"}#`.

| | |
|---|---|
| ✅ Pass | **Heartbeat** straight away (no fault code) |

Switch back to GSM afterwards: `{"command":"set_link_mode","mode":"gsm"}#`.

### 7.6 Firmware update

During the update in Section 5, watch the LED.

| | |
|---|---|
| ✅ Pass | **Fast steady blink** for the whole download |

---

## Section 8 — Every command (45 minutes)

This section tries **all 32 commands** the device accepts, one by one. Send
each command exactly as written (copy and paste it), compare the reply with
the "You should see" column, and tick the box.

- Send over **USB** or **BLE** — both work the same.
- Every command **ends with `#`**.
- `status_code` **0 = success**, **-1 = failure**.
- Values in replies (signal, battery %, ID) will differ on your board — check
  the **shape**, not the exact numbers.
- Do the groups **in order**. The last groups change settings; each says how to
  put things back.

The full technical detail for each command is in [commands.md](commands.md).

### 8.1 Device information

| ✓ | Send | You should see |
|---|---|---|
| ☐ | `{"command":"get_firmware_version"}#` | `{"response_message":"get_firmware_version","status_code":0,"data":"2.1.0.1003"}` — the version on the label |
| ☐ | `{"command":"ncle_get_unique_id"}#` | `"data":"AABBCCDDEEFF"` — 12 letters/numbers, different on every board. **Write it on the result sheet.** |
| ☐ | `{"command":"get_battery_status"}#` | `{"response_message":"battery",...,"data":{"pct":85}}` — note the reply name is `battery`. `0` on USB power with no battery is normal |
| ☐ | `{"command":"self_diagnosis"}#` | An acknowledgement, then one line with the firmware version, hardware ID, and the connected state of MA, WM, `Printer_status` and battery |
| ☐ | `{"command":"diag"}#` | One long line with the active link (`gsm`), its SIM and signal, MQTT counters, buffer state, BLE mode and all port settings |

### 8.2 Link mode and WiFi

| ✓ | Send | You should see |
|---|---|---|
| ☐ | `{"command":"get_link_mode"}#` | `"data":{"mode":"gsm","stored":"gsm"}` — `mode` is what runs now, `stored` what it starts in after a reboot |
| ☐ | `{"command":"wifi_status"}#` | While in GSM mode: `"data":{"running":false,"provisioned":false,...}` (`true` and an `ssid` if WiFi was saved before) |
| ☐ | `{"command":"wifi_config","ssid":"YourWiFi","password":"YourPassword"}#` — use your own network | `"data":{"state":"saved"}` (saved but not tested yet, because WiFi is not running) |
| ☐ | `{"command":"set_link_mode","mode":"wifi"}#` | `"data":{"mode":"wifi","state":"switching"}` straight away. The switch takes up to 30 s |
| ☐ | wait 30 s, then `{"command":"get_link_mode"}#` | `"data":{"mode":"wifi","stored":"wifi"}` |
| ☐ | `{"command":"wifi_status"}#` | `"data":{"connected":true,"bars":4,"rssi":-48,"cloud":true}` — **`cloud":true` is the one that matters** |
| ☐ | `{"command":"set_link_mode","mode":"wifi"}#` again | `"state":"unchanged"` — already in that mode |
| ☐ | `{"command":"set_link_mode","mode":"off"}#` | `"state":"switching"`, then `get_link_mode` shows `"off"`. LED: heartbeat |
| ☐ | `{"command":"set_link_mode","mode":"gsm"}#` | `"state":"switching"`, then `get_link_mode` shows `"gsm"`. LED: solid ON within ~1 min |
| ☐ | `{"command":"set_link_mode","mode":"xyz"}#` | `set_link_mode_failed` with `{"reason":"unknown_mode"}` — a wrong value is refused |

`wifi_erase` is tested in 8.7, because it removes the WiFi you just saved.

### 8.3 Meter and printer settings

First read what is set now, so you can put it back:

| ✓ | Send | You should see |
|---|---|---|
| ☐ | `{"command":"get_current_config"}#` | **Four replies** in a row: `get_ma_config`, `get_wm_config`, `get_printer_config`, then `get_current_config_success`. **Write down the MA, WM and printer values.** |

Now set each one. Use the values your meters actually need — the examples
below are common ones:

| ✓ | Send | You should see |
|---|---|---|
| ☐ | `{"command":"ma_port_config","model":1002,"baud_rate":1200,"data_bits":8,"stop_bits":1,"parity":0,"stream":0}#` | `ma_config_success` |
| ☐ | `{"command":"wm_port_config","model":9001,"baud_rate":9600,"data_bits":8,"stop_bits":1,"parity":0,"stream":1}#` | `wm_config_success` |
| ☐ | `{"command":"printer_port_config","model":8000,"baud_rate":1200,"data_bits":8,"stop_bits":1,"parity":0,"stream":0}#` | `printer_config_success` |
| ☐ | `{"command":"ma_port_config","model":1002,"baud_rate":1200}#` — keys missing on purpose | `ma_config_fail` — **all six settings are required** |
| ☐ | `{"command":"get_current_config"}#` | The four replies again, showing the values you just set |

`parity`: 0 none, 1 odd, 2 even. `stream`: 0 one reading at a time, 1
continuous. The printer needs `stream` too, even though it ignores it.

After setting the ports, take one reading from each meter (Section 2) to prove
they still work.

### 8.4 Printer

| ✓ | Send | You should see |
|---|---|---|
| ☐ | `{"command":"check_printer_status"}#` | `"data":"ready"` (printer connected and on) or `"data":"not_ready"` |
| ☐ | `{"command":"reprint_last_receipt"}#` **before** printing anything since power-on | `reprint_fail_no_receipt` — nothing to reprint yet |
| ☐ | `{"command":"print_receipt","data":"TEST RECEIPT 123"}#` | The printer prints **TEST RECEIPT 123**; reply `print_receipt_success` |
| ☐ | `{"command":"reprint_last_receipt"}#` | The printer prints **TEST RECEIPT 123** again; reply `reprint_success` |

### 8.5 Where readings go

| ✓ | Send | You should see |
|---|---|---|
| ☐ | `{"command":"get_ble_data"}#` | `"data":{"mode":"auto","store_forward":true}` — the defaults |
| ☐ | `{"command":"get_storage_info"}#` | `"data":{"total_bytes":...,"used_pct":0,"records":0,"capacity_left":...,"evicted":0}` — `records` is how many readings are stored; normally `0` while connected |
| ☐ | `{"command":"set_ble_data","mode":"always"}#` | Success. Take an MA reading — it now **also** appears on the phone even though the server is connected |
| ☐ | `{"command":"set_ble_data","mode":"off"}#` | Success. Take an MA reading — it does **not** appear on the phone |
| ☐ | `{"command":"set_ble_data","mode":"auto"}#` | Success — **put it back to `auto`** |
| ☐ | `{"command":"set_store_forward","enable":false}#` | Success |
| ☐ | `{"command":"set_store_forward","enable":true}#` | Success — **put it back to `true`** |
| ☐ | `{"command":"clear_buffer","confirm":1}#` | `"status_code":-1,"data":"confirm_required"` — `confirm` must be the word `true`, not `1` |
| ☐ | `{"command":"clear_buffer","confirm":true}#` | `"data":{"deleted":N}` — how many were removed. `get_storage_info` now shows `"records":0` |

**Do not send `"mode":"app"`** during this test: it turns storage off and
**deletes every stored reading**. It is only for sites with no SIM and no WiFi.

### 8.6 Development commands

| ✓ | Send | You should see |
|---|---|---|
| ☐ | `{"command":"get_cycle_timing"}#` | `"data":{"t0":...,"wm":...,"ma_rx":...,"print":...,"total":...}` in milliseconds; `-1` for a stage that has not run yet |
| ☐ | `{"command":"set_timing_debug","enable":true}#` | `"data":{"receipt_timing":true}`; extra timing lines appear in the log on the next reading |
| ☐ | `{"command":"set_timing_debug","enable":false}#` | `"data":{"receipt_timing":false}`; the extra lines stop |

### 8.7 Settings that are hard to undo — do these last

| ✓ | Send | You should see |
|---|---|---|
| ☐ | `{"command":"wifi_erase"}#` | Success. `wifi_status` now shows `"provisioned":false` |
| ☐ | `{"command":"reset_config"}#` | `reset_config_success`. `get_current_config` shows the factory port settings. **WiFi, link mode and BLE mode are not changed** |

After `reset_config`, **set the ports back** to the values you wrote down in
8.3, and take one reading from each meter.

### 8.8 Firmware update commands

`fota_start` is tested in **Section 5**. The others:

| ✓ | Send | You should see |
|---|---|---|
| ☐ | `{"command":"fota_start"}#` — no `url` on purpose | `"status_code":-1,"data":"missing url"` — refused safely |
| ☐ | `{"command":"fota_rollback"}#` **only after Section 5** | `"data":{"state":"rolling_back"}`, then the device reboots into the previous firmware — check with `get_firmware_version`. On a board that was never updated: `"data":"no_previous_firmware"` |
| ☐ | `{"command":"ota_status"}#` | `"data":{"state":0,"pct":0,"recv":0,"total":0}` — all zero when no update is running |
| ☐ | `{"command":"ota_abort"}#` | `"response_message":"ota_abort","status_code":0` — nothing happens, there is no update to cancel |

`ota_begin`, `ota_write` and `ota_end` send a new firmware **over BLE** in
hundreds of pieces. They cannot be typed by hand — test them only with the
Nitara app's BLE update feature, if the app has it. Otherwise mark N/A.

### 8.9 Commands from the server (MQTT)

Six commands are **refused when they come from the server**, because each one
could cut the device off from the cloud. If you can send commands from the
server side, send this one on `clv4/<device-id>/cmd`:

```
{"command":"set_link_mode","mode":"off"}
```

| ✓ | Expect |
|---|---|
| ☐ | `command_not_allowed_remotely` — and the device **stays** in GSM mode |

The six are: `wifi_config`, `wifi_erase`, `reset_config`, `set_ble_data`,
`set_store_forward`, `set_link_mode`. Everything else works from the server.

### 8.10 Mistakes

The device must answer mistakes clearly, not hang:

| ✓ | Send | You should see |
|---|---|---|
| ☐ | `{"command":"hello"}#` | `undefined_command` |
| ☐ | `{"cmd":"diag"}#` | `command_not_found` — the key must be `command` |
| ☐ | `{"command":"diag"#` — missing `}` | `invalid_json_string` |
| ☐ | `{"command":"apn_config","apn":"airtelgprs.com"}#` | `undefined_command` — **this command does not exist yet**, even though some GSM error messages mention it. The APN is found automatically |

### 8.11 Put everything back

Before the board leaves the bench, confirm:

| ✓ | Send | Must show |
|---|---|---|
| ☐ | `{"command":"get_link_mode"}#` | `gsm` |
| ☐ | `{"command":"get_ble_data"}#` | `auto` |
| ☐ | `{"command":"get_current_config"}#` | the port settings the site needs |
| ☐ | `{"command":"get_storage_info"}#` | `0` stored |

---

## Result sheet

Board serial number: ______________  Date: ____________

Tester: ______________

| Section | Result | Notes |
|---|---|---|
| 1.1 Starts | ☐ Pass ☐ Fail | Version: __________ |
| 1.2 All modules | ☐ Pass ☐ Fail | |
| 1.3 Memory | ☐ Pass ☐ Check | Free heap: __________ |
| 1.4 Six stages | ☐ Pass ☐ Fail | Failed stage: ______ |
| 1.4 GSM status on BLE | ☐ Pass ☐ Fail ☐ N/A | rssi / bars: ______ |
| 1.5 Reaches server | ☐ Pass ☐ Fail | |
| 2.1 Weighing machine | ☐ Pass ☐ Fail | |
| 2.2 Milk analyser | ☐ Pass ☐ Fail | |
| 2.3 Printer | ☐ Pass ☐ Fail | |
| 2.4 Battery | ☐ Pass ☐ Fail | |
| 3.1 Notices SIM removed | ☐ Pass ☐ Fail | |
| 3.2 Readings to app | ☐ Pass ☐ Fail | |
| 3.3 Recovers on SIM back | ☐ Pass ☐ Fail | Took: ______ sec |
| 4.2 Switches to WiFi | ☐ Pass ☐ Fail ☐ N/A | |
| 4.5 Remembers after reboot | ☐ Pass ☐ Fail ☐ N/A | |
| 5.2 Firmware update | ☐ Pass ☐ Fail ☐ N/A | |
| 6 Overnight | ☐ Pass ☐ Fail ☐ N/A | |
| 7.1 LED solid ON when connected | ☐ Pass ☐ Fail | |
| 7.2 LED fault codes (SIM / antenna / data) | ☐ Pass ☐ Fail | Wrong code: ______ |
| 7.3 LED reading blink | ☐ Pass ☐ Fail | |
| 7.4 LED in WiFi mode | ☐ Pass ☐ Fail ☐ N/A | |
| 7.5 LED heartbeat in BLE mode | ☐ Pass ☐ Fail | |
| 7.6 LED during update | ☐ Pass ☐ Fail ☐ N/A | |
| 8 All commands (8.1–8.11) | ☐ Pass ☐ Fail | Failed commands: ______ |
| Device ID (8.1) | | ______________ |

---

## If something fails

**Send the log.** The terminal output is what identifies the problem — a
description of the symptom usually is not enough. Copy everything from
power-up to the failure.

**Note the board serial number** so the fault can be traced to a specific
unit.

**The six-stage check names the fix.** If a stage fails it prints what to
check underneath. Try that first.

---

## Things that look wrong but are not

**`GSM: Module not responding, trying power-on...`** at startup — normal. The
modem is off when the board powers up, so the device turns it on. Takes about
20 seconds.

**`Certificate validated`** repeatedly during a firmware update — normal. Each
piece of the download opens a fresh secure connection.

**`Battery: 0mV (0%)`** on USB power — normal if no battery is fitted.

**`OTA partition already valid (state=-1)`** — normal. It means the firmware
has confirmed itself as working.

**The LED double-blinks for a minute or two after power-on** — normal. It is
connecting. A fault code only appears once a problem has lasted 40 seconds.

**A pause of up to 2 minutes before the network comes up** — normal on a SIM
the device has not seen before. It tries several network settings in turn.
Afterwards it remembers the one that worked and connects in about 6 seconds.
