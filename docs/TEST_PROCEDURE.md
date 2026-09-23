# Test Procedure — NCLite GSM Connector

**For:** the hardware team, testing a board before it goes to a site.

You do not need to know anything about the firmware. Follow the steps in
order, tick each result, and note anything that does not match. Every step
says what you should see and what it means if you see something else.

**Time:** about 90 minutes for the full set. Section 1 alone is 15 minutes and
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

## Result sheet

Board serial number: ______________  Date: ____________

Tester: ______________

| Section | Result | Notes |
|---|---|---|
| 1.1 Starts | ☐ Pass ☐ Fail | Version: __________ |
| 1.2 All modules | ☐ Pass ☐ Fail | |
| 1.3 Memory | ☐ Pass ☐ Check | Free heap: __________ |
| 1.4 Six stages | ☐ Pass ☐ Fail | Failed stage: ______ |
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

**A pause of up to 2 minutes before the network comes up** — normal on a SIM
the device has not seen before. It tries several network settings in turn.
Afterwards it remembers the one that worked and connects in about 6 seconds.
