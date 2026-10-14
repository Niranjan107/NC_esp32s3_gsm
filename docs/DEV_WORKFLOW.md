# Day-to-day commands

Quick reference for building, flashing, and saving work to both repos.

## Repos

| Name | Where |
|---|---|
| `origin` | GitHub: `https://github.com/Niranjan107/NC_esp32s3_gsm.git` |
| `azure` | Azure (office): `https://dev.azure.com/Nitara/Nitara-Embedded-Project/_git/NCLite-Esp32-s3-Connector-GSM-firmware` |

Work on `main`. Push to **both** repos.

## Get the latest code

```
git pull origin main
```

## Save and push your changes

| Step | Command |
|---|---|
| 1. See what changed | `git status` |
| 2. Add the files | `git add <file> <file> ...` (do **not** add `.vscode/settings.json`) |
| 3. Commit | `git commit -m "what you changed"` |
| 4. Push to GitHub | `git push origin main` |
| 5. Push to Azure | `git push azure main` |

If a push says **rejected**, someone else pushed first. Run `git pull origin main`
(or `git pull azure main`), then push again. Never use `--force`: it can delete
other people's work.

## Build and flash

| Step | Command |
|---|---|
| Find the board's port | `ls /dev/cu.usb*` |
| Build | `idf.py build` |
| Flash and watch the log | `idf.py -p /dev/cu.usbmodemXXXX flash monitor` |
| Leave the monitor | **Ctrl + ]** |
| Clean build (after a big pull) | `idf.py fullclean` then `idf.py build` |

## Device commands

The 32 commands the device accepts over BLE / USB / MQTT (`set_link_mode`,
`gsm_enable`, `diag`, ...) are in [commands.md](commands.md).

## Status LED

| LED | Meaning |
|---|---|
| Solid ON | GSM or WiFi connected, cloud (MQTT) working |
| Double blink | Connecting |
| 1–5 blinks, pause | Fault (shown after 40 s). GSM: 1 modem, 2 SIM, 3 signal, 4 network/carrier, 5 data/cloud. WiFi: 1 no WiFi saved, 2 can't join router, 3 no internet |
| Heartbeat | BLE mode |
| 5 quick blinks | Reading received |
| Fast steady blink | Firmware update, do not unplug |
