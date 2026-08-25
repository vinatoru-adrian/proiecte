# GSMRelay

A small ESP32 project that opens a gate/barrier by phone call. It runs on a TTGO T-Call board (ESP32 + SIM800L GSM module) hooked up to a relay. Call the SIM card's number from a whitelisted phone, it recognizes the caller ID, triggers the relay for a second, and hangs up without ever answering the call — so it doesn't cost the caller anything.

Built this for a barrier at a small office building where giving everyone a physical key/remote wasn't practical, but everyone already has a phone.

## How it works

1. SIM800L is registered on the mobile network and listens for incoming calls (`AT+CLIP=1` gives us caller ID on ringing, no need to answer).
2. When a call comes in, the ESP32 reads the `+CLIP` line, extracts and normalizes the number (handles `+40` / `07xx` formats for Romanian numbers).
3. If the number is in the whitelist, it pulses the relay pin for ~1s (enough to trigger the barrier controller) and hangs up (`ATH`).
4. If it's not whitelisted (or the number is hidden/CLIR), it just hangs up.
5. Every call gets logged to `log.csv` on the internal flash (timestamp, number, authorized/denied), pulled from the modem's network time.

No call is ever answered, so there's no airtime cost — it all happens during the ringing phase.

## Managing the whitelist remotely

The board also connects to WiFi and exposes a small HTTP API (port 83) to manage the whitelist without re-flashing or pulling the SD card:

| Endpoint | Method | Description |
|---|---|---|
| `/api/health` | GET | WiFi status, IP, number of whitelisted entries |
| `/api/whitelist` | GET | Raw whitelist as text |
| `/api/whitelist/add` | POST | Add a number (`number=...`) |
| `/api/whitelist/delete` | POST | Remove a number (`number=...`) |
| `/api/whitelist/edit` | POST | Replace one number with another (`old=...&new=...`) |
| `/api/whitelist/replace` | POST | Overwrite the whole list (raw body, one number per line) |

All endpoints (except `/api/health`) require an API key, passed either as `?key=...` or an `X-Api-Key` header. Changes are persisted straight to flash (LittleFS).

## Reliability bits

Since this thing needs to run unattended 24/7 controlling a physical gate, a few things are in there specifically for that:

- **Watchdog timer** — reboots the board if `loop()` ever hangs for more than 30s.
- **Modem health check** — pings the SIM800L every minute; after 3 failed pings in a row it power-cycles the modem hardware and re-initializes it.
- **Whitelist in heap memory** (not `.bss`) — with up to 5000 numbers, it's too large to keep as a static array without wasting RAM at boot.

## Whitelist format

`data/whitelist.txt`, one number per line. `#` for full-line comments, `;` for inline comments. Numbers get normalized on load (`+407...` and `07...` are treated as the same number).

```
# Office
0744123456   ; John
0741234567   ; Maria
```

## Hardware

- TTGO T-Call (ESP32 + SIM800L)
- 1-channel relay module wired to the barrier controller's "open" input
- SIM card with an active number and calling credit (or a plan with free minutes — no minutes are actually used since calls aren't answered)

Pinout is the standard T-Call layout (`MODEM_PWRKEY`, `MODEM_RST`, `MODEM_POWER_ON`, `MODEM_TX/RX`), relay on GPIO 12.

## Setup

1. Copy `secrets.example.h` to `secrets.h` and fill in your WiFi credentials and a proper API key (not `"2005"`).
2. Edit `data/whitelist.txt` with the numbers you want authorized.
3. Flash the sketch, then upload the `data/` folder as a LittleFS image (e.g. via the Arduino "ESP32 Sketch Data Upload" tool or `pio run -t uploadfs` for PlatformIO).
4. Watch the serial monitor at 115200 baud during first boot — it prints modem init, network registration, and the assigned WiFi IP.

Typing `L` in the serial monitor dumps `log.csv` for a quick look at recent activity.

## Known limitations

- Whitelist is managed over plain HTTP on the local network — fine for a trusted LAN, wouldn't expose this directly to the internet as-is.
- No rate limiting on the API key, so a weak key is brute-forceable on a LAN someone has access to.
- Number normalization currently only handles Romanian (`+40`) numbers.
