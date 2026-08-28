# BLE Tracking System

A simple BLE-based proximity detection system using two ESP32 modules:

- **TAG** – continuously broadcasts BLE advertising packets under a fixed name (`TAG_01`), with no connection or pairing required. Powered via USB or a power bank.
- **GATEWAY** – scans for that tag, extracts the signal's RSSI, filters it (3-point median + EMA filter), and plots the raw vs. filtered RSSI in real time via the Serial Plotter.

The goal of this project is to explore proximity estimation based on BLE signal strength (RSSI) and the noise-filtering techniques relevant to radio communication.

## Hardware Required

- 2x ESP32 boards (any BLE-capable variant)
- USB cable for programming/power

## Software Required

- Arduino IDE with the "ESP32 by Espressif Systems" board package installed
- "ESP32 BLE Arduino" library

## Project Structure

- `TAG_Advertiser.ino/` – firmware for the tag module (transmitter)
- `GATEWAY_SerialPlotter.ino/` – firmware for the gateway module (receiver + filtering + plotting)

## Usage

1. Flash `TAG_Advertiser.ino` onto the first ESP32 and power it independently (USB/power bank).
2. Flash `GATEWAY_SerialPlotter.ino` onto the second ESP32, connected to your computer.
3. Open Arduino IDE → Tools → Serial Plotter, baud rate `115200`.
4. You'll see two curves: `rssi_raw` (instantaneous value, median-filtered over 3 readings) and `rssi_avg` (smoothed via EMA filter), useful for observing how the signal varies with distance/obstacles.

## Configurable Parameters

- `TAG_NAME` – tag identifier (allows multiple distinct tags)
- `ADV_INT_MS` / `TX_POWER_DBM` – advertising interval and transmit power, for the power-consumption vs. range trade-off
- `EMA_ALPHA` – aggressiveness of the smoothing filter
- `SCAN_INTERVAL_MS` / `SCAN_WINDOW_MS` – BLE scan parameters on the gateway

## Possible Extensions

- Approximate distance estimation from RSSI (log-distance path loss model)
- Support for multiple simultaneous tags
- Sending data to a server/dashboard over Wi-Fi
