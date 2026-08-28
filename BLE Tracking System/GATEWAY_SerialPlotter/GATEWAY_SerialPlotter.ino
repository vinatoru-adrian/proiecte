/*
  GATEWAY_SerialPlotter_FixedAxis.ino
  - Scans for TAG_NAME and sends two series over Serial: rssi_raw, rssi_avg
  - Forces the Serial Plotter Y axis with "ymin" and "ymax"
  - Throttles the output (decimation) for a longer time window
  - Optional: CSV export (disabled by default)

  Requires: "ESP32 by Espressif Systems" + the "ESP32 BLE Arduino" library
*/

#include <WiFi.h>
#include <BLEDevice.h>

// ---------- USER SETTINGS ----------
#define TAG_NAME          "TAG_01"   // must match the tag's name
#define SERIAL_BAUD       115200
// Fixed Y axis in the Plotter:
#define YMIN_DBM          -100
#define YMAX_DBM          -30
// Output rate to Serial Plotter (higher = graph over a longer period):
const uint32_t OUTPUT_EVERY_MS = 200;   // 200ms ~= 5 Hz
// EMA filter (0..1) - higher = reacts faster, lower = smoother
const float    EMA_ALPHA       = 0.20;
// Enable CSV (separate line "csv,<ms>,<rssi_raw>,<rssi_avg>"), only useful with the Serial Monitor:
#define ENABLE_CSV        0
// BLE scan timing:
#define SCAN_INTERVAL_MS  100
#define SCAN_WINDOW_MS    100
// --------------------------------------

// Variables for filtering / output
volatile int  g_lastRaw = 0;
volatile bool g_haveNew = false;
volatile uint32_t g_lastSeenMs = 0;

float  ema = 0.0f;
bool   haveEma = false;
uint32_t lastOut = 0;

// Anti-spike median over 3 points (inside the callback)
struct AdvCallbacks : public BLEAdvertisedDeviceCallbacks {
  int last1 = 0, last2 = 0;

  static int median3(int a, int b, int c) {
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
  }

  void onResult(BLEAdvertisedDevice d) override {
    if (d.haveName() && d.getName() == TAG_NAME) {
      int rssi = d.getRSSI();
      int med  = median3(last2, last1, rssi);
      last2 = last1;
      last1 = rssi;

      g_lastRaw   = med;
      g_haveNew   = true;
      g_lastSeenMs = millis();
    }
  }
};

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  // optional: turn off Wi-Fi for slightly less RF noise
  WiFi.mode(WIFI_OFF);

  BLEDevice::init(""); // gateway with no name
  BLEScan *scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new AdvCallbacks(), true); // true = report duplicates (more often)
  scan->setActiveScan(true);
  scan->setInterval(SCAN_INTERVAL_MS);
  scan->setWindow(SCAN_WINDOW_MS);
  scan->start(0, nullptr, false); // continuous scan

  Serial.println("# Ready. Open Tools > Serial Plotter @ 115200.");
  Serial.println("# Series: rssi_raw, rssi_avg (ymin/ymax fixează scala).");
}

void loop() {
  const uint32_t now = millis();

  // Decimation: only output every OUTPUT_EVERY_MS
  if (now - lastOut < OUTPUT_EVERY_MS) {
    delay(5);
    return;
  }
  lastOut = now;

  // If we've received something recently, update the EMA
  if (g_haveNew) {
    int raw = g_lastRaw;
    if (!haveEma) {
      ema = raw;         // initialize the EMA with the first value
      haveEma = true;
    } else {
      ema = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * ema;
    }

    // Line for the Serial Plotter (all series on the same line):
    Serial.print("rssi_raw:"); Serial.print(raw);
    Serial.print(" rssi_avg:"); Serial.print(ema, 2);
    Serial.print(" ymin:");     Serial.print(YMIN_DBM);
    Serial.print(" ymax:");     Serial.print(YMAX_DBM);
    Serial.println();

#if ENABLE_CSV
    // CSV line (use the Serial Monitor to copy/paste into Excel)
    Serial.print("csv,"); Serial.print(now);
    Serial.print(",");    Serial.print(raw);
    Serial.print(",");    Serial.println(ema, 2);
#endif

    g_haveNew = false; // wait for new values
  } else {
    // If we haven't seen the tag for a while, flag it as missing (optional)
    if (haveEma && (now - g_lastSeenMs > 3000)) {
      // keep the axis and output going, but hold a constant signal at YMIN as a "lost" indicator
      Serial.print("rssi_raw:"); Serial.print(YMIN_DBM);
      Serial.print(" rssi_avg:"); Serial.print(YMIN_DBM);
      Serial.print(" ymin:");     Serial.print(YMIN_DBM);
      Serial.print(" ymax:");     Serial.print(YMAX_DBM);
      Serial.println();
#if ENABLE_CSV
      Serial.print("csv,"); Serial.print(now);
      Serial.print(",");    Serial.print(YMIN_DBM);
      Serial.print(",");    Serial.println(YMIN_DBM);
#endif
    }
  }
}
