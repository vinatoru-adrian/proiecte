/*
  GATEWAY_SerialPlotter_FixedAxis.ino
  - Scanează TAG_NAME și trimite pe Serial două serii: rssi_raw, rssi_avg
  - Forțează scala Y din Serial Plotter cu "ymin" și "ymax"
  - Rărește ieșirea (decimare) pentru o fereastră de timp mai lungă
  - Opțional: export CSV (dezactivat implicit)

  Necesită: "ESP32 by Espressif Systems" + librăria "ESP32 BLE Arduino"
*/

#include <WiFi.h>
#include <BLEDevice.h>

// ---------- SETĂRI UTILIZATOR ----------
#define TAG_NAME          "TAG_01"   // trebuie să coincidă cu numele tag-ului
#define SERIAL_BAUD       115200
// Axă Y forțată în Plotter:
#define YMIN_DBM          -100
#define YMAX_DBM          -30
// Rată ieșire către Serial Plotter (mai mare = grafic pe perioadă mai lungă):
const uint32_t OUTPUT_EVERY_MS = 200;   // 200ms ≈ 5 Hz
// Filtru EMA (0..1) – mai mare = reacționează mai repede, mai mic = mai neted
const float    EMA_ALPHA       = 0.20;
// Activează CSV (linie separată "csv,<ms>,<rssi_raw>,<rssi_avg>") doar când folosești Serial Monitor:
#define ENABLE_CSV        0
// Timpi scan BLE:
#define SCAN_INTERVAL_MS  100
#define SCAN_WINDOW_MS    100
// --------------------------------------

// Variabile pentru filtrare / ieșire
volatile int  g_lastRaw = 0;
volatile bool g_haveNew = false;
volatile uint32_t g_lastSeenMs = 0;

float  ema = 0.0f;
bool   haveEma = false;
uint32_t lastOut = 0;

// Mediană anti-spike pe 3 puncte (în interiorul callback-ului)
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

  // opțional: oprește Wi-Fi pentru un pic mai puțin zgomot RF
  WiFi.mode(WIFI_OFF);

  BLEDevice::init(""); // gateway fără nume
  BLEScan *scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new AdvCallbacks(), true); // true = raportează duplicate (mai des)
  scan->setActiveScan(true);
  scan->setInterval(SCAN_INTERVAL_MS);
  scan->setWindow(SCAN_WINDOW_MS);
  scan->start(0, nullptr, false); // scan continuu

  Serial.println("# Ready. Open Tools > Serial Plotter @ 115200.");
  Serial.println("# Series: rssi_raw, rssi_avg (ymin/ymax fixează scala).");
}

void loop() {
  const uint32_t now = millis();

  // Decimare: ieșim doar la fiecare OUTPUT_EVERY_MS
  if (now - lastOut < OUTPUT_EVERY_MS) {
    delay(5);
    return;
  }
  lastOut = now;

  // Dacă am primit ceva recent, actualizăm EMA
  if (g_haveNew) {
    int raw = g_lastRaw;
    if (!haveEma) {
      ema = raw;         // initializează EMA cu prima valoare
      haveEma = true;
    } else {
      ema = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * ema;
    }

    // Linia pentru Serial Plotter (toate seriile pe aceeași linie):
    Serial.print("rssi_raw:"); Serial.print(raw);
    Serial.print(" rssi_avg:"); Serial.print(ema, 2);
    Serial.print(" ymin:");     Serial.print(YMIN_DBM);
    Serial.print(" ymax:");     Serial.print(YMAX_DBM);
    Serial.println();

#if ENABLE_CSV
    // Linie CSV (folosește Serial Monitor pentru copy/paste în Excel)
    Serial.print("csv,"); Serial.print(now);
    Serial.print(",");    Serial.print(raw);
    Serial.print(",");    Serial.println(ema, 2);
#endif

    g_haveNew = false; // așteptăm valori noi
  } else {
    // Dacă n-am mai văzut tag-ul de ceva timp, marchează absența (opțional)
    if (haveEma && (now - g_lastSeenMs > 3000)) {
      // menține axa și ieșirea, dar semnal constant la YMIN ca indicator "pierdere"
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
