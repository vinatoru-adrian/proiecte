/*
  TAG_Advertiser.ino
  - Emite pachete BLE advertising cu numele "TAG_01"
  - Nu necesită conexiune la laptop; doar alimentare (USB sau powerbank)
  - Library: ESP32 BLE Arduino (include <BLEDevice.h>)
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define TAG_NAME        "TAG_01"     // Schimbă dacă vrei mai multe tag-uri
#define ADV_INT_MS      300          // Interval advertising (200..1000 ms tipic)
#define TX_POWER_DBM    3            // -12..+9 dBm; 3 e un compromis bun consum/acoperire

void setup() {
  // Inițializează BLE cu nume
  BLEDevice::init(TAG_NAME);

  // Setează puterea de emisie (opțional, afectează autonomia)
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, (esp_power_level_t)ESP_PWR_LVL_P3); // ~+3 dBm
  // Dacă vrei mai tare: ESP_PWR_LVL_P7 (~+7 dBm) – dar consumul crește

  // Pornim advertising
  BLEAdvertising *adv = BLEDevice::getAdvertising();

  BLEAdvertisementData advData;
  advData.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
  advData.setName(TAG_NAME);
  // UUID opțional să poți filtra pe gateway (custom)
  advData.setCompleteServices(BLEUUID("12345678-1234-1234-1234-1234567890ab"));
  adv->setAdvertisementData(advData);

  // Interval advertising
  adv->setMinInterval(ADV_INT_MS / 0.625); // unități de 0.625ms
  adv->setMaxInterval(ADV_INT_MS / 0.625);

  adv->start();
}

void loop() {
  // Nimic – doar emitem. Consum mic.
  delay(1000);
}
