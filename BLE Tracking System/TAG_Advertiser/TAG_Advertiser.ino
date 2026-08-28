/*
  TAG_Advertiser.ino
  - Emits BLE advertising packets under the name "TAG_01"
  - No connection to a laptop needed; just power (USB or power bank)
  - Library: ESP32 BLE Arduino (includes <BLEDevice.h>)
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define TAG_NAME        "TAG_01"     // Change if you want multiple tags
#define ADV_INT_MS      300          // Advertising interval (typically 200..1000 ms)
#define TX_POWER_DBM    3            // -12..+9 dBm; 3 is a good power/range trade-off

void setup() {
  // Initialize BLE with a name
  BLEDevice::init(TAG_NAME);

  // Set the TX power (optional, affects battery life)
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, (esp_power_level_t)ESP_PWR_LVL_P3); // ~+3 dBm
  // For more range: ESP_PWR_LVL_P7 (~+7 dBm) - but power consumption goes up

  // Start advertising
  BLEAdvertising *adv = BLEDevice::getAdvertising();

  BLEAdvertisementData advData;
  advData.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
  advData.setName(TAG_NAME);
  // Optional UUID so the gateway can filter (custom)
  advData.setCompleteServices(BLEUUID("12345678-1234-1234-1234-1234567890ab"));
  adv->setAdvertisementData(advData);

  // Advertising interval
  adv->setMinInterval(ADV_INT_MS / 0.625); // units of 0.625ms
  adv->setMaxInterval(ADV_INT_MS / 0.625);

  adv->start();
}

void loop() {
  // Nothing to do here - just advertising. Low power draw.
  delay(1000);
}
