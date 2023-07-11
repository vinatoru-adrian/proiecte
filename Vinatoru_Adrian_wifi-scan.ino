#define BLYNK_TEMPLATE_ID "TMPLF-BVu5M6"
#define BLYNK_DEVICE_NAME "ESP"
#define BLYNK_AUTH_TOKEN "BXzmowwbVsRnmw2J7RVc7GHfboz5zwcq"
#define BLYNK_PRINT Serial


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include "DHT.h"
#include <TM1637.h>

#define PIN_LED 27
#define PIN_MOTOR 26
#define PIN_DHT 13
#define PIN_HOME_LIGHT 21
#define PIN_LDR 25
#define PIN_POTE 35
#define PIN_CLK 32
#define PIN_4SEG 33
#define PIN_ENC_CLK 34
#define PIN_DT 35
#define PIN_BLYNK_TEMP V1
#define PIN_BLYNK_HUMD V2
#define DHTTYPE DHT22

char auth[] = BLYNK_AUTH_TOKEN;

char ssid[] = "Wokwi-GUEST";
char pass[] = "";

Servo servo_door;
BlynkTimer timer;
DHT dht(PIN_DHT, DHTTYPE);
TM1637 tm(PIN_CLK, PIN_4SEG);

int door_toggle = 0;
int temp_low_limit = 1500;
int enc_pos = 0;
float set_temp = 0.0;

BLYNK_WRITE(V0) // Toggle inchidere/deschidere usa
{
  door_toggle ^= 1;
  digitalWrite(PIN_LED, door_toggle);
  servo_door.write(180*door_toggle);
}

BLYNK_CONNECTED()
{}

void setup()
{
  door_toggle = 0;
  Serial.begin(115200);
  servo_door.attach(PIN_MOTOR);
  servo_door.write(0); 
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);

  tm.init();
  tm.set(BRIGHT_TYPICAL);
  
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_HOME_LIGHT, OUTPUT);
  pinMode(PIN_LDR, INPUT);
  pinMode(PIN_ENC_CLK, INPUT);
  pinMode(PIN_DT, INPUT);
  adjust_temp();
  // Seteaza o functie ce este apelata la fiecare secunda

  //Update-ul senzorului dureaza 2s
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), adjust_temp, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_LDR), check_light, CHANGE);
  timer.setInterval(2000L, update_ui);
}

void loop()
{
  Blynk.run();
  timer.run();

}

void update_ui() {
  Blynk.virtualWrite(PIN_BLYNK_TEMP, dht.readTemperature());
  Blynk.virtualWrite(PIN_BLYNK_HUMD, dht.readHumidity());
}

void adjust_temp(){
  int dt = digitalRead(PIN_DT);
  if (dt == HIGH) {
    enc_pos += 25;
  }
  if (dt == LOW) {
    enc_pos -= 25;
  }
  int t = temp_low_limit + enc_pos; // =~ (15 -> 40)C * 100 range

  if(t > 4000 || t < 1500) enc_pos = 0;

  tm.display(0, (t / 1000) % 10);
  tm.display(1, (t/ 100) % 10);
  tm.display(2, (t / 10) % 10);
  tm.display(3, t % 10);
  //  void showNumberDecEx(int num, uint8_t dots = 0, bool leading_zero = false, uint8_t length = 4, uint8_t pos = 0);
}

void check_light(){
  digitalWrite(PIN_HOME_LIGHT, digitalRead(PIN_LDR));
}
