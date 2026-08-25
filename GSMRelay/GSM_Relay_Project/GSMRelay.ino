#include <Arduino.h>
#include "FS.h"
#include "LittleFS.h"
#include "esp_task_wdt.h"
#include <WiFi.h>
#include <WebServer.h>
#include "secrets.h"  // WIFI_SSID, WIFI_PASS, API_KEY - see secrets.example.h

// ==========================
// Pin configuration for TTGO T-Call
// (values are the usual ones for T-Call SIM800L)
// ==========================
#define MODEM_PWRKEY     4
#define MODEM_RST        5
#define MODEM_POWER_ON   23
#define MODEM_TX         27
#define MODEM_RX         26

// Relay (to the barrier)
#define RELAY_PIN        12
#define RELAY_ACTIVE_MS  1000   // how long we keep the relay active (ms)

// Flash storage
#define SPIFFS LittleFS

// ==========================
// Watchdog and modem health
// ==========================
#define WDT_TIMEOUT_S            30      // reboot the board if loop() blocks for this long
#define MODEM_CHECK_INTERVAL_MS  60000UL // how often we check that the modem still responds
#define MODEM_FAIL_THRESHOLD     3        // after how many failures in a row we reset the modem

int modemFailCount = 0;
unsigned long lastModemCheck = 0;

// ==========================
// WiFi + HTTP API
// ==========================
// WIFI_SSID, WIFI_PASS and API_KEY come from secrets.h (gitignored)

WebServer server(83);

// ==========================
// Whitelist (RAM, heap-allocated to keep it off .bss)
// ==========================
const int MAX_NUMBERS = 5000;
// max length for a normalized number (07xxxxxxxx + '\0' => 11), 16 for headroom
const int NUM_LEN = 16;

char (*whitelist)[NUM_LEN] = nullptr;
int whitelistCount = 0;

// Last time read from the modem (raw string)
String lastTimeString = "";

// ==========================
// Forward declarations
// ==========================
bool initModem();
String sendAT(const String &cmd, uint32_t timeout = 3000);

bool loadWhitelist();
bool saveWhitelist();

bool normalizeNumberToBuf(const String &in, char *out, size_t outSize);
int  findNumberIndex(const char *numNorm);
bool isAuthorizedNumber(const char *numNorm);

void handleIncomingClipLine(const String &clipLine);
void triggerRelay();

void logEvent(const char *number, bool authorized);
void dumpLogToSerial();

void updateTimeFromModem();
String getTimestamp();

void initWatchdog();
String readAndLogModemLine();
bool isURCLine(const String &line);
void dispatchURC(const String &line);
bool modemPing();
void resetModem();
void checkModemHealth();

// API
bool apiAuthOk();
void apiSendJson(int code, const String &json);

void handleApiHealth();
void handleApiWhitelistGet();
void handleApiWhitelistAdd();
void handleApiWhitelistDelete();
void handleApiWhitelistEdit();
void handleApiWhitelistReplace();

// ==========================
// Setup
// ==========================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== GSM relay with ESP32 + SIM800L (TTGO T-Call) + WiFi API ===");

  // reboots the board if something hangs for too long
  initWatchdog();

  // whitelist lives on the heap so MAX_NUMBERS doesn't blow up .bss
  whitelist = (char (*)[NUM_LEN])malloc((size_t)MAX_NUMBERS * (size_t)NUM_LEN);
  if (!whitelist) {
    Serial.println("Error: cannot allocate RAM for whitelist!");
    while (true) delay(1000); // nothing useful to do without it
  }
  for (int i = 0; i < MAX_NUMBERS; i++) whitelist[i][0] = '\0';

  // Mount SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("Error mounting SPIFFS!");
  } else {
    Serial.println("SPIFFS mounted.");
  }

  // Relay pin
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // SIM800L modem pins
  pinMode(MODEM_POWER_ON, OUTPUT);
  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);

  digitalWrite(MODEM_POWER_ON, HIGH);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_PWRKEY, HIGH);

  // Serial1 = SIM800L modem
  Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  // Initialize modem
  if (!initModem()) {
    Serial.println("Modem could not be initialized correctly!");
  }

  // load whitelist
  if (!loadWhitelist()) {
    Serial.println("Warning: could not load whitelist.txt or it's empty!");
  } else {
    Serial.printf("Whitelist loaded: %d numbers.\n", whitelistCount);
  }

  // try to get the time from the network
  updateTimeFromModem();

  // WiFi + HTTP API
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting WiFi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi NOT connected (API unavailable until it connects).");
  }

  const char* hdrs[] = {"X-Api-Key"};
  server.collectHeaders(hdrs, 1);

  server.on("/api/health", HTTP_GET, handleApiHealth);

  server.on("/api/whitelist",         HTTP_GET,  handleApiWhitelistGet);      // text/plain
  server.on("/api/whitelist/add",     HTTP_POST, handleApiWhitelistAdd);      // form: number
  server.on("/api/whitelist/delete",  HTTP_POST, handleApiWhitelistDelete);   // form: number
  server.on("/api/whitelist/edit",    HTTP_POST, handleApiWhitelistEdit);     // form: old,new
  server.on("/api/whitelist/replace", HTTP_POST, handleApiWhitelistReplace);  // body raw text

  server.begin();
  Serial.println("HTTP API started on port 83.");

  Serial.println("Setup done, waiting for calls...");
}

// ==========================
// Main loop
// ==========================
void loop() {
  esp_task_wdt_reset();

  server.handleClient();

  // Commands from the PC over Serial
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'L') {        // Print log to serial
      dumpLogToSerial();
    }
  }

  // read lines from the modem (same path as sendAT, so we don't miss calls)
  if (Serial1.available()) {
    String line = readAndLogModemLine();
    if (line.length() > 0 && isURCLine(line)) {
      dispatchURC(line);
    }
  }

  checkModemHealth();

  // You can do other things here too (e.g. periodic time update)
  static unsigned long lastTimeUpdate = 0;
  if (millis() - lastTimeUpdate > 600000) { // every 10 minutes
    updateTimeFromModem();
    lastTimeUpdate = millis();
  }

  // Small pause
  delay(10);
}

// ==========================
// SIM800L modem initialization
// ==========================
bool initModem() {
  Serial.println("Starting modem...");

  // PWRKEY sequence (depends on the board, but this is the usual one)
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(100);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(1000);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(3000);

  // Check AT (up to x10)
  for (int i = 0; i < 10; i++) {
    String res = sendAT("AT");
    if (res.indexOf("OK") >= 0) {
      Serial.println("Modem AT OK.");
      break;
    }
    delay(1000);
  }

  // Turn off echo
  sendAT("ATE0");

  // Enable Caller ID
  sendAT("AT+CLIP=1");

  // Enable getting time from the network, if supported
  sendAT("AT+CLTS=1");
  sendAT("AT&W"); // save profile

  // Wait for network registration (simplified)
  Serial.println("Waiting for network registration...");
  for (int i = 0; i < 30; i++) {
    String res = sendAT("AT+CREG?");
    Serial.println(res);
    if (res.indexOf("+CREG: 0,1") >= 0 || res.indexOf("+CREG: 0,5") >= 0) {
      Serial.println("Registered on network.");
      return true;
    }
    delay(1000);
  }

  Serial.println("Did not register on the network in time.");
  return false;
}

// ==========================
// Function for sending AT commands
// ==========================
String sendAT(const String &cmd, uint32_t timeout) {
  // Clear buffer before sending
  while (Serial1.available()) {
    Serial1.read();
  }

  Serial.print(">> ");
  Serial.println(cmd);
  Serial1.println(cmd);

  uint32_t start = millis();
  String response;

  while (millis() - start < timeout) {
    esp_task_wdt_reset(); // AT+CREG? in initModem() can take a while

    while (Serial1.available()) {
      String line = readAndLogModemLine();
      if (line.length() == 0) continue;

      // handle calls right away, don't let them get stuck waiting on another AT command
      if (isURCLine(line)) {
        dispatchURC(line);
        continue;
      }

      response += line;
      response += '\n';
      if (line == "OK" || line == "ERROR") {
        return response;
      }
    }
  }

  return response;
}

// ==========================
// Read + log a line from the modem (used by both sendAT and loop)
// ==========================
String readAndLogModemLine() {
  String line = Serial1.readStringUntil('\n');
  line.trim();
  if (line.length() > 0) {
    Serial.print("[MODEM] ");
    Serial.println(line);
  }
  return line;
}

// ==========================
// Unsolicited lines from the modem (URC), not a reply to an AT command
// ==========================
bool isURCLine(const String &line) {
  return line.startsWith("+CLIP:");
}

void dispatchURC(const String &line) {
  if (line.startsWith("+CLIP:")) {
    handleIncomingClipLine(line);
  }
}

// ==========================
// Normalize a number into a fixed buffer (e.g. +4074.. -> 074..)
// ==========================
bool normalizeNumberToBuf(const String &in, char *out, size_t outSize) {
  if (!out || outSize < 2) return false;

  // strip spaces / tabs / - / ( )
  String cleaned;
  cleaned.reserve(in.length());
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == ' ' || c == '\t' || c == '-' || c == '(' || c == ')') continue;
    cleaned += c;
  }
  cleaned.trim();
  if (cleaned.length() == 0) return false;

  // Romania: +407xx -> 07xx
  if (cleaned.startsWith("+40") && cleaned.length() > 3 && cleaned[3] == '7') {
    cleaned = "0" + cleaned.substring(3);
  }

  if (cleaned.length() >= outSize) return false;

  cleaned.toCharArray(out, outSize);
  return (out[0] != '\0');
}

int findNumberIndex(const char *numNorm) {
  if (!numNorm || !whitelist) return -1;
  for (int i = 0; i < whitelistCount; i++) {
    if (strncmp(whitelist[i], numNorm, NUM_LEN) == 0) return i;
  }
  return -1;
}

bool isAuthorizedNumber(const char *numNorm) {
  return (findNumberIndex(numNorm) >= 0);
}

// ==========================
// Load/save whitelist
// ==========================
bool loadWhitelist() {
  if (!whitelist) return false;

  whitelistCount = 0;

  if (!SPIFFS.exists("/whitelist.txt")) {
    Serial.println("/whitelist.txt does not exist in SPIFFS.");
    return false;
  }

  File f = SPIFFS.open("/whitelist.txt", "r");
  if (!f) {
    Serial.println("Cannot open /whitelist.txt");
    return false;
  }

  while (f.available() && whitelistCount < MAX_NUMBERS) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (line[0] == '#') continue;

    // Cut off the comment after ';'
    int semi = line.indexOf(';');
    if (semi >= 0) {
      line = line.substring(0, semi);
      line.trim();
    }
    if (line.length() == 0) continue;

    char norm[NUM_LEN];
    if (!normalizeNumberToBuf(line, norm, sizeof(norm))) continue;

    if (findNumberIndex(norm) >= 0) continue; // skip duplicates

    strncpy(whitelist[whitelistCount], norm, NUM_LEN);
    whitelist[whitelistCount][NUM_LEN - 1] = '\0';
    whitelistCount++;
  }

  f.close();
  return (whitelistCount > 0);
}

bool saveWhitelist() {
  if (!whitelist) return false;

  File f = SPIFFS.open("/whitelist.txt", "w");
  if (!f) {
    Serial.println("Cannot open /whitelist.txt for writing.");
    return false;
  }
  for (int i = 0; i < whitelistCount; i++) {
    f.println(whitelist[i]);
  }
  f.close();
  return true;
}

// ==========================
// Process a +CLIP line
// Example line:
// +CLIP: "0744123456",145,"",,"",0
// ==========================
void handleIncomingClipLine(const String &clipLine) {
  // Find the first and second quotes
  int firstQuote = clipLine.indexOf('"');
  if (firstQuote < 0) return;
  int secondQuote = clipLine.indexOf('"', firstQuote + 1);
  if (secondQuote < 0) return;

  String numberRaw = clipLine.substring(firstQuote + 1, secondQuote);

  char norm[NUM_LEN];
  bool gotNumber = normalizeNumberToBuf(numberRaw, norm, sizeof(norm));

  Serial.print("Call from: ");
  Serial.print(numberRaw);
  Serial.print(" (normalized: ");
  Serial.print(gotNumber ? norm : "");
  Serial.println(")");

  if (!gotNumber) {
    // Hidden/unknown number (CLIR) - reject directly, without comparing against the whitelist
    Serial.println("Hidden/unknown number. Rejecting automatically.");
    sendAT("ATH");
    logEvent("HIDDEN", false);
    return;
  }

  bool auth = isAuthorizedNumber(norm);

  if (auth) {
    Serial.println("Authorized number. Activating relay and hanging up.");
    triggerRelay();
    // hang up the call
    sendAT("ATH");
  } else {
    Serial.println("UNAUTHORIZED number.");
    // hang up the call
    sendAT("ATH");
  }

  logEvent(norm, auth);
}

// ==========================
// Activate the relay
// ==========================
void triggerRelay() {
  digitalWrite(RELAY_PIN, HIGH);
  delay(RELAY_ACTIVE_MS);
  digitalWrite(RELAY_PIN, LOW);
}

// ==========================
// Log event to log.csv
// ==========================
void logEvent(const char *number, bool authorized) {
  File f = SPIFFS.open("/log.csv", FILE_APPEND);
  if (!f) {
    Serial.println("Cannot open /log.csv for append.");
    return;
  }

  String ts = getTimestamp();
  if (ts.length() == 0) ts = String(millis()); // fallback: milliseconds

  f.print(ts);
  f.print(",");
  f.print(number ? number : "");
  f.print(",");
  f.print(authorized ? "AUTHORIZED" : "DENIED");
  f.print("\n");
  f.close();
}

// ==========================
// Print log.csv to Serial
// ==========================
void dumpLogToSerial() {
  if (!SPIFFS.exists("/log.csv")) {
    Serial.println("/log.csv does not exist in SPIFFS.");
    return;
  }

  File f = SPIFFS.open("/log.csv", "r");
  if (!f) {
    Serial.println("Cannot open /log.csv");
    return;
  }

  Serial.println("=== START log.csv ===");
  while (f.available()) Serial.write((char)f.read());
  f.close();
  Serial.println("\n=== END log.csv ===");
}

// ==========================
// Update time from modem (AT+CCLK?)
// ==========================
void updateTimeFromModem() {
  String res = sendAT("AT+CCLK?");
  // Typical response:
  // +CCLK: "24/11/25,12:34:56+08"
  // OK
  int idx = res.indexOf("+CCLK:");
  if (idx < 0) {
    Serial.println("+CCLK not found in response.");
    return;
  }
  int firstQuote = res.indexOf('"', idx);
  int secondQuote = res.indexOf('"', firstQuote + 1);
  if (firstQuote < 0 || secondQuote < 0) {
    Serial.println("Unexpected CCLK format.");
    return;
  }

  lastTimeString = res.substring(firstQuote + 1, secondQuote);
  Serial.print("Time from modem: ");
  Serial.println(lastTimeString);
}

// ==========================
// Turn lastTimeString into something more "human"
// E.g.: "24/11/25,12:34:56+08" -> "24/11/25 12:34:56"
// ==========================
String getTimestamp() {
  if (lastTimeString.length() == 0) return "";

  int comma = lastTimeString.indexOf(',');
  if (comma < 0) return lastTimeString;

  String datePart = lastTimeString.substring(0, comma); // "24/11/25"
  String timePart = lastTimeString.substring(comma + 1); // "12:34:56+08"
  int plusIdx = timePart.indexOf('+');
  if (plusIdx > 0) {
    timePart = timePart.substring(0, plusIdx); // "12:34:56"
  }

  return datePart + " " + timePart; // "24/11/25 12:34:56"
}

// ==========================
// Watchdog
// ==========================
void initWatchdog() {
  // API changed between core versions (2.x vs 3.x)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdtConfig);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
}

// ==========================
// Modem health: ping + hardware reset if needed
// ==========================
bool modemPing() {
  String res = sendAT("AT", 2000);
  return res.indexOf("OK") >= 0;
}

void resetModem() {
  Serial.println("Modem not responding, resetting hardware...");

  digitalWrite(MODEM_PWRKEY, LOW);
  delay(100);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(1000);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(3000);

  initModem();
  modemFailCount = 0;
}

void checkModemHealth() {
  if (millis() - lastModemCheck < MODEM_CHECK_INTERVAL_MS) return;
  lastModemCheck = millis();

  if (modemPing()) {
    modemFailCount = 0;
  } else {
    modemFailCount++;
    Serial.printf("Modem not responding (%d/%d)\n", modemFailCount, MODEM_FAIL_THRESHOLD);
    if (modemFailCount >= MODEM_FAIL_THRESHOLD) {
      resetModem();
    }
  }
}

// ==========================
// API helpers
// ==========================
void apiSendJson(int code, const String &json) {
  server.send(code, "application/json", json);
}

bool apiAuthOk() {
  String keyQ = server.hasArg("key") ? server.arg("key") : "";
  String keyH = server.header("X-Api-Key");
  if (keyQ == API_KEY || keyH == API_KEY) return true;

  apiSendJson(401, "{\"ok\":false,\"error\":\"unauthorized\"}");
  return false;
}

// ==========================
// API endpoints
// ==========================
void handleApiHealth() {
  String ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("disconnected");
  String json = "{\"ok\":true,\"wifi\":\"" + String((WiFi.status()==WL_CONNECTED)?"connected":"disconnected") +
                "\",\"ip\":\"" + ip + "\",\"count\":" + String(whitelistCount) + "}";
  apiSendJson(200, json);
}

// GET /api/whitelist -> text/plain (raw file)
void handleApiWhitelistGet() {
  if (!apiAuthOk()) return;

  if (!SPIFFS.exists("/whitelist.txt")) {
    server.send(200, "text/plain", "");
    return;
  }
  File f = SPIFFS.open("/whitelist.txt", "r");
  if (!f) {
    apiSendJson(500, "{\"ok\":false,\"error\":\"cannot_open_file\"}");
    return;
  }
  server.streamFile(f, "text/plain");
  f.close();
}

// POST /api/whitelist/add (number=...)
void handleApiWhitelistAdd() {
  if (!apiAuthOk()) return;

  String number = server.hasArg("number") ? server.arg("number") : "";
  char norm[NUM_LEN];
  if (!normalizeNumberToBuf(number, norm, sizeof(norm))) {
    apiSendJson(400, "{\"ok\":false,\"error\":\"invalid_number\"}");
    return;
  }

  if (findNumberIndex(norm) >= 0) {
    apiSendJson(409, "{\"ok\":false,\"error\":\"already_exists\"}");
    return;
  }
  if (whitelistCount >= MAX_NUMBERS) {
    apiSendJson(507, "{\"ok\":false,\"error\":\"whitelist_full\"}");
    return;
  }

  strncpy(whitelist[whitelistCount], norm, NUM_LEN);
  whitelist[whitelistCount][NUM_LEN - 1] = '\0';
  whitelistCount++;

  if (!saveWhitelist()) {
    apiSendJson(500, "{\"ok\":false,\"error\":\"save_failed\"}");
    return;
  }

  apiSendJson(200, "{\"ok\":true,\"action\":\"add\",\"count\":" + String(whitelistCount) + "}");
}

// POST /api/whitelist/delete (number=...)
void handleApiWhitelistDelete() {
  if (!apiAuthOk()) return;

  String number = server.hasArg("number") ? server.arg("number") : "";
  char norm[NUM_LEN];
  if (!normalizeNumberToBuf(number, norm, sizeof(norm))) {
    apiSendJson(400, "{\"ok\":false,\"error\":\"invalid_number\"}");
    return;
  }

  int idx = findNumberIndex(norm);
  if (idx < 0) {
    apiSendJson(404, "{\"ok\":false,\"error\":\"not_found\"}");
    return;
  }

  for (int i = idx; i < whitelistCount - 1; i++) {
    strncpy(whitelist[i], whitelist[i + 1], NUM_LEN);
    whitelist[i][NUM_LEN - 1] = '\0';
  }
  whitelistCount--;
  whitelist[whitelistCount][0] = '\0';

  if (!saveWhitelist()) {
    apiSendJson(500, "{\"ok\":false,\"error\":\"save_failed\"}");
    return;
  }

  apiSendJson(200, "{\"ok\":true,\"action\":\"delete\",\"count\":" + String(whitelistCount) + "}");
}

// POST /api/whitelist/edit (old=..., new=...)
void handleApiWhitelistEdit() {
  if (!apiAuthOk()) return;

  String oldNum = server.hasArg("old") ? server.arg("old") : "";
  String newNum = server.hasArg("new") ? server.arg("new") : "";

  char oldNorm[NUM_LEN], newNorm[NUM_LEN];
  if (!normalizeNumberToBuf(oldNum, oldNorm, sizeof(oldNorm)) ||
      !normalizeNumberToBuf(newNum, newNorm, sizeof(newNorm))) {
    apiSendJson(400, "{\"ok\":false,\"error\":\"invalid_number\"}");
    return;
  }

  int idxOld = findNumberIndex(oldNorm);
  if (idxOld < 0) {
    apiSendJson(404, "{\"ok\":false,\"error\":\"old_not_found\"}");
    return;
  }

  int idxNew = findNumberIndex(newNorm);
  if (idxNew >= 0 && idxNew != idxOld) {
    apiSendJson(409, "{\"ok\":false,\"error\":\"new_already_exists\"}");
    return;
  }

  strncpy(whitelist[idxOld], newNorm, NUM_LEN);
  whitelist[idxOld][NUM_LEN - 1] = '\0';

  if (!saveWhitelist()) {
    apiSendJson(500, "{\"ok\":false,\"error\":\"save_failed\"}");
    return;
  }

  apiSendJson(200, "{\"ok\":true,\"action\":\"edit\",\"count\":" + String(whitelistCount) + "}");
}

// POST /api/whitelist/replace
// Raw body: one number per line, '#' and ';' comments accepted (same format as the file)
void handleApiWhitelistReplace() {
  if (!apiAuthOk()) return;

  String body = server.arg("plain");
  if (body.length() == 0) {
    apiSendJson(400, "{\"ok\":false,\"error\":\"empty_body\"}");
    return;
  }

  int newCount = 0;
  int start = 0;

  while (start < (int)body.length() && newCount < MAX_NUMBERS) {
    int end = body.indexOf('\n', start);
    if (end < 0) end = body.length();

    String line = body.substring(start, end);
    start = end + 1;

    line.trim();
    if (line.length() == 0) continue;
    if (line[0] == '#') continue;

    int semi = line.indexOf(';');
    if (semi >= 0) {
      line = line.substring(0, semi);
      line.trim();
    }
    if (line.length() == 0) continue;

    char norm[NUM_LEN];
    if (!normalizeNumberToBuf(line, norm, sizeof(norm))) continue;

    // dedup against what we've already added in this batch
    bool dup = false;
    for (int i = 0; i < newCount; i++) {
      if (strncmp(whitelist[i], norm, NUM_LEN) == 0) { dup = true; break; }
    }
    if (dup) continue;

    strncpy(whitelist[newCount], norm, NUM_LEN);
    whitelist[newCount][NUM_LEN - 1] = '\0';
    newCount++;
  }

  whitelistCount = newCount;

  if (!saveWhitelist()) {
    apiSendJson(500, "{\"ok\":false,\"error\":\"save_failed\"}");
    return;
  }

  apiSendJson(200, "{\"ok\":true,\"action\":\"replace\",\"count\":" + String(whitelistCount) + "}");
}
