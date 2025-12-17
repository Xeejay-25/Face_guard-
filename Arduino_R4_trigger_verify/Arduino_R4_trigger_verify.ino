
// Arduino UNO R4 WiFi - distance trigger -> request JSON from ESP32-CAM over WiFi
// GOOD RANGE: 5cm to 30cm
// Shows result for 5 seconds, and if the result text is long, it scrolls (marquee) left-to-right.

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFiS3.h>
#include <math.h>

// ===================== LCD =====================
LiquidCrystal_I2C lcd(0x27, 16, 2); // change to 0x3F if needed

// ===================== Pins =====================
const int redPin   = 9;
const int greenPin = 10;
const int bluePin  = 11;

const int trigPin  = 7;
const int echoPin  = 6;

// ===================== RGB Type =====================
// COMMON ANODE = true (inverted)
const bool COMMON_ANODE = true;

// ===================== WiFi / ESP32 =====================
const char* WIFI_SSID = "esp32";
const char* WIFI_PASS = "12345678";

// Put the ESP32's IP here (print in ESP32 Serial Monitor)
const char* ESP32_HOST = "192.168.137.155";
const uint16_t ESP32_PORT = 80;

// ===================== Distance rules =====================
const float MIN_CM = 5.0f;            // ignore too-close noise
const float GOOD_CM = 30.0f;          // GOOD RANGE upper limit
const float FAR_CM  = 80.0f;          // presence detection limit
const float HYSTERESIS_CM = 2.5f;     // reduce flicker near boundary

// Must be in-range for N loops before triggering
const int IN_RANGE_HITS_REQUIRED = 10;

// Cooldown after a run (ms)
const unsigned long COOLDOWN_MS = 4500;

// Show result for 5 seconds (per your request)
const unsigned long RESULT_HOLD_MS = 10000;

// ===================== State machine =====================
enum State { IDLE, TOO_FAR, READY_STABLE, TESTING, SHOW_RESULT, COOLDOWN };
State state = IDLE;

int stableHits = 0;
unsigned long stateUntil = 0;

bool lastPass = false;
float lastShownDist = -999;

// ===================== LED helpers =====================
void setColor(int r, int g, int b) {
  if (COMMON_ANODE) { r = 255 - r; g = 255 - g; b = 255 - b; }
  analogWrite(redPin, r);
  analogWrite(greenPin, g);
  analogWrite(bluePin, b);
}

// idle rainbow
unsigned long lastLEDUpdate = 0;
int hue = 0;

void rainbowStep() {
  int r=0,g=0,b=0;
  int x = hue % 1536; // 6 * 256
  int seg = x / 256;
  int off = x % 256;

  switch (seg) {
    case 0: r=255; g=off;  b=0;   break;
    case 1: r=255-off; g=255; b=0; break;
    case 2: r=0;   g=255; b=off;  break;
    case 3: r=0;   g=255-off; b=255; break;
    case 4: r=off; g=0;   b=255; break;
    case 5: r=255; g=0;   b=255-off; break;
  }
  setColor(r,g,b);
  hue = (hue + 10) % 1536;
}

// ===================== LCD helpers =====================
String idleMessage = " Identity Guard  Stand Here     ";
int textPos = 0;

void lcdShow(const String& line1, const String& line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1.substring(0, 16));
  lcd.setCursor(0, 1);
  lcd.print(line2.substring(0, 16));
}

void lcdIdleScroll(unsigned long now) {
  static unsigned long lastScroll = 0;
  if (now - lastScroll < 170) return;
  lastScroll = now;

  int len = idleMessage.length();
  String displayMsg = idleMessage.substring(textPos) + idleMessage.substring(0, textPos);

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(displayMsg.substring(0,16));

  lcd.setCursor(0, 1);
  lcd.print("Stand <= 30cm   ");

  textPos = (textPos + 1) % len;
}

// ===================== Result scroll (16x2 marquee) =====================
String resultText = "";
bool resultScrollable = false;
int scrollPos = 0;
unsigned long lastScrollMs = 0;

// +1 = normal marquee (text moves left). Set -1 to reverse.
const int SCROLL_DIR = +1;
const unsigned long SCROLL_STEP_MS = 220;

void lcdWriteRow(uint8_t row, const String& s) {
  String t = s;
  if (t.length() < 16) t += String(' ', 16 - t.length());
  lcd.setCursor(0, row);
  lcd.print(t.substring(0, 16));
}

void startResultDisplay(const String& msg) {
  resultText = msg;
  scrollPos = 0;
  lastScrollMs = 0;

  // If longer than 32 chars, enable scrolling window across both lines
  resultScrollable = (resultText.length() > 32);

  if (!resultScrollable) {
    lcdWriteRow(0, resultText.substring(0, 16));
    lcdWriteRow(1, resultText.substring(16, 32));
  } else {
    // draw first frame immediately
    String pad = resultText + "                "; // 16 spaces gap
    String loop = pad + pad;
    String view = loop.substring(0, 32);
    lcdWriteRow(0, view.substring(0, 16));
    lcdWriteRow(1, view.substring(16, 32));
  }
}

void tickResultScroll(unsigned long now) {
  if (!resultScrollable) return;
  if (now - lastScrollMs < SCROLL_STEP_MS) return;
  lastScrollMs = now;

  String pad = resultText + "                ";
  int total = pad.length();
  if (total <= 0) return;

  scrollPos = (scrollPos + SCROLL_DIR);
  if (scrollPos < 0) scrollPos += total;
  if (scrollPos >= total) scrollPos -= total;

  String loop = pad + pad;
  String view = loop.substring(scrollPos, scrollPos + 32);

  lcdWriteRow(0, view.substring(0, 16));
  lcdWriteRow(1, view.substring(16, 32));
}

// ===================== Ultrasonic helpers =====================
float readDistanceCM_once() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return -1;
  return duration * 0.034f / 2.0f;
}

float smoothDistanceCM(int samples = 5) {
  float sum = 0;
  int count = 0;

  for (int i = 0; i < samples; i++) {
    float d = readDistanceCM_once();
    if (d > 0) { sum += d; count++; }
    delay(8);
  }
  if (count == 0) return -1;
  return sum / count;
}

// ===================== HTTP helpers =====================
String httpGET(const String& path, int timeoutMs = 7000) {
  WiFiClient client;
  if (!client.connect(ESP32_HOST, ESP32_PORT)) return "";

  client.print("GET " + path + " HTTP/1.1\r\n");
  client.print("Host: " + String(ESP32_HOST) + "\r\n");
  client.print("Connection: close\r\n\r\n");

  unsigned long start = millis();
  while (client.connected() && !client.available()) {
    if (millis() - start > (unsigned long)timeoutMs) { client.stop(); return ""; }
    delay(5);
  }

  String resp;
  while (client.available()) resp += (char)client.read();
  client.stop();

  int idx = resp.indexOf("\r\n\r\n");
  if (idx < 0) return "";
  return resp.substring(idx + 4);
}

bool espSetActive(bool enable) {
  String body = httpGET(String("/active?enable=") + (enable ? "1" : "0"), 4000);
  return body.length() > 0;
}

bool espVerify(String &outJson) {
  outJson = httpGET("/verify", 28000); // capture + upload + inference
  return outJson.length() > 0;
}

// ===================== JSON helpers (no ArduinoJson needed) =====================
String extractJsonStringValue(const String& json, const char* key) {
  String k = String("\"") + key + "\":";
  int p = json.indexOf(k);
  if (p < 0) return "";

  p += k.length();
  while (p < (int)json.length() && (json[p] == ' ' || json[p] == '\n' || json[p] == '\r' || json[p] == '\t')) p++;

  if (p >= (int)json.length() || json[p] != '"') return "";
  p++;

  String out;
  while (p < (int)json.length()) {
    char c = json[p++];
    if (c == '\\') { // minimal escape handling
      if (p < (int)json.length()) out += json[p++];
      continue;
    }
    if (c == '"') break;
    out += c;
  }
  return out;
}

bool inferPassFromJson(const String& json) {
  // If text_LCD contains "Welcome" => pass
  String text = extractJsonStringValue(json, "text_LCD");
  String low = text; low.toLowerCase();
  if (low.indexOf("welcome") >= 0) return true;
  if (low.indexOf("recognized") >= 0) return true;
  if (low.indexOf("not recognize") >= 0) return false;
  if (low.indexOf("not recognized") >= 0) return false;

  // boolean keys fallback
  const char* keys[] = {"\"pass\":", "\"match\":", "\"verified\":", "\"success\":", "\"ok\":"};
  for (auto k : keys) {
    int p = json.indexOf(k);
    if (p >= 0) {
      int t = json.indexOf("true", p);
      int f = json.indexOf("false", p);
      if (t >= 0 && (f < 0 || t < f)) return true;
      if (f >= 0) return false;
    }
  }

  // common status strings
  if (json.indexOf("\"status\":\"pass\"") >= 0) return true;
  if (json.indexOf("\"status\":\"ok\"") >= 0) return true;

  return false;
}

// ===================== Setup / Loop =====================
void setup() {
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  lcd.begin();
  lcd.backlight();
  lcdShow("BOOTING...", "WiFi connect");
  delay(500);

  // WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(350);
    Serial.print(".");
    if (millis() - start > 20000) break;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("R4 IP: "); Serial.println(WiFi.localIP());
    
    // Check ESP32 connection
    lcdShow("CHECKING ESP32", "Please wait...");
    delay(500);
    if (espSetActive(true)) {
      Serial.println("ESP32 connection OK");
      lcdShow("READY", "Stand <= 30cm");
    } else {
      Serial.println("ESP32 connection FAILED");
      lcdShow("ESP32 FAIL", "Check IP/Power");
      setColor(255, 0, 0);
    }
  } else {
    lcdShow("WIFI FAIL", "Check SSID");
    setColor(255, 0, 0);
  }
}

void loop() {
  unsigned long now = millis();

  float dist = smoothDistanceCM(5);

  // presence detection
  bool detected = (dist > 0 && dist <= FAR_CM);

  // ===================== IDLE =====================
  if (!detected) {
    stableHits = 0;
    state = IDLE;

    if (now - lastLEDUpdate > 25) {
      rainbowStep();
      lastLEDUpdate = now;
    }
    lcdIdleScroll(now);
    return;
  }

  // GOOD RANGE check: between MIN_CM and GOOD_CM (with hysteresis)
  bool inGoodRange = (dist >= MIN_CM) && (dist <= (GOOD_CM + HYSTERESIS_CM));

  // ===================== SHOW RESULT (5 seconds + scrolling) =====================
  if (state == SHOW_RESULT) {
    if (now < stateUntil) {
      tickResultScroll(now); // keep scrolling during result display
      setColor(lastPass ? 0 : 255, lastPass ? 255 : 0, 0);
      return;
    }
    state = COOLDOWN;
    stateUntil = now + COOLDOWN_MS;
    lcdShow("PLEASE WAIT", "...");
    setColor(255, 120, 0);
    return;
  }

  // ===================== COOLDOWN =====================
  if (state == COOLDOWN) {
    if (now < stateUntil) {
      setColor(255, 120, 0);
      if (fabs(dist - lastShownDist) > 2.0f) {
        lcdShow("PLEASE WAIT", "Dist:" + String((int)dist) + "cm");
        lastShownDist = dist;
      }
      return;
    }
    stableHits = 0;
    state = TOO_FAR;
  }

  // ===================== TOO FAR =====================
  if (!inGoodRange) {
    state = TOO_FAR;
    stableHits = 0;

    setColor(255, 0, 0);
    if (fabs(dist - lastShownDist) > 1.0f) {
      lcdShow("TOO FAR", "Need <= 30cm");
      lastShownDist = dist;
    }
    return;
  }

  // ===================== READY/STABLE =====================
  if (state != READY_STABLE) {
    state = READY_STABLE;
    stableHits = 0;
    lcdShow("GOOD RANGE", "Hold still...");
    setColor(0, 80, 255);
  }

  stableHits++;
  if (stableHits < IN_RANGE_HITS_REQUIRED) {
    setColor(0, 80, 255);
    return;
  }

  // ===================== RUN VERIFY =====================
  state = TESTING;
  stableHits = 0;

  lcdShow("CHECKING...", "Face verify");
  setColor(0, 0, 255);

  String json;
  bool ok = espVerify(json);

  Serial.println("VERIFY JSON:");
  Serial.println(json);

  if (!ok) {
    lastPass = false;
    startResultDisplay("FAIL: No response from ESP32");
    setColor(255, 0, 0);
  } else {
    lastPass = inferPassFromJson(json);

    String text = extractJsonStringValue(json, "text_LCD");
    if (text.length() == 0) text = "No text_LCD in JSON";
    startResultDisplay(text);

    setColor(lastPass ? 0 : 255, lastPass ? 255 : 0, 0);
  }

  state = SHOW_RESULT;
  stateUntil = millis() + RESULT_HOLD_MS; // 5 seconds
}
