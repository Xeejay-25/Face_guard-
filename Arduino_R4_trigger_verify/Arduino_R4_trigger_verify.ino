
// Arduino UNO R4 WiFi - distance trigger -> request JSON from ESP32-CAM over WiFi
// GOOD RANGE: 5cm to 40cm
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
const char* ESP32_HOST = "192.168.137.77";
const uint16_t ESP32_PORT = 80;

// ===================== Distance rules =====================
const float REGISTER_MIN = 20.0f;     // registration minimum
const float REGISTER_MAX = 25.0f;     // registration maximum
const float MIN_CM = 35.0f;           // minimum detection distance (below = too close)
const float GOOD_CM = 45.0f;          // GOOD RANGE upper limit
const float FAR_CM  = 70.0f;          // presence detection limit
const float HYSTERESIS_CM = 2.5f;     // reduce flicker near boundary

// Must be in-range for N loops before triggering
const int IN_RANGE_HITS_REQUIRED = 10;

// Cooldown after a run (ms)
const unsigned long COOLDOWN_MS = 4500;

// Show result for 5 seconds (per your request)
const unsigned long RESULT_HOLD_MS = 10000;

// ===================== State machine =====================
enum State { IDLE, TOO_CLOSE, TOO_FAR, READY_STABLE, REGISTRATION, TESTING, SHOW_RESULT, COOLDOWN };
State state = IDLE;

// ===================== WiFi check interval =====================
unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL_MS = 3000; // Check WiFi every 3 seconds

// ===================== ESP32 check interval =====================
unsigned long lastESP32Check = 0;
const unsigned long ESP32_CHECK_INTERVAL_MS = 15000; // Check ESP32 every 15 seconds
int esp32FailCount = 0;
const int ESP32_MAX_FAILS = 5; // Reset after 5 consecutive failures

int stableHits = 0;
unsigned long stateUntil = 0;

bool lastPass = false;
float lastShownDist = -999;
bool streamingActive = false; // Track streaming state

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
String idleMessage = "    IDENTITY GUARD   FACE RECOGNITION    ";
int textPos = 0;

// Helper to center text on 16-char LCD line
String centerText(const String& text, int width = 16) {
  int len = text.length();
  if (len >= width) return text.substring(0, width);
  int leftPad = (width - len) / 2;
  String result = "";
  for (int i = 0; i < leftPad; i++) result += " ";
  result += text;
  while (result.length() < width) result += " ";
  return result;
}

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

  // Write directly without clearing to avoid flicker
  lcd.setCursor(0, 0);
  lcd.print(displayMsg.substring(0, 16));

  lcd.setCursor(0, 1);
  lcd.print("                "); // Keep second line clear

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
  if (duration == 0) {
    //Serial.println("[ULTRASONIC] No echo received (timeout)");
    return -1;
  }
  float distance = duration * 0.034f / 2.0f;
  //Serial.print("[ULTRASONIC] Raw distance: ");
  //Serial.print(distance);
  //Serial.println(" cm");
  return distance;
}

float smoothDistanceCM(int samples = 5) {
  //Serial.print("[ULTRASONIC] Starting smoothed reading (");
  //Serial.print(samples);
  //Serial.println(" samples):");
  
  float sum = 0;
  int count = 0;

  for (int i = 0; i < samples; i++) {
    float d = readDistanceCM_once();
    if (d > 0) { sum += d; count++; }
    delay(8);
  }
  
  if (count == 0) {
    Serial.println("[ULTRASONIC] Smoothed result: NO VALID READINGS");
    return -1;
  }
  
  float average = sum / count;
  // Serial.print("[ULTRASONIC] Smoothed result: ");
  // Serial.print(average);
  // Serial.print(" cm (from ");
  // Serial.print(count);
  // Serial.print("/");
  // Serial.print(samples);
  // Serial.println(" valid samples)");
  
  return average;
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
  String body = httpGET(String("/active?enable=") + (enable ? "1" : "0"), 8000);
  return body.length() > 0;
}

bool espSetStreaming(bool enable) {
  String body = httpGET(String("/stream?enable=") + (enable ? "1" : "0"), 2000);
  return body.length() > 0;
}

bool espVerify(String &outJson) {
  outJson = httpGET("/verify", 28000); // capture + upload + inference
  return outJson.length() > 0;
}

// ===================== JSON helpers (no ArduinoJson needed) =====================

// Helper function to remove unwanted patterns from text
String cleanDisplayText(String text) {
  // Remove all possible XXce patterns (where XX is 1-3 digits)
  // Try common patterns: 0ce through 999ce
  for (int i = 0; i < 1000; i++) {
    String pattern = String(i) + "ce";
    text.replace(pattern, "");
  }
  
  // Remove "101" standalone (in case it appears without "ce")
  text.replace("101", "");
  
  // Remove "Verify" suffix
  text.replace(" Verify", "");
  text.replace("Verify", "");
  
  // Remove "verify" lowercase too
  text.replace(" verify", "");
  text.replace("verify", "");
  
  // Clean up multiple spaces
  while (text.indexOf("  ") >= 0) {
    text.replace("  ", " ");
  }
  
  text.trim();
  return text;
}

String extractJsonStringValue(const String& json, const char* key) {
  String k = String("\"") + key + "\":";
  int p = json.indexOf(k);
  if (p < 0) return "";

  p += k.length();
  // Skip whitespace
  while (p < (int)json.length() && (json[p] == ' ' || json[p] == '\n' || json[p] == '\r' || json[p] == '\t')) p++;

  if (p >= (int)json.length() || json[p] != '"') return "";
  p++;

  String out;
  while (p < (int)json.length()) {
    char c = json[p++];
    if (c == '\\') { // escape handling
      if (p < (int)json.length()) {
        char next = json[p++];
        if (next == 'n') out += ' '; // replace newlines with space for LCD
        else if (next == 'r') out += ' ';
        else if (next == 't') out += ' ';
        else out += next;
      }
      continue;
    }
    if (c == '"') break;
    // Clean up control characters for LCD display
    if (c >= 32 && c <= 126) out += c;
    else out += ' ';
  }
  out.trim();
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

// ===================== WiFi reconnect helper =====================
void checkAndReconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  
  Serial.println("WiFi disconnected! Attempting reconnect...");
  lcdShow(centerText("WiFi Lost!"), centerText("Reconnecting"));
  setColor(255, 255, 0); // Yellow
  
  WiFi.disconnect();
  delay(500);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(350);
    Serial.print(".");
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi reconnected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    lcdShow(centerText("WiFi OK!"), centerText("Resuming..."));
    setColor(0, 255, 0);
    delay(1000);
  } else {
    Serial.println("WiFi reconnect failed!");
    lcdShow(centerText("WiFi FAILED"), centerText("Will retry"));
    setColor(255, 0, 0);
    delay(2000);
  }
}

// ===================== ESP32 connection check =====================
void checkESP32Connection() {
  // Only check when idle or cooldown to avoid interference
  if (state != IDLE && state != COOLDOWN) {
    Serial.println("Skipping ESP32 check (system busy)");
    return;
  }
  
  Serial.println("Checking ESP32 connection...");
  
  // Try multiple times before declaring failure
  bool connected = false;
  for (int attempt = 1; attempt <= 2; attempt++) {
    Serial.print("ESP32 check attempt ");
    Serial.print(attempt);
    Serial.print("/2... ");
    
    if (espSetActive(true)) {
      Serial.println("OK");
      connected = true;
      break;
    }
    
    Serial.println("failed");
    if (attempt < 2) delay(800); // Brief delay between retries
  }
  
  if (connected) {
    esp32FailCount = 0; // Reset fail counter on success
    return;
  }
  
  // ESP32 check failed after retries
  esp32FailCount++;
  Serial.print("ESP32 check FAILED! Fail count: ");
  Serial.println(esp32FailCount);
  
  if (esp32FailCount >= ESP32_MAX_FAILS) {
    Serial.println("ESP32 disconnected - Resetting system...");
    lcdShow(centerText("ESP32 LOST!"), centerText("Resetting..."));
    setColor(255, 0, 0);
    delay(2000);
    NVIC_SystemReset(); // Reset Arduino to go back to setup
  } else {
    String failMsg = "Fails: " + String(esp32FailCount) + "/" + String(ESP32_MAX_FAILS);
    lcdShow(centerText("ESP32 Warning"), centerText(failMsg));
    setColor(255, 255, 0); // Yellow for warning
    delay(1000);
  }
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
  lcdShow(centerText("BOOTING..."), centerText("WiFi connect"));
  delay(500);

  // WiFi connection with retry and reset
  int wifiAttempts = 0;
  const int MAX_WIFI_ATTEMPTS = 3;
  
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < MAX_WIFI_ATTEMPTS) {
    wifiAttempts++;
    String attemptMsg = "Try " + String(wifiAttempts) + "/" + String(MAX_WIFI_ATTEMPTS);
    lcdShow(centerText("WiFi Attempt"), centerText(attemptMsg));
    
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long start = millis();
    
    while (WiFi.status() != WL_CONNECTED) {
      delay(350);
      Serial.print(".");
      if (millis() - start > 15000) break; // 15 second timeout per attempt
    }
    Serial.println();
    
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi attempt failed");
      delay(1000);
      WiFi.disconnect();
      delay(500);
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("R4 IP: "); Serial.println(WiFi.localIP());
    
    // Check ESP32 connection with retry loop
    lcdShow(centerText("CHECKING ESP32"), centerText("Please wait"));
    delay(800);
    
    bool esp32Connected = false;
    int esp32Attempts = 0;
    const int MAX_ESP32_ATTEMPTS = 5;
    
    while (!esp32Connected) {
      esp32Attempts++;
      String attemptMsg = "Try " + String(esp32Attempts) + "/" + String(MAX_ESP32_ATTEMPTS);
      lcdShow(centerText("ESP32 Check"), centerText(attemptMsg));
      Serial.print("ESP32 attempt "); Serial.print(esp32Attempts); Serial.println("...");
      
      if (espSetActive(true)) {
        esp32Connected = true;
        Serial.println("ESP32 connection OK");
        lcdShow(centerText("CONNECTED!"), centerText("System Ready"));
        setColor(0, 255, 0); // Green for good connection
        delay(1500);
        lcdShow(centerText("READY"), centerText(""));
        setColor(0, 255, 0);
      } else {
        Serial.println("ESP32 connection attempt failed");
        delay(1500);
        
        if (esp32Attempts >= MAX_ESP32_ATTEMPTS) {
          Serial.println("ESP32 connection FAILED - Resetting to attempt 1");
          lcdShow(centerText("ESP32 FAILED"), centerText("Retrying..."));
          setColor(255, 255, 0); // Yellow for waiting
          delay(2000);
          esp32Attempts = 0; // Reset counter to loop again
        }
      }
    }
  } else {
    Serial.println("WiFi FAILED after all attempts");
    lcdShow(centerText("WiFi FAILED"), centerText("Restarting..."));
    setColor(255, 0, 0);
    delay(3000);
    // Reset to retry from beginning
    NVIC_SystemReset();
  }
}

void loop() {
  unsigned long now = millis();

  // ===================== WiFi check =====================
  if (now - lastWiFiCheck >= WIFI_CHECK_INTERVAL_MS) {
    lastWiFiCheck = now;
    checkAndReconnectWiFi();
  }

  // ===================== ESP32 check =====================
  if (now - lastESP32Check >= ESP32_CHECK_INTERVAL_MS) {
    lastESP32Check = now;
    checkESP32Connection();
  }

  float dist = smoothDistanceCM(5);

  // presence detection
  bool detected = (dist > 0 && dist <= FAR_CM);

  // ===================== IDLE =====================
  if (!detected) {
    stableHits = 0;
    if (state != IDLE) {
      state = IDLE;
    }
    // Turn off streaming when no one detected
    if (streamingActive) {
      espSetStreaming(false);
      streamingActive = false;
      Serial.println("[STREAM] Camera live feed stopped - No person detected");
    }

    if (now - lastLEDUpdate > 10) { // Faster RGB transition (changed from 25 to 10)
      rainbowStep();
      lastLEDUpdate = now;
    }
    lcdIdleScroll(now);
    return;
  }

  // Person detected within 70cm - enable streaming
  if (!streamingActive) {
    espSetStreaming(true);
    streamingActive = true;
    Serial.print("[STREAM] Camera live feed started - Person detected at ");
    Serial.print((int)dist);
    Serial.println(" cm");
  }

  // GOOD RANGE check: between MIN_CM and GOOD_CM (with hysteresis)
  bool inGoodRange = (dist >= MIN_CM) && (dist <= (GOOD_CM + HYSTERESIS_CM));

  // ===================== SHOW RESULT (5 seconds + scrolling) =====================
  if (state == SHOW_RESULT) {
    if (now < stateUntil) {
      tickResultScroll(now); // keep scrolling during result display
      setColor(lastPass ? 0 : 255, lastPass ? 255 : 0, 0); // Green for pass, Red for fail
      return;
    }
    state = COOLDOWN;
    stateUntil = now + COOLDOWN_MS;
    lcdShow(centerText("PLEASE WAIT"), centerText("Cooldown..."));
    setColor(255, 255, 0); // Yellow for please wait
    return;
  }

  // ===================== COOLDOWN =====================
  if (state == COOLDOWN) {
    if (now < stateUntil) {
      setColor(255, 255, 0); // Yellow for please wait
      if (fabs(dist - lastShownDist) > 2.0f) {
        String distMsg = String((int)dist) + " cm";
        lcdShow(centerText("PLEASE WAIT"), centerText(distMsg));
        lastShownDist = dist;
        delay(100); // LCD update delay
      }
      return;
    }
    // Transition delay
    delay(200);
    stableHits = 0;
    state = TOO_FAR;
  }

  // ===================== REGISTRATION MODE (20-25cm) =====================
    if (dist > 0 && dist >= REGISTER_MIN && dist <= REGISTER_MAX) {
    state = REGISTRATION;
    stableHits = 0;

    setColor(0, 255, 0); // Green LED for registration
    if (fabs(dist - lastShownDist) > 1.0f) {
      String distMsg = String((int)dist) + " cm";
      lcdShow(centerText("REGISTRATION"), centerText("Mode Active"));
      lastShownDist = dist;
      delay(150); // LCD update delay
    }
    return;
  }

  // ===================== TOO CLOSE =====================
  if (dist > 0 && dist < MIN_CM) {
    state = TOO_CLOSE;
    stableHits = 0;

    setColor(255, 0, 0); // Red for error
    if (fabs(dist - lastShownDist) > 1.0f) {
      String distMsg = String((int)dist) + " cm";
      lcdShow(centerText("TOO CLOSE!"), centerText(distMsg));
      lastShownDist = dist;
      delay(150); // LCD update delay
    }
    return;
  }

  // ===================== TOO FAR =====================
  if (!inGoodRange) {
    state = TOO_FAR;
    stableHits = 0;

    setColor(255, 0, 0);
    if (fabs(dist - lastShownDist) > 1.0f) {
      String distMsg = String((int)dist) + " cm";
      lcdShow(centerText("TOO FAR!"), centerText(distMsg));
      lastShownDist = dist;
      delay(150); // LCD update delay
    }
    return;
  }

  // ===================== READY/STABLE =====================
  if (state != READY_STABLE) {
    state = READY_STABLE;
    stableHits = 0;
    lcdShow(centerText("GOOD RANGE"), centerText("Hold still..."));
    setColor(255, 255, 255); // White for lighting in good range
    delay(250); // State transition delay
  }

  stableHits++;
  if (stableHits < IN_RANGE_HITS_REQUIRED) {
    setColor(255, 255, 255); // White for lighting in good range
    return;
  }

  // ===================== RUN VERIFY =====================
  state = TESTING;
  stableHits = 0;

  // 3 second countdown before capture
  for (int countdown = 3; countdown >= 1; countdown--) {
    String countStr = String(countdown);
    lcdShow(centerText("GET READY!"), centerText(countStr));
    setColor(255, 255, 255);; // Blue for checking/verifying
    delay(1000); // 1 second per count
  }

  lcdShow(centerText("CHECKING..."), centerText("Face verify"));
  setColor(0, 0, 255); // Blue for checking/verifying

  String json;
  bool ok = espVerify(json);

  Serial.println("VERIFY JSON:");
  Serial.println(json);

  delay(300); // Brief delay before processing result

  if (!ok || json.length() < 5) {
    lastPass = false;
    String errMsg = "ERROR: No ESP32 response - Check connection";
    startResultDisplay(errMsg);
    setColor(255, 0, 0);
    Serial.println(errMsg);
  } else {
    lastPass = inferPassFromJson(json);

    // Try multiple JSON keys for display text
    String text = extractJsonStringValue(json, "text_LCD");
    if (text.length() == 0) text = extractJsonStringValue(json, "message");
    if (text.length() == 0) text = extractJsonStringValue(json, "result");
    if (text.length() == 0) text = extractJsonStringValue(json, "status");
    
    // If still no text, provide meaningful default
    if (text.length() == 0) {
      if (lastPass) {
        text = "WELCOME! Access Granted";
      } else {
        text = "ACCESS DENIED - Face Not Recognized";
      }
    }
    
    // Clean up text for better LCD display
    text.replace("\\n", " ");
    text.replace("\\r", "");
    
    // Debug: show text BEFORE cleaning
    Serial.print("Display text (BEFORE cleaning): ");
    Serial.println(text);
    
    // Use cleaning function to remove varying patterns (XXce, 101, Verify)
    text = cleanDisplayText(text);
    
    Serial.print("Display text (AFTER cleaning): ");
    Serial.println(text);
    
    startResultDisplay(text);
    setColor(lastPass ? 0 : 255, lastPass ? 255 : 0, 0);
    
    delay(500); // Allow LCD to update
  }

  state = SHOW_RESULT;
  stateUntil = millis() + RESULT_HOLD_MS; // 5 seconds
}
