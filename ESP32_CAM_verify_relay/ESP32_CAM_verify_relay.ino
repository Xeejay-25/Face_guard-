// ESP32-CAM (AI Thinker) - Modern UI (clean controls) + live view (snapshot loop)
// + verify/register passthrough JSON
//
// Endpoints:
//   GET /                      -> Modern UI dashboard
//   GET /help                  -> plain text help
//   GET /active?enable=1|0     -> arms/disarms verification
//   GET /capture               -> returns a single JPEG frame
//   GET /verify                -> captures JPEG, POSTs to FACE_API_URL, returns server JSON (passthrough)
//   GET /register?name=...     -> captures JPEG, POSTs to REGISTER_API_BASE + urlencoded(name), returns JSON (passthrough)
//   GET /state                 -> device state JSON

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>

// -------------------- WiFi --------------------
const char* WIFI_SSID = "esp32";
const char* WIFI_PASS = "12345678";

// -------------------- Face server --------------------
const char* FACE_API_URL        = "http://192.168.137.49:8000/identify";
const char* REGISTER_API_BASE   = "http://192.168.137.49:8000/register?name=";
const char* FACE_API_KEY        = "CARL_PHILIP_RENCE";

// -------------------- Camera pins (AI Thinker ESP32-CAM) --------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// -------------------- Globals --------------------
WebServer server(80);
SemaphoreHandle_t camMutex;

bool g_active = true;
bool g_busy   = false;
bool g_streaming = false;

unsigned long g_lastMs = 0;
String g_lastJson = "{\"status\":\"none\"}"; // always valid JSON for /state
String g_lastErr  = "";

// -------------------- WiFi monitoring --------------------
unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 3000; // Check every 3 seconds

// -------------------- UI HTML --------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>ESP32-CAM Dashboard</title>
  <style>
    :root{
      --bg1:#0b1020; --bg2:#111a33;
      --card: rgba(255,255,255,.07);
      --card2: rgba(255,255,255,.10);
      --line: rgba(255,255,255,.12);
      --txt: rgba(255,255,255,.92);
      --muted: rgba(255,255,255,.70);
      --good:#22c55e; --bad:#ef4444; --warn:#f59e0b; --blue:#3b82f6;
      --shadow: 0 16px 40px rgba(0,0,0,.45);
      --r: 18px;
    }
    *{box-sizing:border-box}
    body{
      margin:0; font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial;
      color:var(--txt);
      background: radial-gradient(1200px 700px at 20% 0%, #1b2a6b 0%, transparent 55%),
                  radial-gradient(1000px 600px at 90% 10%, #3b0f5a 0%, transparent 60%),
                  linear-gradient(180deg, var(--bg1), var(--bg2));
      min-height:100vh;
    }
    .wrap{max-width:1080px; margin:0 auto; padding:22px;}
    .topbar{display:flex; align-items:center; justify-content:space-between; margin-bottom:16px;}
    .title{display:flex; gap:12px; align-items:center;}
    .logo{
      width:44px; height:44px; border-radius:14px;
      background: linear-gradient(135deg, rgba(59,130,246,.9), rgba(34,197,94,.85));
      box-shadow: var(--shadow);
    }
    h1{margin:0; font-size:18px; letter-spacing:.2px}
    .sub{margin:2px 0 0; color:var(--muted); font-size:12px}
    .grid{display:grid; gap:16px; grid-template-columns: 1.2fr .8fr;}
    @media (max-width: 900px){ .grid{grid-template-columns: 1fr;} }

    .card{
      background: var(--card);
      border: 1px solid var(--line);
      border-radius: var(--r);
      box-shadow: var(--shadow);
      overflow:hidden;
      backdrop-filter: blur(10px);
    }
    .cardHeader{
      padding:14px 14px 10px;
      display:flex; align-items:center; justify-content:space-between;
      border-bottom:1px solid var(--line);
    }
    .cardHeader .h{font-size:13px; color:var(--muted); letter-spacing:.2px;}
    .cardBody{padding:14px;}

    .videoWrap{
      position:relative; border-radius: 16px; overflow:hidden;
      background: rgba(0,0,0,.35);
      border: 1px solid rgba(255,255,255,.10);
    }
    img#cam{
      width:100%; display:block;
      aspect-ratio: 4 / 3;
      object-fit: cover;
      background: rgba(0,0,0,.25);
    }
    .overlay{
      position:absolute; inset:0;
      display:flex; align-items:center; justify-content:center;
      background: rgba(0,0,0,.45);
      color: rgba(255,255,255,.92);
      font-weight:600;
      text-align:center;
      padding:16px;
      opacity:0; pointer-events:none;
      transition: opacity .2s ease;
    }
    .overlay.show{opacity:1;}

    .row{display:flex; gap:10px; flex-wrap:wrap; align-items:center;}
    .chip{
      display:inline-flex; align-items:center; gap:8px;
      padding:8px 10px;
      border-radius: 999px;
      border:1px solid var(--line);
      background: var(--card2);
      font-size:12px;
      color: var(--muted);
    }
    .dot{width:10px; height:10px; border-radius:999px; background: var(--warn);}
    .dot.good{background:var(--good)}
    .dot.bad{background:var(--bad)}
    .dot.blue{background:var(--blue)}

    .btn{
      appearance:none; border:1px solid var(--line);
      background: rgba(255,255,255,.10);
      color: var(--txt);
      padding:10px 12px;
      border-radius: 14px;
      cursor:pointer;
      font-weight:600;
      font-size:13px;
      transition: transform .06s ease, background .15s ease;
      display:inline-flex; align-items:center; justify-content:center;
    }
    .btn:hover{background: rgba(255,255,255,.14)}
    .btn:active{transform: translateY(1px)}
    .btn.primary{background: rgba(59,130,246,.22); border-color: rgba(59,130,246,.35)}
    .btn.danger{background: rgba(239,68,68,.18); border-color: rgba(239,68,68,.35)}
    .btn:disabled{opacity:.55; cursor:not-allowed}

    input[type=text]{
      background: rgba(0,0,0,.2); border:1px solid var(--line);
      color: var(--txt); padding:8px 12px; border-radius:12px;
      outline:none; font-size:13px; width:100%;
    }
    input[type=text]:focus{border-color:var(--blue)}

    .switch{display:inline-flex; align-items:center; gap:10px; user-select:none;}
    .toggle{
      width:48px; height:28px; border-radius:999px;
      border:1px solid var(--line);
      background: rgba(255,255,255,.10);
      position:relative;
      cursor:pointer;
    }
    .knob{
      width:22px; height:22px; border-radius:999px;
      position:absolute; top:2px; left:2px;
      background: rgba(255,255,255,.92);
      transition: left .15s ease;
    }
    .toggle.on{background: rgba(34,197,94,.22); border-color: rgba(34,197,94,.35)}
    .toggle.on .knob{left:24px}

    .kv{
      display:grid; grid-template-columns: 120px 1fr;
      gap:10px; font-size:13px;
      padding:10px 0; border-bottom:1px dashed rgba(255,255,255,.14);
    }
    .kv:last-child{border-bottom:none}
    .k{color:var(--muted)}

    pre{
      margin:0; padding:12px;
      background: rgba(0,0,0,.35);
      border:1px solid rgba(255,255,255,.10);
      border-radius: 14px;
      overflow:auto;
      max-height: 310px;
      color: rgba(255,255,255,.90);
      font-size: 12px;
      line-height: 1.35;
    }

    .bigStatus{
      font-size: 16px;
      font-weight: 800;
      letter-spacing: .2px;
      padding:10px 12px;
      border-radius: 14px;
      border: 1px solid rgba(255,255,255,.12);
      background: rgba(0,0,0,.25);
    }
    .bigStatus.good{border-color: rgba(34,197,94,.35); background: rgba(34,197,94,.12)}
    .bigStatus.bad{border-color: rgba(239,68,68,.35); background: rgba(239,68,68,.12)}
    .mini{font-size:12px; color:var(--muted); margin-top:8px}
    .spacer{height:10px}

    /* --- Modern controls area --- */
    .controls {
      display: grid;
      gap: 12px;
      margin-top: 12px;
    }
    .panel {
      background: rgba(0,0,0,.18);
      border: 1px solid rgba(255,255,255,.10);
      border-radius: 16px;
      padding: 12px;
    }
    .panelTitle {
      font-size: 12px;
      color: var(--muted);
      margin-bottom: 10px;
      letter-spacing: .2px;
    }
    .statusBar {
      display:flex;
      align-items:center;
      justify-content:space-between;
      gap: 10px;
      flex-wrap: wrap;
    }
    .actionsGrid {
      display:grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }
    @media (max-width: 700px){
      .actionsGrid { grid-template-columns: 1fr; }
    }
    .btnWide {
      width: 100%;
      justify-content: center;
      padding: 12px 14px;
      border-radius: 16px;
      font-size: 14px;
    }
    .fieldRow {
      display:flex;
      gap: 8px;
      align-items:center;
    }
    .fieldRow input[type=text] {
      height: 42px;
      border-radius: 14px;
    }
    .btnSmall {
      padding: 10px 12px;
      border-radius: 14px;
      height: 42px;
      white-space: nowrap;
    }
    .secondaryRow {
      display:flex;
      gap: 10px;
      flex-wrap: wrap;
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="topbar">
      <div class="title">
        <div class="logo"></div>
        <div>
          <h1>ESP32-CAM • Face Verify Dashboard</h1>
          <div class="sub">Modern controls + Live view + Verify/Register passthrough JSON</div>
        </div>
      </div>
    </div>

    <div class="grid">
      <div class="card">
        <div class="cardHeader">
          <div class="h">Live Camera</div>
        </div>

        <div class="cardBody">
          <div class="videoWrap">
            <img id="cam" alt="camera feed"/>
            <div class="overlay" id="overlay">Inactive or busy…</div>
          </div>

          <div class="controls">

            <!-- Status / Active -->
            <div class="panel">
              <div class="statusBar">
                <div class="switch">
                  <div class="toggle" id="toggleActive"><div class="knob"></div></div>
                  <div>
                    <div style="font-weight:800">Active</div>
                    <div class="sub" id="activeHint">Device is armed</div>
                  </div>
                </div>

                <div class="chip">
                  <span class="dot blue" id="dotLive"></span>
                  <span id="liveLabel">LIVE</span>
                </div>
              </div>
            </div>

            <!-- Primary actions -->
            <div class="panel">
              <div class="panelTitle">Actions</div>
              <div class="actionsGrid">
                <button class="btn primary btnWide" id="btnVerify">Verify Now</button>

                <div>
                  <div class="fieldRow">
                    <input type="text" id="regName" placeholder="Name for registration" />
                    <button class="btn btnSmall" id="btnRegister">Register</button>
                  </div>
                  <div class="sub" style="margin-top:6px;">Tip: Face the camera and keep steady.</div>
                </div>
              </div>
            </div>

            <!-- Secondary controls -->
            <div class="panel">
              <div class="panelTitle">Live Controls</div>
              <div class="secondaryRow">
                <button class="btn danger" id="btnStop">Stop Live</button>
                <button class="btn" id="btnStart">Start Live</button>
                <button class="btn" id="btnRefresh">Refresh</button>
                <button class="btn" id="btnSnap">Capture JPG</button>
              </div>
            </div>

          </div>

          <div class="spacer"></div>

          <div class="bigStatus" id="bigStatus">Waiting for verify…</div>
          <div class="mini" id="miniStatus">Tip: Verify/Register pauses live view briefly to avoid camera busy.</div>
        </div>
      </div>

      <div class="card">
        <div class="cardHeader">
          <div class="h">Device State</div>
          <div class="row">
            <div class="chip"><span class="dot" id="dotState"></span><span id="stateLabel">UNKNOWN</span></div>
          </div>
        </div>
        <div class="cardBody">
          <div class="kv"><div class="k">IP</div><div id="ip">-</div></div>
          <div class="kv"><div class="k">RSSI</div><div id="rssi">-</div></div>
          <div class="kv"><div class="k">Busy</div><div id="busy">-</div></div>
          <div class="kv"><div class="k">Last Error</div><div id="lerr">-</div></div>

          <div class="spacer"></div>
          <div class="h" style="margin-bottom:8px; color:rgba(255,255,255,.72)">Last JSON</div>
          <pre id="jsonBox">{}</pre>
        </div>
      </div>
    </div>
  </div>

<script>
  let liveTimer = null;
  let liveFps = 6;

  const $ = (id)=>document.getElementById(id);

  function setOverlay(show, text){
    const ov = $("overlay");
    if(text) ov.textContent = text;
    ov.classList.toggle("show", !!show);
  }

  function stopLive(){
    if(liveTimer){ clearInterval(liveTimer); liveTimer=null; }
    $("liveLabel").textContent = "STOPPED";
    $("dotLive").className = "dot bad";
  }

  function startLive(){
    stopLive();
    $("liveLabel").textContent = "LIVE";
    $("dotLive").className = "dot good";
    setOverlay(false);

    liveTimer = setInterval(()=>{
      $("cam").src = "/capture?t=" + Date.now();
    }, Math.max(120, Math.floor(1000/liveFps)));
  }

  function pretty(obj){
    try { return JSON.stringify(obj, null, 2); }
    catch(e){ return String(obj); }
  }

  function parseMaybeJson(text){
    try { return JSON.parse(text); } catch(e){ return { raw: text }; }
  }

  function setBigStatus(kind, text){
    const el = $("bigStatus");
    el.className = "bigStatus" + (kind ? (" " + kind) : "");
    el.textContent = text || "—";
  }

  function extractStatusText(obj){
    if(!obj || typeof obj !== "object") return "";
    return obj.text_LCD || obj.message || obj.status || obj.result || obj.raw || "";
  }

  function isWelcomeText(s){
    if(!s) return false;
    s = String(s).toLowerCase();
    return s.includes("welcome") || s.includes("recognized") || s.includes("verified");
  }

  async function refreshState(){
    try{
      const st = await fetch("/state", {cache:"no-store"}).then(r=>r.json());
      $("ip").textContent = st.ip || "-";
      $("rssi").textContent = (st.rssi!=null) ? (st.rssi + " dBm") : "-";
      $("busy").textContent = st.busy ? "true" : "false";
      $("lerr").textContent = st.lastError || "";

      const t = $("toggleActive");
      t.classList.toggle("on", !!st.active);
      $("activeHint").textContent = st.active ? "Device is armed" : "Device is inactive";

      if(!st.active){
        $("dotState").className = "dot bad";
        $("stateLabel").textContent = "INACTIVE";
        setOverlay(true, "Inactive. Turn Active ON to view & verify.");
        stopLive();
      }else if(st.busy){
        $("dotState").className = "dot";
        $("stateLabel").textContent = "BUSY";
        setOverlay(true, "Busy… (verify/register/capture running)");
      }else{
        $("dotState").className = "dot good";
        $("stateLabel").textContent = "READY";
        setOverlay(false);
      }

      // Auto-start/stop live stream based on streaming state
      if(st.streaming && !liveTimer && st.active && !st.busy){
        startLive();
      }else if(!st.streaming && liveTimer){
        stopLive();
      }

      $("jsonBox").textContent = pretty(st.last || {});
    }catch(e){
      $("dotState").className = "dot bad";
      $("stateLabel").textContent = "OFFLINE";
      setOverlay(true, "Cannot reach ESP32. Check WiFi/IP.");
      stopLive();
    }
  }

  async function setActive(enable){
    await fetch("/active?enable=" + (enable ? "1":"0"), {cache:"no-store"});
    await refreshState();
    if(enable && !liveTimer) startLive();
    if(!enable) stopLive();
  }

  async function verifyNow(){
    const wasLive = !!liveTimer;
    stopLive();

    $("btnVerify").disabled = true;
    setBigStatus("", "Verifying…");

    try{
      const respText = await fetch("/verify", {cache:"no-store"}).then(r=>r.text());
      const obj = parseMaybeJson(respText);
      $("jsonBox").textContent = pretty(obj);

      const msg = extractStatusText(obj) || respText;

      if(obj && obj.error){
        setBigStatus("bad", "Error: " + (obj.error || msg));
      }else if(isWelcomeText(msg)){
        setBigStatus("good", msg);
      }else{
        setBigStatus("bad", msg || "Not recognized");
      }
    }catch(e){
      setBigStatus("bad", "Verify failed: " + e.message);
    }finally{
      $("btnVerify").disabled = false;
      await refreshState();
      if(wasLive && $("toggleActive").classList.contains("on")) startLive();
    }
  }

  async function registerFace(){
    const name = $("regName").value.trim();
    if(!name){ alert("Please enter a name first."); return; }

    const wasLive = !!liveTimer;
    stopLive();

    $("btnRegister").disabled = true;
    setBigStatus("", "Registering " + name + "…");

    try{
      const url = "/register?name=" + encodeURIComponent(name);
      const respText = await fetch(url, {cache:"no-store"}).then(r=>r.text());
      const obj = parseMaybeJson(respText);
      $("jsonBox").textContent = pretty(obj);

      const msg = extractStatusText(obj) || respText;

      if(obj && obj.error){
        setBigStatus("bad", "Reg Error: " + (obj.error || msg));
      }else{
        setBigStatus("good", msg || ("Registered: " + name));
      }
    }catch(e){
      setBigStatus("bad", "Register failed: " + e.message);
    }finally{
      $("btnRegister").disabled = false;
      await refreshState();
      if(wasLive && $("toggleActive").classList.contains("on")) startLive();
    }
  }

  // UI events
  $("btnRefresh").onclick = refreshState;
  $("btnStart").onclick = startLive;
  $("btnStop").onclick = stopLive;

  $("btnSnap").onclick = ()=>{
    window.open("/capture?t=" + Date.now(), "_blank");
  };

  $("btnVerify").onclick = verifyNow;
  $("btnRegister").onclick = registerFace;

  $("toggleActive").onclick = async ()=>{
    const isOn = $("toggleActive").classList.contains("on");
    await setActive(!isOn);
  };

  $("cam").onerror = ()=>{
    setOverlay(true, "No frame (inactive/busy). Try Refresh or Active ON.");
  };

  // boot
  (async function boot(){
    await refreshState();
    if ($("toggleActive").classList.contains("on")) startLive();
    setInterval(refreshState, 2500);
  })();
</script>
</body>
</html>
)HTML";

// -------------------- Helpers --------------------
static String jsonEscape(const String& s) {
  String out; out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '\\') out += "\\\\";
    else if (c == '"') out += "\\\"";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else out += c;
  }
  return out;
}

static String normalizeJson(const String& s) {
  String t = s;
  t.trim();
  if (t.length() == 0) return "{}";
  if (t[0] == '{' || t[0] == '[') return t;
  return String("{\"raw\":\"") + jsonEscape(t) + "\"}";
}

static void sendJson(int code, const String& body) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

static String urlEncode(const String& s) {
  const char* hex = "0123456789ABCDEF";
  String out; out.reserve(s.length() * 3);
  for (size_t i = 0; i < s.length(); i++) {
    uint8_t c = (uint8_t)s[i];
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
        c == '-' || c == '_' || c == '.' || c == '~') {
      out += (char)c;
    } else {
      out += '%';
      out += hex[(c >> 4) & 0xF];
      out += hex[c & 0xF];
    }
  }
  return out;
}

// -------------------- WiFi Helper --------------------
void checkAndReconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  
  Serial.println("[WiFi] Disconnected! Reconnecting...");
  
  WiFi.disconnect();
  delay(500);
  
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Reconnected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Reconnect failed, will retry...");
  }
}

// -------------------- Routes --------------------
void handleHelp() {
  String msg =
    "ESP32-CAM is online.\n\n"
    "GET /                 -> UI dashboard\n"
    "GET /help             -> this help\n"
    "GET /capture          -> JPEG frame\n"
    "GET /verify           -> capture + POST to FACE_API_URL + return JSON\n"
    "GET /register?name=.. -> capture + POST to REGISTER_API_BASE + name\n"
    "GET /state            -> state JSON\n"
    "GET /active?enable=1|0\n"
    "GET /stream?enable=1|0\n";
  server.send(200, "text/plain", msg);
}

void handleUI() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleActive() {
  if (server.hasArg("enable")) {
    String v = server.arg("enable");
    g_active = (v == "1" || v == "true");
  }
  // Quick response - don't let this timeout
  String response = String("{\"active\":") + (g_active ? "true" : "false") + ",\"busy\":" + (g_busy ? "true" : "false") + "}";
  sendJson(200, response);
}

void handleStream() {
  if (server.hasArg("enable")) {
    String v = server.arg("enable");
    g_streaming = (v == "1" || v == "true");
  }
  sendJson(200, String("{\"streaming\":") + (g_streaming ? "true" : "false") + "}");
}

void handleState() {
  String json = "{";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"active\":" + String(g_active ? "true" : "false") + ",";
  json += "\"busy\":" + String(g_busy ? "true" : "false") + ",";
  json += "\"streaming\":" + String(g_streaming ? "true" : "false") + ",";
  json += "\"lastAtMs\":" + String(g_lastMs) + ",";
  json += "\"last\":" + g_lastJson + ",";
  json += "\"lastError\":\"" + jsonEscape(g_lastErr) + "\"";
  json += "}";
  sendJson(200, json);
}

void handleCapture() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");

  if (!g_active) { server.send(503, "text/plain", "Inactive"); return; }
  if (g_busy)    { server.send(503, "text/plain", "Busy"); return; }

  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
    server.send(503, "text/plain", "Camera busy");
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    xSemaphoreGive(camMutex);
    server.send(500, "text/plain", "Capture failed");
    return;
  }

  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);

  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);
}

// Captures a JPEG and POSTs it to targetUrl as raw bytes (Content-Type: image/jpeg).
String postFrameToFaceServer(camera_fb_t* fb, int &outHttpCode, const char* targetUrl) {
  HTTPClient http;
  WiFiClient client;

  http.setTimeout(25000);
  if (!http.begin(client, targetUrl)) {
    outHttpCode = -1;
    return "{\"error\":\"http_begin_failed\"}";
  }

  http.addHeader("Content-Type", "image/jpeg");
  if (FACE_API_KEY && strlen(FACE_API_KEY) > 0) {
    http.addHeader("X-API-Key", FACE_API_KEY);
  }

  outHttpCode = http.POST(fb->buf, fb->len);
  String payload = http.getString();
  http.end();

  if (outHttpCode <= 0) return "{\"error\":\"face_server_unreachable\"}";
  if (payload.length() == 0) return "{\"error\":\"empty_response_from_server\"}";
  return payload;
}

void handleVerify() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");

  if (!g_active) { server.send(503, "text/plain", "Inactive"); return; }
  if (g_busy)    { server.send(503, "text/plain", "Busy"); return; }

  g_busy = true;
  g_lastErr = "";

  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(4000)) != pdTRUE) {
    g_busy = false;
    g_lastErr = "camera_busy";
    g_lastJson = "{\"error\":\"camera_busy\"}";
    sendJson(503, g_lastJson);
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    xSemaphoreGive(camMutex);
    g_busy = false;
    g_lastErr = "capture_failed";
    g_lastJson = "{\"error\":\"capture_failed\"}";
    sendJson(500, g_lastJson);
    return;
  }

  int faceHttp = 0;
  String faceResp = postFrameToFaceServer(fb, faceHttp, FACE_API_URL);

  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);

  g_lastMs = millis();
  g_lastJson = normalizeJson(faceResp);
  if (faceHttp != 200) g_lastErr = "face_http_" + String(faceHttp);

  g_busy = false;

  // passthrough (raw server response)
  sendJson(200, faceResp);
}

void handleRegister() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");

  if (!server.hasArg("name")) { sendJson(400, "{\"error\":\"missing_name\"}"); return; }
  String name = server.arg("name");
  if (name.length() == 0)      { sendJson(400, "{\"error\":\"empty_name\"}"); return; }

  if (!g_active) { server.send(503, "text/plain", "Inactive"); return; }
  if (g_busy)    { server.send(503, "text/plain", "Busy"); return; }

  g_busy = true;
  g_lastErr = "";

  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(4000)) != pdTRUE) {
    g_busy = false;
    g_lastErr = "camera_busy";
    g_lastJson = "{\"error\":\"camera_busy\"}";
    sendJson(503, g_lastJson);
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    xSemaphoreGive(camMutex);
    g_busy = false;
    g_lastErr = "capture_failed";
    g_lastJson = "{\"error\":\"capture_failed\"}";
    sendJson(500, g_lastJson);
    return;
  }

  int regHttp = 0;
  String url = String(REGISTER_API_BASE) + urlEncode(name);
  String regResp = postFrameToFaceServer(fb, regHttp, url.c_str());

  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);

  g_lastMs = millis();
  g_lastJson = normalizeJson(regResp);
  if (regHttp != 200) g_lastErr = "register_http_" + String(regHttp);

  g_busy = false;

  // passthrough (raw server response)
  sendJson(200, regResp);
}

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  camMutex = xSemaphoreCreateMutex();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size   = FRAMESIZE_QVGA; // 320x240
  config.jpeg_quality = 12;
  config.fb_count     = psramFound() ? 2 : 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }

  // WiFi setup with better stability
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false); // Disable WiFi sleep for stability
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) { 
    delay(400); 
    Serial.print("."); 
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("WiFi connection failed! Restarting...");
    ESP.restart();
  }

  server.on("/", handleUI);
  server.on("/help", handleHelp);
  server.on("/active", handleActive);
  server.on("/stream", handleStream);
  server.on("/state", handleState);
  server.on("/capture", handleCapture);
  server.on("/verify", handleVerify);
  server.on("/register", handleRegister);

  server.begin();
  Serial.println("HTTP server started on port 80");
}

void loop() {
  // Check WiFi connection periodically
  unsigned long now = millis();
  if (now - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = now;
    checkAndReconnectWiFi();
  }
  
  server.handleClient();
}
