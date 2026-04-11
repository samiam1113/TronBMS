// ============================================================================
// bms_wifi.cpp — WiFi Access Point + HTTP/WebSocket telemetry server
//
// HOW IT WORKS:
//   The ESP32 creates its own WiFi Access Point (no router needed).
//   Connect your phone/laptop to the "TronBMS" network, then open a browser
//   to http://192.168.4.1  
//   password is "bart1234"
//   will need to add xTaskCreatePinnedToCore(task_wifi_broadcast, "wifi_bc", 4096, nullptr, 1, nullptr, 0) to main :)
//

#include "bms_wifi.h"
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "bms_config.h"
#include "bms_types.h"
#include "bms_fault.h"
#include "bms_state.h"
#include "freertos/semphr.h"
#include "bms_fault.h"
#include "bms_fsm.h"
extern BmsFsm g_fsm;


// ── AP credentials ─────────────────────────────────────────────────────────
#define WIFI_SSID       "TronBMS"
#define WIFI_PASSWORD   "bart1234"   // min 8 chars; set "" for open network
#define WIFI_CHANNEL    6
#define WIFI_MAX_CONN   4

// ── Server objects ─────────────────────────────────────────────────────────
static AsyncWebServer s_server(80);
static AsyncWebSocket s_ws("/ws");

// ── Latest telemetry snapshot (double-buffered for thread safety) ───────────
static measurement_data_t s_meas_snap = {};
static BmsState           s_state_snap = BmsState::INIT;
static uint16_t           s_fault_snap = 0;
static SemaphoreHandle_t  s_snap_mutex = nullptr;
static volatile bool      s_snap_dirty = false;

// ============================================================================
// HTML / CSS / JS — served as a single self-contained page
// ============================================================================
// The GUI is stored in program flash (PROGMEM) to avoid SPIFFS dependency.
// It is ~12 KB and served on first connect; subsequent updates are WebSocket.
static const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>TronBMS</title>
<style>
  /* ── Reset & base ── */
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
  :root {
    --bg:       #171518;
    --surface:  #2d2a2e;
    --border:   #a6c7ed;
    --accent:   #00e5ff;
    --warn:     #ffb300;
    --danger:   #ff1744;
    --ok:       #00e676;
    --muted:    #769aab;
    --text:     #cfd8dc;
    --text-dim: #a3d5ec;
    --radius:   6px;
    --cell-h:   56px;
  }
  html, body { height: 100%; background: var(--bg); color: var(--text);
               font-family: 'Courier New', monospace; font-size: 13px; }
  body { display: flex; flex-direction: column; overflow: hidden; }

  /* ── Header ── */
  header {
    display: flex; align-items: center; justify-content: space-between;
    padding: 10px 18px; background: var(--surface);
    border-bottom: 1px solid var(--border); flex-shrink: 0;
  }
  header h1 { font-size: 18px; letter-spacing: 4px; color: var(--accent);
               text-transform: uppercase; }
  #status-dot { width: 10px; height: 10px; border-radius: 50%;
                background: var(--muted); transition: background .4s; display: inline-block; margin-right: 6px; }
  #status-dot.connected { background: var(--ok); box-shadow: 0 0 8px var(--ok); }
  #status-dot.error     { background: var(--danger); }
  #conn-label { font-size: 11px; color: var(--text-dim); }

  /* ── Fault banner ── */
  #fault-banner {
    display: none; background: var(--danger); color: #fff;
    text-align: center; padding: 6px; font-size: 12px;
    letter-spacing: 2px; font-weight: bold; flex-shrink: 0;
  }
  #fault-banner.visible { display: block; }

  /* ── Layout ── */
  main { flex: 1; overflow-y: auto; padding: 14px 16px; display: grid;
         grid-template-columns: 1fr 1fr; grid-template-rows: auto auto auto;
         gap: 14px; }
  @media (max-width: 640px) {
    main { grid-template-columns: 1fr; }
  }

  /* ── Cards ── */
  .card { background: var(--surface); border: 1px solid var(--border);
          border-radius: var(--radius); padding: 12px; }
  .card-title { font-size: 10px; letter-spacing: 3px; color: var(--text-dim);
                text-transform: uppercase; margin-bottom: 10px; }

  /* ── Cell grid ── */
  #cells-card { grid-column: 1 / -1; }
  #cell-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(120px, 1fr)); gap: 8px; }
  .cell {
    border-radius: var(--radius); border: 1px solid var(--border);
    padding: 6px 8px; height: var(--cell-h); display: flex; flex-direction: column;
    justify-content: space-between; position: relative; overflow: hidden;
    transition: border-color .3s;
  }
  .cell-fill {
    position: absolute; bottom: 0; left: 0; right: 0;
    transition: height .6s ease, background .6s ease;
    border-radius: 0 0 var(--radius) var(--radius);
    z-index: 0;
  }
  .cell-label, .cell-volt { position: relative; z-index: 1; }
  .cell-label { font-size: 10px; color: var(--text-dim); }
  .cell-volt  { font-size: 15px; font-weight: bold; letter-spacing: 1px; }
  .cell.ov { border-color: var(--danger) !important; border-width: 3px; }
  .cell.uv { border-color: var(--warn) !important; border-width: 3px;}
  .cell.bal { border-color: var(--accent) !important; border-width: 3px;}

  /* ── Stat rows ── */
  .stat-row { display: flex; justify-content: space-between; align-items: baseline;
              padding: 4px 0; border-bottom: 1px solid var(--border); }
  .stat-row:last-child { border-bottom: none; }
  .stat-key   { color: var(--text-dim); }
  .stat-value { font-size: 15px; font-weight: bold; }

  /* ── Temp bars ── */
  .temp-row { display: flex; align-items: center; gap: 8px; margin-bottom: 6px; }
  .temp-row:last-child { margin-bottom: 0; }
  .temp-label { width: 60px; color: var(--text-dim); flex-shrink: 0; }
  .temp-bar-track { flex: 1; height: 8px; background: var(--border); border-radius: 4px; overflow: hidden; }
  .temp-bar-fill  { height: 100%; border-radius: 4px; transition: width .5s ease, background .5s; }
  .temp-value { width: 54px; text-align: right; font-size: 12px; }

  /* ── Faults list ── */
  #fault-list { min-height: 40px; }
  .fault-item { display: flex; align-items: center; gap: 8px; padding: 3px 0; color: var(--danger); }
  .fault-item::before { content: '⚠'; font-size: 11px; }
  .no-faults { color: var(--ok); }

  /* ── State badge ── */
  #state-badge {
    display: inline-block; padding: 2px 10px; border-radius: 99px;
    font-size: 11px; letter-spacing: 2px; border: 1px solid var(--accent);
    color: var(--accent);
  }
  #state-badge.fault { border-color: var(--danger); color: var(--danger); }

  /* ── Footer ── */
  footer { padding: 6px 18px; font-size: 10px; color: var(--text-dim);
           text-align: right; border-top: 1px solid var(--border); flex-shrink: 0; }
</style>
</head>
<body>

<header>
  <h1>TronBMS</h1>
  <div style="display:flex;align-items:center;gap:8px;">
    <span id="status-dot"></span>
    <span id="conn-label">Connecting…</span>
    <span id="state-badge">—</span>
  </div>
</header>

<div id="fault-banner">⚠ FAULT ACTIVE</div>

<main>
  <!-- Cell voltages -->
  <div class="card" id="cells-card">
    <div class="card-title">Cell Voltages</div>
    <div id="cell-grid"></div>
  </div>

  <!-- Temperatures -->
  <div class="card">
    <div class="card-title">Temperatures</div>
    <div id="temp-rows"></div>
  </div>

  <!-- Current & pack -->
  <div class="card">
    <div class="card-title">Pack</div>
    <div class="stat-row">
      <span class="stat-key">Current</span>
      <span class="stat-value" id="val-current">—</span>
    </div>
    <div class="stat-row">
      <span class="stat-key">Pack Voltage</span>
      <span class="stat-value" id="val-pack-v">—</span>
    </div>
    <div class="stat-row">
      <span class="stat-key">Avg Cell</span>
      <span class="stat-value" id="val-avg-cell">—</span>
    </div>
    <div class="stat-row">
      <span class="stat-key">Min Cell</span>
      <span class="stat-value" id="val-min-cell">—</span>
    </div>
    <div class="stat-row">
      <span class="stat-key">Max Cell</span>
      <span class="stat-value" id="val-max-cell">—</span>
    </div>
    <div class="stat-row">
      <span class="stat-key">Delta</span>
      <span class="stat-value" id="val-delta">—</span>
    </div>
    <div class="stat-row">
      <span class="stat-key">Last Update</span>
      <span class="stat-value" id="val-time" style="font-size:11px;">—</span>
    </div>
    
  </div>

  <!-- Active faults -->
<div class="card">
  <div class="card-title">Faults</div>
  <div id="fault-list"><span class="no-faults">No active faults</span></div>
  <button id="clear-btn" onclick="clearFault()"
    style="margin-top:10px;width:100%;padding:6px;background:transparent;
           border:1px solid var(--danger);color:var(--danger);
           border-radius:4px;cursor:pointer;font-family:inherit;
           font-size:11px;letter-spacing:2px;">
    CLEAR FAULT
  </button>
</div>
</main>

<footer id="footer-ts">Waiting for data…</footer>

<script>
// ── Constants ──────────────────────────────────────────────────────────────
const CELL_UV   = 2.75;
const CELL_OV   = 4.10;
const CELL_NOM  = 3.60;   // nominal, used for fill colouring mid-point
const TEMP_MAX  = 70;     // °C hard cutoff from bms_config.h
const TEMP_WARN = 60;

const FAULT_NAMES = [
  null,
  'Overvoltage',
  'Undervoltage',
  'Overcurrent',
  'Overtemperature',
  'Balance Overtemp',
  'SPI LTC6811',
  'SPI ADS131',
  'Startup',
];

const STATE_NAMES = ['INIT','STARTUP','NORMAL','BALANCE','SLEEP','FAULT'];

// ── DOM refs ───────────────────────────────────────────────────────────────
const $dot       = document.getElementById('status-dot');
const $connLabel = document.getElementById('conn-label');
const $banner    = document.getElementById('fault-banner');
const $stateBdg  = document.getElementById('state-badge');
const $cellGrid  = document.getElementById('cell-grid');
const $tempRows  = document.getElementById('temp-rows');
const $faultList = document.getElementById('fault-list');

// ── Build cell tiles on load ───────────────────────────────────────────────
const NUM_CELLS = 20;  // must match firmware TOTAL_IC * CELLS_PER_IC
for (let i = 0; i < NUM_CELLS; i++) {
  const ic   = Math.floor(i / 10);
  const cell = (i % 10);
  const el   = document.createElement('div');
  el.className = 'cell';
  el.id = `cell-${i}`;
  el.innerHTML = `
    <div class="cell-fill" id="fill-${i}"></div>
    <span class="cell-label">IC${ic+1} C${String(cell+1).padStart(2,'0')}</span>
    <span class="cell-volt" id="volt-${i}">—.——</span>`;
  $cellGrid.appendChild(el);
}

// ── Build temp rows ────────────────────────────────────────────────────────
const NUM_TEMPS = 5;
for (let i = 0; i < NUM_TEMPS; i++) {
  const el = document.createElement('div');
  el.className = 'temp-row';
  el.innerHTML = `
    <span class="temp-label">Sensor ${i}</span>
    <div class="temp-bar-track"><div class="temp-bar-fill" id="tbar-${i}" style="width:0%"></div></div>
    <span class="temp-value" id="tval-${i}">—.— °C</span>`;
  $tempRows.appendChild(el);
}

// ── Update helpers ─────────────────────────────────────────────────────────
function lerp(a, b, t) { return a + (b - a) * Math.clamp(t, 0, 1); }
Math.clamp = (v, lo, hi) => Math.min(Math.max(v, lo), hi);

function voltToFill(v) {
  return Math.clamp((v - CELL_UV) / (CELL_OV - CELL_UV), 0, 1);
}
function fillColor(pct) {
  // green at high, amber at 30%, red below 10%
  if (pct > 0.30) return `hsl(${Math.round(lerp(40, 140, (pct - 0.30) / 0.70))}, 80%, 30%)`;
  return `hsl(${Math.round(lerp(0, 40, pct / 0.30))}, 80%, 25%)`;
}

function clearFault() {
  fetch('/clear_fault', { method: 'POST' })
    .then(r => { if (!r.ok) console.error('Clear failed'); });
}

function applyData(d) {
  // Cells
  const voltages = d.cells;  // flat array length 20
  let minV = Infinity, maxV = -Infinity, packV = 0;
  voltages.forEach((v, i) => {
    packV += v;
    if (v < minV) minV = v;
    if (v > maxV) maxV = v;
    const pct = voltToFill(v);
    const avgV = packV / voltages.length;
    document.getElementById('val-avg-cell').textContent = avgV.toFixed(4) + ' V';
    document.getElementById(`fill-${i}`).style.height    = `${(pct * 100).toFixed(1)}%`;
    document.getElementById(`fill-${i}`).style.background = fillColor(pct);
    document.getElementById(`volt-${i}`).textContent      = v.toFixed(4) + ' V';
    const tile = document.getElementById(`cell-${i}`);
    tile.classList.toggle('ov', v >= CELL_OV);
    tile.classList.toggle('uv', v > 0.1 && v <= CELL_UV);
    tile.classList.toggle('bal', d.balance && d.balance[i]);
    
  });

  document.getElementById('val-pack-v').textContent   = packV.toFixed(2) + ' V';
  document.getElementById('val-min-cell').textContent = minV.toFixed(4) + ' V';
  document.getElementById('val-max-cell').textContent = maxV.toFixed(4) + ' V';
  document.getElementById('val-delta').textContent    = ((maxV - minV) * 1000).toFixed(1) + ' mV';

  // Current
  const amps = d.current;
  const aEl  = document.getElementById('val-current');
  aEl.textContent   = (amps >= 0 ? '+' : '') + amps.toFixed(1) + ' A';
  aEl.style.color   = Math.abs(amps) > 350 ? 'var(--danger)' : Math.abs(amps) > 280 ? 'var(--warn)' : '';

  // Temps
  d.temps.forEach((t, i) => {
    const pct = Math.clamp(t / TEMP_MAX, 0, 1);
    const bar = document.getElementById(`tbar-${i}`);
    bar.style.width = `${(pct * 100).toFixed(1)}%`;
    bar.style.background = t >= TEMP_MAX ? 'var(--danger)' : t >= TEMP_WARN ? 'var(--warn)' : 'var(--ok)';
    document.getElementById(`tval-${i}`).textContent = t.toFixed(1) + ' °C';
  });

  // State
  const stateName = STATE_NAMES[d.state] || '?';
  $stateBdg.textContent = stateName;
  $stateBdg.className   = 'state-badge' + (d.state === 5 ? ' fault' : '');

  // Faults
  const faultBits = d.faults;
  $banner.classList.toggle('visible', faultBits !== 0);
  if (faultBits === 0) {
    $faultList.innerHTML = '<span class="no-faults">✓ No active faults</span>';
  } else {
    let html = '';
    for (let b = 1; b <= 8; b++) {
      if (faultBits & (1 << (b - 1))) {
        html += `<div class="fault-item">${FAULT_NAMES[b] || 'Fault ' + b}</div>`;
      }
    }
    $faultList.innerHTML = html;
  }

  // Timestamp
  const now = new Date();
  document.getElementById('val-time').textContent =
    `${String(now.getHours()).padStart(2,'0')}:${String(now.getMinutes()).padStart(2,'0')}:${String(now.getSeconds()).padStart(2,'0')}`;
  document.getElementById('footer-ts').textContent =
    `Last update: ${now.toLocaleTimeString()} · TronBMS v1`;
}

// ── WebSocket ──────────────────────────────────────────────────────────────
let ws, reconnectTimer;

function connect() {
  ws = new WebSocket(`ws://${location.hostname}/ws`);

  ws.onopen = () => {
    $dot.className       = 'connected';
    $connLabel.textContent = 'Connected';
    clearTimeout(reconnectTimer);
  };
  ws.onclose = ws.onerror = () => {
    $dot.className       = 'error';
    $connLabel.textContent = 'Disconnected — retrying…';
    reconnectTimer = setTimeout(connect, 2500);
  };
  ws.onmessage = (ev) => {
    try { applyData(JSON.parse(ev.data)); }
    catch(e) { console.error('Parse error', e); }
  };
}

connect();
</script>
</body>
</html>
)rawhtml";

// ============================================================================
// WebSocket event handler
// ============================================================================
static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[wifi] WS client #%u connected from %s\n",
                      client->id(), client->remoteIP().toString().c_str());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[wifi] WS client #%u disconnected\n", client->id());
    }
}

void wifi_push_telemetry(const measurement_data_t *meas,
                         BmsState state,
                         uint16_t fault_reg)
{
    if (!meas) return;
    if (xSemaphoreTake(s_snap_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(&s_meas_snap, meas, sizeof(measurement_data_t));
        s_state_snap = state;
        s_fault_snap = fault_reg;
        s_snap_dirty = true;
        xSemaphoreGive(s_snap_mutex);
    }
}

// builds json
void task_wifi_broadcast(void *pv)
{
    // Reuse a static JSON document to avoid repeated heap allocation
    static JsonDocument doc;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));   // 4 Hz

        if (!s_ws.count()) continue;      // no clients — skip JSON build

        if (!s_snap_dirty) continue;

        measurement_data_t snap;
        BmsState  state;
        uint16_t  faults;

        if (xSemaphoreTake(s_snap_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(&snap, &s_meas_snap, sizeof(snap));
            state  = s_state_snap;
            faults = s_fault_snap;
            s_snap_dirty = false;
            xSemaphoreGive(s_snap_mutex);
        } else {
            continue;
        }

        doc.clear();

        // Flat cell voltage array (20 elements)
        JsonArray cells = doc["cells"].to<JsonArray>();
        for (int ic = 0; ic < TOTAL_IC; ic++)
            for (int c = 0; c < CELLS_PER_IC; c++)
                cells.add(serialized(String(snap.cell_v[ic][c], 4)));

        // Balance flags
        JsonArray bal = doc["balance"].to<JsonArray>();
        for (int ic = 0; ic < TOTAL_IC; ic++)
            for (int c = 0; c < CELLS_PER_IC; c++)
                bal.add(snap.balance_cells[ic][c]);

        // Temperatures
        JsonArray temps = doc["temps"].to<JsonArray>();
        for (int i = 0; i < NUM_TEMP_SENSORS; i++)
            temps.add(serialized(String(snap.temps[i], 1)));

        doc["current"] = serialized(String(snap.current_a, 2));
        doc["state"]   = (int)state;
        doc["faults"]  = faults;

        // Serialize to a char buffer on the stack
        char buf[2048];
        size_t len = serializeJson(doc, buf, sizeof(buf));

        s_ws.textAll(buf, len);
    }
}

// ============================================================================
// Public API
// ============================================================================
void wifi_server_init(void)
{
    s_snap_mutex = xSemaphoreCreateMutex();

    // ── Start AP ──────────────────────────────────────────────────────────
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD[0] ? WIFI_PASSWORD : nullptr,
                WIFI_CHANNEL, 0, WIFI_MAX_CONN);
    Serial.printf("[wifi] AP started: SSID=%s  IP=%s\n",
                  WIFI_SSID, WiFi.softAPIP().toString().c_str());

    // ── WebSocket ─────────────────────────────────────────────────────────
    s_ws.onEvent(onWsEvent);
    s_server.addHandler(&s_ws);

    // ── HTTP: serve the GUI ───────────────────────────────────────────────
    s_server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send_P(200, "text/html", INDEX_HTML);
    });

    s_server.on("/clear_fault", HTTP_POST, [](AsyncWebServerRequest *req) {
    fault_clear(static_cast<uint16_t>(FaultCode::OVERVOLTAGE));
    fault_clear(static_cast<uint16_t>(FaultCode::UNDERVOLTAGE));
    fault_clear(static_cast<uint16_t>(FaultCode::OVERCURRENT));
    fault_clear(static_cast<uint16_t>(FaultCode::OVERTEMP));
    fault_clear(static_cast<uint16_t>(FaultCode::BAL_OVERTEMP));
    fault_clear(static_cast<uint16_t>(FaultCode::SPI_LTC));
    fault_clear(static_cast<uint16_t>(FaultCode::SPI_ADS));
    fault_clear(static_cast<uint16_t>(FaultCode::STARTUP));
    fsm_set_state(g_fsm, BmsState::INIT);
    req->send(200, "text/plain", "OK");
});

    // ── HTTP: JSON REST endpoint (optional polling fallback) ──────────────
    s_server.on("/data", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        if (xSemaphoreTake(s_snap_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            JsonArray cells = doc["cells"].to<JsonArray>();
            for (int ic = 0; ic < TOTAL_IC; ic++)
                for (int c = 0; c < CELLS_PER_IC; c++)
                    cells.add(s_meas_snap.cell_v[ic][c]);
            JsonArray temps = doc["temps"].to<JsonArray>();
            for (int i = 0; i < NUM_TEMP_SENSORS; i++)
                temps.add(s_meas_snap.temps[i]);
            JsonArray bal = doc["balance"].to<JsonArray>();
            for (int ic = 0; ic < TOTAL_IC; ic++)
                for (int c = 0; c < CELLS_PER_IC; c++)
                    bal.add(s_meas_snap.balance_cells[ic][c]);
            doc["current"] = s_meas_snap.current_a;
            doc["state"]   = (int)s_state_snap;
            doc["faults"]  = s_fault_snap;
            xSemaphoreGive(s_snap_mutex);
        }
        char buf[2048];
        serializeJson(doc, buf, sizeof(buf));
        req->send(200, "application/json", buf);
    });

    s_server.onNotFound([](AsyncWebServerRequest *req) {
        req->redirect("/");
    });

    s_server.begin();
    Serial.println("[wifi] HTTP server started on port 80");
}

