/*
# SPDX-FileCopyrightText: 2024 iowlabs <contacto@iowlabs.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later.txt
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SHT31.h>
#include <ArduinoJson.h>
#include <ContinuousStepper.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Heater/H-bridge pins
#define ENA 25
#define ENB 26
#define IN1 33
#define IN2 32

// Stepper pins
#define PIN_DIR 27
#define PIN_STEP 14

// PWM channels
#define CH_H 5
#define CH_C 4

// Safe defaults and limits
constexpr float DEFAULT_TEMP_SP_C = 37.0f;
constexpr long DEFAULT_RPM = 60;
constexpr bool DEFAULT_RUN_ENABLED = false;

constexpr float TEMP_MIN_C = 15.0f;
constexpr float TEMP_MAX_C = 60.0f;
constexpr long RPM_MIN = 30;
constexpr long RPM_MAX = 90;

constexpr float STEPS_PER_REV = 1600.0f;
constexpr float RPM_TO_STEPS_PER_SEC = STEPS_PER_REV / 60.0f;

constexpr uint32_t CONTROL_INTERVAL_MS = 500;
constexpr uint32_t SSE_INTERVAL_MS = 500;

// Access point credentials (WPA2)
constexpr char AP_SSID[] = "Incubadora-ESP32";
constexpr char AP_PASSWORD[] = "incubadora32";

SemaphoreHandle_t stateMutex;

Adafruit_SHT31 sht31 = Adafruit_SHT31();
ContinuousStepper<StepperDriver> stepper;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// PID setup
static double setpointPid = DEFAULT_TEMP_SP_C;
static double inputPid = 0.0;
static double outputPid = 0.0;
static double kp = 15.0;
static double ki = 2.0;
static double kd = 10.0;
PID temperaturePid(&inputPid, &outputPid, &setpointPid, kp, ki, kd, DIRECT);

// Web server + SSE
AsyncWebServer server(80);
AsyncEventSource events("/api/stream");

struct IncubatorState
{
  float temp_c;
  float temp_sp_c;
  long rpm;
  int pwm;
  bool run_enabled;
  uint32_t uptime_ms;
  char last_error[96];
};

IncubatorState gState;

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Incubadora Control</title>
  <style>
    :root {
      --bg: radial-gradient(circle at 20% 20%, #14213d 0%, #0d1b2a 45%, #08121f 100%);
      --card: rgba(255, 255, 255, 0.08);
      --card-border: rgba(255, 255, 255, 0.18);
      --text: #f1f5f9;
      --muted: #cbd5e1;
      --ok: #34d399;
      --warn: #f59e0b;
      --bad: #f87171;
      --accent: #38bdf8;
      --accent-2: #22d3ee;
    }

    * { box-sizing: border-box; }

    body {
      margin: 0;
      min-height: 100vh;
      font-family: "Trebuchet MS", "Segoe UI", sans-serif;
      color: var(--text);
      background: var(--bg);
      display: grid;
      place-items: center;
      padding: 20px;
    }

    .panel {
      width: min(960px, 100%);
      border: 1px solid var(--card-border);
      background: var(--card);
      backdrop-filter: blur(8px);
      border-radius: 20px;
      padding: 20px;
      box-shadow: 0 24px 80px rgba(0, 0, 0, 0.35);
      animation: rise 450ms ease-out;
    }

    @keyframes rise {
      from { opacity: 0; transform: translateY(8px); }
      to { opacity: 1; transform: translateY(0); }
    }

    h1 {
      margin: 0 0 6px;
      font-size: clamp(1.2rem, 2.4vw, 1.8rem);
      letter-spacing: 0.04em;
      text-transform: uppercase;
    }

    .sub {
      margin: 0 0 18px;
      color: var(--muted);
      font-size: 0.95rem;
    }

    .grid {
      display: grid;
      gap: 12px;
      grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
      margin-bottom: 18px;
    }

    .card {
      background: rgba(2, 6, 23, 0.35);
      border: 1px solid rgba(148, 163, 184, 0.25);
      border-radius: 14px;
      padding: 12px;
    }

    .label {
      color: var(--muted);
      font-size: 0.78rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }

    .value {
      font-size: 1.4rem;
      margin-top: 4px;
      font-weight: 700;
    }

    .controls {
      display: grid;
      gap: 12px;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
    }

    .input-group {
      display: grid;
      gap: 6px;
    }

    label {
      font-size: 0.85rem;
      color: var(--muted);
    }

    input {
      width: 100%;
      border-radius: 10px;
      border: 1px solid rgba(148, 163, 184, 0.4);
      background: rgba(2, 6, 23, 0.55);
      color: var(--text);
      font-size: 1rem;
      padding: 10px 12px;
      outline: none;
    }

    input:focus {
      border-color: var(--accent);
      box-shadow: 0 0 0 2px rgba(56, 189, 248, 0.25);
    }

    .actions {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-top: 2px;
    }

    button {
      border: none;
      border-radius: 10px;
      padding: 10px 14px;
      color: #042f2e;
      font-weight: 700;
      cursor: pointer;
      transition: transform 120ms ease, filter 120ms ease;
      background: linear-gradient(135deg, var(--accent), var(--accent-2));
    }

    button:hover { transform: translateY(-1px); filter: brightness(1.06); }

    .btn-stop {
      color: #450a0a;
      background: linear-gradient(135deg, #fca5a5, #f87171);
    }

    .btn-export {
      color: #082f49;
      background: linear-gradient(135deg, #93c5fd, #60a5fa);
    }

    .btn-clear {
      color: #3f2a00;
      background: linear-gradient(135deg, #fde68a, #fbbf24);
    }

    .status {
      margin-top: 14px;
      font-size: 0.88rem;
      color: var(--muted);
      min-height: 1.2rem;
    }

    .pill {
      display: inline-block;
      border-radius: 999px;
      padding: 3px 10px;
      font-size: 0.75rem;
      font-weight: 700;
      letter-spacing: 0.05em;
      margin-left: 6px;
    }

    .ok { background: rgba(52, 211, 153, 0.2); color: #86efac; }
    .warn { background: rgba(245, 158, 11, 0.2); color: #fcd34d; }
    .bad { background: rgba(248, 113, 113, 0.2); color: #fecaca; }

    .chart-card {
      margin-top: 14px;
    }

    .chart-wrap {
      margin-top: 10px;
      width: 100%;
      height: 230px;
      border-radius: 12px;
      border: 1px solid rgba(148, 163, 184, 0.3);
      background: rgba(2, 6, 23, 0.55);
      overflow: hidden;
    }

    #graphCanvas {
      width: 100%;
      height: 100%;
      display: block;
    }

    .chart-legend {
      margin-top: 10px;
      display: flex;
      gap: 16px;
      flex-wrap: wrap;
      color: var(--muted);
      font-size: 0.82rem;
    }

    .chart-key {
      display: inline-flex;
      align-items: center;
      gap: 7px;
    }

    .chart-dot {
      width: 10px;
      height: 10px;
      border-radius: 999px;
      display: inline-block;
    }

    .dot-temp { background: #22d3ee; }
    .dot-pwm { background: #f59e0b; }
  </style>
</head>
<body>
  <main class="panel">
    <h1>Incubadora + Shaker</h1>
    <p class="sub">
      AP control interface
      <span id="connPill" class="pill warn">CONNECTING</span>
    </p>

    <section class="grid">
      <article class="card"><div class="label">Temperature (C)</div><div id="tempVal" class="value">--</div></article>
      <article class="card"><div class="label">Setpoint (C)</div><div id="spVal" class="value">--</div></article>
      <article class="card"><div class="label">PWM</div><div id="pwmVal" class="value">--</div></article>
      <article class="card"><div class="label">RPM</div><div id="rpmVal" class="value">--</div></article>
      <article class="card"><div class="label">Run State</div><div id="runVal" class="value">--</div></article>
      <article class="card"><div class="label">Uptime (s)</div><div id="upVal" class="value">--</div></article>
    </section>

    <section class="controls">
      <div class="input-group">
        <label for="tempInput">Temperature Setpoint (15.0 to 60.0 C)</label>
        <input id="tempInput" type="number" min="15" max="60" step="0.1" value="37.0" />
      </div>
      <div class="input-group">
        <label for="rpmInput">Shaker RPM (30 to 90)</label>
        <input id="rpmInput" type="number" min="30" max="90" step="1" value="60" />
      </div>
    </section>

    <div class="actions">
      <button id="applyBtn">Apply</button>
      <button id="startBtn">Start</button>
      <button id="stopBtn" class="btn-stop">Stop</button>
      <button id="clearBtn" class="btn-clear">Clear History</button>
      <button id="downloadBtn" class="btn-export">Download CSV</button>
    </div>

    <div id="statusMsg" class="status"></div>

    <section class="card chart-card">
      <div class="label">Live Temperature and PWM</div>
      <div class="chart-wrap">
        <canvas id="graphCanvas"></canvas>
      </div>
      <div class="chart-legend">
        <span class="chart-key"><i class="chart-dot dot-temp"></i>Temperature (C)</span>
        <span class="chart-key"><i class="chart-dot dot-pwm"></i>PWM (0-255)</span>
      </div>
    </section>
  </main>

  <script>
    const TEMP_MIN = 15.0;
    const TEMP_MAX = 60.0;
    const RPM_MIN = 30;
    const RPM_MAX = 90;
    const PWM_MIN = 0;
    const PWM_MAX = 255;
    const CSV_MAX_ROWS = 5000;
    const HISTORY_STORAGE_KEY = 'incubadora_history_v1';
    const HISTORY_SAVE_DEBOUNCE_MS = 300;
    const UPTIME_RESET_TOLERANCE_MS = 2000;

    const els = {
      tempVal: document.getElementById('tempVal'),
      spVal: document.getElementById('spVal'),
      pwmVal: document.getElementById('pwmVal'),
      rpmVal: document.getElementById('rpmVal'),
      runVal: document.getElementById('runVal'),
      upVal: document.getElementById('upVal'),
      tempInput: document.getElementById('tempInput'),
      rpmInput: document.getElementById('rpmInput'),
      applyBtn: document.getElementById('applyBtn'),
      startBtn: document.getElementById('startBtn'),
      stopBtn: document.getElementById('stopBtn'),
      clearBtn: document.getElementById('clearBtn'),
      downloadBtn: document.getElementById('downloadBtn'),
      statusMsg: document.getElementById('statusMsg'),
      connPill: document.getElementById('connPill'),
      graphCanvas: document.getElementById('graphCanvas')
    };

    const graphHistory = {
      timeS: [],
      temp: [],
      pwm: []
    };
    const csvHistory = [];
    let lastGraphUptime = -1;
    let historySaveTimer = null;

    function setConnState(kind, label) {
      els.connPill.className = 'pill ' + kind;
      els.connPill.textContent = label;
    }

    function setStatus(text, isError = false) {
      els.statusMsg.textContent = text || '';
      els.statusMsg.style.color = isError ? '#fecaca' : '#cbd5e1';
    }

    function fmt1(v) {
      return Number.isFinite(v) ? v.toFixed(1) : '--';
    }

    function clamp(v, min, max) {
      return Math.max(min, Math.min(max, v));
    }

    function addGraphPoint(uptimeMs, temp, pwm) {
      if (!Number.isFinite(uptimeMs) || !Number.isFinite(temp) || !Number.isFinite(pwm)) {
        return;
      }

      graphHistory.timeS.push(uptimeMs / 1000);
      graphHistory.temp.push(temp);
      graphHistory.pwm.push(pwm);

      drawGraph();
      scheduleHistorySave();
    }

    function formatElapsed(seconds) {
      const s = Math.max(0, Math.round(seconds));
      const h = Math.floor(s / 3600);
      const m = Math.floor((s % 3600) / 60);
      const sec = s % 60;
      if (h > 0) {
        return h + 'h ' + String(m).padStart(2, '0') + 'm';
      }
      if (m > 0) {
        return m + 'm ' + String(sec).padStart(2, '0') + 's';
      }
      return sec + 's';
    }

    function addCsvRow(sample) {
      csvHistory.push(sample);
      if (csvHistory.length > CSV_MAX_ROWS) {
        csvHistory.shift();
      }
      scheduleHistorySave();
    }

    function saveHistoryNow() {
      const history = {
        v: 1,
        lastGraphUptime,
        graphHistory,
        csvHistory
      };
      try {
        localStorage.setItem(HISTORY_STORAGE_KEY, JSON.stringify(history));
      } catch (_) {
        // Ignore storage quota/private-mode failures.
      }
    }

    function scheduleHistorySave() {
      if (historySaveTimer !== null) {
        return;
      }
      historySaveTimer = setTimeout(() => {
        historySaveTimer = null;
        saveHistoryNow();
      }, HISTORY_SAVE_DEBOUNCE_MS);
    }

    function clearHistory(persist = true) {
      graphHistory.timeS.length = 0;
      graphHistory.temp.length = 0;
      graphHistory.pwm.length = 0;
      csvHistory.length = 0;
      lastGraphUptime = -1;
      drawGraph();
      if (persist) {
        saveHistoryNow();
      }
    }

    function restoreHistory() {
      let raw = '';
      try {
        raw = localStorage.getItem(HISTORY_STORAGE_KEY) || '';
      } catch (_) {
        return;
      }
      if (!raw) {
        return;
      }

      try {
        const parsed = JSON.parse(raw);
        if (!parsed || typeof parsed !== 'object') {
          return;
        }

        const g = parsed.graphHistory;
        const c = parsed.csvHistory;
        if (!g || !Array.isArray(g.timeS) || !Array.isArray(g.temp) || !Array.isArray(g.pwm) || !Array.isArray(c)) {
          return;
        }

        const n = Math.min(g.timeS.length, g.temp.length, g.pwm.length);
        for (let i = 0; i < n; i += 1) {
          const t = Number(g.timeS[i]);
          const temp = Number(g.temp[i]);
          const pwm = Number(g.pwm[i]);
          if (!Number.isFinite(t) || !Number.isFinite(temp) || !Number.isFinite(pwm)) {
            continue;
          }
          graphHistory.timeS.push(t);
          graphHistory.temp.push(temp);
          graphHistory.pwm.push(pwm);
        }

        const rowsToCopy = c.slice(-CSV_MAX_ROWS);
        for (const row of rowsToCopy) {
          if (!row || typeof row !== 'object') {
            continue;
          }
          csvHistory.push(row);
        }

        const persistedUptime = Number(parsed.lastGraphUptime);
        if (Number.isFinite(persistedUptime)) {
          lastGraphUptime = persistedUptime;
        } else if (graphHistory.timeS.length > 0) {
          lastGraphUptime = Math.round(graphHistory.timeS[graphHistory.timeS.length - 1] * 1000);
        }
      } catch (_) {
        // Corrupt storage should not break UI.
      }
    }

    function csvEscape(value) {
      const text = String(value ?? '');
      if (text.includes(',') || text.includes('"') || text.includes('\n')) {
        return '"' + text.replaceAll('"', '""') + '"';
      }
      return text;
    }

    function buildCsv() {
      const header = [
        'captured_at_iso',
        'uptime_ms',
        'temperature_c',
        'pwm',
        'rpm',
        'temp_sp_c',
        'run_enabled'
      ];
      const lines = [header.join(',')];

      for (const row of csvHistory) {
        lines.push([
          csvEscape(row.captured_at_iso),
          csvEscape(row.uptime_ms),
          csvEscape(row.temperature_c),
          csvEscape(row.pwm),
          csvEscape(row.rpm),
          csvEscape(row.temp_sp_c),
          csvEscape(row.run_enabled)
        ].join(','));
      }

      return lines.join('\n');
    }

    function downloadCsv() {
      if (csvHistory.length === 0) {
        setStatus('No live samples yet. Wait for telemetry before exporting.', true);
        return;
      }

      const csvText = buildCsv();
      const stamp = new Date().toISOString().replace(/[:]/g, '-').replace(/\..+/, '');
      const fileName = 'incubadora_temp_pwm_' + stamp + '.csv';
      const blob = new Blob([csvText], { type: 'text/csv;charset=utf-8;' });
      const url = URL.createObjectURL(blob);

      const link = document.createElement('a');
      link.href = url;
      link.download = fileName;
      document.body.appendChild(link);
      link.click();
      link.remove();
      URL.revokeObjectURL(url);

      setStatus('CSV downloaded (' + csvHistory.length + ' samples).');
    }

    function drawGraph() {
      const canvas = els.graphCanvas;
      if (!canvas) {
        return;
      }

      const ctx = canvas.getContext('2d');
      if (!ctx) {
        return;
      }

      const width = canvas.clientWidth || 300;
      const height = canvas.clientHeight || 220;
      const dpr = window.devicePixelRatio || 1;
      const targetW = Math.max(1, Math.floor(width * dpr));
      const targetH = Math.max(1, Math.floor(height * dpr));

      if (canvas.width !== targetW || canvas.height !== targetH) {
        canvas.width = targetW;
        canvas.height = targetH;
      }

      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, width, height);

      const padLeft = 38;
      const padRight = 38;
      const padTop = 12;
      const padBottom = 30;
      const plotLeft = padLeft;
      const plotTop = padTop;
      const plotWidth = Math.max(10, width - padLeft - padRight);
      const plotHeight = Math.max(10, height - padTop - padBottom);
      const plotBottom = plotTop + plotHeight;
      const plotRight = plotLeft + plotWidth;

      ctx.fillStyle = 'rgba(2, 6, 23, 0.65)';
      ctx.fillRect(plotLeft, plotTop, plotWidth, plotHeight);

      ctx.strokeStyle = 'rgba(148, 163, 184, 0.22)';
      ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i += 1) {
        const y = plotTop + (plotHeight * i) / 4;
        ctx.beginPath();
        ctx.moveTo(plotLeft, y);
        ctx.lineTo(plotRight, y);
        ctx.stroke();
      }

      ctx.fillStyle = 'rgba(203, 213, 225, 0.9)';
      ctx.font = '11px Trebuchet MS';
      ctx.textAlign = 'left';
      ctx.fillText(TEMP_MAX.toFixed(0) + 'C', 4, plotTop + 4);
      ctx.fillText(TEMP_MIN.toFixed(0) + 'C', 4, plotBottom + 4);
      ctx.textAlign = 'right';
      ctx.fillText(String(PWM_MAX), width - 4, plotTop + 4);
      ctx.fillText(String(PWM_MIN), width - 4, plotBottom + 4);
      ctx.textAlign = 'left';

      const points = graphHistory.temp.length;
      if (points < 1) {
        return;
      }

      const startTimeS = graphHistory.timeS[0];
      const endTimeS = graphHistory.timeS[points - 1];
      const spanS = Math.max(1, endTimeS - startTimeS);

      ctx.strokeStyle = 'rgba(148, 163, 184, 0.20)';
      ctx.lineWidth = 1;
      ctx.textAlign = 'center';
      for (let i = 0; i <= 4; i += 1) {
        const x = plotLeft + (plotWidth * i) / 4;
        ctx.beginPath();
        ctx.moveTo(x, plotTop);
        ctx.lineTo(x, plotBottom);
        ctx.stroke();

        const tickElapsed = (spanS * i) / 4;
        ctx.fillStyle = 'rgba(203, 213, 225, 0.9)';
        ctx.fillText(formatElapsed(tickElapsed), x, height - 6);
      }
      ctx.textAlign = 'left';

      const xForTime = (timeS) => plotLeft + (plotWidth * (timeS - startTimeS)) / spanS;
      const yTemp = (val) => {
        const norm = (clamp(val, TEMP_MIN, TEMP_MAX) - TEMP_MIN) / (TEMP_MAX - TEMP_MIN);
        return plotBottom - (norm * plotHeight);
      };
      const yPwm = (val) => {
        const norm = (clamp(val, PWM_MIN, PWM_MAX) - PWM_MIN) / (PWM_MAX - PWM_MIN);
        return plotBottom - (norm * plotHeight);
      };

      ctx.strokeStyle = '#22d3ee';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(xForTime(graphHistory.timeS[0]), yTemp(graphHistory.temp[0]));
      for (let i = 1; i < points; i += 1) {
        ctx.lineTo(xForTime(graphHistory.timeS[i]), yTemp(graphHistory.temp[i]));
      }
      ctx.stroke();

      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(xForTime(graphHistory.timeS[0]), yPwm(graphHistory.pwm[0]));
      for (let i = 1; i < points; i += 1) {
        ctx.lineTo(xForTime(graphHistory.timeS[i]), yPwm(graphHistory.pwm[i]));
      }
      ctx.stroke();
    }

    function renderState(s) {
      const tempNum = Number(s.temp_c);
      const setpointNum = Number(s.temp_sp_c);
      const pwmNum = Number(s.pwm);
      const rpmNum = Number(s.rpm);
      const uptimeNum = Number(s.uptime_ms);

      els.tempVal.textContent = fmt1(tempNum);
      els.spVal.textContent = fmt1(Number(s.temp_sp_c));
      els.pwmVal.textContent = Number.isFinite(pwmNum) ? pwmNum : '--';
      els.rpmVal.textContent = Number.isFinite(rpmNum) ? rpmNum : '--';
      els.runVal.textContent = s.run_enabled ? 'RUNNING' : 'STOPPED';
      els.upVal.textContent = Number.isFinite(uptimeNum) ? Math.floor(uptimeNum / 1000) : '--';

      if (Number.isFinite(setpointNum)) {
        if (document.activeElement !== els.tempInput) {
          els.tempInput.value = setpointNum.toFixed(1);
        }
      }
      if (Number.isFinite(rpmNum)) {
        if (document.activeElement !== els.rpmInput) {
          els.rpmInput.value = String(Math.round(rpmNum));
        }
      }

      if (Number.isFinite(tempNum) && Number.isFinite(pwmNum) && Number.isFinite(uptimeNum) && uptimeNum !== lastGraphUptime) {
        if (lastGraphUptime >= 0 && (uptimeNum + UPTIME_RESET_TOLERANCE_MS) < lastGraphUptime) {
          clearHistory(false);
          setStatus('Device uptime reset detected. History restarted.');
        }
        addGraphPoint(uptimeNum, tempNum, pwmNum);
        addCsvRow({
          captured_at_iso: new Date().toISOString(),
          uptime_ms: Math.round(uptimeNum),
          temperature_c: tempNum.toFixed(3),
          pwm: Math.round(pwmNum),
          rpm: Number.isFinite(rpmNum) ? Math.round(rpmNum) : '',
          temp_sp_c: Number.isFinite(setpointNum) ? setpointNum.toFixed(3) : '',
          run_enabled: Boolean(s.run_enabled)
        });
        lastGraphUptime = uptimeNum;
      }
    }

    async function jsonFetch(url, options = {}) {
      const res = await fetch(url, {
        headers: { 'Content-Type': 'application/json' },
        ...options
      });
      const text = await res.text();
      let payload = {};
      try { payload = text ? JSON.parse(text) : {}; } catch (_) {}
      if (!res.ok) {
        const msg = payload.error || ('HTTP ' + res.status);
        throw new Error(msg);
      }
      return payload;
    }

    function parseInputs() {
      const temp = Number(els.tempInput.value);
      const rpm = Number(els.rpmInput.value);

      if (!Number.isFinite(temp) || !Number.isFinite(rpm)) {
        throw new Error('Setpoint and RPM must be numeric.');
      }

      return {
        temp_sp_c: Math.max(TEMP_MIN, Math.min(TEMP_MAX, temp)),
        rpm: Math.max(RPM_MIN, Math.min(RPM_MAX, Math.round(rpm)))
      };
    }

    async function loadInitialState() {
      try {
        const s = await jsonFetch('/api/state');
        renderState(s);
      } catch (err) {
        setStatus('Failed to load initial state: ' + err.message, true);
      }
    }

    async function applySetpoints() {
      try {
        const body = parseInputs();
        const res = await jsonFetch('/api/setpoints', {
          method: 'POST',
          body: JSON.stringify(body)
        });
        renderState(res);

        const tempClamp = res.validation && res.validation.temp_sp_c_clamped;
        const rpmClamp = res.validation && res.validation.rpm_clamped;
        if (tempClamp || rpmClamp) {
          setStatus('Values were clamped by firmware safety limits.');
        } else if (!res.run_enabled) {
          setStatus('Setpoints updated. Press Start to run shaker.');
        } else {
          setStatus('Setpoints updated.');
        }
      } catch (err) {
        setStatus(err.message, true);
      }
    }

    async function setRun(enabled) {
      try {
        const res = await jsonFetch('/api/run', {
          method: 'POST',
          body: JSON.stringify({ run_enabled: enabled })
        });
        renderState(res);
        setStatus(enabled ? 'Outputs started.' : 'Outputs stopped.');
      } catch (err) {
        setStatus(err.message, true);
      }
    }

    function connectStream() {
      const es = new EventSource('/api/stream');

      es.onopen = () => {
        setConnState('ok', 'LIVE');
      };

      es.onerror = () => {
        setConnState('bad', 'DISCONNECTED');
      };

      es.addEventListener('state', (ev) => {
        try {
          const payload = JSON.parse(ev.data);
          renderState(payload);
        } catch (_) {
          setStatus('Malformed live telemetry.', true);
        }
      });
    }

    els.applyBtn.addEventListener('click', applySetpoints);
    els.tempInput.addEventListener('keydown', (ev) => {
      if (ev.key === 'Enter') {
        applySetpoints();
      }
    });
    els.rpmInput.addEventListener('keydown', (ev) => {
      if (ev.key === 'Enter') {
        applySetpoints();
      }
    });
    els.startBtn.addEventListener('click', () => setRun(true));
    els.stopBtn.addEventListener('click', () => setRun(false));
    els.clearBtn.addEventListener('click', () => {
      clearHistory();
      setStatus('History cleared.');
    });
    els.downloadBtn.addEventListener('click', downloadCsv);
    window.addEventListener('resize', drawGraph);
    window.addEventListener('beforeunload', saveHistoryNow);

    restoreHistory();
    drawGraph();
    loadInitialState();
    connectStream();
  </script>
</body>
</html>
)HTML";

float clampTemperature(float value, bool *wasClamped)
{
  float bounded = value;
  if (bounded < TEMP_MIN_C)
  {
    bounded = TEMP_MIN_C;
  }
  if (bounded > TEMP_MAX_C)
  {
    bounded = TEMP_MAX_C;
  }
  if (wasClamped != nullptr)
  {
    *wasClamped = (bounded != value);
  }
  return bounded;
}

long clampRpm(long value, bool *wasClamped)
{
  long bounded = value;
  if (bounded < RPM_MIN)
  {
    bounded = RPM_MIN;
  }
  if (bounded > RPM_MAX)
  {
    bounded = RPM_MAX;
  }
  if (wasClamped != nullptr)
  {
    *wasClamped = (bounded != value);
  }
  return bounded;
}

void setLastError(const char *msg)
{
  if (stateMutex == nullptr)
  {
    return;
  }

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
  {
    strncpy(gState.last_error, msg, sizeof(gState.last_error) - 1);
    gState.last_error[sizeof(gState.last_error) - 1] = '\0';
    xSemaphoreGive(stateMutex);
  }
}

IncubatorState snapshotState()
{
  IncubatorState snapshot{};
  if (stateMutex == nullptr)
  {
    return snapshot;
  }

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
  {
    snapshot = gState;
    xSemaphoreGive(stateMutex);
  }
  return snapshot;
}

void updateStateTelemetry(float temp, int pwm, uint32_t uptime)
{
  if (stateMutex == nullptr)
  {
    return;
  }

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
  {
    gState.temp_c = temp;
    gState.pwm = pwm;
    gState.uptime_ms = uptime;
    xSemaphoreGive(stateMutex);
  }
}

void updateStateRunEnabled(bool runEnabled)
{
  if (stateMutex == nullptr)
  {
    return;
  }

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
  {
    gState.run_enabled = runEnabled;
    if (!runEnabled)
    {
      gState.pwm = 0;
    }
    xSemaphoreGive(stateMutex);
  }
}

void updateStateSetpoints(bool hasTemp, float tempValue, bool hasRpm, long rpmValue, bool *tempClamped, bool *rpmClamped)
{
  bool localTempClamped = false;
  bool localRpmClamped = false;

  if (stateMutex == nullptr)
  {
    if (tempClamped != nullptr)
    {
      *tempClamped = false;
    }
    if (rpmClamped != nullptr)
    {
      *rpmClamped = false;
    }
    return;
  }

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
  {
    if (hasTemp)
    {
      gState.temp_sp_c = clampTemperature(tempValue, &localTempClamped);
    }
    if (hasRpm)
    {
      gState.rpm = clampRpm(rpmValue, &localRpmClamped);
    }
    xSemaphoreGive(stateMutex);
  }

  if (tempClamped != nullptr)
  {
    *tempClamped = localTempClamped;
  }
  if (rpmClamped != nullptr)
  {
    *rpmClamped = localRpmClamped;
  }
}

String buildStateJson()
{
  IncubatorState state = snapshotState();
  StaticJsonDocument<384> doc;
  doc["temp_c"] = state.temp_c;
  doc["temp_sp_c"] = state.temp_sp_c;
  doc["rpm"] = state.rpm;
  doc["pwm"] = state.pwm;
  doc["run_enabled"] = state.run_enabled;
  doc["uptime_ms"] = state.uptime_ms;
  doc["last_error"] = state.last_error;

  String body;
  serializeJson(doc, body);
  return body;
}

String buildLegacySerialTelemetry()
{
  IncubatorState state = snapshotState();
  StaticJsonDocument<256> doc;
  doc["time"] = state.uptime_ms;
  doc["temp"] = state.temp_c;
  doc["temp_sp"] = state.temp_sp_c;
  doc["pwm"] = state.pwm;
  doc["rpm"] = state.rpm;
  doc["run_enabled"] = state.run_enabled;

  String body;
  serializeJson(doc, body);
  return body;
}

void sendJsonError(AsyncWebServerRequest *request, const char *errorMsg, const char *field)
{
  StaticJsonDocument<192> doc;
  doc["error"] = errorMsg;
  doc["field"] = field;

  String body;
  serializeJson(doc, body);
  request->send(400, "application/json", body);
}

void handleRunBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
  if (!request->contentType().startsWith("application/json"))
  {
    sendJsonError(request, "Content-Type must be application/json", "content_type");
    return;
  }

  if (index == 0)
  {
    request->_tempObject = new String();
    String *body = reinterpret_cast<String *>(request->_tempObject);
    body->reserve(total + 1);
  }

  String *body = reinterpret_cast<String *>(request->_tempObject);
  if (body == nullptr)
  {
    sendJsonError(request, "Request body buffer allocation failed", "body");
    return;
  }

  body->concat(reinterpret_cast<const char *>(data), len);

  if ((index + len) < total)
  {
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, *body);
  delete body;
  request->_tempObject = nullptr;

  if (err)
  {
    sendJsonError(request, "Invalid JSON body", "body");
    return;
  }

  if (!doc.containsKey("run_enabled") || !doc["run_enabled"].is<bool>())
  {
    sendJsonError(request, "run_enabled must be boolean", "run_enabled");
    return;
  }

  bool runEnabled = doc["run_enabled"].as<bool>();
  Serial.printf("[API] /api/run run_enabled=%s\n", runEnabled ? "true" : "false");
  updateStateRunEnabled(runEnabled);

  request->send(200, "application/json", buildStateJson());
}

void handleSetpointsBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
  if (!request->contentType().startsWith("application/json"))
  {
    sendJsonError(request, "Content-Type must be application/json", "content_type");
    return;
  }

  if (index == 0)
  {
    request->_tempObject = new String();
    String *body = reinterpret_cast<String *>(request->_tempObject);
    body->reserve(total + 1);
  }

  String *body = reinterpret_cast<String *>(request->_tempObject);
  if (body == nullptr)
  {
    sendJsonError(request, "Request body buffer allocation failed", "body");
    return;
  }

  body->concat(reinterpret_cast<const char *>(data), len);

  if ((index + len) < total)
  {
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, *body);
  delete body;
  request->_tempObject = nullptr;

  if (err)
  {
    sendJsonError(request, "Invalid JSON body", "body");
    return;
  }

  bool hasTemp = doc.containsKey("temp_sp_c");
  bool hasRpm = doc.containsKey("rpm");

  if (!hasTemp && !hasRpm)
  {
    sendJsonError(request, "At least one of temp_sp_c or rpm is required", "body");
    return;
  }

  float tempValue = 0.0f;
  long rpmValue = 0;

  if (hasTemp)
  {
    if (!doc["temp_sp_c"].is<float>() && !doc["temp_sp_c"].is<int>() && !doc["temp_sp_c"].is<double>())
    {
      sendJsonError(request, "temp_sp_c must be numeric", "temp_sp_c");
      return;
    }
    tempValue = doc["temp_sp_c"].as<float>();
  }

  if (hasRpm)
  {
    if (!doc["rpm"].is<long>() && !doc["rpm"].is<int>() && !doc["rpm"].is<float>() && !doc["rpm"].is<double>())
    {
      sendJsonError(request, "rpm must be numeric", "rpm");
      return;
    }
    rpmValue = lround(doc["rpm"].as<float>());
  }

  bool tempClamped = false;
  bool rpmClamped = false;
  updateStateSetpoints(hasTemp, tempValue, hasRpm, rpmValue, &tempClamped, &rpmClamped);
  Serial.printf("[API] /api/setpoints hasTemp=%s temp=%.2f hasRpm=%s rpm=%ld clamped(temp=%s,rpm=%s)\n",
                hasTemp ? "true" : "false",
                tempValue,
                hasRpm ? "true" : "false",
                rpmValue,
                tempClamped ? "true" : "false",
                rpmClamped ? "true" : "false");

  IncubatorState state = snapshotState();
  StaticJsonDocument<448> responseDoc;
  responseDoc["temp_c"] = state.temp_c;
  responseDoc["temp_sp_c"] = state.temp_sp_c;
  responseDoc["rpm"] = state.rpm;
  responseDoc["pwm"] = state.pwm;
  responseDoc["run_enabled"] = state.run_enabled;
  responseDoc["uptime_ms"] = state.uptime_ms;
  responseDoc["last_error"] = state.last_error;

  JsonObject validation = responseDoc.createNestedObject("validation");
  if (hasTemp)
  {
    validation["temp_sp_c_clamped"] = tempClamped;
  }
  if (hasRpm)
  {
    validation["rpm_clamped"] = rpmClamped;
  }

  String bodyOut;
  serializeJson(responseDoc, bodyOut);
  request->send(200, "application/json", bodyOut);
}

void configureWebServer()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/html", INDEX_HTML); });

  server.on("/api/state", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "application/json", buildStateJson()); });

  server.on("/api/run", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, handleRunBody);

  server.on("/api/setpoints", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, handleSetpointsBody);

  events.onConnect([](AsyncEventSourceClient *client)
                   {
					 String payload = buildStateJson();
					 client->send(payload.c_str(), "state", millis()); });

  server.addHandler(&events);
  server.begin();
}

void configureAccessPoint()
{
  IPAddress apIp(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(apIp, gateway, subnet);

  bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD);
  if (!ok)
  {
    setLastError("Failed to start AP");
    Serial.println("Failed to start AP");
    return;
  }

  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

void processSerialCommand()
{
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, Serial);
  if (err)
  {
    setLastError("Serial JSON parse error");
    return;
  }

  const char *cmd = doc["cmd"];
  if (cmd == nullptr)
  {
    setLastError("Missing serial cmd");
    return;
  }

  if (strcmp(cmd, "sp") == 0)
  {
    if (!doc["arg"].is<float>() && !doc["arg"].is<int>() && !doc["arg"].is<double>())
    {
      setLastError("sp arg must be numeric");
      return;
    }
    bool tempClamped = false;
    updateStateSetpoints(true, doc["arg"].as<float>(), false, 0, &tempClamped, nullptr);
    if (tempClamped)
    {
      setLastError("sp clamped to safety range");
    }
  }
  else if (strcmp(cmd, "rpm") == 0)
  {
    if (!doc["arg"].is<float>() && !doc["arg"].is<int>() && !doc["arg"].is<double>())
    {
      setLastError("rpm arg must be numeric");
      return;
    }
    bool rpmClamped = false;
    updateStateSetpoints(false, 0.0f, true, lround(doc["arg"].as<float>()), nullptr, &rpmClamped);
    if (rpmClamped)
    {
      setLastError("rpm clamped to safety range");
    }
  }
  else if (strcmp(cmd, "start") == 0)
  {
    updateStateRunEnabled(true);
  }
  else if (strcmp(cmd, "stop") == 0)
  {
    updateStateRunEnabled(false);
  }
  else
  {
    setLastError("Unknown serial command");
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  stateMutex = xSemaphoreCreateMutex();
  if (stateMutex == nullptr)
  {
    Serial.println("Failed to create state mutex");
  }

  memset(&gState, 0, sizeof(gState));
  gState.temp_sp_c = DEFAULT_TEMP_SP_C;
  gState.rpm = DEFAULT_RPM;
  gState.run_enabled = DEFAULT_RUN_ENABLED;
  strncpy(gState.last_error, "", sizeof(gState.last_error) - 1);

  if (!sht31.begin(0x44))
  {
    setLastError("SHT31 not found at 0x44");
    Serial.println("SHT31 not found at 0x44");
  }

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  stepper.begin(PIN_STEP, PIN_DIR);
  stepper.stop();
  stepper.powerOff();

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);

  ledcSetup(CH_H, 2000, 8);
  ledcAttachPin(ENA, CH_H);
  ledcSetup(CH_C, 2000, 8);
  ledcAttachPin(ENB, CH_C);
  ledcWrite(CH_H, 0);
  ledcWrite(CH_C, 0);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Incubadora AP");
  lcd.setCursor(0, 1);
  lcd.print("Boot: STOPPED");

  temperaturePid.SetMode(AUTOMATIC);
  temperaturePid.SetOutputLimits(0, 255);

  configureAccessPoint();
  configureWebServer();

  Serial.println("ready");
}

void applyActuators(const IncubatorState &state, bool tempValid, int pwmToApply)
{
  static bool motorPowered = false;
  static long appliedRpm = -1;
  static bool lastRunEnabled = false;

  if (!state.run_enabled)
  {
    ledcWrite(CH_H, 0);
    ledcWrite(CH_C, 0);
    if (motorPowered)
    {
      stepper.stop();
      stepper.powerOff();
      motorPowered = false;
      appliedRpm = -1;
      Serial.println("[MOTOR] Stopped (run_enabled=false)");
    }
    lastRunEnabled = false;
    return;
  }

  if (!motorPowered)
  {
    stepper.powerOn();
    motorPowered = true;
    Serial.println("[MOTOR] Powered on");
  }

  if (!lastRunEnabled)
  {
    Serial.println("[MOTOR] Run enabled");
  }
  lastRunEnabled = true;

  if (appliedRpm != state.rpm)
  {
    float stepsPerSecond = state.rpm * RPM_TO_STEPS_PER_SEC;
    stepper.spin(stepsPerSecond);
    appliedRpm = state.rpm;
    Serial.printf("[MOTOR] RPM=%ld -> %.1f steps/s\n", state.rpm, stepsPerSecond);
  }

  if (state.rpm == 0)
  {
    stepper.stop();
    Serial.println("[MOTOR] RPM=0, stop command issued");
  }

  if (!tempValid)
  {
    ledcWrite(CH_H, 0);
    ledcWrite(CH_C, 0);
    return;
  }

  if (state.temp_c < state.temp_sp_c)
  {
    ledcWrite(CH_H, pwmToApply);
    ledcWrite(CH_C, 0);
  }
  else
  {
    ledcWrite(CH_H, 0);
    ledcWrite(CH_C, (255 - pwmToApply));
  }
}

void updateLcd(const IncubatorState &state)
{
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(state.temp_c, 1);
  lcd.print("/");
  lcd.print(state.temp_sp_c, 1);
  lcd.print("C   ");

  lcd.setCursor(0, 1);
  lcd.print("R:");
  lcd.print(state.rpm);
  lcd.print(" ");
  lcd.print(state.run_enabled ? "ON " : "OFF");
  lcd.print("    ");
}

void loop()
{
  static uint32_t lastControlMs = 0;
  static uint32_t lastSseMs = 0;

  // Keep all stepper operations in a single task context.
  stepper.loop();

  uint32_t now = millis();

  if (Serial.available() > 0)
  {
    processSerialCommand();
  }

  if ((now - lastControlMs) >= CONTROL_INTERVAL_MS)
  {
    lastControlMs = now;

    IncubatorState state = snapshotState();
    float measuredTemp = sht31.readTemperature();
    bool tempValid = !isnan(measuredTemp);
    int pwmToApply = 0;

    if (tempValid)
    {
      setpointPid = state.temp_sp_c;
      inputPid = static_cast<double>(measuredTemp);
      temperaturePid.Compute();
      pwmToApply = constrain(static_cast<int>(lround(outputPid)), 0, 255);
    }
    else
    {
      setLastError("Temperature read failed");
      measuredTemp = state.temp_c;
    }

    if (!state.run_enabled)
    {
      pwmToApply = 0;
    }

    updateStateTelemetry(measuredTemp, pwmToApply, now);
    state = snapshotState();
    applyActuators(state, tempValid, pwmToApply);
    updateLcd(state);

    String serialTelemetry = buildLegacySerialTelemetry();
    Serial.println(serialTelemetry);
  }

  if ((now - lastSseMs) >= SSE_INTERVAL_MS)
  {
    lastSseMs = now;
    String payload = buildStateJson();
    events.send(payload.c_str(), "state", now);
  }
}
