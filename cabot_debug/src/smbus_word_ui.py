#!/usr/bin/env python3

import argparse
import json
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List
from urllib.parse import parse_qs, urlparse

from read_smbus_word_map import (
    SMBusCanClient,
    parse_locations,
    read_battery_state_snapshots,
    read_registers,
)


HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SMBus Word Monitor</title>
  <style>
    :root {
      --bg: #f3efe8;
      --panel: #fffdf9;
      --ink: #1d2a33;
      --muted: #667784;
      --line: #d6cbc0;
      --accent: #0d6b78;
      --warn: #b44b2a;
      --warn-soft: #fde7df;
      --ok: #2e6a45;
      --ok-soft: #e2f3e7;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Segoe UI", "Hiragino Sans", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, #fff8ee 0, #fff8ee 18%, transparent 45%),
        linear-gradient(180deg, #efe7dc 0, var(--bg) 18%, #ece8e1 100%);
      min-height: 100vh;
    }
    .wrap {
      max-width: 1400px;
      margin: 0 auto;
      padding: 24px;
    }
    .hero {
      display: grid;
      gap: 16px;
      grid-template-columns: 1.2fr 1fr;
      align-items: end;
      margin-bottom: 20px;
    }
    .title {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 20px;
      padding: 20px 22px;
      box-shadow: 0 12px 32px rgba(29, 42, 51, 0.06);
    }
    h1 {
      margin: 0 0 8px;
      font-size: clamp(28px, 4vw, 42px);
      line-height: 1;
      letter-spacing: -0.04em;
    }
    .subtitle {
      margin: 0;
      color: var(--muted);
      font-size: 15px;
    }
    .controls {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 20px;
      padding: 18px;
      display: grid;
      gap: 12px;
      box-shadow: 0 12px 32px rgba(29, 42, 51, 0.06);
    }
    .row {
      display: grid;
      gap: 12px;
      grid-template-columns: repeat(4, minmax(0, 1fr));
    }
    label {
      display: grid;
      gap: 6px;
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--muted);
    }
    input {
      width: 100%;
      border: 1px solid var(--line);
      border-radius: 12px;
      padding: 11px 12px;
      font-size: 14px;
      color: var(--ink);
      background: #fff;
    }
    .toolbar {
      display: flex;
      gap: 10px;
      align-items: center;
      flex-wrap: wrap;
    }
    button {
      border: 0;
      border-radius: 999px;
      padding: 11px 18px;
      background: var(--accent);
      color: white;
      font-weight: 700;
      cursor: pointer;
    }
    button.secondary {
      background: #d7e6e8;
      color: var(--ink);
    }
    .badge {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 8px 12px;
      border-radius: 999px;
      background: #f5f0e8;
      color: var(--muted);
      font-size: 13px;
    }
    .dot {
      width: 9px;
      height: 9px;
      border-radius: 50%;
      background: var(--accent);
    }
    .grid {
      display: grid;
      gap: 16px;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
    }
    .card {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 20px;
      overflow: hidden;
      box-shadow: 0 12px 32px rgba(29, 42, 51, 0.06);
    }
    .card-head {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 16px 18px;
      border-bottom: 1px solid var(--line);
      background: linear-gradient(180deg, #fff 0, #faf5ee 100%);
    }
    .card-head h2 {
      margin: 0;
      font-size: 18px;
    }
    .summary {
      font-size: 12px;
      color: var(--muted);
    }
    .state-summary {
      display: grid;
      gap: 10px;
      padding: 14px 14px 4px;
      border-bottom: 1px solid #eee5da;
      background: linear-gradient(180deg, #f8f5ef 0, #fffdf9 100%);
    }
    .state-head {
      display: flex;
      justify-content: space-between;
      align-items: baseline;
      gap: 12px;
      flex-wrap: wrap;
    }
    .state-title {
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--muted);
    }
    .state-serial {
      font-size: 13px;
      font-weight: 700;
    }
    .state-grid {
      display: grid;
      gap: 8px;
      grid-template-columns: repeat(auto-fit, minmax(110px, 1fr));
    }
    .state-item {
      border: 1px solid #eee5da;
      border-radius: 12px;
      padding: 10px 11px;
      background: #fff;
    }
    .state-label {
      color: var(--muted);
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 0.06em;
      margin-bottom: 4px;
    }
    .state-value {
      font-size: 16px;
      font-weight: 700;
      line-height: 1.25;
      overflow-wrap: anywhere;
    }
    .register-list {
      display: grid;
      gap: 10px;
      padding: 12px;
    }
    .register {
      display: grid;
      gap: 10px;
      padding: 14px;
      border: 1px solid #eee5da;
      border-radius: 16px;
      background: linear-gradient(180deg, #fff 0, #fcf8f2 100%);
    }
    .register-header {
      display: flex;
      gap: 10px;
      align-items: flex-start;
      justify-content: space-between;
    }
    .register-name {
      display: grid;
      gap: 4px;
      min-width: 0;
    }
    .register-title {
      font-weight: 700;
      font-size: 14px;
      line-height: 1.2;
    }
    .register-addr {
      color: var(--muted);
      font-size: 12px;
      letter-spacing: 0.04em;
    }
    .register-main {
      display: grid;
      gap: 6px;
      min-width: 0;
      padding-top: 2px;
    }
    .decoded {
      font-size: 15px;
      font-weight: 700;
      line-height: 1.45;
      overflow-wrap: anywhere;
    }
    .rawline {
      color: var(--muted);
      font-size: 12px;
      line-height: 1.4;
    }
    .status {
      display: inline-block;
      min-width: 72px;
      text-align: center;
      padding: 4px 8px;
      border-radius: 999px;
      font-weight: 700;
      font-size: 11px;
    }
    .ok {
      background: var(--ok-soft);
      color: var(--ok);
    }
    .timeout {
      background: var(--warn-soft);
      color: var(--warn);
    }
    .error {
      margin-bottom: 16px;
      padding: 14px 16px;
      border-radius: 16px;
      border: 1px solid #efc8ba;
      background: var(--warn-soft);
      color: #7d2f16;
      display: none;
    }
    @media (max-width: 900px) {
      .hero, .row { grid-template-columns: 1fr; }
      .wrap { padding: 14px; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <div class="title">
        <h1>SMBus Word Monitor</h1>
        <p class="subtitle">Power board firmware が対応している SMBus Word Read を location ごとに可視化します。</p>
      </div>
      <div class="controls">
        <div class="row">
          <label>Interface
            <input id="interface" value="can0">
          </label>
          <label>Locations
            <input id="locations" value="1,2,3,4">
          </label>
          <label>Timeout (s)
            <input id="timeout" value="0.5">
          </label>
          <label>Auto refresh (s)
            <input id="refresh" value="0">
          </label>
        </div>
        <div class="toolbar">
          <button id="read">Read now</button>
          <button id="stop" class="secondary">Stop auto refresh</button>
          <span class="badge"><span class="dot"></span><span id="meta">Idle</span></span>
        </div>
      </div>
    </section>
    <div id="error" class="error"></div>
    <section id="grid" class="grid"></section>
  </div>
  <script>
    const state = { timer: null };

    function esc(text) {
      return String(text)
        .replace(/&/g, "&amp;")
        .replace(/</g, "&lt;")
        .replace(/>/g, "&gt;")
        .replace(/"/g, "&quot;");
    }

    function setMeta(text) {
      document.getElementById("meta").textContent = text;
    }

    function showError(text) {
      const el = document.getElementById("error");
      el.textContent = text;
      el.style.display = text ? "block" : "none";
    }

    function byLocation(rows) {
      const grouped = {};
      for (const row of rows) {
        if (!grouped[row.location]) grouped[row.location] = [];
        grouped[row.location].push(row);
      }
      return grouped;
    }

    function fmtNumber(value, digits, suffix = "") {
      if (value === null || value === undefined) return "not available";
      return `${Number(value).toFixed(digits)}${suffix}`;
    }

    function renderStateSummary(state) {
      if (!state) {
        return `
          <div class="state-summary">
            <div class="state-head">
              <div class="state-title">Battery State Summary</div>
              <div class="state-serial">serial: not available</div>
            </div>
          </div>
        `;
      }

      const items = [
        ["Percentage", state.percentage === null ? "not available" : `${Math.round(state.percentage * 100)}%`],
        ["Voltage", fmtNumber(state.voltage, 3, " V")],
        ["Current", fmtNumber(state.current, 3, " A")],
        ["Temperature", fmtNumber(state.temperature, 1, " C")],
        ["Charge", fmtNumber(state.charge, 3, " Ah")],
        ["Capacity", fmtNumber(state.capacity, 3, " Ah")],
        ["Design", fmtNumber(state.design_capacity, 3, " Ah")],
      ];

      return `
        <div class="state-summary">
          <div class="state-head">
            <div class="state-title">Battery State Summary</div>
            <div class="state-serial">serial: ${esc(state.serial_number ?? "not available")}</div>
          </div>
          <div class="state-grid">
            ${items.map(([label, value]) => `
              <div class="state-item">
                <div class="state-label">${esc(label)}</div>
                <div class="state-value">${esc(value)}</div>
              </div>
            `).join("")}
          </div>
        </div>
      `;
    }

    function renderRegisters(rows) {
      return `
        <div class="register-list">
          ${rows.map((row) => `
            <div class="register">
              <div class="register-header">
                <div class="register-name">
                  <div class="register-title">${esc(row.name)}</div>
                  <div class="register-addr">${esc("0x" + row.addr.toString(16).toUpperCase().padStart(2, "0"))}</div>
                </div>
                <div><span class="status ${row.status}">${esc(row.status)}</span></div>
              </div>
              <div class="register-main">
                <div class="decoded">${esc(row.decoded)}</div>
                <div class="rawline">raw: ${esc(row.raw_hex ?? "-")} / value: ${esc(row.value ?? "-")}</div>
              </div>
            </div>
          `).join("")}
        </div>
      `;
    }

    function render(data) {
      const grouped = byLocation(data.results);
      const cards = Object.keys(grouped)
        .sort((a, b) => Number(a) - Number(b))
        .map((location) => {
          const rows = grouped[location];
          const okCount = rows.filter((row) => row.status === "ok").length;
          return `
            <article class="card">
              <div class="card-head">
                <h2>Location ${esc(location)}</h2>
                <div class="summary">${okCount}/${rows.length} responses</div>
              </div>
              ${renderStateSummary(data.battery_states[location])}
              ${renderRegisters(rows)}
            </article>
          `;
        }).join("");
      document.getElementById("grid").innerHTML = cards;
    }

    async function readNow() {
      const interfaceName = document.getElementById("interface").value.trim();
      const locations = document.getElementById("locations").value.trim();
      const timeout = document.getElementById("timeout").value.trim();
      const query = new URLSearchParams({ interface: interfaceName, locations, timeout });
      setMeta("Reading...");
      showError("");
      try {
        const response = await fetch(`/api/read?${query.toString()}`);
        const data = await response.json();
        if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
        render(data);
        setMeta(`Updated ${new Date(data.timestamp * 1000).toLocaleTimeString()}`);
      } catch (error) {
        document.getElementById("grid").innerHTML = "";
        showError(error.message);
        setMeta("Read failed");
      }
      resetTimer();
    }

    function resetTimer() {
      if (state.timer) {
        clearTimeout(state.timer);
        state.timer = null;
      }
      const seconds = Number(document.getElementById("refresh").value);
      if (seconds > 0) {
        state.timer = setTimeout(readNow, seconds * 1000);
      }
    }

    document.getElementById("read").addEventListener("click", readNow);
    document.getElementById("stop").addEventListener("click", () => {
      document.getElementById("refresh").value = "0";
      resetTimer();
      setMeta("Auto refresh stopped");
    });
    ["interface", "locations", "timeout", "refresh"].forEach((id) => {
      document.getElementById(id).addEventListener("change", resetTimer);
    });
    readNow();
  </script>
</body>
</html>
"""


def build_payload(interface: str, locations: List[int], timeout: float) -> Dict[str, object]:
    client = SMBusCanClient(interface, timeout)
    try:
        results = read_registers(client, locations, timeout)
    finally:
        client.close()
    return {
        "interface": interface,
        "locations": locations,
        "timeout": timeout,
        "timestamp": time.time(),
        "battery_states": {
            str(location): snapshot.to_dict()
            for location, snapshot in read_battery_state_snapshots(interface, locations, max(timeout, 1.2)).items()
        },
        "results": [result.to_dict() for result in results],
    }


class Handler(BaseHTTPRequestHandler):
    interface = "can0"
    locations = [1, 2, 3, 4]
    timeout = 0.5

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self.respond_html(HTML)
            return
        if parsed.path == "/api/read":
            self.handle_api_read(parsed.query)
            return
        self.send_error(404, "Not found")

    def log_message(self, fmt: str, *args) -> None:
        sys.stderr.write("%s - - [%s] %s\n" % (self.address_string(), self.log_date_time_string(), fmt % args))

    def handle_api_read(self, query: str) -> None:
        params = parse_qs(query)
        interface = params.get("interface", [self.interface])[0]
        locations_text = params.get("locations", [",".join(str(v) for v in self.locations)])[0]
        timeout_text = params.get("timeout", [str(self.timeout)])[0]

        try:
            locations = parse_locations(locations_text)
            timeout = float(timeout_text)
            payload = build_payload(interface, locations, timeout)
        except Exception as exc:
            self.respond_json({"error": str(exc)}, status=400)
            return

        self.respond_json(payload)

    def respond_html(self, body: str) -> None:
        raw = body.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def respond_json(self, payload: Dict[str, object], status: int = 200) -> None:
        raw = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)


def main() -> int:
    parser = argparse.ArgumentParser(description="Serve a local SMBus Word Read dashboard.")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8765, help="Bind port (default: 8765)")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface name (default: can0)")
    parser.add_argument(
        "--locations",
        type=parse_locations,
        default=[1, 2, 3, 4],
        help="Comma-separated battery locations to query by default (default: 1,2,3,4)",
    )
    parser.add_argument("--timeout", type=float, default=0.5, help="Response timeout in seconds (default: 0.5)")
    args = parser.parse_args()

    Handler.interface = args.interface
    Handler.locations = args.locations
    Handler.timeout = args.timeout

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Serving SMBus UI on http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
