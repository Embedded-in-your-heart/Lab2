#!/usr/bin/env python3
"""
TCP Server + WebSocket Dashboard for STM32 B-L475E-IOT01A Sensor Visualization

Architecture:
    STM32 --TCP--> Python server --WebSocket--> Browser (Plotly.js)

Usage:
    python server.py [--host HOST] [--port PORT] [--ws-port WS_PORT]

Default: TCP on 0.0.0.0:25550, Dashboard on http://localhost:8050
"""

import asyncio
import json
import socket
import threading
import argparse
import time
from collections import deque
from datetime import datetime
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

import websockets

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
sample_count = 0
connected = threading.Event()
client_addr_str = ""

sig_motion_times = deque(maxlen=100)

# WebSocket clients (managed from asyncio thread)
ws_clients: set[websockets.ServerProtocol] = set()
ws_loop: asyncio.AbstractEventLoop | None = None

# ---------------------------------------------------------------------------
# Broadcast helper (thread-safe)
# ---------------------------------------------------------------------------

def broadcast(msg: dict):
    """Send a message to all connected WebSocket browser clients."""
    if ws_loop is None or not ws_clients:
        return
    payload = json.dumps(msg)
    asyncio.run_coroutine_threadsafe(_broadcast(payload), ws_loop)


async def _broadcast(payload: str):
    if ws_clients:
        await asyncio.gather(
            *(client.send(payload) for client in ws_clients),
            return_exceptions=True,
        )


def send_status():
    """Broadcast TCP connection status to all browser clients."""
    if connected.is_set():
        state = "connected"
    else:
        state = "disconnected"

    broadcast({
        "type": "status",
        "state": state,
        "client": client_addr_str,
        "sample_count": sample_count,
    })

# ---------------------------------------------------------------------------
# TCP Server (receives data from STM32)
# ---------------------------------------------------------------------------

def tcp_server(host, port):
    global client_addr_str, sample_count

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"[TCP] Listening on {host}:{port} ...")

    while True:
        conn, addr = srv.accept()
        client_addr_str = f"{addr[0]}:{addr[1]}"
        print(f"[TCP] Client connected: {client_addr_str}")
        connected.set()
        send_status()

        conn.settimeout(5.0)
        buf = ""
        try:
            while True:
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    print("[TCP] No data for 5s, assuming client disconnected")
                    break
                if not data:
                    break
                buf += data.decode("utf-8", errors="replace")

                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                    except json.JSONDecodeError:
                        continue

                    now = datetime.now().strftime("%H:%M:%S.") + \
                          f"{datetime.now().microsecond // 1000:03d}"
                    sample_count += 1

                    sig = msg.get("sig_motion", 0)
                    if sig == 1:
                        sig_motion_times.append(now)
                        print("[TCP] *** Significant Motion Detected! ***")

                    # Push data to browser via WebSocket
                    broadcast({
                        "type": "data",
                        "acc": msg.get("acc", {}),
                        "gyro": msg.get("gyro", {}),
                        "sig_motion": sig,
                        "time": now,
                        "sample_count": sample_count,
                    })

        except (ConnectionResetError, BrokenPipeError, OSError):
            pass
        finally:
            conn.close()
            connected.clear()
            print(f"[TCP] Client disconnected: {client_addr_str}")
            print("[TCP] Waiting for reconnection ...")
            send_status()

# ---------------------------------------------------------------------------
# WebSocket Server (pushes data to browser)
# ---------------------------------------------------------------------------

async def ws_handler(websocket):
    ws_clients.add(websocket)
    print(f"[WS] Browser connected ({len(ws_clients)} total)")
    send_status()

    # Send recent sig_motion events so new browsers see history
    events = list(sig_motion_times)
    if events:
        await websocket.send(json.dumps({
            "type": "sig_motion_history",
            "times": events,
        }))

    try:
        async for _ in websocket:
            pass  # browser doesn't send data, just keep alive
    finally:
        ws_clients.discard(websocket)
        print(f"[WS] Browser disconnected ({len(ws_clients)} total)")


async def ws_main(ws_port):
    global ws_loop
    ws_loop = asyncio.get_event_loop()

    async with websockets.serve(ws_handler, "0.0.0.0", ws_port):
        print(f"[WS] WebSocket server on ws://0.0.0.0:{ws_port}")
        await asyncio.Future()  # run forever

# ---------------------------------------------------------------------------
# HTTP Server (serves dashboard.html)
# ---------------------------------------------------------------------------

def http_server(http_port, ws_port):
    static_dir = Path(__file__).parent

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(static_dir), **kwargs)

        def do_GET(self):
            if self.path == "/":
                self.path = "/dashboard.html"
            elif self.path == "/config":
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"ws_port": ws_port}).encode())
                return
            return super().do_GET()

        def log_message(self, format, *args):
            pass  # suppress access logs

    server = HTTPServer(("0.0.0.0", http_port), Handler)
    print(f"[HTTP] Dashboard: http://localhost:{http_port}")
    server.serve_forever()

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="STM32 Sensor TCP Server + WebSocket Dashboard")
    parser.add_argument("--host", default="0.0.0.0",
                        help="TCP bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=25550,
                        help="TCP port (default: 25550)")
    parser.add_argument("--ws-port", type=int, default=8765,
                        help="WebSocket port (default: 8765)")
    parser.add_argument("--http-port", type=int, default=8050,
                        help="Dashboard HTTP port (default: 8050)")
    args = parser.parse_args()

    # TCP server thread (receives STM32 data)
    threading.Thread(target=tcp_server, args=(args.host, args.port),
                     daemon=True).start()

    # HTTP server thread (serves dashboard page)
    threading.Thread(target=http_server, args=(args.http_port, args.ws_port),
                     daemon=True).start()

    # WebSocket server (asyncio main loop)
    print(f"[Dashboard] Open browser: http://localhost:{args.http_port}")
    asyncio.run(ws_main(args.ws_port))


if __name__ == "__main__":
    main()
