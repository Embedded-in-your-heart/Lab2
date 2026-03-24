#!/usr/bin/env python3
"""
TCP Server + Dash Dashboard for STM32 B-L475E-IOT01A Sensor Visualization

Receives accelerometer, gyroscope, and significant motion data from
the STM32 IoT node over TCP, and displays a unified real-time dashboard
in the browser using Dash & Plotly.

Usage:
    python server.py [--host HOST] [--port PORT] [--dash-port DASH_PORT]

Default: TCP on 0.0.0.0:25550, Dashboard on http://localhost:8050
"""

import socket
import json
import threading
import argparse
import time
from collections import deque
from datetime import datetime

from dash import Dash, html, dcc
from dash.dependencies import Input, Output
import plotly.graph_objs as go
from plotly.subplots import make_subplots

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
MAX_POINTS = 300

# ---------------------------------------------------------------------------
# Shared data
# ---------------------------------------------------------------------------
acc_x = deque(maxlen=MAX_POINTS)
acc_y = deque(maxlen=MAX_POINTS)
acc_z = deque(maxlen=MAX_POINTS)
gyro_x = deque(maxlen=MAX_POINTS)
gyro_y = deque(maxlen=MAX_POINTS)
gyro_z = deque(maxlen=MAX_POINTS)
time_labels = deque(maxlen=MAX_POINTS)

sig_motion_times = deque(maxlen=100)
sig_motion_lock = threading.Lock()

sample_count = 0
connected = threading.Event()
client_addr_str = ""

# ---------------------------------------------------------------------------
# TCP Server
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

        buf = ""
        try:
            while True:
                data = conn.recv(4096)
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

                    now = datetime.now().strftime("%H:%M:%S.") + f"{datetime.now().microsecond // 1000:03d}"
                    time_labels.append(now)
                    sample_count += 1

                    acc = msg.get("acc", {})
                    acc_x.append(acc.get("x", 0))
                    acc_y.append(acc.get("y", 0))
                    acc_z.append(acc.get("z", 0))

                    gyro = msg.get("gyro", {})
                    gyro_x.append(gyro.get("x", 0.0))
                    gyro_y.append(gyro.get("y", 0.0))
                    gyro_z.append(gyro.get("z", 0.0))

                    if msg.get("sig_motion", 0) == 1:
                        with sig_motion_lock:
                            sig_motion_times.append(now)
                        print(f"[TCP] *** Significant Motion Detected! ***")

        except (ConnectionResetError, BrokenPipeError, OSError):
            pass
        finally:
            conn.close()
            connected.clear()
            print(f"[TCP] Client disconnected: {client_addr_str}")

# ---------------------------------------------------------------------------
# Dash App
# ---------------------------------------------------------------------------

app = Dash(__name__)

app.layout = html.Div(
    style={
        "backgroundColor": "#0f0f1a",
        "minHeight": "100vh",
        "padding": "20px",
        "fontFamily": "'Segoe UI', Arial, sans-serif",
        "color": "#e0e0e0",
    },
    children=[
        # Title bar
        html.Div(
            style={
                "display": "flex",
                "justifyContent": "space-between",
                "alignItems": "center",
                "marginBottom": "16px",
                "padding": "12px 20px",
                "backgroundColor": "#1a1a2e",
                "borderRadius": "10px",
                "border": "1px solid #2a2a4a",
            },
            children=[
                html.H1(
                    "STM32 B-L475E-IOT01A  Sensor Dashboard",
                    style={"margin": 0, "fontSize": "22px", "color": "#ffffff"},
                ),
                html.Div(id="status-badge", style={"fontSize": "14px"}),
            ],
        ),

        # Latest value cards
        html.Div(
            id="value-cards",
            style={
                "display": "grid",
                "gridTemplateColumns": "repeat(4, 1fr)",
                "gap": "12px",
                "marginBottom": "16px",
            },
        ),

        # Main chart (accelerometer + gyroscope combined)
        dcc.Graph(id="main-chart", style={"height": "520px"}),

        # Significant motion event log
        html.Div(
            style={
                "marginTop": "16px",
                "padding": "14px 20px",
                "backgroundColor": "#1a1a2e",
                "borderRadius": "10px",
                "border": "1px solid #2a2a4a",
            },
            children=[
                html.H3(
                    "Significant Motion Events",
                    style={"margin": "0 0 10px 0", "fontSize": "16px", "color": "#ffd43b"},
                ),
                html.Div(id="sig-motion-log", style={"maxHeight": "120px", "overflowY": "auto"}),
            ],
        ),

        # Auto-refresh timer
        dcc.Interval(id="interval", interval=200, n_intervals=0),
    ],
)


def _make_card(title, value, unit, color):
    return html.Div(
        style={
            "backgroundColor": "#1a1a2e",
            "borderRadius": "10px",
            "padding": "14px 18px",
            "border": f"1px solid {color}33",
        },
        children=[
            html.Div(title, style={"fontSize": "12px", "color": "#888", "marginBottom": "4px"}),
            html.Span(value, style={"fontSize": "22px", "fontWeight": "bold", "color": color}),
            html.Span(f" {unit}", style={"fontSize": "12px", "color": "#888"}),
        ],
    )


@app.callback(
    [
        Output("main-chart", "figure"),
        Output("status-badge", "children"),
        Output("value-cards", "children"),
        Output("sig-motion-log", "children"),
    ],
    Input("interval", "n_intervals"),
)
def update(_n):
    t = list(time_labels)
    n = len(t)

    # --- Build combined figure with 2 subplots sharing x-axis ---
    fig = make_subplots(
        rows=2, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.08,
        subplot_titles=("Accelerometer  (mg)", "Gyroscope  (mdps)"),
        row_heights=[0.5, 0.5],
    )

    axis_colors = {"X": "#ff6b6b", "Y": "#51cf66", "Z": "#339af0"}

    if n > 0:
        xs = list(range(n))

        # Accelerometer traces
        for name, data, color in [
            ("X", list(acc_x), axis_colors["X"]),
            ("Y", list(acc_y), axis_colors["Y"]),
            ("Z", list(acc_z), axis_colors["Z"]),
        ]:
            fig.add_trace(
                go.Scatter(
                    x=xs, y=data, mode="lines",
                    name=f"Acc {name}", line=dict(color=color, width=1.5),
                    legendgroup="acc",
                ),
                row=1, col=1,
            )

        # Gyroscope traces
        for name, data, color in [
            ("X", list(gyro_x), axis_colors["X"]),
            ("Y", list(gyro_y), axis_colors["Y"]),
            ("Z", list(gyro_z), axis_colors["Z"]),
        ]:
            fig.add_trace(
                go.Scatter(
                    x=xs, y=data, mode="lines",
                    name=f"Gyro {name}", line=dict(color=color, width=1.5, dash="dot"),
                    legendgroup="gyro",
                ),
                row=2, col=1,
            )

        # Mark significant motion events as vertical lines on both subplots
        with sig_motion_lock:
            sm_times_set = set(sig_motion_times)
        for i, label in enumerate(t):
            if label in sm_times_set:
                for row in [1, 2]:
                    fig.add_vline(
                        x=i, line_width=2, line_dash="dash",
                        line_color="#ffd43b", opacity=0.7,
                        row=row, col=1,
                    )

    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#0f0f1a",
        plot_bgcolor="#16162a",
        font=dict(color="#ccc"),
        margin=dict(l=60, r=30, t=40, b=40),
        legend=dict(
            orientation="h", yanchor="bottom", y=1.02, xanchor="center", x=0.5,
            font=dict(size=11),
        ),
        hovermode="x unified",
    )
    fig.update_xaxes(showgrid=True, gridcolor="#222244")
    fig.update_yaxes(showgrid=True, gridcolor="#222244")

    # --- Status badge ---
    if connected.is_set():
        badge = html.Span(
            [
                html.Span("● ", style={"color": "#51cf66"}),
                f"Connected: {client_addr_str}   |   Samples: {sample_count}",
            ],
            style={"color": "#51cf66"},
        )
    else:
        badge = html.Span(
            [html.Span("● ", style={"color": "#888"}), "Waiting for STM32 ..."],
            style={"color": "#888"},
        )

    # --- Value cards ---
    if n > 0:
        cards = [
            _make_card("Acc X / Y / Z", f"{list(acc_x)[-1]}  {list(acc_y)[-1]}  {list(acc_z)[-1]}", "mg", "#ff6b6b"),
            _make_card("Gyro X", f"{list(gyro_x)[-1]:.1f}", "mdps", "#51cf66"),
            _make_card("Gyro Y", f"{list(gyro_y)[-1]:.1f}", "mdps", "#339af0"),
            _make_card("Gyro Z", f"{list(gyro_z)[-1]:.1f}", "mdps", "#cc5de8"),
        ]
    else:
        cards = [
            _make_card("Acc X / Y / Z", "-- -- --", "mg", "#555"),
            _make_card("Gyro X", "--", "mdps", "#555"),
            _make_card("Gyro Y", "--", "mdps", "#555"),
            _make_card("Gyro Z", "--", "mdps", "#555"),
        ]

    # --- Significant motion log ---
    with sig_motion_lock:
        events = list(sig_motion_times)
    if events:
        log_items = [
            html.Div(
                f"⚡ {t}",
                style={"color": "#ffd43b", "fontSize": "13px", "padding": "2px 0"},
            )
            for t in reversed(events[-20:])
        ]
    else:
        log_items = [html.Div("No events yet.", style={"color": "#555", "fontSize": "13px"})]

    return fig, badge, cards, log_items


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="STM32 Sensor TCP Server + Dashboard")
    parser.add_argument("--host", default="0.0.0.0", help="TCP bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=25550, help="TCP port (default: 25550)")
    parser.add_argument("--dash-port", type=int, default=8050, help="Dashboard port (default: 8050)")
    args = parser.parse_args()

    # Start TCP server in background
    srv = threading.Thread(target=tcp_server, args=(args.host, args.port), daemon=True)
    srv.start()

    print(f"[Dashboard] Open browser: http://localhost:{args.dash_port}")
    app.run(debug=False, host="0.0.0.0", port=args.dash_port)


if __name__ == "__main__":
    main()
