#!/usr/bin/env python3
import argparse
import csv
import html
import math
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
from tier4_vehicle_msgs.msg import ActuationCommandStamped


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def lerp(a, b, t):
    return int(round(a + (b - a) * t))


def rgb(c):
    return f"rgb({c[0]},{c[1]},{c[2]})"


def mix(c1, c2, t):
    t = clamp(t, 0.0, 1.0)
    return rgb(tuple(lerp(a, b, t) for a, b in zip(c1, c2)))


BLUE = (43, 108, 176)
NEUTRAL = (215, 215, 215)
RED = (202, 53, 53)
ORANGE = (232, 126, 42)
PURPLE = (113, 72, 170)
GREEN = (42, 145, 95)
DARK = (32, 37, 43)


def percentile(values, p, default=0.0):
    vals = sorted(v for v in values if math.isfinite(v))
    if not vals:
        return default
    if len(vals) == 1:
        return vals[0]
    idx = (len(vals) - 1) * clamp(p, 0.0, 1.0)
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return vals[lo]
    f = idx - lo
    return vals[lo] * (1.0 - f) + vals[hi] * f


def moving_average(values, n=5):
    if n <= 1 or len(values) < 2:
        return list(values)
    out = []
    half = n // 2
    for i in range(len(values)):
        a = max(0, i - half)
        b = min(len(values), i + half + 1)
        out.append(sum(values[a:b]) / (b - a))
    return out


class RaceTelemetry(Node):
    def __init__(self, sample_hz=20.0):
        super().__init__("race_telemetry")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.x = None
        self.y = None
        self.speed_mps = None
        self.steer_rad = None
        self.accel_cmd = 0.0
        self.brake_cmd = 0.0

        self.got_odom = False
        self.got_velocity = False
        self.rows = []
        self.t0 = time.monotonic()

        self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_cb, qos
        )
        self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self.velocity_cb, qos
        )
        self.create_subscription(
            SteeringReport, "/vehicle/status/steering_status", self.steer_cb, qos
        )
        self.create_subscription(
            ActuationCommandStamped,
            "/control/command/actuation_cmd",
            self.actuation_cb,
            qos,
        )

        self.create_timer(1.0 / sample_hz, self.sample)
        self.get_logger().info(
            "Recording telemetry. Start the simulator, then press Ctrl+C when the run is finished."
        )

    def odom_cb(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.got_odom = True

    def velocity_cb(self, msg):
        self.speed_mps = float(msg.longitudinal_velocity)
        self.got_velocity = True

    def steer_cb(self, msg):
        self.steer_rad = float(msg.steering_tire_angle)

    def actuation_cb(self, msg):
        act = getattr(msg, "actuation", None)
        if act is None:
            return
        self.accel_cmd = float(getattr(act, "accel_cmd", 0.0))
        self.brake_cmd = float(getattr(act, "brake_cmd", 0.0))

    def sample(self):
        if not (self.got_odom and self.got_velocity):
            return

        self.rows.append(
            {
                "t_s": time.monotonic() - self.t0,
                "x_m": self.x,
                "y_m": self.y,
                "speed_mps": abs(self.speed_mps),
                "steering_rad": self.steer_rad if self.steer_rad is not None else 0.0,
                "accel_cmd": self.accel_cmd,
                "brake_cmd": self.brake_cmd,
            }
        )


def process_rows(rows, base_speed_kmh):
    if not rows:
        return []

    # Remove exact duplicate positions only when the car is essentially stationary.
    clean = []
    for r in rows:
        if clean:
            dx = r["x_m"] - clean[-1]["x_m"]
            dy = r["y_m"] - clean[-1]["y_m"]
            if abs(dx) < 1e-6 and abs(dy) < 1e-6 and r["speed_mps"] < 0.05:
                continue
        clean.append(dict(r))

    speeds = [r["speed_mps"] for r in clean]
    smooth_speed = moving_average(speeds, 7)

    distance = 0.0
    for i, r in enumerate(clean):
        r["speed_kmh"] = r["speed_mps"] * 3.6
        r["steering_deg"] = math.degrees(r["steering_rad"])

        if i == 0:
            r["distance_m"] = 0.0
            r["accel_mps2"] = 0.0
            r["time_loss_vs_base_s"] = 0.0
            continue

        prev = clean[i - 1]
        dt = max(1e-4, r["t_s"] - prev["t_s"])
        dx = r["x_m"] - prev["x_m"]
        dy = r["y_m"] - prev["y_m"]
        ds = math.hypot(dx, dy)

        # Ignore obvious localization/reset jumps when accumulating track distance.
        if ds > 5.0:
            ds = 0.0

        distance += ds
        r["distance_m"] = distance

        # Central-ish derivative on smoothed speed.
        if 0 < i < len(clean) - 1:
            dt2 = max(1e-4, clean[i + 1]["t_s"] - clean[i - 1]["t_s"])
            accel = (smooth_speed[i + 1] - smooth_speed[i - 1]) / dt2
        else:
            accel = (smooth_speed[i] - smooth_speed[i - 1]) / dt
        r["accel_mps2"] = accel

        base_mps = base_speed_kmh / 3.6
        actual_mps = max(0.5, r["speed_mps"])
        if ds > 0.0:
            r["time_loss_vs_base_s"] = ds / actual_mps - ds / base_mps
        else:
            r["time_loss_vs_base_s"] = 0.0

    return clean


def save_csv(rows, path):
    fields = [
        "t_s", "distance_m", "x_m", "y_m",
        "speed_mps", "speed_kmh", "accel_mps2",
        "steering_rad", "steering_deg",
        "accel_cmd", "brake_cmd", "time_loss_vs_base_s",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, 0.0) for k in fields})


def norm_track(rows, width=1000, height=650, margin=35):
    xs = [r["x_m"] for r in rows]
    ys = [r["y_m"] for r in rows]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    xr = max(1e-6, xmax - xmin)
    yr = max(1e-6, ymax - ymin)
    scale = min((width - 2 * margin) / xr, (height - 2 * margin) / yr)
    used_w = xr * scale
    used_h = yr * scale
    ox = (width - used_w) / 2
    oy = (height - used_h) / 2

    def p(x, y):
        sx = ox + (x - xmin) * scale
        sy = height - (oy + (y - ymin) * scale)
        return sx, sy

    return p, width, height


def svg_track(rows, metric, title, color_fn, legend_text):
    p, width, height = norm_track(rows)
    pieces = [
        f'<svg class="track" viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)}">'
    ]
    # faint course trace
    pts = " ".join(f"{p(r['x_m'], r['y_m'])[0]:.1f},{p(r['x_m'], r['y_m'])[1]:.1f}" for r in rows)
    pieces.append(f'<polyline points="{pts}" fill="none" stroke="#d7dbe0" stroke-width="8" stroke-linecap="round" stroke-linejoin="round"/>')

    for i in range(1, len(rows)):
        a, b = rows[i - 1], rows[i]
        x1, y1 = p(a["x_m"], a["y_m"])
        x2, y2 = p(b["x_m"], b["y_m"])
        v = b[metric]
        c = color_fn(v)
        tip = (
            f"{title}: {v:.3f} | speed {b['speed_kmh']:.1f} km/h | "
            f"ax {b['accel_mps2']:+.2f} m/s² | steer {b['steering_deg']:+.1f}° | "
            f"brake {b['brake_cmd']:.3f}"
        )
        pieces.append(
            f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
            f'stroke="{c}" stroke-width="5.5" stroke-linecap="round"><title>{html.escape(tip)}</title></line>'
        )

    pieces.append("</svg>")
    pieces.append(f'<div class="legend-note">{html.escape(legend_text)}</div>')
    return "\n".join(pieces)


def svg_line(rows, xkey, ykey, title, y_label, ref=None):
    width, height, margin_l, margin_r, margin_t, margin_b = 1000, 280, 70, 25, 28, 45
    xs = [r[xkey] for r in rows]
    ys = [r[ykey] for r in rows]
    if ref is not None:
        ys = ys + [ref]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    if abs(ymax - ymin) < 1e-9:
        ymin -= 1.0
        ymax += 1.0
    pad = 0.08 * (ymax - ymin)
    ymin -= pad
    ymax += pad
    xr = max(1e-6, xmax - xmin)
    yr = max(1e-6, ymax - ymin)

    def sx(x):
        return margin_l + (x - xmin) / xr * (width - margin_l - margin_r)

    def sy(y):
        return height - margin_b - (y - ymin) / yr * (height - margin_t - margin_b)

    pts = " ".join(f"{sx(r[xkey]):.1f},{sy(r[ykey]):.1f}" for r in rows)
    out = [
        f'<svg class="linechart" viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)}">',
        f'<line x1="{margin_l}" y1="{height-margin_b}" x2="{width-margin_r}" y2="{height-margin_b}" stroke="#8a9199" stroke-width="1"/>',
        f'<line x1="{margin_l}" y1="{margin_t}" x2="{margin_l}" y2="{height-margin_b}" stroke="#8a9199" stroke-width="1"/>',
    ]
    if ref is not None and ymin <= ref <= ymax:
        y = sy(ref)
        out.append(f'<line x1="{margin_l}" y1="{y:.1f}" x2="{width-margin_r}" y2="{y:.1f}" stroke="#8f969e" stroke-width="1.5" stroke-dasharray="7 6"/>')
    out.append(f'<polyline points="{pts}" fill="none" stroke="#2f6fbb" stroke-width="2"/>')
    out.append(f'<text x="{margin_l}" y="18" font-size="15" fill="#343a40">{html.escape(title)}</text>')
    out.append(f'<text x="8" y="{margin_t+15}" font-size="12" fill="#555">{html.escape(y_label)} {ymax:.2f}</text>')
    out.append(f'<text x="8" y="{height-margin_b}" font-size="12" fill="#555">{ymin:.2f}</text>')
    out.append(f'<text x="{width-145}" y="{height-10}" font-size="12" fill="#555">distance [m]</text>')
    out.append("</svg>")
    return "\n".join(out)


def generate_html(rows, path, base_speed_kmh):
    speed_delta = max(4.0, percentile([abs(r["speed_kmh"] - base_speed_kmh) for r in rows], 0.98, 4.0))
    decel_scale = max(0.5, percentile([max(0.0, -r["accel_mps2"]) for r in rows], 0.98, 0.5))
    steer_scale = max(3.0, percentile([abs(r["steering_deg"]) for r in rows], 0.98, 3.0))
    brake_scale = max(0.05, percentile([max(0.0, r["brake_cmd"]) for r in rows], 0.98, 0.05))

    def speed_color(v):
        d = v - base_speed_kmh
        if d < 0:
            return mix(NEUTRAL, BLUE, abs(d) / speed_delta)
        return mix(NEUTRAL, RED, d / speed_delta)

    def decel_color(a):
        if a >= 0:
            return mix(NEUTRAL, GREEN, min(a / max(0.5, percentile([max(0.0, r["accel_mps2"]) for r in rows], 0.98, 0.5)), 1.0))
        return mix(NEUTRAL, RED, (-a) / decel_scale)

    def steer_color(deg):
        return mix(NEUTRAL, PURPLE, abs(deg) / steer_scale)

    def brake_color(v):
        return mix(NEUTRAL, ORANGE, max(0.0, v) / brake_scale)

    duration = rows[-1]["t_s"] - rows[0]["t_s"]
    distance = rows[-1]["distance_m"]
    speeds = [r["speed_kmh"] for r in rows]
    accels = [r["accel_mps2"] for r in rows]
    steers = [abs(r["steering_deg"]) for r in rows]
    brakes = [max(0.0, r["brake_cmd"]) for r in rows]
    below = sum(1 for r in rows if r["speed_kmh"] < base_speed_kmh) / len(rows) * 100.0
    above = sum(1 for r in rows if r["speed_kmh"] > base_speed_kmh) / len(rows) * 100.0
    time_loss = sum(r["time_loss_vs_base_s"] for r in rows)

    track_speed = svg_track(
        rows, "speed_kmh", f"Speed vs {base_speed_kmh:.0f} km/h", speed_color,
        f"Blue: below {base_speed_kmh:.0f} km/h / gray: near baseline / red: above baseline"
    )
    track_decel = svg_track(
        rows, "accel_mps2", "Longitudinal acceleration", decel_color,
        "Red: braking/deceleration / green: acceleration"
    )
    track_steer = svg_track(
        rows, "steering_deg", "Steering intensity", steer_color,
        f"Darker purple = larger |steering angle| (98th percentile ≈ {steer_scale:.1f}°)"
    )
    track_brake = svg_track(
        rows, "brake_cmd", "Brake command", brake_color,
        f"Darker orange = stronger brake command (98th percentile ≈ {brake_scale:.3f})"
    )

    charts = "\n".join([
        svg_line(rows, "distance_m", "speed_kmh", "Speed", "km/h", ref=base_speed_kmh),
        svg_line(rows, "distance_m", "accel_mps2", "Longitudinal acceleration", "m/s²", ref=0.0),
        svg_line(rows, "distance_m", "steering_deg", "Steering tire angle", "deg", ref=0.0),
        svg_line(rows, "distance_m", "brake_cmd", "Brake command", "cmd", ref=0.0),
    ])

    doc = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Race Telemetry</title>
<style>
:root {{
  color-scheme: light dark;
  --bg:#f5f7f9; --card:#ffffff; --text:#20252b; --muted:#68707a; --border:#d9dee5;
}}
@media (prefers-color-scheme: dark) {{
  :root {{ --bg:#171a1f; --card:#20252b; --text:#edf0f3; --muted:#aab2bc; --border:#3a414a; }}
}}
* {{ box-sizing:border-box; }}
body {{ margin:0; font-family:system-ui,-apple-system,Segoe UI,sans-serif; background:var(--bg); color:var(--text); }}
main {{ max-width:1400px; margin:auto; padding:24px; }}
h1 {{ margin:0 0 6px; }}
.sub {{ color:var(--muted); margin-bottom:20px; }}
.summary {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(170px,1fr)); gap:10px; margin-bottom:20px; }}
.stat {{ background:var(--card); border:1px solid var(--border); border-radius:12px; padding:12px; }}
.stat b {{ display:block; font-size:1.35rem; margin-top:3px; }}
.grid {{ display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:14px; }}
.card {{ background:var(--card); border:1px solid var(--border); border-radius:14px; padding:12px; min-width:0; }}
.card h2 {{ margin:2px 0 8px; font-size:1.05rem; }}
.track,.linechart {{ width:100%; height:auto; display:block; }}
.legend-note {{ color:var(--muted); font-size:.88rem; margin-top:6px; }}
.charts {{ margin-top:18px; display:grid; gap:12px; }}
@media(max-width:850px) {{ .grid {{ grid-template-columns:1fr; }} main {{ padding:12px; }} }}
</style>
</head>
<body>
<main>
<h1>Race telemetry</h1>
<div class="sub">Baseline speed: {base_speed_kmh:.1f} km/h · Hover track segments for values</div>

<div class="summary">
  <div class="stat">Duration<b>{duration:.2f} s</b></div>
  <div class="stat">Distance<b>{distance:.1f} m</b></div>
  <div class="stat">Average speed<b>{sum(speeds)/len(speeds):.1f} km/h</b></div>
  <div class="stat">Maximum speed<b>{max(speeds):.1f} km/h</b></div>
  <div class="stat">Minimum speed<b>{min(speeds):.1f} km/h</b></div>
  <div class="stat">Below baseline<b>{below:.1f}%</b></div>
  <div class="stat">Above baseline<b>{above:.1f}%</b></div>
  <div class="stat">Max deceleration<b>{min(accels):.2f} m/s²</b></div>
  <div class="stat">Max |steering|<b>{max(steers):.1f}°</b></div>
  <div class="stat">Max brake cmd<b>{max(brakes):.3f}</b></div>
  <div class="stat">Time loss vs {base_speed_kmh:.0f}<b>{time_loss:+.2f} s</b></div>
</div>

<div class="grid">
  <section class="card"><h2>Speed map</h2>{track_speed}</section>
  <section class="card"><h2>Acceleration / deceleration map</h2>{track_decel}</section>
  <section class="card"><h2>Steering map</h2>{track_steer}</section>
  <section class="card"><h2>Brake map</h2>{track_brake}</section>
</div>

<div class="charts">
  <section class="card">{charts}</section>
</div>
</main>
</body>
</html>
"""
    path.write_text(doc, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="ROS 2 race telemetry recorder and HTML analyzer")
    parser.add_argument("--base-speed", type=float, default=32.0, help="baseline speed in km/h (default: 32)")
    parser.add_argument("--sample-hz", type=float, default=20.0, help="sampling rate (default: 20 Hz)")
    parser.add_argument("--out", default="telemetry_runs", help="output root directory")
    args = parser.parse_args()

    if args.base_speed <= 0:
        raise SystemExit("--base-speed must be > 0")
    if args.sample_hz <= 0 or args.sample_hz > 100:
        raise SystemExit("--sample-hz must be in (0, 100]")

    run_dir = Path(args.out) / datetime.now().strftime("run_%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = RaceTelemetry(sample_hz=args.sample_hz)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rows = process_rows(node.rows, args.base_speed)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if len(rows) < 10:
        print("Not enough samples were recorded. Check ROS_DOMAIN_ID and topic availability.")
        return 1

    csv_path = run_dir / "telemetry.csv"
    html_path = run_dir / "telemetry.html"
    save_csv(rows, csv_path)
    generate_html(rows, html_path, args.base_speed)

    print()
    print(f"Saved CSV : {csv_path.resolve()}")
    print(f"Saved HTML: {html_path.resolve()}")
    print(f"Samples   : {len(rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
