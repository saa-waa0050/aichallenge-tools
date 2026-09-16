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


BLUE = (37, 99, 235)
CYAN = (6, 182, 212)
NEUTRAL = (148, 163, 184)
RED = (220, 38, 38)
ORANGE = (234, 88, 12)
YELLOW = (245, 158, 11)
PURPLE = (126, 34, 206)
GREEN = (22, 163, 74)
INDIGO = (49, 46, 129)
DARK = (17, 24, 39)


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


def piecewise_color(value, stops):
    """Interpolate a value across [(value, rgb_tuple), ...]."""
    if value <= stops[0][0]:
        return rgb(stops[0][1])
    if value >= stops[-1][0]:
        return rgb(stops[-1][1])

    for (v0, c0), (v1, c1) in zip(stops[:-1], stops[1:]):
        if v0 <= value <= v1:
            if abs(v1 - v0) < 1e-12:
                return rgb(c1)
            t = (value - v0) / (v1 - v0)
            return mix(c0, c1, t)

    return rgb(stops[-1][1])


def css_rgb(c):
    return f"rgb({c[0]},{c[1]},{c[2]})"


def colorbar(stops, labels, note=""):
    gradient = ", ".join(
        f"{css_rgb(c)} {100.0 * i / max(1, len(stops) - 1):.1f}%"
        for i, c in enumerate(stops)
    )
    ticks = "".join(f"<span>{html.escape(str(label))}</span>" for label in labels)
    note_html = f'<div class="legend-note">{html.escape(note)}</div>' if note else ""
    return (
        '<div class="colorbar-wrap">'
        f'<div class="colorbar-gradient" style="background:linear-gradient(90deg,{gradient})"></div>'
        f'<div class="colorbar-ticks">{ticks}</div>'
        f'{note_html}'
        '</div>'
    )


def svg_track(rows, metric, title, color_fn, legend_html):
    p, width, height = norm_track(rows)
    pieces = [
        f'<svg class="track" viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)}">'
    ]

    # Thick neutral underlay: keeps the course visible even for pale metric colors.
    pts = " ".join(
        f"{p(r['x_m'], r['y_m'])[0]:.1f},{p(r['x_m'], r['y_m'])[1]:.1f}"
        for r in rows
    )
    pieces.append(
        f'<polyline points="{pts}" fill="none" stroke="#cbd5e1" stroke-width="10" '
        'stroke-linecap="round" stroke-linejoin="round"/>'
    )
    pieces.append(
        f'<polyline points="{pts}" fill="none" stroke="#ffffff" stroke-width="7" '
        'stroke-linecap="round" stroke-linejoin="round"/>'
    )

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
            f'stroke="{c}" stroke-width="6.5" stroke-linecap="round">'
            f'<title>{html.escape(tip)}</title></line>'
        )

    pieces.append("</svg>")
    pieces.append(legend_html)
    return "\n".join(pieces)


def svg_line(rows, xkey, ykey, title, y_label, ref=None, stroke="#0057d9"):
    width, height, margin_l, margin_r, margin_t, margin_b = 1000, 300, 76, 28, 30, 48
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
        f'<rect x="{margin_l}" y="{margin_t}" width="{width-margin_l-margin_r}" '
        f'height="{height-margin_t-margin_b}" fill="#f8fafc" rx="6"/>',
    ]

    # Horizontal grid.
    for j in range(5):
        val = ymin + (ymax - ymin) * j / 4.0
        y = sy(val)
        out.append(
            f'<line x1="{margin_l}" y1="{y:.1f}" x2="{width-margin_r}" y2="{y:.1f}" '
            'stroke="#dbe3ec" stroke-width="1"/>'
        )
        out.append(
            f'<text x="{margin_l-8}" y="{y+4:.1f}" text-anchor="end" '
            f'font-size="12" fill="#334155">{val:.1f}</text>'
        )

    # Reference line (baseline / zero).
    if ref is not None and ymin <= ref <= ymax:
        y = sy(ref)
        out.append(
            f'<line x1="{margin_l}" y1="{y:.1f}" x2="{width-margin_r}" y2="{y:.1f}" '
            'stroke="#dc2626" stroke-width="2" stroke-dasharray="8 6"/>'
        )
        out.append(
            f'<text x="{width-margin_r-4}" y="{y-7:.1f}" text-anchor="end" '
            f'font-size="12" font-weight="700" fill="#b91c1c">{ref:.1f}</text>'
        )

    out.append(
        f'<polyline points="{pts}" fill="none" stroke="{stroke}" '
        'stroke-width="3" stroke-linejoin="round" stroke-linecap="round"/>'
    )
    out.append(
        f'<text x="{margin_l}" y="20" font-size="16" font-weight="700" fill="#111827">'
        f'{html.escape(title)}</text>'
    )
    out.append(
        f'<text x="10" y="{margin_t+13}" font-size="12" fill="#475569">{html.escape(y_label)}</text>'
    )
    out.append(
        f'<text x="{width-150}" y="{height-12}" font-size="12" fill="#475569">distance [m]</text>'
    )
    out.append("</svg>")
    return "\n".join(out)


def generate_html(rows, path, base_speed_kmh):
    # High-contrast speed scale. With the default 32 km/h baseline this becomes:
    # 20 / 24 / 28 / 32 / 36 km/h.
    speed_ticks = [
        max(0.0, base_speed_kmh - 12.0),
        max(0.0, base_speed_kmh - 8.0),
        max(0.0, base_speed_kmh - 4.0),
        base_speed_kmh,
        base_speed_kmh + 4.0,
    ]
    speed_colors = [
        (30, 58, 138),   # deep blue
        (37, 99, 235),   # blue
        (6, 182, 212),   # cyan
        (22, 163, 74),   # green = baseline
        (220, 38, 38),   # red
    ]
    speed_stops = list(zip(speed_ticks, speed_colors))

    neg_scale = max(
        0.5,
        percentile([max(0.0, -r["accel_mps2"]) for r in rows], 0.98, 0.5),
    )
    pos_scale = max(
        0.5,
        percentile([max(0.0, r["accel_mps2"]) for r in rows], 0.98, 0.5),
    )
    steer_scale = max(
        3.0,
        percentile([abs(r["steering_deg"]) for r in rows], 0.98, 3.0),
    )
    brake_scale = max(
        0.05,
        percentile([max(0.0, r["brake_cmd"]) for r in rows], 0.98, 0.05),
    )

    def speed_color(v):
        return piecewise_color(v, speed_stops)

    def accel_color(a):
        if a < 0.0:
            # zero -> medium gray; stronger decel -> vivid red
            return mix((148, 163, 184), RED, min((-a) / neg_scale, 1.0))
        if a > 0.0:
            # zero -> medium gray; stronger accel -> vivid green
            return mix((148, 163, 184), GREEN, min(a / pos_scale, 1.0))
        return rgb((148, 163, 184))

    def steer_color(deg):
        # Keep even near-zero steering visibly purple rather than near-white.
        t = clamp(abs(deg) / steer_scale, 0.0, 1.0)
        return mix((196, 181, 253), (88, 28, 135), t)

    def brake_color(v):
        t = clamp(max(0.0, v) / brake_scale, 0.0, 1.0)
        return mix((203, 213, 225), ORANGE, t)

    duration = rows[-1]["t_s"] - rows[0]["t_s"]
    distance = rows[-1]["distance_m"]
    speeds = [r["speed_kmh"] for r in rows]
    accels = [r["accel_mps2"] for r in rows]
    steers = [abs(r["steering_deg"]) for r in rows]
    brakes = [max(0.0, r["brake_cmd"]) for r in rows]
    below = sum(1 for r in rows if r["speed_kmh"] < base_speed_kmh) / len(rows) * 100.0
    above = sum(1 for r in rows if r["speed_kmh"] > base_speed_kmh) / len(rows) * 100.0
    time_loss = sum(r["time_loss_vs_base_s"] for r in rows)

    speed_legend = colorbar(
        speed_colors,
        [f"{v:.0f}" for v in speed_ticks],
        f"Speed [km/h]. Green marks the {base_speed_kmh:.0f} km/h baseline.",
    )
    accel_legend = colorbar(
        [RED, (148, 163, 184), GREEN],
        [f"-{neg_scale:.1f}", "0", f"+{pos_scale:.1f}"],
        "Longitudinal acceleration [m/s²]: red = deceleration, green = acceleration.",
    )
    steer_legend = colorbar(
        [(196, 181, 253), (126, 34, 206), (88, 28, 135)],
        ["0°", f"{steer_scale/2:.1f}°", f"{steer_scale:.1f}°"],
        f"Absolute steering angle. Upper scale is the 98th percentile ({steer_scale:.1f}°).",
    )

    no_brake = max(brakes) < 1e-6
    if no_brake:
        brake_legend = (
            '<div class="no-data">No brake command detected in this run '
            '(brake_cmd stayed at 0).</div>'
        )
    else:
        brake_legend = colorbar(
            [(203, 213, 225), YELLOW, ORANGE],
            ["0", f"{brake_scale/2:.3f}", f"{brake_scale:.3f}"],
            f"Brake command. Upper scale is the 98th percentile ({brake_scale:.3f}).",
        )

    track_speed = svg_track(
        rows, "speed_kmh", "Speed", speed_color, speed_legend
    )
    track_decel = svg_track(
        rows, "accel_mps2", "Longitudinal acceleration", accel_color, accel_legend
    )
    track_steer = svg_track(
        rows, "steering_deg", "Steering intensity", steer_color, steer_legend
    )
    track_brake = svg_track(
        rows, "brake_cmd", "Brake command", brake_color, brake_legend
    )

    charts = "\n".join([
        svg_line(
            rows, "distance_m", "speed_kmh",
            "Speed", "km/h", ref=base_speed_kmh, stroke="#0057d9"
        ),
        svg_line(
            rows, "distance_m", "accel_mps2",
            "Longitudinal acceleration", "m/s²", ref=0.0, stroke="#0a7f3f"
        ),
        svg_line(
            rows, "distance_m", "steering_deg",
            "Steering tire angle", "deg", ref=0.0, stroke="#7e22ce"
        ),
        svg_line(
            rows, "distance_m", "brake_cmd",
            "Brake command", "cmd", ref=0.0, stroke="#ea580c"
        ),
    ])

    doc = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Race Telemetry</title>
<style>
:root {{
  color-scheme: light;
  --bg:#eef2f6;
  --card:#ffffff;
  --text:#111827;
  --muted:#475569;
  --border:#cbd5e1;
}}
* {{ box-sizing:border-box; }}
body {{
  margin:0;
  font-family:system-ui,-apple-system,"Segoe UI",sans-serif;
  background:var(--bg);
  color:var(--text);
}}
main {{ max-width:1440px; margin:auto; padding:24px; }}
h1 {{ margin:0 0 6px; color:#0f172a; }}
.sub {{ color:var(--muted); margin-bottom:20px; font-weight:600; }}
.summary {{
  display:grid;
  grid-template-columns:repeat(auto-fit,minmax(170px,1fr));
  gap:10px;
  margin-bottom:20px;
}}
.stat {{
  background:var(--card);
  border:1px solid var(--border);
  border-radius:12px;
  padding:12px;
}}
.stat b {{
  display:block;
  font-size:1.35rem;
  margin-top:3px;
  color:#0f172a;
}}
.grid {{
  display:grid;
  grid-template-columns:repeat(2,minmax(0,1fr));
  gap:14px;
}}
.card {{
  background:var(--card);
  border:1px solid var(--border);
  border-radius:14px;
  padding:14px;
  min-width:0;
}}
.card h2 {{
  margin:2px 0 10px;
  font-size:1.08rem;
  color:#0f172a;
}}
.track,.linechart {{ width:100%; height:auto; display:block; }}
.legend-note {{ color:#334155; font-size:.88rem; margin-top:7px; }}
.colorbar-wrap {{ margin:8px 8px 2px; }}
.colorbar-gradient {{
  height:18px;
  border-radius:5px;
  border:1px solid #64748b;
}}
.colorbar-ticks {{
  display:flex;
  justify-content:space-between;
  gap:8px;
  margin-top:4px;
  color:#1e293b;
  font-size:.82rem;
  font-weight:700;
}}
.no-data {{
  margin:9px 8px 2px;
  padding:10px 12px;
  border:1px solid #f59e0b;
  border-radius:8px;
  background:#fffbeb;
  color:#92400e;
  font-weight:700;
}}
.charts {{ margin-top:18px; display:grid; gap:12px; }}
@media(max-width:850px) {{
  .grid {{ grid-template-columns:1fr; }}
  main {{ padding:12px; }}
}}
</style>
</head>
<body>
<main>
<h1>Race telemetry</h1>
<div class="sub">
  Baseline speed: {base_speed_kmh:.1f} km/h · Hover track segments for exact values
</div>

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
