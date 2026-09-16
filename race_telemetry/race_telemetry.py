#!/usr/bin/env python3
import argparse
import csv
import html
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message

from nav_msgs.msg import Odometry
from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
from tier4_vehicle_msgs.msg import ActuationCommandStamped


# ============================================================
# 基本ユーティリティ
# ============================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def lerp(a, b, t):
    return a + (b - a) * t


def lerp_int(a, b, t):
    return int(round(lerp(a, b, t)))


def rgb(c):
    return f"rgb({c[0]},{c[1]},{c[2]})"


def mix(c1, c2, t):
    t = clamp(t, 0.0, 1.0)
    return rgb(tuple(lerp_int(a, b, t) for a, b in zip(c1, c2)))


BLUE = (37, 99, 235)
CYAN = (6, 182, 212)
NEUTRAL = (148, 163, 184)
RED = (220, 38, 38)
ORANGE = (234, 88, 12)
YELLOW = (245, 158, 11)
PURPLE = (126, 34, 206)
GREEN = (22, 163, 74)
DARK_PURPLE = (88, 28, 135)


def percentile(values, p, default=0.0):
    vals = sorted(v for v in values if isinstance(v, (int, float)) and math.isfinite(v))
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


def moving_average(values, n=7):
    if n <= 1 or len(values) < 2:
        return list(values)
    out = []
    half = n // 2
    for i in range(len(values)):
        a = max(0, i - half)
        b = min(len(values), i + half + 1)
        out.append(sum(values[a:b]) / (b - a))
    return out


def safe_mean(values, default=0.0):
    vals = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
    return sum(vals) / len(vals) if vals else default


# ============================================================
# ROS 2 収集
# ============================================================

class RaceTelemetry(Node):
    def __init__(self, sample_hz=20.0):
        super().__init__("race_telemetry")

        self.qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 実車両状態
        self.x = None
        self.y = None
        self.speed_mps = None
        self.steer_rad = None

        # tier4_vehicle_msgs/ActuationCommandStamped
        self.actuation_accel_cmd = 0.0
        self.actuation_brake_cmd = 0.0
        self.actuation_seen = False

        # /control/command/control_cmd
        # 型は環境差を吸収するためROS graphから動的取得する。
        self.control_speed_mps = 0.0
        self.control_accel_mps2 = 0.0
        self.control_steer_rad = 0.0
        self.control_seen = False
        self.control_sub = None
        self.control_type_name = ""

        self.got_odom = False
        self.got_velocity = False
        self.rows = []
        self.t0 = time.monotonic()

        self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.odom_cb,
            self.qos,
        )
        self.create_subscription(
            VelocityReport,
            "/vehicle/status/velocity_status",
            self.velocity_cb,
            self.qos,
        )
        self.create_subscription(
            SteeringReport,
            "/vehicle/status/steering_status",
            self.steer_cb,
            self.qos,
        )
        self.create_subscription(
            ActuationCommandStamped,
            "/control/command/actuation_cmd",
            self.actuation_cb,
            self.qos,
        )

        # control_cmdの型を1秒ごとに探し、見つかった時点で購読する。
        self.create_timer(1.0, self.ensure_control_subscription)
        self.create_timer(1.0 / sample_hz, self.sample)

        self.get_logger().info(
            "テレメトリ記録を開始しました。シミュレータを通常どおり走行させ、"
            "終了後にこのターミナルで Ctrl+C を押してください。"
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
        self.actuation_accel_cmd = float(getattr(act, "accel_cmd", 0.0))
        self.actuation_brake_cmd = float(getattr(act, "brake_cmd", 0.0))
        self.actuation_seen = True

    def ensure_control_subscription(self):
        if self.control_sub is not None:
            return

        try:
            topic_map = dict(self.get_topic_names_and_types())
            types = topic_map.get("/control/command/control_cmd", [])
            if not types:
                return

            msg_type_name = types[0]
            msg_cls = get_message(msg_type_name)
            self.control_sub = self.create_subscription(
                msg_cls,
                "/control/command/control_cmd",
                self.control_cb,
                self.qos,
            )
            self.control_type_name = msg_type_name
            self.get_logger().info(
                f"/control/command/control_cmd を購読しました ({msg_type_name})"
            )
        except Exception as e:
            # 競技環境の起動途中では型解決に失敗することがあるので、
            # timerで次回再試行する。
            self.get_logger().debug(f"control_cmd購読待機中: {e}")

    def control_cb(self, msg):
        lon = getattr(msg, "longitudinal", None)
        lat = getattr(msg, "lateral", None)

        if lon is not None:
            # AckermannControlCommand系はspeed、新Control系はvelocityの場合がある。
            if hasattr(lon, "speed"):
                self.control_speed_mps = float(lon.speed)
            elif hasattr(lon, "velocity"):
                self.control_speed_mps = float(lon.velocity)

            if hasattr(lon, "acceleration"):
                self.control_accel_mps2 = float(lon.acceleration)

        if lat is not None and hasattr(lat, "steering_tire_angle"):
            self.control_steer_rad = float(lat.steering_tire_angle)

        self.control_seen = True

    def sample(self):
        if not (self.got_odom and self.got_velocity):
            return

        self.rows.append(
            {
                "t_s": time.monotonic() - self.t0,
                "x_m": float(self.x),
                "y_m": float(self.y),
                "speed_mps": abs(float(self.speed_mps)),
                "steering_rad": float(self.steer_rad or 0.0),

                "actuation_accel_cmd": float(self.actuation_accel_cmd),
                "actuation_brake_cmd": float(self.actuation_brake_cmd),
                "actuation_seen": 1 if self.actuation_seen else 0,

                "control_speed_mps": float(self.control_speed_mps),
                "control_accel_mps2": float(self.control_accel_mps2),
                "control_steer_rad": float(self.control_steer_rad),
                "control_seen": 1 if self.control_seen else 0,
            }
        )


# ============================================================
# 前処理
# ============================================================

def process_rows(rows, base_speed_kmh):
    if not rows:
        return []

    # 完全停止中の同一点サンプルのみ削除。
    # 走行開始前後のログは残すが、周回解析では移動開始点から切り出す。
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
        r["sample_index"] = i
        r["lap"] = 0

        r["speed_kmh"] = r["speed_mps"] * 3.6
        r["steering_deg"] = math.degrees(r["steering_rad"])

        r["control_speed_kmh"] = r["control_speed_mps"] * 3.6
        r["control_steer_deg"] = math.degrees(r["control_steer_rad"])

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

        # reset / localization jumpを総距離へ混ぜない。
        if ds > 5.0:
            ds = 0.0

        distance += ds
        r["distance_m"] = distance

        if 0 < i < len(clean) - 1:
            dt2 = max(1e-4, clean[i + 1]["t_s"] - clean[i - 1]["t_s"])
            accel = (smooth_speed[i + 1] - smooth_speed[i - 1]) / dt2
        else:
            accel = (smooth_speed[i] - smooth_speed[i - 1]) / dt
        r["accel_mps2"] = accel

        base_mps = base_speed_kmh / 3.6
        actual_mps = max(0.5, r["speed_mps"])
        r["time_loss_vs_base_s"] = (
            ds / actual_mps - ds / base_mps if ds > 0.0 else 0.0
        )

    return clean


# ============================================================
# 周回検出
# ============================================================

def _find_heading_point(rows, start_idx, min_forward_distance=5.0):
    x0 = rows[start_idx]["x_m"]
    y0 = rows[start_idx]["y_m"]

    for i in range(start_idx + 1, len(rows)):
        dx = rows[i]["x_m"] - x0
        dy = rows[i]["y_m"] - y0
        if math.hypot(dx, dy) >= min_forward_distance:
            n = math.hypot(dx, dy)
            return i, dx / n, dy / n

    return None, None, None


def detect_laps(
    rows,
    start_speed_kmh=3.0,
    min_lap_distance_m=80.0,
    gate_half_width_m=10.0,
):
    """
    移動開始点をスタートライン中心とし、最初の数mの進行方向に直交する
    仮想スタートラインを作る。同じ向きでラインを再通過したところをラップ境界とする。

    戻り値:
      laps: 完走ラップの行リスト
      info: 検出情報
    """
    if len(rows) < 20:
        return [], {"method": "none", "reason": "サンプル不足"}

    start_idx = None
    for i, r in enumerate(rows):
        if r["speed_kmh"] >= start_speed_kmh:
            start_idx = i
            break

    if start_idx is None:
        return [], {"method": "none", "reason": "走行開始を検出できませんでした"}

    heading_idx, hx, hy = _find_heading_point(rows, start_idx)
    if heading_idx is None:
        return [], {"method": "none", "reason": "スタート方向を推定できませんでした"}

    x0 = rows[start_idx]["x_m"]
    y0 = rows[start_idx]["y_m"]
    d0 = rows[start_idx]["distance_m"]
    t0 = rows[start_idx]["t_s"]

    def coords(r):
        dx = r["x_m"] - x0
        dy = r["y_m"] - y0
        along = dx * hx + dy * hy
        lateral = -dx * hy + dy * hx
        return along, lateral

    crossings = []
    last_boundary_idx = start_idx
    last_boundary_distance = d0
    last_boundary_time = t0

    # まず仮想スタートラインの同方向通過で検出。
    for i in range(max(start_idx + 2, heading_idx), len(rows)):
        prev = rows[i - 1]
        cur = rows[i]
        prev_along, _ = coords(prev)
        cur_along, cur_lat = coords(cur)

        seg_dx = cur["x_m"] - prev["x_m"]
        seg_dy = cur["y_m"] - prev["y_m"]
        forward_motion = seg_dx * hx + seg_dy * hy

        traveled = cur["distance_m"] - last_boundary_distance
        elapsed = cur["t_s"] - last_boundary_time

        if (
            prev_along < 0.0 <= cur_along
            and abs(cur_lat) <= gate_half_width_m
            and forward_motion > 0.0
            and traveled >= min_lap_distance_m
            and elapsed >= 8.0
        ):
            crossings.append(i)
            last_boundary_idx = i
            last_boundary_distance = cur["distance_m"]
            last_boundary_time = cur["t_s"]

    method = "start_line"

    # ライン交差が取れなかった場合のみ、スタート近傍への再進入でフォールバック。
    if not crossings:
        inner_radius = min(7.0, max(4.0, gate_half_width_m * 0.7))
        outer_radius = max(inner_radius + 4.0, gate_half_width_m + 3.0)

        outside = False
        last_boundary_distance = d0
        last_boundary_time = t0

        for i in range(start_idx + 1, len(rows)):
            cur = rows[i]
            dist_start = math.hypot(cur["x_m"] - x0, cur["y_m"] - y0)
            traveled = cur["distance_m"] - last_boundary_distance
            elapsed = cur["t_s"] - last_boundary_time

            if dist_start >= outer_radius:
                outside = True

            if (
                outside
                and dist_start <= inner_radius
                and traveled >= min_lap_distance_m
                and elapsed >= 8.0
            ):
                # 進行方向が概ねスタート時と同方向か確認。
                if i > 0:
                    seg_dx = cur["x_m"] - rows[i - 1]["x_m"]
                    seg_dy = cur["y_m"] - rows[i - 1]["y_m"]
                    if seg_dx * hx + seg_dy * hy <= 0.0:
                        continue

                crossings.append(i)
                last_boundary_distance = cur["distance_m"]
                last_boundary_time = cur["t_s"]
                outside = False

        method = "start_radius"

    if not crossings:
        return [], {
            "method": "none",
            "reason": "完走ラップのスタート再通過を検出できませんでした",
            "start_idx": start_idx,
        }

    boundaries = [start_idx] + crossings
    laps = []

    for lap_no in range(1, len(boundaries)):
        a = boundaries[lap_no - 1]
        b = boundaries[lap_no]

        lap_rows = []
        d_base = rows[a]["distance_m"]
        t_base = rows[a]["t_s"]

        for r in rows[a:b + 1]:
            q = dict(r)
            q["lap"] = lap_no
            q["distance_m"] = max(0.0, r["distance_m"] - d_base)
            q["t_s"] = max(0.0, r["t_s"] - t_base)
            lap_rows.append(q)
            rows[r["sample_index"]]["lap"] = lap_no

        if len(lap_rows) >= 10 and lap_rows[-1]["distance_m"] >= min_lap_distance_m:
            laps.append(lap_rows)

    tail_start = crossings[-1] if crossings else start_idx
    tail_distance = rows[-1]["distance_m"] - rows[tail_start]["distance_m"]

    return laps, {
        "method": method,
        "start_idx": start_idx,
        "crossings": crossings,
        "incomplete_tail_distance_m": max(0.0, tail_distance),
        "gate_half_width_m": gate_half_width_m,
        "min_lap_distance_m": min_lap_distance_m,
    }


# ============================================================
# 全周平均
# ============================================================

AVERAGE_KEYS = [
    "x_m",
    "y_m",
    "speed_mps",
    "speed_kmh",
    "accel_mps2",
    "steering_rad",
    "steering_deg",
    "actuation_accel_cmd",
    "actuation_brake_cmd",
    "control_speed_mps",
    "control_speed_kmh",
    "control_accel_mps2",
    "control_steer_rad",
    "control_steer_deg",
]


def interp_by_distance(rows, target_d, key):
    if target_d <= rows[0]["distance_m"]:
        return float(rows[0].get(key, 0.0))
    if target_d >= rows[-1]["distance_m"]:
        return float(rows[-1].get(key, 0.0))

    lo = 0
    hi = len(rows) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if rows[mid]["distance_m"] < target_d:
            lo = mid
        else:
            hi = mid

    a = rows[lo]
    b = rows[hi]
    da = a["distance_m"]
    db = b["distance_m"]
    if db <= da + 1e-9:
        return float(b.get(key, 0.0))

    t = (target_d - da) / (db - da)
    return lerp(float(a.get(key, 0.0)), float(b.get(key, 0.0)), t)


def average_laps(laps, base_speed_kmh, points=320):
    if not laps:
        return []

    points = max(80, min(1000, int(points)))
    mean_length = safe_mean([lap[-1]["distance_m"] for lap in laps], 0.0)
    mean_time = safe_mean([lap[-1]["t_s"] for lap in laps], 0.0)

    any_control = any(any(r.get("control_seen", 0) for r in lap) for lap in laps)
    any_actuation = any(any(r.get("actuation_seen", 0) for r in lap) for lap in laps)

    out = []
    for j in range(points):
        progress = j / (points - 1)
        q = {
            "sample_index": j,
            "lap": -1,
            "distance_m": mean_length * progress,
            "t_s": mean_time * progress,
            "control_seen": 1 if any_control else 0,
            "actuation_seen": 1 if any_actuation else 0,
        }

        for key in AVERAGE_KEYS:
            vals = []
            for lap in laps:
                target = lap[-1]["distance_m"] * progress
                vals.append(interp_by_distance(lap, target, key))
            q[key] = safe_mean(vals, 0.0)

        out.append(q)

    # 平均速度から基準速度比タイムロスを再計算。
    base_mps = base_speed_kmh / 3.6
    for i, r in enumerate(out):
        if i == 0:
            r["time_loss_vs_base_s"] = 0.0
            continue
        ds = r["distance_m"] - out[i - 1]["distance_m"]
        actual_mps = max(0.5, r["speed_mps"])
        r["time_loss_vs_base_s"] = ds / actual_mps - ds / base_mps

    return out


# ============================================================
# CSV
# ============================================================

CSV_FIELDS = [
    "sample_index",
    "lap",
    "t_s",
    "distance_m",
    "x_m",
    "y_m",
    "speed_mps",
    "speed_kmh",
    "accel_mps2",
    "steering_rad",
    "steering_deg",
    "control_seen",
    "control_speed_mps",
    "control_speed_kmh",
    "control_accel_mps2",
    "control_steer_rad",
    "control_steer_deg",
    "actuation_seen",
    "actuation_accel_cmd",
    "actuation_brake_cmd",
    "time_loss_vs_base_s",
]


def save_csv(rows, path):
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, 0.0) for k in CSV_FIELDS})


# ============================================================
# SVG / HTML
# ============================================================

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


def tooltip_text(r, title, value):
    return (
        f"{title}: {value:.3f} | "
        f"速度 {r['speed_kmh']:.1f} km/h | "
        f"実加速度 {r['accel_mps2']:+.2f} m/s² | "
        f"実操舵 {r['steering_deg']:+.1f}° | "
        f"制御目標速度 {r.get('control_speed_kmh', 0.0):.1f} km/h | "
        f"制御加速度指令 {r.get('control_accel_mps2', 0.0):+.2f} m/s² | "
        f"actuation brake_cmd {r.get('actuation_brake_cmd', 0.0):.3f}"
    )


def svg_track(rows, metric, title, color_fn, legend_html):
    p, width, height = norm_track(rows)
    pieces = [
        f'<svg class="track" viewBox="0 0 {width} {height}" '
        f'role="img" aria-label="{html.escape(title)}">'
    ]

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
        v = float(b.get(metric, 0.0))
        c = color_fn(v)
        tip = tooltip_text(b, title, v)
        pieces.append(
            f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
            f'stroke="{c}" stroke-width="6.5" stroke-linecap="round">'
            f'<title>{html.escape(tip)}</title></line>'
        )

    pieces.append("</svg>")
    pieces.append(legend_html)
    return "\n".join(pieces)


def svg_line_multi(rows, series, title, y_label, ref=None):
    """
    series = [(key, label, color), ...]
    """
    width, height = 1000, 320
    margin_l, margin_r, margin_t, margin_b = 78, 28, 42, 50

    xs = [r["distance_m"] for r in rows]
    ys = []
    active_series = []

    for key, label, color in series:
        vals = [float(r.get(key, 0.0)) for r in rows]
        if vals:
            ys.extend(vals)
            active_series.append((key, label, color))

    if ref is not None:
        ys.append(ref)

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

    out = [
        f'<svg class="linechart" viewBox="0 0 {width} {height}" '
        f'role="img" aria-label="{html.escape(title)}">',
        f'<rect x="{margin_l}" y="{margin_t}" width="{width-margin_l-margin_r}" '
        f'height="{height-margin_t-margin_b}" fill="#f8fafc" rx="6"/>',
    ]

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

    if ref is not None and ymin <= ref <= ymax:
        y = sy(ref)
        out.append(
            f'<line x1="{margin_l}" y1="{y:.1f}" x2="{width-margin_r}" y2="{y:.1f}" '
            'stroke="#64748b" stroke-width="1.7" stroke-dasharray="8 6"/>'
        )
        out.append(
            f'<text x="{width-margin_r-4}" y="{y-7:.1f}" text-anchor="end" '
            f'font-size="12" font-weight="700" fill="#475569">{ref:.1f}</text>'
        )

    for key, label, color in active_series:
        pts = " ".join(
            f"{sx(r['distance_m']):.1f},{sy(float(r.get(key, 0.0))):.1f}"
            for r in rows
        )
        out.append(
            f'<polyline points="{pts}" fill="none" stroke="{color}" '
            'stroke-width="3" stroke-linejoin="round" stroke-linecap="round"/>'
        )

    out.append(
        f'<text x="{margin_l}" y="21" font-size="16" font-weight="700" fill="#111827">'
        f'{html.escape(title)}</text>'
    )
    out.append(
        f'<text x="10" y="{margin_t+13}" font-size="12" fill="#475569">'
        f'{html.escape(y_label)}</text>'
    )
    out.append(
        f'<text x="{width-150}" y="{height-12}" font-size="12" fill="#475569">'
        '距離 [m]</text>'
    )

    # 凡例
    legend_x = margin_l + 200
    for i, (_, label, color) in enumerate(active_series):
        x = legend_x + i * 190
        out.append(
            f'<line x1="{x}" y1="18" x2="{x+28}" y2="18" '
            f'stroke="{color}" stroke-width="4"/>'
        )
        out.append(
            f'<text x="{x+35}" y="22" font-size="12" fill="#334155">'
            f'{html.escape(label)}</text>'
        )

    out.append("</svg>")
    return "\n".join(out)


def build_scales(reference_rows, base_speed_kmh):
    speed_ticks = [
        max(0.0, base_speed_kmh - 12.0),
        max(0.0, base_speed_kmh - 8.0),
        max(0.0, base_speed_kmh - 4.0),
        base_speed_kmh,
        base_speed_kmh + 4.0,
    ]
    speed_colors = [
        (30, 58, 138),
        BLUE,
        CYAN,
        GREEN,
        RED,
    ]
    speed_stops = list(zip(speed_ticks, speed_colors))

    actual_neg = max(
        0.5,
        percentile([max(0.0, -r["accel_mps2"]) for r in reference_rows], 0.98, 0.5),
    )
    actual_pos = max(
        0.5,
        percentile([max(0.0, r["accel_mps2"]) for r in reference_rows], 0.98, 0.5),
    )
    steer_scale = max(
        3.0,
        percentile([abs(r["steering_deg"]) for r in reference_rows], 0.98, 3.0),
    )
    ctrl_neg = max(
        0.5,
        percentile(
            [max(0.0, -r.get("control_accel_mps2", 0.0)) for r in reference_rows],
            0.98,
            0.5,
        ),
    )
    ctrl_pos = max(
        0.5,
        percentile(
            [max(0.0, r.get("control_accel_mps2", 0.0)) for r in reference_rows],
            0.98,
            0.5,
        ),
    )
    brake_scale = max(
        0.05,
        percentile(
            [max(0.0, r.get("actuation_brake_cmd", 0.0)) for r in reference_rows],
            0.98,
            0.05,
        ),
    )

    return {
        "speed_ticks": speed_ticks,
        "speed_colors": speed_colors,
        "speed_stops": speed_stops,
        "actual_neg": actual_neg,
        "actual_pos": actual_pos,
        "steer_scale": steer_scale,
        "ctrl_neg": ctrl_neg,
        "ctrl_pos": ctrl_pos,
        "brake_scale": brake_scale,
    }


def view_summary(rows, base_speed_kmh):
    duration = rows[-1]["t_s"] - rows[0]["t_s"]
    distance = rows[-1]["distance_m"] - rows[0]["distance_m"]
    speeds = [r["speed_kmh"] for r in rows]
    accels = [r["accel_mps2"] for r in rows]
    steers = [abs(r["steering_deg"]) for r in rows]
    ctrl_accels = [
        r.get("control_accel_mps2", 0.0)
        for r in rows
        if r.get("control_seen", 0)
    ]
    brakes = [
        max(0.0, r.get("actuation_brake_cmd", 0.0))
        for r in rows
        if r.get("actuation_seen", 0)
    ]

    below = sum(1 for r in rows if r["speed_kmh"] < base_speed_kmh) / len(rows) * 100.0
    above = sum(1 for r in rows if r["speed_kmh"] > base_speed_kmh) / len(rows) * 100.0
    time_loss = sum(r["time_loss_vs_base_s"] for r in rows)

    avg_speed = distance / max(duration, 1e-6) * 3.6

    return {
        "duration": duration,
        "distance": distance,
        "avg_speed": avg_speed,
        "max_speed": max(speeds),
        "min_speed": min(speeds),
        "below": below,
        "above": above,
        "max_decel": min(accels),
        "max_steer": max(steers),
        "min_ctrl_accel": min(ctrl_accels) if ctrl_accels else None,
        "max_brake": max(brakes) if brakes else None,
        "time_loss": time_loss,
        "control_seen": bool(ctrl_accels),
        "actuation_seen": bool(brakes) or any(r.get("actuation_seen", 0) for r in rows),
    }


def render_summary_cards(summary, base_speed_kmh):
    ctrl = (
        f"{summary['min_ctrl_accel']:+.2f} m/s²"
        if summary["min_ctrl_accel"] is not None
        else "未受信"
    )
    brake = (
        f"{summary['max_brake']:.3f}"
        if summary["max_brake"] is not None
        else "未受信"
    )

    return f"""
<div class="summary">
  <div class="stat">ラップタイム<b>{summary['duration']:.2f} s</b></div>
  <div class="stat">走行距離<b>{summary['distance']:.1f} m</b></div>
  <div class="stat">平均速度<b>{summary['avg_speed']:.1f} km/h</b></div>
  <div class="stat">最高速度<b>{summary['max_speed']:.1f} km/h</b></div>
  <div class="stat">最低速度<b>{summary['min_speed']:.1f} km/h</b></div>
  <div class="stat">{base_speed_kmh:.0f} km/h未満<b>{summary['below']:.1f}%</b></div>
  <div class="stat">最大実減速度<b>{summary['max_decel']:.2f} m/s²</b></div>
  <div class="stat">最大|実操舵角|<b>{summary['max_steer']:.1f}°</b></div>
  <div class="stat">最小制御加速度指令<b>{ctrl}</b></div>
  <div class="stat">最大actuation brake_cmd<b>{brake}</b></div>
  <div class="stat">{base_speed_kmh:.0f} km/h比タイム差<b>{summary['time_loss']:+.2f} s</b></div>
</div>
"""


def render_view(rows, label, base_speed_kmh, scales):
    speed_ticks = scales["speed_ticks"]
    speed_colors = scales["speed_colors"]

    def speed_color(v):
        return piecewise_color(v, scales["speed_stops"])

    def actual_accel_color(a):
        if a < 0.0:
            return mix(NEUTRAL, RED, min((-a) / scales["actual_neg"], 1.0))
        if a > 0.0:
            return mix(NEUTRAL, GREEN, min(a / scales["actual_pos"], 1.0))
        return rgb(NEUTRAL)

    def steer_color(deg):
        t = clamp(abs(deg) / scales["steer_scale"], 0.0, 1.0)
        return mix((196, 181, 253), DARK_PURPLE, t)

    def control_accel_color(a):
        if a < 0.0:
            return mix(NEUTRAL, RED, min((-a) / scales["ctrl_neg"], 1.0))
        if a > 0.0:
            return mix(NEUTRAL, GREEN, min(a / scales["ctrl_pos"], 1.0))
        return rgb(NEUTRAL)

    def brake_color(v):
        t = clamp(max(0.0, v) / scales["brake_scale"], 0.0, 1.0)
        return mix((203, 213, 225), ORANGE, t)

    speed_legend = colorbar(
        speed_colors,
        [f"{v:.0f}" for v in speed_ticks],
        f"速度 [km/h]。緑が基準速度 {base_speed_kmh:.0f} km/h。",
    )
    actual_accel_legend = colorbar(
        [RED, NEUTRAL, GREEN],
        [f"-{scales['actual_neg']:.1f}", "0", f"+{scales['actual_pos']:.1f}"],
        "実加速度 [m/s²]。赤=実減速、緑=実加速。",
    )
    steer_legend = colorbar(
        [(196, 181, 253), PURPLE, DARK_PURPLE],
        ["0°", f"{scales['steer_scale']/2:.1f}°", f"{scales['steer_scale']:.1f}°"],
        f"実ステア角の絶対値。上限表示は全ラップ98パーセンタイル "
        f"({scales['steer_scale']:.1f}°)。",
    )

    control_seen = any(r.get("control_seen", 0) for r in rows)
    actuation_seen = any(r.get("actuation_seen", 0) for r in rows)
    brake_max = max((r.get("actuation_brake_cmd", 0.0) for r in rows), default=0.0)

    if control_seen:
        control_accel_legend = colorbar(
            [RED, NEUTRAL, GREEN],
            [f"-{scales['ctrl_neg']:.1f}", "0", f"+{scales['ctrl_pos']:.1f}"],
            "control_cmd の longitudinal.acceleration [m/s²]。"
            "赤=減速指令、緑=加速指令。これは実加速度とは別。",
        )
    else:
        control_accel_legend = (
            '<div class="warning">/control/command/control_cmd を受信できなかったため、'
            '制御加速度指令は表示できません。</div>'
        )

    if not actuation_seen:
        brake_legend = (
            '<div class="warning">/control/command/actuation_cmd を受信できませんでした。</div>'
        )
    elif brake_max < 1e-6:
        brake_legend = (
            '<div class="info">'
            'この区間では <code>actuation_cmd.actuation.brake_cmd</code> は 0 のままでした。'
            'これは「車両が減速していない」「ブレーキ相当の制御が無い」という意味ではありません。'
            '実際の減速は「実加速度」、制御側の減速要求は「control_cmd 加速度指令」を確認してください。'
            '</div>'
        )
    else:
        brake_legend = colorbar(
            [(203, 213, 225), YELLOW, ORANGE],
            ["0", f"{scales['brake_scale']/2:.3f}", f"{scales['brake_scale']:.3f}"],
            "actuation_cmd.actuation.brake_cmd。車両制御経路の一つであり、"
            "減速全体を表す値ではありません。",
        )

    summary = view_summary(rows, base_speed_kmh)

    speed_map = svg_track(
        rows, "speed_kmh", "速度", speed_color, speed_legend
    )
    accel_map = svg_track(
        rows, "accel_mps2", "実加速度", actual_accel_color, actual_accel_legend
    )
    steer_map = svg_track(
        rows, "steering_deg", "実ステア角", steer_color, steer_legend
    )
    ctrl_map = svg_track(
        rows,
        "control_accel_mps2",
        "control_cmd 加速度指令",
        control_accel_color,
        control_accel_legend,
    )
    brake_map = svg_track(
        rows,
        "actuation_brake_cmd",
        "actuation brake_cmd",
        brake_color,
        brake_legend,
    )

    speed_series = [("speed_kmh", "実速度", "#0057d9")]
    if control_seen:
        speed_series.append(("control_speed_kmh", "制御目標速度", "#ea580c"))

    accel_series = [("accel_mps2", "実加速度", "#15803d")]
    if control_seen:
        accel_series.append(("control_accel_mps2", "制御加速度指令", "#dc2626"))

    steer_series = [("steering_deg", "実ステア角", "#7e22ce")]
    if control_seen:
        steer_series.append(("control_steer_deg", "制御ステア指令", "#ea580c"))

    charts = "\n".join([
        svg_line_multi(
            rows,
            speed_series,
            "速度",
            "km/h",
            ref=base_speed_kmh,
        ),
        svg_line_multi(
            rows,
            accel_series,
            "実加速度 / 制御加速度指令",
            "m/s²",
            ref=0.0,
        ),
        svg_line_multi(
            rows,
            steer_series,
            "ステアリング",
            "deg",
            ref=0.0,
        ),
        svg_line_multi(
            rows,
            [("actuation_brake_cmd", "actuation brake_cmd", "#ea580c")],
            "actuation brake_cmd",
            "cmd",
            ref=0.0,
        ),
    ])

    return f"""
<div class="view-title">{html.escape(label)}</div>
{render_summary_cards(summary, base_speed_kmh)}

<div class="grid">
  <section class="card"><h2>速度マップ</h2>{speed_map}</section>
  <section class="card"><h2>実加減速マップ</h2>{accel_map}</section>
  <section class="card"><h2>実ステアリングマップ</h2>{steer_map}</section>
  <section class="card"><h2>control_cmd 加速度指令マップ</h2>{ctrl_map}</section>
  <section class="card wide"><h2>actuation brake_cmd マップ</h2>{brake_map}</section>
</div>

<div class="charts">
  <section class="card">{charts}</section>
</div>
"""


def lap_table_html(laps, base_speed_kmh):
    if not laps:
        return ""

    summaries = [view_summary(lap, base_speed_kmh) for lap in laps]
    best_idx = min(range(len(summaries)), key=lambda i: summaries[i]["duration"])

    rows_html = []
    for i, s in enumerate(summaries):
        best = ' <span class="best">BEST</span>' if i == best_idx else ""
        ctrl = (
            f"{s['min_ctrl_accel']:+.2f}"
            if s["min_ctrl_accel"] is not None
            else "未受信"
        )
        brake = (
            f"{s['max_brake']:.3f}"
            if s["max_brake"] is not None
            else "未受信"
        )
        rows_html.append(
            "<tr>"
            f"<td>Lap {i+1}{best}</td>"
            f"<td>{s['duration']:.2f} s</td>"
            f"<td>{s['distance']:.1f} m</td>"
            f"<td>{s['avg_speed']:.1f} km/h</td>"
            f"<td>{s['max_speed']:.1f} km/h</td>"
            f"<td>{ctrl} m/s²</td>"
            f"<td>{brake}</td>"
            "</tr>"
        )

    return f"""
<section class="card lap-table-card">
  <h2>ラップ一覧</h2>
  <div class="table-scroll">
    <table class="lap-table">
      <thead>
        <tr>
          <th>ラップ</th>
          <th>タイム</th>
          <th>距離</th>
          <th>平均速度</th>
          <th>最高速度</th>
          <th>最小 control_cmd 加速度</th>
          <th>最大 actuation brake_cmd</th>
        </tr>
      </thead>
      <tbody>
        {''.join(rows_html)}
      </tbody>
    </table>
  </div>
</section>
"""


def generate_html(
    all_rows,
    laps,
    average_rows,
    path,
    base_speed_kmh,
    lap_info,
):
    # 色スケールは全完走ラップをまとめた値から作り、各Lap間で同じ色が同じ意味になるようにする。
    reference_rows = [r for lap in laps for r in lap] if laps else all_rows
    scales = build_scales(reference_rows, base_speed_kmh)

    method_text = {
        "start_line": "スタートライン再通過",
        "start_radius": "スタート近傍再進入（フォールバック）",
        "none": "未検出",
    }.get(lap_info.get("method"), str(lap_info.get("method", "未検出")))

    if laps:
        tab_buttons = [
            '<button class="tab active" type="button" data-target="average">全周平均</button>'
        ]
        panels = [
            '<section class="lap-panel active" data-panel="average">'
            + render_view(average_rows, f"全周平均（{len(laps)}周）", base_speed_kmh, scales)
            + "</section>"
        ]

        for i, lap in enumerate(laps, 1):
            tab_buttons.append(
                f'<button class="tab" type="button" data-target="lap{i}">Lap {i}</button>'
            )
            panels.append(
                f'<section class="lap-panel" data-panel="lap{i}">'
                + render_view(lap, f"Lap {i}", base_speed_kmh, scales)
                + "</section>"
            )

        incomplete = lap_info.get("incomplete_tail_distance_m", 0.0)
        incomplete_note = ""
        if incomplete >= 10.0:
            incomplete_note = (
                f'<div class="info">最後に約 {incomplete:.1f} m の未完走区間があります。'
                '全周平均には含めていません。</div>'
            )

        lap_notice = f"""
<div class="info">
  完走ラップを <b>{len(laps)}周</b> 検出しました。
  検出方式: {html.escape(method_text)}。<br>
  「全周平均」は各ラップを走行距離 0〜100% に正規化し、
  同じ進捗位置の速度・加速度・操舵・制御指令・座標を平均しています。
</div>
{incomplete_note}
"""
    else:
        tab_buttons = [
            '<button class="tab active" type="button" data-target="whole">走行全体</button>'
        ]
        panels = [
            '<section class="lap-panel active" data-panel="whole">'
            + render_view(all_rows, "走行全体", base_speed_kmh, scales)
            + "</section>"
        ]
        reason = lap_info.get("reason", "周回境界を検出できませんでした")
        lap_notice = f"""
<div class="warning">
  周回を自動分割できませんでした: {html.escape(reason)}。<br>
  データ自体は失われていません。走行全体として表示しています。
  必要なら <code>--lap-gate-width</code> や <code>--lap-min-distance</code> を調整してください。
</div>
"""

    lap_table = lap_table_html(laps, base_speed_kmh)

    doc = f"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>レーステレメトリ解析</title>
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
  font-family:system-ui,-apple-system,"Segoe UI","Yu Gothic UI","Meiryo",sans-serif;
  background:var(--bg);
  color:var(--text);
}}
main {{ max-width:1480px; margin:auto; padding:24px; }}
h1 {{ margin:0 0 6px; color:#0f172a; }}
.sub {{ color:var(--muted); margin-bottom:16px; font-weight:600; }}
.tabs {{
  display:flex;
  gap:8px;
  flex-wrap:wrap;
  margin:16px 0;
}}
.tab {{
  border:1px solid #94a3b8;
  background:#ffffff;
  color:#1e293b;
  border-radius:999px;
  padding:9px 16px;
  font-weight:700;
  cursor:pointer;
}}
.tab:hover {{ background:#f8fafc; }}
.tab.active {{
  background:#0f172a;
  color:#ffffff;
  border-color:#0f172a;
}}
.lap-panel {{ display:none; }}
.lap-panel.active {{ display:block; }}
.view-title {{
  font-size:1.4rem;
  font-weight:800;
  margin:18px 0 12px;
  color:#0f172a;
}}
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
  font-size:1.3rem;
  margin-top:3px;
  color:#0f172a;
}}
.grid {{
  display:grid;
  grid-template-columns:repeat(2,minmax(0,1fr));
  gap:14px;
}}
.wide {{ grid-column:1 / -1; }}
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
.legend-note {{ color:#334155; font-size:.88rem; margin-top:7px; line-height:1.5; }}
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
.info,.warning {{
  margin:12px 0;
  padding:11px 13px;
  border-radius:9px;
  line-height:1.6;
}}
.info {{
  border:1px solid #38bdf8;
  background:#f0f9ff;
  color:#0c4a6e;
}}
.warning {{
  border:1px solid #f59e0b;
  background:#fffbeb;
  color:#92400e;
}}
code {{
  background:#e2e8f0;
  color:#0f172a;
  padding:1px 5px;
  border-radius:4px;
}}
.charts {{ margin-top:18px; display:grid; gap:12px; }}
.table-scroll {{ overflow-x:auto; }}
.lap-table {{
  border-collapse:collapse;
  width:100%;
  min-width:900px;
}}
.lap-table th,.lap-table td {{
  border-bottom:1px solid #e2e8f0;
  padding:9px 10px;
  text-align:right;
  white-space:nowrap;
}}
.lap-table th:first-child,.lap-table td:first-child {{ text-align:left; }}
.lap-table th {{ background:#f8fafc; color:#334155; }}
.best {{
  display:inline-block;
  background:#dcfce7;
  color:#166534;
  border:1px solid #86efac;
  border-radius:999px;
  padding:1px 7px;
  margin-left:5px;
  font-size:.72rem;
  font-weight:800;
}}
@media(max-width:850px) {{
  .grid {{ grid-template-columns:1fr; }}
  .wide {{ grid-column:auto; }}
  main {{ padding:12px; }}
}}
</style>
</head>
<body>
<main>
<h1>レーステレメトリ解析</h1>
<div class="sub">
  基準速度: {base_speed_kmh:.1f} km/h ・
  コース上の線にマウスを置くと詳細値を表示します
</div>

{lap_notice}
{lap_table}

<nav class="tabs" aria-label="ラップ切り替え">
  {''.join(tab_buttons)}
</nav>

{''.join(panels)}

</main>
<script>
(() => {{
  const tabs = Array.from(document.querySelectorAll('.tab'));
  const panels = Array.from(document.querySelectorAll('.lap-panel'));

  tabs.forEach((tab) => {{
    tab.addEventListener('click', () => {{
      const target = tab.dataset.target;
      tabs.forEach((x) => x.classList.toggle('active', x === tab));
      panels.forEach((panel) => {{
        panel.classList.toggle('active', panel.dataset.panel === target);
      }});
    }});
  }});
}})();
</script>
</body>
</html>
"""
    path.write_text(doc, encoding="utf-8")


# ============================================================
# main
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="ROS 2 レーステレメトリ記録・周回解析ツール"
    )
    parser.add_argument(
        "--base-speed",
        type=float,
        default=32.0,
        help="比較基準速度 [km/h] (default: 32)",
    )
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=20.0,
        help="記録周波数 [Hz] (default: 20)",
    )
    parser.add_argument(
        "--out",
        default="telemetry_runs",
        help="出力先ルートディレクトリ",
    )
    parser.add_argument(
        "--lap-start-speed",
        type=float,
        default=3.0,
        help="走行開始判定速度 [km/h] (default: 3)",
    )
    parser.add_argument(
        "--lap-min-distance",
        type=float,
        default=80.0,
        help="同一ラップと誤検出しないための最小ラップ距離 [m] (default: 80)",
    )
    parser.add_argument(
        "--lap-gate-width",
        type=float,
        default=10.0,
        help="仮想スタートラインの片側許容幅 [m] (default: 10)",
    )
    parser.add_argument(
        "--average-points",
        type=int,
        default=320,
        help="全周平均ラップの補間点数 (default: 320)",
    )
    args = parser.parse_args()

    if args.base_speed <= 0:
        raise SystemExit("--base-speed は 0 より大きくしてください")
    if args.sample_hz <= 0 or args.sample_hz > 100:
        raise SystemExit("--sample-hz は 0 < Hz <= 100 の範囲にしてください")
    if args.lap_min_distance <= 10:
        raise SystemExit("--lap-min-distance は 10 m より大きくしてください")
    if args.lap_gate_width <= 1:
        raise SystemExit("--lap-gate-width は 1 m より大きくしてください")

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
        print("記録サンプルが不足しています。ROS_DOMAIN_ID とトピック受信を確認してください。")
        return 1

    laps, lap_info = detect_laps(
        rows,
        start_speed_kmh=args.lap_start_speed,
        min_lap_distance_m=args.lap_min_distance,
        gate_half_width_m=args.lap_gate_width,
    )

    average_rows = average_laps(
        laps,
        args.base_speed,
        points=args.average_points,
    )

    # 全ログ
    csv_path = run_dir / "telemetry.csv"
    save_csv(rows, csv_path)

    # ラップ別CSV
    for i, lap in enumerate(laps, 1):
        save_csv(lap, run_dir / f"lap_{i:02d}.csv")

    if average_rows:
        save_csv(average_rows, run_dir / "lap_average.csv")

    html_path = run_dir / "telemetry.html"
    generate_html(
        rows,
        laps,
        average_rows,
        html_path,
        args.base_speed,
        lap_info,
    )

    print()
    print(f"CSV       : {csv_path.resolve()}")
    print(f"HTML      : {html_path.resolve()}")
    print(f"サンプル数: {len(rows)}")
    print(f"完走ラップ: {len(laps)}")

    for i, lap in enumerate(laps, 1):
        print(
            f"  Lap {i}: {lap[-1]['t_s']:.2f} s / "
            f"{lap[-1]['distance_m']:.1f} m"
        )

    if laps:
        print(f"全周平均CSV: {(run_dir / 'lap_average.csv').resolve()}")
    else:
        print(
            "周回分割はできませんでした。telemetry.html には走行全体を表示しています。"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
