#!/usr/bin/env python3
"""
Slow endstop calibration for Feetech STS3215/SCS-like protocol.

Flow per ID:
1) Move slowly to negative direction until stall/load threshold.
2) Move slowly to positive direction until stall/load threshold.
3) Midpoint = circular midpoint(min_stop, max_stop)
4) Move to midpoint and report zero_tick suggestion.
5) Optional: write persistent offset register.

Default is SAFE-DRY-RUN (no writes).
"""

import argparse
import math
import threading
import time
from dataclasses import dataclass
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Callable, Dict, List, Optional

import serial

TICKS = 4096


@dataclass
class ServoResult:
    servo_id: int
    min_stop: Optional[int]
    max_stop: Optional[int]
    midpoint: Optional[int]
    offset_written: Optional[int]


def wrap_tick(v: int, mod: int = TICKS) -> int:
    return v % mod


def circular_delta(a: int, b: int, mod: int = TICKS) -> int:
    """Shortest signed delta from a -> b on circular ring."""
    d = (b - a) % mod
    if d > mod // 2:
        d -= mod
    return d


def circular_span(samples: List[int], mod: int = TICKS) -> int:
    """Smallest span containing all sample points on circular ring."""
    if not samples:
        return 0
    if len(samples) == 1:
        return 0
    pts = sorted([s % mod for s in samples])
    gaps = [pts[i + 1] - pts[i] for i in range(len(pts) - 1)]
    gaps.append((pts[0] + mod) - pts[-1])
    largest_gap = max(gaps)
    return mod - largest_gap


def circular_midpoint(a: int, b: int, mod: int = TICKS) -> int:
    d = circular_delta(a, b, mod)
    return wrap_tick(a + d // 2, mod)


class Backend:
    def read_u16(self, servo_id: int, addr: int) -> Optional[int]:
        raise NotImplementedError

    def write_u16(self, servo_id: int, addr: int, value: int, dry_run: bool) -> bool:
        raise NotImplementedError


class SerialBackend(Backend):
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=0.002)

    def close(self):
        self.ser.close()

    @staticmethod
    def checksum(body: List[int]) -> int:
        return (~(sum(body) & 0xFF)) & 0xFF

    def pkt_read(self, servo_id: int, addr: int, n: int) -> bytes:
        body = [servo_id, 0x04, 0x02, addr, n]
        return bytes([0xFF, 0xFF] + body + [self.checksum(body)])

    def pkt_write(self, servo_id: int, addr: int, data: List[int]) -> bytes:
        body = [servo_id, 3 + len(data), 0x03, addr] + data
        return bytes([0xFF, 0xFF] + body + [self.checksum(body)])

    def recv_frame(self, timeout: float = 0.05) -> Optional[bytes]:
        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < timeout:
            b = self.ser.read(1)
            if not b:
                continue
            buf += b
            while len(buf) >= 2 and not (buf[0] == 0xFF and buf[1] == 0xFF):
                buf.pop(0)
            if len(buf) >= 4:
                ln = buf[3]
                total = 2 + 1 + 1 + ln
                if len(buf) >= total:
                    return bytes(buf[:total])
        return None

    def read_u16(self, servo_id: int, addr: int) -> Optional[int]:
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write(self.pkt_read(servo_id, addr, 2))
        fr = self.recv_frame()
        if not fr or len(fr) < 8 or fr[2] != servo_id:
            return None
        if fr[4] != 0:
            return None
        return fr[5] | (fr[6] << 8)

    def write_u16(self, servo_id: int, addr: int, value: int, dry_run: bool) -> bool:
        if dry_run:
            return True
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        lo, hi = value & 0xFF, (value >> 8) & 0xFF
        self.ser.write(self.pkt_write(servo_id, addr, [lo, hi]))
        fr = self.recv_frame()
        return bool(fr and len(fr) >= 6 and fr[2] == servo_id and fr[4] == 0)


class SimBackend(Backend):
    """Simple wrap-around simulator with external mechanical stops."""

    def __init__(self, ids: List[int]):
        self.state: Dict[int, Dict[str, int]] = {}
        for i, sid in enumerate(ids):
            mn = wrap_tick(300 + i * 80)
            mx = wrap_tick(3300 - i * 50)
            p = circular_midpoint(mn, mx)
            self.state[sid] = {
                "pos": p,
                "goal": p,
                "min": mn,
                "max": mx,
                "load": 120,
                "offset": 0,
            }

    def _within_arc(self, x: int, a: int, b: int) -> bool:
        # true if x is on arc a->b (positive direction)
        if a <= b:
            return a <= x <= b
        return x >= a or x <= b

    def _clamp_to_stops(self, s: Dict[str, int], goal: int) -> int:
        mn, mx = s["min"], s["max"]
        # allowed arc is mn -> mx positive direction
        if self._within_arc(goal, mn, mx):
            return goal
        d_to_mn = abs(circular_delta(goal, mn))
        d_to_mx = abs(circular_delta(goal, mx))
        return mn if d_to_mn <= d_to_mx else mx

    def read_u16(self, servo_id: int, addr: int) -> Optional[int]:
        s = self.state.get(servo_id)
        if not s:
            return None
        if addr == 56:  # present pos
            return s["pos"]
        if addr == 60:  # present load
            return s["load"]
        if addr == 31:  # offset
            return s["offset"] & 0xFFFF
        return 0

    def write_u16(self, servo_id: int, addr: int, value: int, dry_run: bool) -> bool:
        s = self.state.get(servo_id)
        if not s:
            return False
        if dry_run:
            return True
        if addr == 42:  # goal pos
            g = wrap_tick(value)
            clamped = self._clamp_to_stops(s, g)
            moved = abs(circular_delta(s["pos"], clamped))
            stalled = clamped != g
            s["pos"] = clamped
            s["goal"] = g
            s["load"] = 500 if stalled else max(80, min(300, 80 + moved * 3))
            return True
        if addr == 31:  # offset
            old = u16_to_i16(s["offset"] & 0xFFFF)
            new = u16_to_i16(value & 0xFFFF)
            delta = new - old
            # Simulate logical remap caused by offset write.
            s["offset"] = value & 0xFFFF
            s["pos"] = wrap_tick(s["pos"] + delta)
            s["goal"] = wrap_tick(s["goal"] + delta)
            s["min"] = wrap_tick(s["min"] + delta)
            s["max"] = wrap_tick(s["max"] + delta)
            return True
        return True


class SimulationVisualizer:
    def __init__(self, sim: SimBackend, ids: List[int], refresh_ms: int = 40):
        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            raise SystemExit(
                "--visualize requires matplotlib (pip install matplotlib)"
            ) from exc

        self.sim = sim
        self.ids = ids
        self.refresh_s = max(0.01, refresh_ms / 1000.0)
        self._last_update = 0.0

        self.plt = plt
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.ax.set_title("Endstop simulator (2D)")
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(-1.8, 1.8)
        self.ax.set_ylim(-1.8, 1.8)
        self.ax.grid(True, alpha=0.25)

        self._pos = {}
        self._goal = {}

        for idx, sid in enumerate(ids):
            ring = 0.6 + idx * 0.3
            state = self.sim.state[sid]
            mn, mx = state["min"], state["max"]
            arc_x, arc_y = self._arc_points(mn, mx, ring)
            self.ax.plot(arc_x, arc_y, linewidth=2, alpha=0.35)
            self.ax.text(0, ring + 0.06, f"ID {sid}", fontsize=8, ha="center")

            pos_dot = self.ax.plot([], [], "o", markersize=8)[0]
            goal_dot = self.ax.plot([], [], "x", markersize=8)[0]
            self._pos[sid] = (ring, pos_dot)
            self._goal[sid] = (ring, goal_dot)

        self.info = self.ax.text(-1.75, -1.7, "", fontsize=8, va="bottom")
        self.plt.ion()
        self.update(force=True)

    @staticmethod
    def _tick_to_xy(tick: int, r: float) -> tuple[float, float]:
        import math

        th = (tick % TICKS) / TICKS * (2.0 * math.pi)
        return r * math.cos(th), r * math.sin(th)

    def _arc_points(self, a: int, b: int, r: float, n: int = 90):
        import math

        d = circular_delta(a, b)
        pts_x = []
        pts_y = []
        for i in range(n + 1):
            t = i / n
            tick = wrap_tick(int(round(a + d * t)))
            th = tick / TICKS * (2.0 * math.pi)
            pts_x.append(r * math.cos(th))
            pts_y.append(r * math.sin(th))
        return pts_x, pts_y

    def update(self, force: bool = False) -> None:
        now = time.time()
        if not force and (now - self._last_update) < self.refresh_s:
            return
        self._last_update = now

        lines = []
        for sid in self.ids:
            s = self.sim.state[sid]
            r, pos_dot = self._pos[sid]
            x, y = self._tick_to_xy(s["pos"], r)
            pos_dot.set_data([x], [y])

            r, goal_dot = self._goal[sid]
            xg, yg = self._tick_to_xy(s["goal"], r)
            goal_dot.set_data([xg], [yg])

            lines.append(f"ID {sid}: pos={s['pos']} goal={s['goal']} load={s['load']} off={u16_to_i16(s['offset'])}")

        self.info.set_text("\n".join(lines))
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    def close(self) -> None:
        self.plt.ioff()
        self.plt.show(block=False)


class WebSimulationVisualizer:
    def __init__(
        self,
        sim: SimBackend,
        ids: List[int],
        refresh_ms: int = 120,
        bind: str = "127.0.0.1",
        port: int = 8765,
    ):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:
            raise SystemExit("--visualize-web requires matplotlib (pip install matplotlib)") from exc

        self.sim = sim
        self.ids = ids
        self.refresh_s = max(0.04, refresh_ms / 1000.0)
        self._last_update = 0.0
        self._frame_no = 0
        self.plt = plt

        self._tmp = TemporaryDirectory(prefix="endstop-viz-")
        self.root = Path(self._tmp.name)
        self.frame_path = self.root / "frame.png"
        self._write_index_html(bind, port)

        self.fig, self.ax = plt.subplots(figsize=(7, 7), dpi=120)
        self.ax.set_title("Endstop simulator (2D web)")
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(-1.8, 1.8)
        self.ax.set_ylim(-1.8, 1.8)
        self.ax.grid(True, alpha=0.25)

        self._pos = {}
        self._goal = {}
        for idx, sid in enumerate(ids):
            ring = 0.6 + idx * 0.3
            state = self.sim.state[sid]
            mn, mx = state["min"], state["max"]
            arc_x, arc_y = self._arc_points(mn, mx, ring)
            self.ax.plot(arc_x, arc_y, linewidth=2, alpha=0.35)
            self.ax.text(0, ring + 0.06, f"ID {sid}", fontsize=8, ha="center")
            pos_dot = self.ax.plot([], [], "o", markersize=8)[0]
            goal_dot = self.ax.plot([], [], "x", markersize=8)[0]
            self._pos[sid] = (ring, pos_dot)
            self._goal[sid] = (ring, goal_dot)

        self.info = self.ax.text(-1.75, -1.7, "", fontsize=8, va="bottom")

        class NoCacheHandler(SimpleHTTPRequestHandler):
            def end_headers(self):
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                super().end_headers()

        handler = partial(NoCacheHandler, directory=str(self.root))
        self.httpd = ThreadingHTTPServer((bind, port), handler)
        self._thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self._thread.start()
        print(f"Web visualization: http://{bind}:{port}")
        print("Tip: if running remote, expose this port via tunnel/Tailscale.")

        self.update(force=True)

    def _write_index_html(self, bind: str, port: int) -> None:
        html = f"""<!doctype html>
<html>
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>Endstop Simulator</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 10px; background:#111; color:#eee; }}
    img {{ width: 100%; max-width: 900px; border:1px solid #444; background:#fff; }}
    .meta {{ color:#bbb; margin: 8px 0; }}
  </style>
</head>
<body>
  <h3>Endstop simulator 2D view</h3>
  <div class=\"meta\">Source: {bind}:{port}</div>
  <img id=\"frame\" src=\"frame.png\" />
  <script>
    const img = document.getElementById('frame');
    setInterval(() => {{
      img.src = 'frame.png?t=' + Date.now();
    }}, 250);
  </script>
</body>
</html>
"""
        (self.root / "index.html").write_text(html, encoding="utf-8")

    @staticmethod
    def _tick_to_xy(tick: int, r: float) -> tuple[float, float]:
        import math

        th = (tick % TICKS) / TICKS * (2.0 * math.pi)
        return r * math.cos(th), r * math.sin(th)

    def _arc_points(self, a: int, b: int, r: float, n: int = 90):
        import math

        d = circular_delta(a, b)
        pts_x = []
        pts_y = []
        for i in range(n + 1):
            t = i / n
            tick = wrap_tick(int(round(a + d * t)))
            th = tick / TICKS * (2.0 * math.pi)
            pts_x.append(r * math.cos(th))
            pts_y.append(r * math.sin(th))
        return pts_x, pts_y

    def update(self, force: bool = False) -> None:
        now = time.time()
        if not force and (now - self._last_update) < self.refresh_s:
            return
        self._last_update = now

        lines = []
        for sid in self.ids:
            s = self.sim.state[sid]
            r, pos_dot = self._pos[sid]
            x, y = self._tick_to_xy(s["pos"], r)
            pos_dot.set_data([x], [y])

            r, goal_dot = self._goal[sid]
            xg, yg = self._tick_to_xy(s["goal"], r)
            goal_dot.set_data([xg], [yg])
            lines.append(f"ID {sid}: pos={s['pos']} goal={s['goal']} load={s['load']} off={u16_to_i16(s['offset'])}")

        self._frame_no += 1
        lines.append(f"frame={self._frame_no} t={time.strftime('%H:%M:%S')}")
        self.info.set_text("\n".join(lines))
        self.fig.tight_layout()
        self.fig.savefig(self.frame_path)

    def close(self) -> None:
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass
        try:
            self._tmp.cleanup()
        except Exception:
            pass


def u16_to_i16(v: int) -> int:
    return v if v < 32768 else v - 65536


def write_i16(backend: Backend, servo_id: int, addr: int, value: int, dry_run: bool) -> bool:
    value = max(-32768, min(32767, int(value)))
    return backend.write_u16(servo_id, addr, value & 0xFFFF, dry_run)


def detect_stop(
    backend: Backend,
    servo_id: int,
    direction: int,
    pos_addr: int,
    load_addr: int,
    goal_addr: int,
    step_ticks: int,
    pause_s: float,
    stall_window: int,
    move_eps: int,
    load_threshold: int,
    dry_run: bool,
    no_wrap: bool,
    hard_min: int,
    hard_max: int,
    on_step: Optional[Callable[[], None]] = None,
) -> Optional[int]:
    pos_hist: List[int] = []

    current = backend.read_u16(servo_id, pos_addr)
    if current is None:
        return None

    for _ in range(2000):
        proposed = current + direction * step_ticks
        boundary_hit = False
        if no_wrap:
            if proposed < hard_min:
                proposed = hard_min
                boundary_hit = True
            elif proposed > hard_max:
                proposed = hard_max
                boundary_hit = True
            target = proposed
        else:
            target = wrap_tick(proposed)

        ok = backend.write_u16(servo_id, goal_addr, target, dry_run)
        if not ok:
            return None

        if on_step is not None:
            on_step()

        time.sleep(pause_s)

        pos = backend.read_u16(servo_id, pos_addr)
        load = backend.read_u16(servo_id, load_addr)
        if pos is None or load is None:
            return None

        pos_hist.append(pos)
        if len(pos_hist) > stall_window:
            pos_hist.pop(0)

        load_mag = load & 0x3FF

        if on_step is not None:
            on_step()

        if len(pos_hist) >= stall_window:
            moved = circular_span(pos_hist)
            if moved <= move_eps and load_mag >= load_threshold:
                return pos

        current = pos

        if no_wrap and boundary_hit:
            # Reached configured command boundary without wrapping.
            return current

    return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port (not needed with --simulate)")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", nargs="+", type=int, required=True)
    ap.add_argument("--simulate", action="store_true", help="run against built-in simulator")
    ap.add_argument("--visualize", action="store_true", help="show live 2D desktop view (simulate mode)")
    ap.add_argument("--visualize-web", action="store_true", help="serve live 2D view over HTTP (simulate mode)")
    ap.add_argument("--viz-refresh-ms", type=int, default=40, help="visualization refresh period")
    ap.add_argument("--web-bind", default="127.0.0.1", help="bind address for --visualize-web")
    ap.add_argument("--web-port", type=int, default=8765, help="port for --visualize-web")
    ap.add_argument("--apply", action="store_true", help="actually send goal positions (default dry-run)")
    ap.add_argument("--demo-loop", action="store_true", help="run continuous simulated motion demo (simulate mode)")
    ap.add_argument("--step", type=int, default=4, help="ticks per step")
    ap.add_argument("--pause-ms", type=int, default=60)
    ap.add_argument("--stall-window", type=int, default=6)
    ap.add_argument("--move-eps", type=int, default=2)
    ap.add_argument("--load-threshold", type=int, default=300)
    ap.add_argument("--no-wrap", action="store_true", default=True, help="never command setpoints across 0/4095 during calibration")
    ap.add_argument("--allow-wrap", action="store_true", help="disable no-wrap protection")
    ap.add_argument("--hard-min", type=int, default=0)
    ap.add_argument("--hard-max", type=int, default=4095)

    # register defaults for STS-like map
    ap.add_argument("--addr-goal", type=int, default=42)
    ap.add_argument("--addr-pos", type=int, default=56)
    ap.add_argument("--addr-load", type=int, default=60)
    ap.add_argument("--addr-offset", type=int, default=31)

    ap.add_argument("--write-offset", action="store_true", help="write computed i16 offset to motor register")
    ap.add_argument("--offset-center-tick", type=int, default=2048, help="target center tick after calibration")
    ap.add_argument("--offset-assist", action="store_true", help="allow bounded offset shifts if a stop is unreachable without wrapping")
    ap.add_argument("--assist-step", type=int, default=80, help="offset ticks per assist step")
    ap.add_argument("--assist-max-total", type=int, default=800, help="max absolute assist offset from initial offset")
    ap.add_argument("--assist-max-attempts", type=int, default=12, help="max detect/assist attempts per servo")

    args = ap.parse_args()

    if (args.visualize or args.visualize_web or args.demo_loop) and not args.simulate:
        raise SystemExit("--visualize/--visualize-web/--demo-loop only work with --simulate")
    if args.visualize and args.visualize_web:
        raise SystemExit("choose only one: --visualize OR --visualize-web")

    dry_run = not args.apply
    no_wrap = args.no_wrap and not args.allow_wrap
    print(f"Mode: {'DRY-RUN' if dry_run else 'APPLY'}")
    print(f"Setpoint wrapping: {'DISABLED' if no_wrap else 'ENABLED'}")

    backend: Backend
    serial_backend: Optional[SerialBackend] = None
    visualizer: Optional[object] = None

    if args.simulate:
        sim_backend = SimBackend(args.ids)
        backend = sim_backend
        if args.visualize:
            visualizer = SimulationVisualizer(sim_backend, args.ids, refresh_ms=args.viz_refresh_ms)
        elif args.visualize_web:
            visualizer = WebSimulationVisualizer(
                sim_backend,
                args.ids,
                refresh_ms=max(80, args.viz_refresh_ms),
                bind=args.web_bind,
                port=args.web_port,
            )
        print("Backend: SIM")
    else:
        if not args.port:
            raise SystemExit("--port is required unless --simulate is set")
        serial_backend = SerialBackend(args.port, args.baud)
        backend = serial_backend
        print(f"Backend: SERIAL {args.port} @ {args.baud}")

    if args.demo_loop:
        if not args.apply:
            print("[demo] enabling apply mode automatically for simulator demo")
        print("[demo] running continuous motion demo. Press Ctrl+C to stop.")
        t0 = time.time()
        try:
            while True:
                t = time.time() - t0
                for i, sid in enumerate(args.ids):
                    base = 2048 + int(900 * math.sin(t * 0.9 + i * 0.7))
                    goal = wrap_tick(base)
                    backend.write_u16(sid, args.addr_goal, goal, dry_run=False)
                if visualizer:
                    visualizer.update(force=True)
                time.sleep(max(0.02, args.viz_refresh_ms / 1000.0))
        except KeyboardInterrupt:
            print("\n[demo] stopped")
        finally:
            if serial_backend is not None:
                serial_backend.close()
            if visualizer is not None:
                visualizer.close()
        return

    results: List[ServoResult] = []

    try:
        for sid in args.ids:
            print(f"\n== Calibrating ID {sid} ==")

            raw_off = backend.read_u16(sid, args.addr_offset)
            initial_offset = u16_to_i16(raw_off) if raw_off is not None else 0
            current_offset = initial_offset

            min_stop: Optional[int] = None
            max_stop: Optional[int] = None

            for attempt in range(1, args.assist_max_attempts + 1):
                min_stop = detect_stop(
                    backend,
                    sid,
                    direction=-1,
                    pos_addr=args.addr_pos,
                    load_addr=args.addr_load,
                    goal_addr=args.addr_goal,
                    step_ticks=args.step,
                    pause_s=args.pause_ms / 1000.0,
                    stall_window=args.stall_window,
                    move_eps=args.move_eps,
                    load_threshold=args.load_threshold,
                    dry_run=dry_run,
                    no_wrap=no_wrap,
                    hard_min=args.hard_min,
                    hard_max=args.hard_max,
                    on_step=visualizer.update if visualizer else None,
                )
                max_stop = detect_stop(
                    backend,
                    sid,
                    direction=+1,
                    pos_addr=args.addr_pos,
                    load_addr=args.addr_load,
                    goal_addr=args.addr_goal,
                    step_ticks=args.step,
                    pause_s=args.pause_ms / 1000.0,
                    stall_window=args.stall_window,
                    move_eps=args.move_eps,
                    load_threshold=args.load_threshold,
                    dry_run=dry_run,
                    no_wrap=no_wrap,
                    hard_min=args.hard_min,
                    hard_max=args.hard_max,
                    on_step=visualizer.update if visualizer else None,
                )

                print(f"ID {sid} attempt={attempt} min_stop={min_stop} max_stop={max_stop} offset={current_offset}")

                min_limited = min_stop == args.hard_min
                max_limited = max_stop == args.hard_max
                if min_stop is not None and max_stop is not None and not (min_limited or max_limited):
                    break

                if not args.offset_assist or dry_run:
                    break

                shift = 0
                if max_limited and not min_limited:
                    shift = -abs(args.assist_step)
                elif min_limited and not max_limited:
                    shift = abs(args.assist_step)
                elif min_limited and max_limited:
                    shift = 0
                elif min_stop is None and max_stop is not None:
                    shift = abs(args.assist_step)
                elif max_stop is None and min_stop is not None:
                    shift = -abs(args.assist_step)

                if shift == 0:
                    break

                proposed = current_offset + shift
                if abs(proposed - initial_offset) > abs(args.assist_max_total):
                    print(f"ID {sid} offset-assist limit reached (initial={initial_offset}, current={current_offset})")
                    break

                ok_assist = write_i16(backend, sid, args.addr_offset, proposed, dry_run=False)
                if not ok_assist:
                    print(f"ID {sid} offset-assist write failed")
                    break
                current_offset = proposed
                if visualizer:
                    visualizer.update(force=True)
                print(f"ID {sid} offset-assist applied: {current_offset}")

            midpoint = None
            offset_written = None
            if min_stop is not None and max_stop is not None:
                midpoint = circular_midpoint(min_stop, max_stop)
                ok = backend.write_u16(sid, args.addr_goal, midpoint, dry_run)
                if visualizer:
                    visualizer.update(force=True)
                print(f"ID {sid} midpoint={midpoint}, move_ok={ok}")

                if args.write_offset:
                    # Offset is applied so midpoint maps toward offset_center_tick.
                    offset = circular_delta(midpoint, args.offset_center_tick)
                    ok_off = write_i16(backend, sid, args.addr_offset, offset, dry_run)
                    if visualizer:
                        visualizer.update(force=True)
                    offset_written = offset if ok_off else None
                    print(
                        f"ID {sid} offset_write addr={args.addr_offset} value={offset} ok={ok_off}"
                    )

            results.append(ServoResult(sid, min_stop, max_stop, midpoint, offset_written))

    finally:
        if serial_backend is not None:
            serial_backend.close()
        if visualizer is not None:
            visualizer.close()

    print("\n=== Suggested zero ticks ===")
    for r in results:
        print(
            f"ID {r.servo_id}: zero_tick={r.midpoint} (min={r.min_stop}, max={r.max_stop}), "
            f"offset_written={r.offset_written}"
        )


if __name__ == "__main__":
    main()
