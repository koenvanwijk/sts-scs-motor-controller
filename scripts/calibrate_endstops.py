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
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

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
            s["offset"] = value & 0xFFFF
            return True
        return True


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

        time.sleep(pause_s)

        pos = backend.read_u16(servo_id, pos_addr)
        load = backend.read_u16(servo_id, load_addr)
        if pos is None or load is None:
            return None

        pos_hist.append(pos)
        if len(pos_hist) > stall_window:
            pos_hist.pop(0)

        load_mag = load & 0x3FF

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
    ap.add_argument("--apply", action="store_true", help="actually send goal positions (default dry-run)")
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

    args = ap.parse_args()

    dry_run = not args.apply
    no_wrap = args.no_wrap and not args.allow_wrap
    print(f"Mode: {'DRY-RUN' if dry_run else 'APPLY'}")
    print(f"Setpoint wrapping: {'DISABLED' if no_wrap else 'ENABLED'}")

    backend: Backend
    serial_backend: Optional[SerialBackend] = None

    if args.simulate:
        backend = SimBackend(args.ids)
        print("Backend: SIM")
    else:
        if not args.port:
            raise SystemExit("--port is required unless --simulate is set")
        serial_backend = SerialBackend(args.port, args.baud)
        backend = serial_backend
        print(f"Backend: SERIAL {args.port} @ {args.baud}")

    results: List[ServoResult] = []

    try:
        for sid in args.ids:
            print(f"\n== Calibrating ID {sid} ==")

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
            )
            print(f"ID {sid} min_stop={min_stop}")

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
            )
            print(f"ID {sid} max_stop={max_stop}")

            midpoint = None
            offset_written = None
            if min_stop is not None and max_stop is not None:
                midpoint = circular_midpoint(min_stop, max_stop)
                ok = backend.write_u16(sid, args.addr_goal, midpoint, dry_run)
                print(f"ID {sid} midpoint={midpoint}, move_ok={ok}")

                if args.write_offset:
                    # Offset is applied so midpoint maps toward offset_center_tick.
                    offset = circular_delta(midpoint, args.offset_center_tick)
                    ok_off = write_i16(backend, sid, args.addr_offset, offset, dry_run)
                    offset_written = offset if ok_off else None
                    print(
                        f"ID {sid} offset_write addr={args.addr_offset} value={offset} ok={ok_off}"
                    )

            results.append(ServoResult(sid, min_stop, max_stop, midpoint, offset_written))

    finally:
        if serial_backend is not None:
            serial_backend.close()

    print("\n=== Suggested zero ticks ===")
    for r in results:
        print(
            f"ID {r.servo_id}: zero_tick={r.midpoint} (min={r.min_stop}, max={r.max_stop}), "
            f"offset_written={r.offset_written}"
        )


if __name__ == "__main__":
    main()
