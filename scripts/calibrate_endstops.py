#!/usr/bin/env python3
"""
Slow endstop calibration for Feetech STS3215/SCS-like protocol (half-duplex TTL).

Flow per ID:
1) Move slowly to negative direction until stall/load threshold.
2) Move slowly to positive direction until stall/load threshold.
3) Midpoint = (min_stop + max_stop) / 2
4) Move to midpoint and report zero_tick suggestion.

Default is SAFE-DRY-RUN (no writes).
"""

import argparse
import time
from dataclasses import dataclass
from typing import List, Optional

import serial


@dataclass
class ServoResult:
    servo_id: int
    min_stop: Optional[int]
    max_stop: Optional[int]
    midpoint: Optional[int]
    offset_written: Optional[int]


def checksum(body: List[int]) -> int:
    return (~(sum(body) & 0xFF)) & 0xFF


def pkt_read(servo_id: int, addr: int, n: int) -> bytes:
    body = [servo_id, 0x04, 0x02, addr, n]
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def pkt_write(servo_id: int, addr: int, data: List[int]) -> bytes:
    body = [servo_id, 3 + len(data), 0x03, addr] + data
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def recv_frame(ser: serial.Serial, timeout: float = 0.05) -> Optional[bytes]:
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < timeout:
        b = ser.read(1)
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


def read_u16(ser: serial.Serial, servo_id: int, addr: int) -> Optional[int]:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(pkt_read(servo_id, addr, 2))
    fr = recv_frame(ser)
    if not fr or len(fr) < 8 or fr[2] != servo_id:
        return None
    if fr[4] != 0:
        return None
    return fr[5] | (fr[6] << 8)


def write_u16(ser: serial.Serial, servo_id: int, addr: int, value: int, dry_run: bool) -> bool:
    if dry_run:
        return True
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    lo, hi = value & 0xFF, (value >> 8) & 0xFF
    ser.write(pkt_write(servo_id, addr, [lo, hi]))
    fr = recv_frame(ser)
    return bool(fr and len(fr) >= 6 and fr[2] == servo_id and fr[4] == 0)


def write_i16(ser: serial.Serial, servo_id: int, addr: int, value: int, dry_run: bool) -> bool:
    value = max(-32768, min(32767, int(value)))
    return write_u16(ser, servo_id, addr, value & 0xFFFF, dry_run)


def detect_stop(
    ser: serial.Serial,
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
    hard_min: int,
    hard_max: int,
    dry_run: bool,
) -> Optional[int]:
    pos_hist: List[int] = []

    current = read_u16(ser, servo_id, pos_addr)
    if current is None:
        return None

    for _ in range(2000):
        target = current + direction * step_ticks
        target = max(hard_min, min(hard_max, target))

        ok = write_u16(ser, servo_id, goal_addr, target, dry_run)
        if not ok:
            return None

        time.sleep(pause_s)

        pos = read_u16(ser, servo_id, pos_addr)
        load = read_u16(ser, servo_id, load_addr)
        if pos is None or load is None:
            return None

        pos_hist.append(pos)
        if len(pos_hist) > stall_window:
            pos_hist.pop(0)

        # Feetech load often encodes sign in top bit; use magnitude-like mask
        load_mag = load & 0x3FF

        if len(pos_hist) >= stall_window:
            moved = max(pos_hist) - min(pos_hist)
            if moved <= move_eps and load_mag >= load_threshold:
                return pos

        current = pos

        if current in (hard_min, hard_max):
            return current

    return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", nargs="+", type=int, required=True)
    ap.add_argument("--apply", action="store_true", help="actually send goal positions (default dry-run)")
    ap.add_argument("--step", type=int, default=4, help="ticks per step")
    ap.add_argument("--pause-ms", type=int, default=60)
    ap.add_argument("--stall-window", type=int, default=6)
    ap.add_argument("--move-eps", type=int, default=2)
    ap.add_argument("--load-threshold", type=int, default=300)
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
    print(f"Mode: {'DRY-RUN' if dry_run else 'APPLY'}")

    ser = serial.Serial(args.port, args.baud, timeout=0.002)
    results: List[ServoResult] = []

    try:
        for sid in args.ids:
            print(f"\n== Calibrating ID {sid} ==")

            min_stop = detect_stop(
                ser,
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
                hard_min=args.hard_min,
                hard_max=args.hard_max,
                dry_run=dry_run,
            )
            print(f"ID {sid} min_stop={min_stop}")

            max_stop = detect_stop(
                ser,
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
                hard_min=args.hard_min,
                hard_max=args.hard_max,
                dry_run=dry_run,
            )
            print(f"ID {sid} max_stop={max_stop}")

            midpoint = None
            offset_written = None
            if min_stop is not None and max_stop is not None:
                midpoint = (min_stop + max_stop) // 2
                ok = write_u16(ser, sid, args.addr_goal, midpoint, dry_run)
                print(f"ID {sid} midpoint={midpoint}, move_ok={ok}")

                if args.write_offset:
                    # Offset is applied so midpoint maps toward offset_center_tick.
                    offset = args.offset_center_tick - midpoint
                    ok_off = write_i16(ser, sid, args.addr_offset, offset, dry_run)
                    offset_written = offset if ok_off else None
                    print(
                        f"ID {sid} offset_write addr={args.addr_offset} value={offset} ok={ok_off}"
                    )

            results.append(ServoResult(sid, min_stop, max_stop, midpoint, offset_written))

    finally:
        ser.close()

    print("\n=== Suggested zero ticks ===")
    for r in results:
        print(
            f"ID {r.servo_id}: zero_tick={r.midpoint} (min={r.min_stop}, max={r.max_stop}), "
            f"offset_written={r.offset_written}"
        )


if __name__ == "__main__":
    main()
