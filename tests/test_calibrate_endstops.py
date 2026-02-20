import importlib.util
import pathlib
import unittest


SCRIPT_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "calibrate_endstops.py"
spec = importlib.util.spec_from_file_location("calibrate_endstops", SCRIPT_PATH)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(mod)


class TestCalibrateEndstops(unittest.TestCase):
    def test_circular_delta_wrap(self):
        self.assertEqual(mod.circular_delta(4090, 2), 8)
        self.assertEqual(mod.circular_delta(2, 4090), -8)

    def test_circular_delta_boundary(self):
        # exact half-turn chooses positive branch in current implementation
        self.assertEqual(mod.circular_delta(0, 2048), 2048)
        self.assertEqual(mod.circular_delta(2048, 0), 2048)

    def test_circular_midpoint_wrap(self):
        # midpoint on shortest arc between 300 and 3300 in 0..4095 ring
        self.assertEqual(mod.circular_midpoint(300, 3300), 3848)

    def test_circular_span_wrap_small_motion(self):
        # samples crossing 4095->0 should still be a small span
        span = mod.circular_span([4094, 4095, 0, 1, 2])
        self.assertEqual(span, 4)

    def test_sim_dry_run_does_not_modify_state(self):
        sim = mod.SimBackend([1])
        before_pos = sim.read_u16(1, 56)
        before_off = sim.read_u16(1, 31)

        self.assertTrue(sim.write_u16(1, 42, 1234, dry_run=True))
        self.assertTrue(mod.write_i16(sim, 1, 31, -120, dry_run=True))

        self.assertEqual(sim.read_u16(1, 56), before_pos)
        self.assertEqual(sim.read_u16(1, 31), before_off)

    def test_sim_detect_stops(self):
        sim = mod.SimBackend([1])
        min_stop = mod.detect_stop(
            sim,
            servo_id=1,
            direction=-1,
            pos_addr=56,
            load_addr=60,
            goal_addr=42,
            step_ticks=8,
            pause_s=0,
            stall_window=4,
            move_eps=2,
            load_threshold=300,
            dry_run=False,
        )
        max_stop = mod.detect_stop(
            sim,
            servo_id=1,
            direction=+1,
            pos_addr=56,
            load_addr=60,
            goal_addr=42,
            step_ticks=8,
            pause_s=0,
            stall_window=4,
            move_eps=2,
            load_threshold=300,
            dry_run=False,
        )
        self.assertEqual(min_stop, 300)
        self.assertEqual(max_stop, 3300)

    def test_detect_stop_requires_load_threshold(self):
        sim = mod.SimBackend([1])
        # Unrealistically high threshold means no stop should be declared
        min_stop = mod.detect_stop(
            sim,
            servo_id=1,
            direction=-1,
            pos_addr=56,
            load_addr=60,
            goal_addr=42,
            step_ticks=8,
            pause_s=0,
            stall_window=4,
            move_eps=2,
            load_threshold=1200,
            dry_run=False,
        )
        self.assertIsNone(min_stop)

    def test_sim_offset_write_to_center_2048(self):
        sim = mod.SimBackend([1])
        min_stop = 300
        max_stop = 3300
        midpoint = mod.circular_midpoint(min_stop, max_stop)
        offset = mod.circular_delta(midpoint, 2048)
        ok = mod.write_i16(sim, 1, 31, offset, dry_run=False)
        self.assertTrue(ok)
        raw = sim.read_u16(1, 31)
        if raw >= 32768:
            raw -= 65536
        self.assertEqual(raw, offset)

    def test_multi_servo_calibration_and_no_wrap_operational_margin(self):
        sim = mod.SimBackend([1, 2, 3, 4, 5, 6])
        for sid in [1, 2, 3, 4, 5, 6]:
            mn = mod.detect_stop(
                sim,
                servo_id=sid,
                direction=-1,
                pos_addr=56,
                load_addr=60,
                goal_addr=42,
                step_ticks=8,
                pause_s=0,
                stall_window=4,
                move_eps=2,
                load_threshold=300,
                dry_run=False,
            )
            mx = mod.detect_stop(
                sim,
                servo_id=sid,
                direction=+1,
                pos_addr=56,
                load_addr=60,
                goal_addr=42,
                step_ticks=8,
                pause_s=0,
                stall_window=4,
                move_eps=2,
                load_threshold=300,
                dry_run=False,
            )
            self.assertIsNotNone(mn)
            self.assertIsNotNone(mx)
            midpoint = mod.circular_midpoint(mn, mx)

            # In expected normal use, commanded motion should stay within a half-turn from calibrated center.
            # This gives a no-wrap operational assumption after calibration.
            for delta in [-900, -300, 0, 300, 900]:
                cmd = mod.wrap_tick(midpoint + delta)
                dist = abs(mod.circular_delta(midpoint, cmd))
                self.assertLess(dist, 2048)


if __name__ == "__main__":
    unittest.main()
