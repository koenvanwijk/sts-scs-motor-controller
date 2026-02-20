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

    def test_circular_midpoint_wrap(self):
        # midpoint on shortest arc between 300 and 3300 in 0..4095 ring
        self.assertEqual(mod.circular_midpoint(300, 3300), 3848)

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


if __name__ == "__main__":
    unittest.main()
