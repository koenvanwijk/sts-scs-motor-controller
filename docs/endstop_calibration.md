# Endstop calibration (slow + load monitored)

This procedure is designed to find mechanical limits gently and define a logical `zero_tick` at the center.

## Visual flow

```text
Tick axis

[min stop] ------------------------ [midpoint / zero_tick] ------------------------ [max stop]
    ^                                                                                 ^
    |                                                                                 |
slow approach + load check                                                   slow approach + load check
(direction -)                                                                 (direction +)
```

## Per-servo sequence

1. Enable torque.
2. Move in tiny steps toward negative direction.
3. Watch **position delta** and **load**:
   - If position barely changes for several samples AND load crosses threshold => stop found.
4. Repeat to positive direction.
5. Compute midpoint:

```text
zero_tick = (min_stop + max_stop) / 2
```

6. Move servo to `zero_tick`.
7. (Optional) write motor offset register so this midpoint becomes persistent in hardware.
8. Repeat for next ID.

## Safety defaults

- very small steps (e.g. 4 ticks)
- pause between steps (e.g. 60 ms)
- max loops bounded
- load threshold required to declare stop
- optional hard min/max clamp

## Script

Use:

```bash
python3 scripts/calibrate_endstops.py \
  --port /dev/tty_pink_follower_so101 \
  --baud 1000000 \
  --ids 1 2 3 4 5 6
```

By default this is **DRY-RUN**. Use `--apply` to actually send goal updates.

To also persist midpoint in motor offset register:

```bash
python3 scripts/calibrate_endstops.py \
  --port /dev/tty_pink_follower_so101 \
  --baud 1000000 \
  --ids 1 \
  --apply \
  --write-offset
```

Default offset register address is `31` (configurable via `--addr-offset`).

Start with one servo first:

```bash
python3 scripts/calibrate_endstops.py \
  --port /dev/tty_pink_follower_so101 \
  --baud 1000000 \
  --ids 1 \
  --apply
```
