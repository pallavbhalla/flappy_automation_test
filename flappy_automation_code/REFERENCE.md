# Flappy Bird Controller Reference

## State Machine

```
CRUISE ──(range_front < pipe_detect_dist)──→ ALIGN
ALIGN  ──(walls both sides + front clear + aligned + settled)──→ TRANSIT
ALIGN  ──(front clear + no walls)──→ CRUISE
TRANSIT ──(walls gone)──→ CRUISE
```

## States

### CRUISE
- **When:** Open field, no pipe ahead
- **X:** Full speed: `vx_target = vx_gain * range_front`
- **Y:** Damp only: `acc_y = -Kd_y * vy`

### ALIGN
- **When:** Pipe detected ahead (`range_front < pipe_detect_dist`)
- **X:** Slow down, more if gap off-center: `vx_target = vx_gain * range_front * (1 - slow_factor * |err_y|)`
- **Y:** Steer toward gap: `acc_y = Kp_y * err_y - Kd_y * vy`

### TRANSIT
- **When:** Inside gap (walls above AND below, front clear, `|err_y| < 0.05`, `|vy| < 0.15`)
- **X:** Boost speed: `vx_target = vx_transit`
- **Y:** Aggressive damp: `acc_y = -Kd_transit * vy`

## Gap Detection

- Range²-weighted angular average across all 9 rays
- Long-range rays dominate (r² weighting)
- Blended with prior from last known gap (30% weight when in CRUISE)
- EMA filtered with `alpha`

## Sensor Layout

```
Ray 0: -45° (down)    Ray 4: 0° (forward)    Ray 8: +45° (up)
Increment: 11.25°     range_front = min(ray3, ray4, ray5)
```

## Parameters (controller.yaml)

| Param | Effect | Tune direction |
|---|---|---|
| `pipe_detect_dist` | When ALIGN starts | ↑ = more reaction time, slower |
| `alpha` | EMA responsiveness | ↑ = faster response, noisier |
| `vx_gain` | Cruise speed | ↑ = faster, less braking margin |
| `vx_transit` | Gap transit speed | ↑ = less blind time in gap |
| `slow_factor` | ALIGN slowdown for y-error | ↑ = slower approach when misaligned |
| `Kp_x` | X accel response | Keep Kp_x * max_vx_error < 3.0 |
| `Kp_y` | Y steering strength | Keep Kp_y * 0.4 < 35 (no saturation) |
| `Kd_y` | Y damping | ≈ 2√Kp_y for critical damping |
| `Kd_transit` | Transit damping | Higher = faster vy kill in gap |
| `gap_threshold` | Wall detection range | Rays below this = pipe beside bird |

## Tuning Order

1. Set `Kp_y=0, Kd_y=0`. Tune `Kp_x` and `vx_gain` until x-motion is smooth
2. Set `Kp_y=20, Kd_y=6`. Increase `Kp_y` until bird reaches gap. Add `Kd_y` if oscillating
3. Keep `Kd_y ≈ 2√Kp_y`
4. Verify no saturation: `Kp_y * max_err_y` should stay below ~25 m/s²
5. Tune `vx_transit` and `Kd_transit` for gap crossing
6. Increase speed: `vx_gain` → `vx_transit` → `Kp_x` (in that order)

## Limits

- Max acc x: 3.0 m/s²
- Max acc y: 35.0 m/s²
- Max laser range: 3.55m (355 pixels × 0.01)
- FOV: 90° (9 rays)
- Update rate: 30 Hz
- Game duration: 60s
