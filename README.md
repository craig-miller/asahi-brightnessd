# asahi-brightnessd

Ambient-light-driven auto-brightness for Apple Silicon MacBooks running
Asahi Linux. Drives both the display backlight and the keyboard backlight
from a single ALS reading, with smooth transitions, per-channel manual
override, and AC-vs-battery curve scaling.

## How it works

```
VD6286 ALS sensor → aop_als kernel driver → IIO sysfs
    │
    └─ asahi-brightnessd polls
       /sys/bus/iio/devices/iio:deviceN/in_illuminance_input

       lux                          power source
        │                                │
        ├──── screen curve + AC boost ───┤
        │                                │
        ├──── kbd curve ─────────────────┘
        │
        ├──→ /sys/class/backlight/apple-panel-bl/brightness
        └──→ /sys/class/leds/kbd_backlight/brightness
```

**Two curves, one sensor.** A piecewise-linear lux → percentage table
maps the same ALS reading to two outputs: the screen curve rises with
ambient light (the way macOS does), and the keyboard curve does the
inverse — bright kbd in a dark room, off in daylight.

**Smooth transitions.** 30 Hz polling, exponential smoothing — each
tick moves 1/16 of the remaining gap. ~1.2 s for 90 % convergence on
the largest possible swing (full cover → full daylight), with per-tick
deltas small enough to fall below perceptual threshold.

**Manual override per channel.** Each channel tracks the last value it
wrote and compares against the observed value on the next read. When a
discrepancy is seen — typically from `brightnessctl` invoked by a
compositor keybind — the daemon yields that channel and stops writing.
Ambient tracking resumes once lux shifts by ≥ 75 % (or ≥ 5 absolute in
low light), at which point the daemon ramps smoothly from the user's
value to the new ambient-driven target.

**AC awareness.** When `/sys/class/power_supply/macsmc-ac/online`
reports 1, the screen target gets a configurable percentage-point
boost (default `+30`), clamped to 100. The keyboard curve is
unaffected — visibility in the dark is power-source neutral. The AC
state is re-read once per second; transitions on plug/unplug ride the
same smoothing loop as ambient changes.

## Prerequisites

- M-series MacBook running Asahi Linux (kernel with
  `CONFIG_IIO_AOP_SENSOR_ALS=m`)
- ALS calibration blob extracted from macOS and installed at
  `/lib/firmware/apple/aop-als-cal.bin`. The
  [juicecultus/asahi-auto-brightness](https://github.com/juicecultus/asahi-auto-brightness)
  repo contains the `extract-als-cal.py` script and the recipe
- A C compiler (any working `cc`)

## Build & install (manual)

```sh
make
sudo make install                                    # → /usr/sbin/asahi-brightnessd
sudo install -m755 asahi-brightnessd.openrc /etc/init.d/asahi-brightnessd
sudo rc-update add asahi-brightnessd default
sudo rc-service asahi-brightnessd start
```

On Gentoo, an ebuild lives in `craig-miller/zentoo-overlay` (and is
documented in the install-gentoo guide).

## Compositor key bindings

The daemon stays out of the way when something else writes the
brightness file. Bind `XF86MonBrightnessUp`/`Down` to `brightnessctl`
normally; the daemon detects the external write and yields the channel
until ambient changes again.

niri (`~/.config/niri/config.kdl`):

```kdl
binds {
    XF86MonBrightnessUp   { spawn "brightnessctl" "set" "5%+"; }
    XF86MonBrightnessDown { spawn "brightnessctl" "set" "5%-"; }

    // M1 Pro has no dedicated keyboard-backlight key — pick a binding:
    Mod+F1 { spawn "brightnessctl" "--device=kbd_backlight" "set" "10%-"; }
    Mod+F2 { spawn "brightnessctl" "--device=kbd_backlight" "set" "10%+"; }
}
```

## Tuning

All tunables are compile-time constants at the top of
`asahi-brightnessd.c`. Edit and rebuild — no config file by design.

| Constant            | Default       | Effect                                                                  |
|---------------------|---------------|-------------------------------------------------------------------------|
| `POLL_NSEC`         | `33_000_000`  | Polling interval in nanoseconds. 33 ms = ~30 Hz.                        |
| `SMOOTH_SHIFT`      | `4`           | Exponential smoothing exponent. Per-tick move = `delta >> SMOOTH_SHIFT`. Higher = slower & smoother. |
| `LUX_RESUME_PCT`    | `75`          | Lux delta (%) needed to clear an override.                              |
| `LUX_RESUME_ABS`    | `5`           | Absolute lux floor for override-clear (matters in dark conditions).     |
| `SCREEN_CURVE[]`    | 10 points     | lux → screen brightness percentage. Lifted from juicecultus/auto-brightness. |
| `KBD_CURVE[]`       | 5 points      | lux → keyboard backlight percentage (inverse).                          |
| `SCREEN_FLOOR_PCT`  | `2`           | Minimum screen brightness — prevents fully-dark screen.                 |
| `KBD_FLOOR_PCT`     | `0`           | Minimum keyboard backlight.                                             |
| `AC_SCREEN_BOOST`   | `30`          | Percentage-point bump added to screen target when on AC, clamped to 100.|
| `AC_RECHECK_TICKS`  | `30`          | How often to re-read AC state (in ticks). 30 ticks @ 30 Hz = 1 Hz.      |

## License

MIT. Curve constants and the override-hysteresis idea are adapted from
[juicecultus/asahi-auto-brightness](https://github.com/juicecultus/asahi-auto-brightness)
(also MIT).
