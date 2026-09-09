# ShaiaController

The `ShimonController-choreo` C++ controller, rebuilt for Shaia (mini-Shimon):
2 slider arms + 4 strikers on one OpenCR board, and a 5 axis Dynamixel head.

It keeps the choreo architecture — OSC in, per-arm command queues, a striker
scheduler, gesture objects that push head primitives onto a queue — and swaps
in Shaia's hardware and Shaia's path planning.

```
master OSC client ──UDP──▶ OscListener ──▶ ArmController ──▶ McuTransport ──serial──▶ Strikers.ino ──CAN──▶ 2 sliders + 4 strikers
                                 │                └─▶ StrikerController ──┘
                                 └──────────────▶ HeadController ──▶ MotorController ──▶ DxlController ──▶ 5 Dynamixels
                                 ▲                        └─▶ Gestures (dance / look / breathe / headbang)
Shaia ──UDP──▶ OscTransmitter ───┘  (arm positions + status back to master)
```

## Build

```bash
mkdir -p build && cd build
cmake ..
make -j
./ShaiaController
```

Run it from `build/` — `HM_CONFIG_FILE` and `DANCE_MODES_FILE` in `Include/Def.h`
are the relative paths `../Config/...`.

`Strikers/Strikers.ino` is **not** part of the CMake build; it is an
Arduino/OpenCR sketch flashed onto the MCU.

### Compile-time switches (`Include/Def.h`)

- `SIMULATE` — stub out every serial and Dynamixel write. The planner, the
  gesture engine and the OSC layer all still run, so you can exercise the whole
  controller with no hardware attached.
- `USE_ARMS` / `USE_HEAD` — gate which subsystem `Shaia::init` brings up.
- `MCU_PORT` / `DXL_PORT` — the two USB devices. `MCU_PORT` defaults to the
  OpenCR (`/dev/tty.usbmodemFFFFFFFEFFFF1`), `DXL_PORT` to the head's FTDI
  adapter (`/dev/tty.usbserial-FT62AP2P`).

## OSC API

Listens on port 9000 (`PORT`), transmits arm positions and status to
`MASTER_HOST:MASTER_PORT`.

| Route | Args | Effect |
| --- | --- | --- |
| `/arm` | `<note> <velocity>` | MIDI note trigger; velocity 0 is ignored |
| `/arm` | `<cmd>` | `HOME`, `RESTART` (`ON`/`OFF`/`CLEAR` are accepted and logged as unsupported) |
| `/arm` | `<armId> <pos> <acc> <v_max>` | Raw move, `armId` is 1-indexed, `pos` in rail mm |
| `/arm_home` | – | Home the sliders |
| `/arm_reset` | – | Restart the MCU controller |
| `/striker` | `<ids> <position> <time_ms>` | Choreo move, `ids` is a bitmask |
| `/striker` | `<mode> <ids> <velocity> <channel>` | Direct strike, mode `s`/`t`/`p` |
| `/head` | `<gesture> [bpm]` | Start a looping gesture from `Config/danceModes.json` |
| `/head_stop` | – | Stop the current gesture |
| `/head_look` | – | One-shot "look at the audience" pose |
| `/head-commands` | `<axis> <pos> [vel]` | Low level move. `axis` ∈ HEADTURN, HEADTILT, MOUTH, NECKTILT, NECKTURN; `pos` and `vel` normalized [0, 1] |
| `/head-commands` | `BREATHING` / `STOPBREATH` / `BEAT <ms>` / `LOOK` / `STOP` | Behavioural commands |
| `/shimon` | `quit` / `exit` / `sethost` | System messages |

`osc_test.py` drives a few of these for a smoke test.

## Calibrating the key positions

`kNotePositionTable` in `Include/Def.h` is the one thing that can only come off
the physical rig. `calibrate_keys.py` captures it by hand:

```bash
# stop ShaiaController first - both want the same serial port
python3 calibrate_keys.py --port /dev/tty.usbmodemFFFFFFFEFFFF1 --arm 1
```

It homes the sliders, releases the chosen arm so you can push it, then walks
that arm's own range. Park the arm over a bar and press Enter to record that key;
`p` peeks without saving, `s` skips, `b` goes back, `q` finishes early, and
typing a number records that rail position directly. The arm is always
re-engaged on the way out — including on Ctrl-C — holding wherever you left it
rather than snapping back to its last commanded target.

Each arm walks 30 of the 31 keys, and the two ranges differ: an arm cannot reach
the key at the *other* arm's home end, because that arm is parked on it.

| Arm | Walks | Gives up |
| --- | --- | --- |
| 1 (low end, parks at rail 0) | F3..A#5 (53–82) | B5 (83) — arm 2 parks there |
| 2 (high end, parks at `SLIDER_LIMIT`) | F#3..B5 (54–83) | F3 (53) — arm 1 parks there |

So one pass can never fill the whole table. Run both arms and merge, or keep the
existing `Def.h` value for the one key the pass skipped. `--full-range` walks
every key anyway if the rig disagrees.

It prints a ready-to-paste `kNotePositionTable` (and the `defs.py` form), plus
per-key deltas and a monotonicity check, and writes `Config/keyPositions.json`.
The pasteable table always spans the full `NUM_KEYS`, with `None` in the slot
this arm could not reach — so it drops into `Def.h` without renumbering
`MIN_NOTE`/`MAX_NOTE`. Both arms write the same file, so the second pass
overwrites the first.

Calibrate with **arm 1**: its rail transform is the identity, so the readings
*are* rail millimetres. Arm 2 needs the second pass below.

### Arm 2's offset

`SLIDER_LIMIT` is arm 2's home expressed in rail coordinates, and it is the only
thing tying arm 2's readings to arm 1's — get it wrong and every arm 2 position
shifts by the same error. Arm 2 counts ticks *inward* from its home, so parking
it over a key whose rail position you already know gives it up:

```
SLIDER_LIMIT = railMm(key) + ticks / ticksPerMm
```

```bash
python3 calibrate_keys.py --port /dev/tty.usbmodemFFFFFFFEFFFF1 --mode arm2
```

It homes, releases arm 2, and asks you to park it over B5 (the highest key). The
reference position comes from the `keyPositions.json` you just captured, falling
back to `Def.h`'s table, or `--ref-mm` — so run the arm 1 pass first and the
reference is measured rather than assumed. It prints the `SLIDER_LIMIT` to paste
into `Def.h`, how far off the current value is, and how much clearance is left
between the highest key and arm 2's home. Output lands in
`Config/arm2Calibration.json`.

Referencing a shared bar beats measuring the two homes against each other — the
other arm is usually parked in the way.

Passing `--ref-note` two or more times (`--ref-note 83 --ref-note 72 --ref-note
60`) fits the encoder scale at the same time by least squares. If the fitted
ticks/mm disagrees with `SLIDER_TICKS_PER_REV` by more than 2% it says so, which
is the same class of error that put the nameplate 25.6 ticks/mm three times off
the real 76.8.

Constants are read out of `Def.h` at startup (encoder scale, rail length, arm
transforms), so a calibration can't silently disagree with what the controller
believes; `--ticks-per-mm` overrides the encoder scale if you are calibrating
that too.

Arm numbers and slider ids are not the same thing: the two drives are addressed
the other way round from the arm order, so arm 1 is CAN node 2 and arm 2 is node
1. `kArmToSliderId` in `Def.h` is the single place that mapping lives —
`McuTransport` and `calibrate_keys.py` both read it, and `--arm` stays the arm
number you see on the rig.

This needs the three calibration frames in the firmware — `f` (release), `l`
(re-engage), `g` (report position, replying `POS <id> <ticks>`). Position is read
over SDO, so it works while the drive is disabled.

## What changed from ShimonController-choreo

### Path planning (`Shaia/Src/ArmController/`)

Rewritten to match Shaia's Python runtime (`shaia_targetUpdated/arms.py`), which
is the calibrated source of truth for the rig:

- **Fixed strike latency.** Every move is given the full `DLY` (500 ms) window
  instead of being squeezed into the gap since the arm's last arrival, so short
  and long moves share one gentle acceleration profile. Peak velocity and
  acceleration then scale with distance rather than distance/time².
- **Busy arms are skipped, not re-timed.** If `mostRecentArrival > now` the arm
  cannot make the beat, so it drops out of the candidate list for that octave.
- **Ranked by travel distance**, not by acceleration as choreo did.
- **Same-note buffer.** A repeat of the same incoming note within
  `NOTE_BUFFER_WINDOW` (500 ms) reuses the previous arm and sends only the
  striker frame — the slider is already in position, and this stops the
  busy-arm check from bouncing a tremolo across the rail.
- Only 3 transpositions are tried (`0, +1, -1`); choreo also tried `+2`.

### Geometry (`Include/Def.h`)

| | choreo (Shimon) | ShaiaController |
| --- | --- | --- |
| Arms | 4 | 2 |
| Strikers | 8 | 4 |
| Note range | 48–95 (49 keys) | 53–83, F3–B5 (31 keys) |
| Rail | 1385 mm | 682 mm |
| Note spacing | measured per key | uniform 20 mm, +35 mm at each B→C |
| Latency | 465 ms | 500 ms |

`kMmToTicks` is 76.8 ticks/mm (3072 counts/rev over 40 mm/rev) — the empirical
figure from `defs.py`, three times the EC45 nameplate, which is what the rig
actually measures.

### Slider and striker I/O

`IAIController` (Modbus/RTU over RS-485) and `StrikerHandler` are replaced by
one `McuTransport`: both sliders and strikers are EPOS4 nodes on the MCU's CAN
bus, so the host only talks to the MCU. The wire format matches
`shaia_targetUpdated/transport.py`, so the existing Python tooling still works
against the same firmware:

```
slider   'm' sliderId  tickHi tickLo durHi durLo '\n'   (7 B)
striker  mode bitmask  midiVelocity                '\n' (4 B)
choreo   'c' bitmask   posHi  posLo  msHi  msLo   '\n'  (7 B)
home     'h' 0 0                                   '\n' (4 B)
restart  'r' 0 0                                   '\n' (4 B)
free     'f' sliderId 0                            '\n' (4 B)   calibration
lock     'l' sliderId 0                            '\n' (4 B)   calibration
get pos  'g' sliderId 0                            '\n' (4 B)   -> "POS <id> <ticks>"
```

Striker ids run on from the sliders — arm 0 gets `{black: 3, white: 4}`, arm 1
`{black: 5, white: 6}` — and go out as a bitmask, `1 << (id - 1)`.

The host still models each arm's trajectory locally (`SliderModel`) purely so it
can report position back to the master at 1 kHz; the MCU owns the real motion.

### Head

The Copley/HD half is gone. All five axes are protocol 2.0 Dynamixels on one
bus, so `CopleyASCIIController`, `HDMotor`, `DofLimits` and the `Motor` type
hierarchy are dropped in favour of `DxlController` + `HeadMotor`.

Positions are **normalized [0, 1]** and map onto a per-axis degree range from
`Config/HeadMotorConfig.yaml`, exactly like Shaia's Python:

```
goal_deg = normalized * (max - min) + min
```

Several axes have `min > max` on purpose — that is how the ranges are wired in
`Shaia/GestureInput/Shaiahead.py`. NeckTurn runs in extended position mode
(operating mode 4) because its range crosses 0°.

### Gestures (`Shaia/Src/HeadController/Include/Gestures/`)

- **`DanceGesture`** — new. A C++ port of `head.py`'s `GestureRunner`, driven by
  `Config/danceModes.json` (the same file Shaia's Python uses). One gesture
  loops at a time; `play()` swaps atomically via a generation counter. Beat
  offsets and periods are scaled by 60/bpm, and the incoming tempo is folded by
  factors of 2 into the `bpm_min`/`bpm_max` range each mode was choreographed
  for (from `dance.py`). Parsed with yaml-cpp — JSON is a subset of YAML 1.2, so
  no extra dependency.
  Ships with: `nod_sway`, `ar_sway`, `lq_sway`, `head_circle`, `circle`,
  `build`, `ballet_nod`, `ballet_sway`.
- **`LookGesture`** — new. The one-shot pose from `head.py`'s `look()`.
- **`BreatheGesture`** / **`HeadBangGesture`** — the choreo gestures, retargeted
  from Copley radians to Shaia's NeckTilt/HeadTilt in normalized units.

### Firmware (`Strikers/`)

`Strikers.ino` is new: it drives 2 sliders as well as 4 strikers, and parses the
5 frame types above rather than choreo's single 4-byte striker frame.

- **`src/slider.h`** — new. An EPOS4 slider in Cyclic Synchronous Position mode.
  `moveTo(ticks, ms)` runs a linear segment with parabolic blends at both ends,
  evaluated per 1 ms PDO tick so no multi-thousand point array is needed.
  `home()` runs the drive into its hard stop on a current threshold.
- **`src/slider.h`** also carries `release()`/`hold()`/`readPosition()` for hand
  calibration; `hold()` re-engages at the axis' *actual* position, so releasing
  and re-locking never makes the arm jump.
- **`src/shaiaController.h`** — replaces `strikerController.h`. Fans CAN receive
  and the PDO tick out to two different node types, and packs the motor arrays
  instead of indexing them by node id (each `Striker` carries a 20 KB trajectory
  buffer, so the choreo layout would have wasted ~60 KB of SRAM here).
- **`src/striker.h`** — the choreo striker, with one fix: the choreo build read
  the choreo-mode position and time parameters in the opposite order from the
  one the host sends them in.

### Robustness fixes carried over the port

- `OscListener::ProcessPacket` catches parse errors. oscpack constructs
  `ReceivedMessage` outside `ProcessMessage`'s try block, so in the choreo build
  a single malformed UDP datagram terminated the process.
- `stop()` marks both subsystems uninitialized, and `main` tears the controller
  down before returning. Previously the destructors ran during static
  destruction and reached into serial/Dynamixel singletons that had already been
  destroyed.

## Layout

```
Include/            Def.h (all geometry + timing), Util.h, logging, error codes
Config/             HeadMotorConfig.yaml, danceModes.json, calibration output
calibrate_keys.py   hand-capture the key table (--mode keys) and arm 2's offset (--mode arm2)
Shaia/Include/      Shaia, OscListener/Transmitter, CommandManager, SerialDevice
Shaia/Src/ArmController/    Arm, ArmController (planner), StrikerController, McuTransport
Shaia/Src/HeadController/   HeadController, MotorController, HeadMotor, DxlController, Gestures/
Strikers/           OpenCR sketch + EPOS4 stack
3rdparty/           oscpack, serialib, DynamixelSDK, yaml-cpp, spdlog
```
