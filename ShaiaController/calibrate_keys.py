"""Hand-calibrate Shaia's rail: the key positions, and arm 2's offset.

Stop ShaiaController before running this - both want the same serial port.

--mode keys (default)
    Homes the sliders, releases one arm so you can push it by hand, then walks
    the key range: park the arm over a bar, press Enter, and its encoder
    position is recorded for that MIDI note. The result is the
    NOTE_POSITION_TABLE that Include/Def.h (and defs.py) reference.

        python3 calibrate_keys.py --port /dev/tty.usbmodem... --arm 1

--mode arm2
    Derives SLIDER_LIMIT, which is arm 2's home expressed in rail coordinates
    and the only thing tying arm 2's readings to arm 1's. Park arm 2 over a key
    whose rail position is already known - B5, the highest, by default - and

        SLIDER_LIMIT = railMm(key) + ticks / ticksPerMm

    Run it after a keys pass on arm 1, so the reference position is measured
    rather than assumed:

        python3 calibrate_keys.py --port /dev/tty.usbmodem... --mode arm2

    Give --ref-note twice or more to fit the encoder scale at the same time.

At each keys-mode prompt:
    Enter    save the arm's current position for this key
    <mm>     type a number to record that rail position by hand instead
    p        peek: print where the arm is without saving
    s        skip this key (left as a gap in the table)
    b        go back to the previous key
    q        stop here and write out what has been captured

Requires the firmware in Strikers/ (the 'f'/'l'/'g' frames).
"""

import argparse
import json
import os
import queue
import re
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")


HERE = os.path.dirname(os.path.abspath(__file__))
DEF_H = os.path.join(HERE, "Include", "Def.h")

NOTE_NAMES = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")

POS_RE = re.compile(r"^POS\s+(\d+)\s+(-?\d+)\s*$")


def note_name(midi: int) -> str:
    return f"{NOTE_NAMES[midi % 12]}{midi // 12 - 1}"


def notes_for_arm(arm_index: int, notes):
    """The keys this arm can actually reach, and the one key it gives up.

    Each arm loses one key at the far end of the rail, because the other arm is
    parked there and in the way: arm 2 homes 6 mm above the highest key and arm
    1 homes 6 mm below the lowest, so neither can park a striker over the key at
    the other's home end.

        arm 1 (index 0, low end)   gives up the HIGHEST key
        arm 2 (index 1, high end)  gives up the LOWEST key

    That makes the two arms' ranges differ by one key at each end, which is also
    why a keys pass on one arm cannot fill in the whole table on its own.
    """
    notes = list(notes)
    if len(notes) < 2:
        return notes, None
    if arm_index == 0:
        return notes[:-1], notes[-1]
    return notes[1:], notes[0]


# --------------------------------------------------------------------------
# Constants come from Def.h so a calibration can never silently disagree with
# what the controller believes about the rail.
# --------------------------------------------------------------------------

DEFAULTS = {
    "MIN_NOTE": 53,
    "MAX_NOTE": 83,
    "SLIDER_LIMIT": 682,
    "ARM0_HOME_MM": 682,
    "ARM1_HOME_MM": 0,
    "SLIDER_TICKS_PER_REV": 3072,
    "SLIDER_MM_PER_REV": 40,
}


def parse_def_h(path: str) -> dict:
    out = dict(DEFAULTS)
    out["kSliderW"] = None
    out["kSliderB"] = None
    out["kArmToSliderId"] = None
    out["note_table"] = {}

    try:
        with open(path) as f:
            text = f.read()
    except OSError as ex:
        print(f"! could not read {path} ({ex}); using built-in defaults")
        return out

    for key in DEFAULTS:
        m = re.search(rf"^#define\s+{key}\s+(-?\d+)", text, re.M)
        if m:
            out[key] = int(m.group(1))

    # A define may alias another define rather than give a literal -
    # ARM1_HOME_MM is #define'd to SLIDER_LIMIT. Resolve those afterwards, and
    # repeat so a chain of aliases settles.
    for _ in range(len(DEFAULTS)):
        changed = False
        for key in DEFAULTS:
            m = re.search(rf"^#define\s+{key}\s+(-?)([A-Za-z_]\w*)", text, re.M)
            if m and m.group(2) in out:
                value = int(out[m.group(2)]) * (-1 if m.group(1) else 1)
                if out[key] != value:
                    out[key] = value
                    changed = True
        if not changed:
            break

    # The arrays mix literals and macro names - kSliderB is { 0, SLIDER_LIMIT } -
    # so resolve identifiers against the defines we just read.
    for name in ("kSliderW", "kSliderB", "kArmToSliderId"):
        m = re.search(rf"(?:const|constexpr) int {name}\[NUM_ARMS\]\s*=\s*\{{([^}}]*)\}}", text)
        if not m:
            continue

        values = []
        for token in (t.strip() for t in m.group(1).split(",")):
            if not token:
                continue
            # A macro may be negated in place - kSliderB is { 0, -ARM1_HOME_MM }.
            sign = 1
            if token.startswith("-") and not re.fullmatch(r"-?\d+", token):
                sign, token = -1, token[1:].strip()
            if re.fullmatch(r"-?\d+", token):
                values.append(sign * int(token))
            elif token in out:
                values.append(sign * int(out[token]))
            else:
                print(f"! cannot resolve '{token}' in {name}; falling back to defaults")
                values = []
                break

        # NUM_ARMS is 2; anything else means the parse went wrong.
        if len(values) == 2:
            out[name] = values

    if not out["kSliderW"]:
        out["kSliderW"] = [1, 1]
    if not out["kSliderB"]:
        out["kSliderB"] = [0, -out["ARM1_HOME_MM"]]
    if not out["kArmToSliderId"]:
        out["kArmToSliderId"] = [2, 1]

    # The note table, so arm 2 can be referenced against a key whose rail
    # position is already known.
    m = re.search(r"const int kNotePositionTable\[NUM_KEYS\]\s*=\s*\{([^}]*)\}", text)
    if m:
        values = [int(v) for v in re.findall(r"-?\d+", m.group(1))]
        out["note_table"] = {out["MIN_NOTE"] + i: v for i, v in enumerate(values)}

    return out


# --------------------------------------------------------------------------
# MCU link
# --------------------------------------------------------------------------

class Mcu:
    """The same frames McuTransport.h sends, plus the calibration ones."""

    def __init__(self, port: str, baud: int, verbose: bool):
        self._ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        self._verbose = verbose
        self._lines: "queue.Queue[str]" = queue.Queue()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, name="mcu-rx", daemon=True)
        self._reader.start()

    def _read_loop(self) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except Exception:
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, _, buf = buf.partition(b"\n")
                text = line.rstrip(b"\r").decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                self._lines.put(text)
                if self._verbose or not POS_RE.match(text):
                    print(f"    [mcu] {text}")

    def _send(self, frame: bytes) -> None:
        self._ser.write(frame)

    def drain(self) -> None:
        while True:
            try:
                self._lines.get_nowait()
            except queue.Empty:
                return

    def home(self) -> None:
        self._send(b"h\x00\x00\n")

    def free(self, slider_id: int) -> None:
        self._send(bytes([ord("f"), slider_id, 0, ord("\n")]))

    def lock(self, slider_id: int) -> None:
        self._send(bytes([ord("l"), slider_id, 0, ord("\n")]))

    def position(self, slider_id: int, timeout: float = 2.0):
        """Ticks reported by slider_id, or None on timeout."""
        self.drain()
        self._send(bytes([ord("g"), slider_id, 0, ord("\n")]))

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = self._lines.get(timeout=0.1)
            except queue.Empty:
                continue
            m = POS_RE.match(line)
            if m and int(m.group(1)) == slider_id:
                return int(m.group(2))
        return None

    def close(self) -> None:
        self._stop.set()
        self._reader.join(timeout=0.5)
        self._ser.close()


# --------------------------------------------------------------------------
# Calibration
# --------------------------------------------------------------------------

# The "g" frame reports the drive's raw encoder value (Epos4::getActualPosition
# does NOT apply m_iDirMultiplier, unlike PDO_setPosition). Each slider homes by
# driving POSITIVE into its own hard stop and reads 0 there, so every position
# in the key range is NEGATIVE raw - the mirror image of the positive ticks the
# host commands. Convert once, here, rather than sprinkling minus signs around.
RAW_TICKS_SIGN = -1


class Calibration:
    def __init__(self, cfg: dict, arm_index: int, mm_to_ticks: float):
        self.cfg = cfg
        self.arm_index = arm_index
        # The wire id is the drive's CAN node id, and the two slider drives are
        # addressed the other way round from the arm order - so it is not
        # arm_index + 1. Def.h's kArmToSliderId is the one source of truth.
        self.slider_id = cfg["kArmToSliderId"][arm_index]
        self.mm_to_ticks = mm_to_ticks
        self.w = cfg["kSliderW"][arm_index]
        self.b = cfg["kSliderB"][arm_index]

    def ticks_to_rail_mm(self, ticks: int) -> float:
        """Raw encoder ticks (as the 'g' frame reports them) -> rail mm.

        Def.h maps the other way: ticks = (W * railMm + B) * mmToTicks, in the
        host's commanded sign. RAW_TICKS_SIGN converts the drive's raw reading
        into that convention first.
        """
        arm_mm = (RAW_TICKS_SIGN * ticks) / self.mm_to_ticks
        return (arm_mm - self.b) / self.w

    def rail_mm_to_ticks(self, rail_mm: float) -> int:
        """Rail mm -> raw encoder ticks, comparable with a 'g' reading."""
        commanded = (self.w * rail_mm + self.b) * self.mm_to_ticks
        return int(round(RAW_TICKS_SIGN * commanded))


def format_table(notes, positions, per_row: int = 10) -> str:
    cells = []
    for n in notes:
        p = positions.get(n)
        cells.append("None" if p is None else str(int(round(p))))
    width = max(len(c) for c in cells) if cells else 1

    rows = []
    for i in range(0, len(cells), per_row):
        rows.append("    " + " ".join(f"{c:>{width}}," for c in cells[i:i + per_row]))
    return "\n".join(rows)


def report(notes, positions, cal: Calibration, table_notes=None) -> None:
    """`notes` is what this arm walked; `table_notes` is the instrument's full
    key range. They differ because each arm gives up one key at the far end of
    the rail (see notes_for_arm), and Def.h's table has to span the instrument,
    not one arm's reach - so the pasteable block below is emitted over
    `table_notes` with a None wherever this pass has no value."""
    if table_notes is None:
        table_notes = notes

    captured = [n for n in notes if positions.get(n) is not None]
    missing = [n for n in notes if positions.get(n) is None]
    unreachable = [n for n in table_notes if n not in notes]

    print()
    print("=" * 66)
    print(f"Captured {len(captured)} of {len(notes)} keys")
    if missing:
        print("Missing: " + ", ".join(f"{note_name(n)}({n})" for n in missing))
    if unreachable:
        print("Out of this arm's reach: "
              + ", ".join(f"{note_name(n)}({n})" for n in unreachable)
              + " - calibrate with the other arm to fill these in")

    if len(captured) >= 2:
        print()
        print("  note        midi     rail mm      ticks     delta mm")
        prev = None
        for n in captured:
            mm = positions[n]
            delta = "" if prev is None else f"{mm - positions[prev]:8.1f}"
            print(f"  {note_name(n):<6}  {n:>6}  {mm:10.1f}  {cal.rail_mm_to_ticks(mm):9d}  {delta:>10}")
            prev = n

        deltas = [positions[b] - positions[a] for a, b in zip(captured, captured[1:])]
        if any(d <= 0 for d in deltas):
            print("\n! positions are not monotonically increasing - check the flagged steps above")
        print(f"\n  span {positions[captured[0]]:.1f} -> {positions[captured[-1]]:.1f} mm"
              f"   (min step {min(deltas):.1f}, max step {max(deltas):.1f})")

        # max(), not the last entry - the table can be captured out of order.
        highest = max(positions[n] for n in captured)
        print(f"\n  SLIDER_LIMIT must be at least {int(highest) + 6} mm"
              f" (furthest key {highest:.1f} + 6 mm clearance); Def.h currently says {cal.cfg['SLIDER_LIMIT']}")

    print()
    print("-- Include/Def.h " + "-" * 49)
    if unreachable:
        print(f"// {len(unreachable)} key(s) show as None - this arm cannot reach them.")
        print("// Run the other arm and merge, or keep the existing Def.h values.")
    print(f"#define MIN_NOTE {table_notes[0]}")
    print(f"#define MAX_NOTE {table_notes[-1]}")
    print(f"#define NUM_KEYS {len(table_notes)}")
    print("const int kNotePositionTable[NUM_KEYS] = {")
    print(format_table(table_notes, positions))
    print("};")

    print()
    print("-- defs.py " + "-" * 55)
    print("NOTE_POSITION_TABLE = (")
    print(format_table(table_notes, positions))
    print(")")
    print("=" * 66)


def prompt_loop(mcu: Mcu, cal: Calibration, notes) -> dict:
    positions: dict = {}
    idx = 0

    print()
    print(f"Arm {cal.arm_index} (slider {cal.slider_id}) is released - push it by hand.")
    print("Enter = save, <mm> = type a position, p = peek, s = skip, b = back, q = done")
    print()

    while idx < len(notes):
        note = notes[idx]
        have = positions.get(note)
        suffix = f"  (currently {have:.1f} mm)" if have is not None else ""
        try:
            answer = input(f"[{idx + 1}/{len(notes)}] {note_name(note)} (MIDI {note}){suffix} > ").strip().lower()
        except EOFError:
            print()
            break

        if answer == "q":
            break

        if answer == "b":
            idx = max(0, idx - 1)
            continue

        if answer == "s":
            positions[note] = None
            idx += 1
            continue

        if answer in ("", "p"):
            ticks = mcu.position(cal.slider_id)
            if ticks is None:
                print("    ! no reply from the MCU - is the firmware running? (not saved)")
                continue

            mm = cal.ticks_to_rail_mm(ticks)
            if answer == "p":
                print(f"    peek: {mm:.1f} mm rail  ({ticks} ticks)")
                continue

            positions[note] = mm
            print(f"    saved {note_name(note)} = {mm:.1f} mm rail  ({ticks} ticks)")
            idx += 1
            continue

        try:
            mm = float(answer)
        except ValueError:
            print("    ? unrecognised input")
            continue

        positions[note] = mm
        print(f"    saved {note_name(note)} = {mm:.1f} mm rail (entered by hand)")
        idx += 1

    return positions


# --------------------------------------------------------------------------
# Arm 2 offset
#
# Arm 2 parks at the far end of the rail and counts ticks *inward*, so Def.h
# describes it as  ticks = (SLIDER_LIMIT - railMm) * ticksPerMm.  SLIDER_LIMIT
# is therefore arm 2's home expressed in rail coordinates, and it is the one
# number that ties arm 2's readings to arm 1's.
#
# Park arm 2 over a key whose rail position is already known (B5, the highest,
# by default) and the offset falls out:
#
#     SLIDER_LIMIT = railMm(key) + ticks / ticksPerMm
#
# Referencing a shared bar beats pushing arm 1 into arm 2's parking spot - the
# other arm is usually in the way.
# --------------------------------------------------------------------------

def resolve_reference_mm(notes, explicit, keys_json: str, cfg: dict):
    """Rail position for each reference note: CLI > a previous run > Def.h."""
    resolved = {}
    source = {}

    for i, note in enumerate(notes):
        if explicit and i < len(explicit):
            resolved[note] = explicit[i]
            source[note] = "--ref-mm"

    captured = {}
    if keys_json and os.path.exists(keys_json):
        try:
            with open(keys_json) as f:
                data = json.load(f)
            for k, v in (data.get("positions_mm") or {}).items():
                if v is not None:
                    captured[int(k)] = float(v)
        except (OSError, ValueError, TypeError) as ex:
            print(f"! could not read {keys_json} ({ex})")

    for note in notes:
        if note in resolved:
            continue
        if note in captured:
            resolved[note] = captured[note]
            source[note] = os.path.basename(keys_json)
        elif note in cfg["note_table"]:
            resolved[note] = float(cfg["note_table"][note])
            source[note] = "Def.h"

    return resolved, source


def fit_home_offset(samples, mm_to_ticks: float):
    """Derive ARM1_HOME_MM (and, with 2+ samples, the true ticks/mm).

    samples: [(railMm, ticks), ...]   ticks = (railMm - home) * ticksPerMm

    Both arms count ticks the same way - up the rail, away from a zero at the
    low end - because the firmware gives every slider one homing method and no
    per-node direction. So the only thing separating arm index 1 from arm index
    0 is where its zero landed, which is what this measures.

    (This used to solve for SLIDER_LIMIT under the old ticks = (limit - railMm)
    model, where arm 1 was assumed to home at the high end and count inward.
    The rig disagreed - see kSliderW in Def.h.)
    """
    if not samples:
        return None, None

    if len(samples) == 1:
        rail, ticks = samples[0]
        return rail - ticks / mm_to_ticks, None

    # Least squares on ticks = k*railMm - k*home; the slope gives k and the
    # intercept over -k gives the home offset.
    n = len(samples)
    sx = sum(r for r, _ in samples)
    sy = sum(t for _, t in samples)
    sxx = sum(r * r for r, _ in samples)
    sxy = sum(r * t for r, t in samples)

    denom = (n * sxx) - (sx * sx)
    if abs(denom) < 1e-9:      # every sample at the same place
        rail, ticks = samples[0]
        return rail - ticks / mm_to_ticks, None

    slope = ((n * sxy) - (sx * sy)) / denom
    intercept = (sy - slope * sx) / n

    fitted_k = slope
    if fitted_k <= 0:
        # Ticks FALLING as rail position rises means this arm is still wired
        # the old inward way; fall back rather than inventing a scale.
        rail, ticks = samples[0]
        return rail - ticks / mm_to_ticks, None

    return -intercept / fitted_k, fitted_k


def run_arm2(mcu: Mcu, cfg: dict, mm_to_ticks: float, ref_notes, ref_mm, source):
    """Park arm 2 over each reference key; return [(railMm, ticks, note), ...]."""
    samples = []
    arm2_id = cfg["kArmToSliderId"][1]

    print()
    print(f"Arm 2 (slider {arm2_id}) is released - push it by hand.")
    print("Enter = record, p = peek, s = skip this key, q = stop")
    print()

    for note in ref_notes:
        rail = ref_mm[note]
        while True:
            try:
                answer = input(
                    f"Park arm 2 over {note_name(note)} (MIDI {note}, rail {rail:.1f} mm"
                    f" per {source[note]}) > "
                ).strip().lower()
            except EOFError:
                print()
                return samples

            if answer == "q":
                return samples
            if answer == "s":
                break

            if answer not in ("", "p"):
                print("    ? unrecognised input")
                continue

            ticks = mcu.position(arm2_id)
            if ticks is None:
                print("    ! no reply from the MCU (not recorded)")
                continue

            if answer == "p":
                print(f"    peek: {ticks} ticks -> ARM1_HOME_MM would be {rail - ticks / mm_to_ticks:.1f} mm")
                continue

            samples.append((rail, ticks, note))
            print(f"    recorded {ticks} ticks -> ARM1_HOME_MM {rail - ticks / mm_to_ticks:.1f} mm")
            if RAW_TICKS_SIGN * ticks < 0:
                print("    ! this reads as beyond the arm's own home -"
                      " check homing and the reference position")
            break

    return samples


def report_arm2(samples, cfg: dict, mm_to_ticks: float, highest_key_mm):
    print()
    print("=" * 66)

    if not samples:
        print("Nothing recorded.")
        print("=" * 66)
        return None, None

    print("  ref key    rail mm     ticks     implied ARM1_HOME_MM")
    for rail, ticks, note in samples:
        print(f"  {note_name(note):<6} {note:>4}  {rail:9.1f}  {ticks:8d}  {rail - ticks / mm_to_ticks:16.1f}")

    home, fitted_k = fit_home_offset([(r, t) for r, t, _ in samples], mm_to_ticks)

    cfg_home = -cfg["kSliderB"][1]      # kSliderB[1] is -ARM1_HOME_MM
    print()
    print(f"  ARM1_HOME_MM = {home:.1f} mm   ->  #define ARM1_HOME_MM {int(round(home))}")
    print(f"  Def.h currently implies {cfg_home:.0f}"
          f"  (off by {home - cfg_home:+.1f} mm)")
    print(f"  kSliderW stays {{ 1, 1 }}; kSliderB is {{ 0, -ARM1_HOME_MM }}")

    if fitted_k is not None:
        drift = (fitted_k - mm_to_ticks) / mm_to_ticks * 100.0
        print()
        print(f"  {len(samples)} reference keys also fit the encoder scale:"
              f" {fitted_k:.2f} ticks/mm vs the configured {mm_to_ticks:.2f} ({drift:+.1f}%)")
        if abs(drift) > 2.0:
            print("  ! that is a big disagreement - SLIDER_TICKS_PER_REV is probably wrong,")
            print("    which would also skew every arm 2 position. Re-check before trusting this.")

    if highest_key_mm is not None:
        reach = highest_key_mm - home
        print()
        if reach < 0:
            print(f"  ! arm 2's home is {-reach:.1f} mm ABOVE the highest key -"
                  " every key would need negative ticks, which the wire format"
                  " cannot express")
        else:
            print(f"  arm 2 must travel up to {reach:.1f} mm from its home to reach"
                  f" the highest key ({int(round(reach * mm_to_ticks))} ticks)")

    print("=" * 66)
    return home, fitted_k


# --------------------------------------------------------------------------

def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", choices=("keys", "arm2"), default="keys",
                   help="keys: capture the note position table (default). "
                        "arm2: derive SLIDER_LIMIT from arm 2 parked over a known key")
    p.add_argument("--port", default="/dev/tty.usbmodemFFFFFFFEFFFF1")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--arm", type=int, choices=(1, 2),
                   help="keys mode: slider id to calibrate with (1-indexed). Prompted if "
                        "omitted; arm 1 is preferable, its rail transform is the identity")
    p.add_argument("--min-note", type=int, help="lowest MIDI note (default: Def.h MIN_NOTE)")
    p.add_argument("--max-note", type=int, help="highest MIDI note (default: Def.h MAX_NOTE)")
    p.add_argument("--ref-note", type=int, action="append",
                   help="arm2 mode: reference key to park over; repeatable, and with 2+ the "
                        "encoder scale is fitted too (default: the highest note)")
    p.add_argument("--ref-mm", type=float, action="append",
                   help="arm2 mode: rail position of the matching --ref-note, if it is not "
                        "already in keyPositions.json or Def.h")
    p.add_argument("--keys-json", default=os.path.join(HERE, "Config", "keyPositions.json"),
                   help="arm2 mode: a previous keys run to take reference positions from")
    p.add_argument("--full-range", action="store_true",
                   help="walk every key, including the one at the other arm's"
                        " home end that this arm cannot normally reach")
    p.add_argument("--no-home", action="store_true",
                   help="skip homing (positions stay relative to power on)")
    p.add_argument("--ticks-per-mm", type=float, help="override the Def.h encoder calibration")
    p.add_argument("--defs", default=DEF_H, help="path to Def.h")
    p.add_argument("--out", help="output JSON (default depends on --mode)")
    p.add_argument("-v", "--verbose", action="store_true", help="echo every MCU line")
    args = p.parse_args()

    cfg = parse_def_h(args.defs)
    mm_to_ticks = args.ticks_per_mm or (cfg["SLIDER_TICKS_PER_REV"] / cfg["SLIDER_MM_PER_REV"])

    lo = args.min_note if args.min_note is not None else cfg["MIN_NOTE"]
    hi = args.max_note if args.max_note is not None else cfg["MAX_NOTE"]
    if hi < lo:
        return p.error("--max-note must be >= --min-note")
    notes = list(range(lo, hi + 1))

    if args.mode == "arm2":
        return run_arm2_mode(p, args, cfg, mm_to_ticks, notes)
    return run_keys_mode(p, args, cfg, mm_to_ticks, notes)


def run_keys_mode(p, args, cfg, mm_to_ticks, notes) -> int:
    out_path = args.out or os.path.join(HERE, "Config", "keyPositions.json")

    arm = args.arm
    while arm not in (1, 2):
        raw = input("Calibrate with which arm? [1/2, default 1] ").strip()
        arm = 1 if raw == "" else (int(raw) if raw.isdigit() else 0)

    cal = Calibration(cfg, arm - 1, mm_to_ticks)
    slider_id = cal.slider_id

    # `notes` stays the instrument's full key range - that is what Def.h's table
    # has to cover. `walk` is the subset this arm can physically reach.
    if args.full_range:
        walk, dropped = list(notes), None
    else:
        walk, dropped = notes_for_arm(cal.arm_index, notes)

    print(f"Rail 0..{cfg['SLIDER_LIMIT']} mm at {mm_to_ticks:.1f} ticks/mm"
          f"   arm {cal.arm_index}: railMm -> ticks = ({cal.w} * mm + {cal.b}) * {mm_to_ticks:.1f}")
    print(f"Addressing arm {cal.arm_index} as slider {slider_id} (its CAN node id)")
    print(f"Keys {note_name(walk[0])}..{note_name(walk[-1])}"
          f" (MIDI {walk[0]}..{walk[-1]}), {len(walk)} of {len(notes)}")
    if dropped is not None:
        other = 2 if arm == 1 else 1
        print(f"  skipping {note_name(dropped)}({dropped}) - arm {other} parks there,"
              f" so arm {arm} cannot reach it. --full-range walks it anyway.")

    if arm == 2:
        print("\n! arm 2's rail coordinate is derived from SLIDER_LIMIT, which is itself")
        print("  a calibrated value. If it is wrong every position here shifts with it.")
        print("  Prefer arm 1, or run '--mode arm2' first to pin SLIDER_LIMIT down.")

    mcu = Mcu(args.port, args.baud, args.verbose)
    positions = {}

    try:
        # The board resets when the port opens; let the sketch come up.
        time.sleep(2.0)
        mcu.drain()

        if not args.no_home:
            print("\nHoming sliders - keep clear of the rail...")
            mcu.home()
            time.sleep(8.0)

        mcu.free(slider_id)
        time.sleep(0.5)

        if mcu.position(slider_id) is None:
            print("\n! the MCU is not answering position queries.")
            print("  Check the port, and that Strikers.ino is the build with the 'g' frame.")
            return 1

        positions = prompt_loop(mcu, cal, walk)

    except (KeyboardInterrupt, EOFError):
        print("\ninterrupted")
    finally:
        print("\nRe-engaging the arm...")
        try:
            mcu.lock(slider_id)
            time.sleep(0.5)
        finally:
            mcu.close()

    if not positions:
        print("Nothing captured.")
        return 0

    report(walk, positions, cal, table_notes=notes)

    payload = {
        "arm": cal.arm_index,
        "slider_id": slider_id,
        # The range this arm walked...
        "min_note": walk[0],
        "max_note": walk[-1],
        "skipped_note": dropped,
        # ...inside the instrument's full range, which is what the table spans.
        "table_min_note": notes[0],
        "table_max_note": notes[-1],
        "ticks_per_mm": mm_to_ticks,
        "slider_w": cal.w,
        "slider_b": cal.b,
        # Keyed over the full range so a key this arm could not reach is present
        # as null rather than absent - resolve_reference_mm() skips nulls and
        # falls back to Def.h.
        "positions_mm": {str(n): positions.get(n) for n in notes},
    }

    write_json(out_path, payload)
    return 0


def run_arm2_mode(p, args, cfg, mm_to_ticks, notes) -> int:
    out_path = args.out or os.path.join(HERE, "Config", "arm2Calibration.json")

    # This mode is always about arm 2, i.e. arm index 1 - but its drive is not
    # slider 2. Go through Def.h's kArmToSliderId like everything else does.
    arm2_id = cfg["kArmToSliderId"][1]

    # Arm 2 gives up the lowest key (arm 1 is parked there), so a reference note
    # has to come from the range arm 2 can actually reach.
    reachable, dropped = notes_for_arm(1, notes)
    if args.full_range:
        reachable, dropped = list(notes), None

    ref_notes = args.ref_note or [reachable[-1]]

    if dropped is not None and dropped in ref_notes:
        return p.error(f"--ref-note {dropped} ({note_name(dropped)}) is the key arm 1"
                       f" parks on; arm 2 cannot reach it. Pick another, or pass"
                       f" --full-range if the rig says otherwise.")
    if args.ref_mm and len(args.ref_mm) != len(ref_notes):
        return p.error("give one --ref-mm per --ref-note, in the same order")

    ref_mm, source = resolve_reference_mm(ref_notes, args.ref_mm, args.keys_json, cfg)

    unknown = [n for n in ref_notes if n not in ref_mm]
    if unknown:
        names = ", ".join(f"{note_name(n)}({n})" for n in unknown)
        print(f"! no known rail position for {names}.")
        print("  Run '--mode keys --arm 1' first, or pass --ref-mm.")
        return 1

    print(f"Arm 2 offset from {len(ref_notes)} reference key(s), at {mm_to_ticks:.1f} ticks/mm")
    print("Arm 2 counts ticks inward from its home, so")
    print("    SLIDER_LIMIT = railMm(key) + ticks / ticksPerMm")

    if args.no_home:
        print("\n! --no-home: arm 2's encoder zero must be its parked position or every")
        print("  number below is meaningless. Only skip homing if it is already homed.")

    highest_key_mm = None
    known = [cfg["note_table"].get(n) for n in notes]
    known = [v for v in known if v is not None]
    if known:
        highest_key_mm = float(max(known))

    mcu = Mcu(args.port, args.baud, args.verbose)
    samples = []

    try:
        time.sleep(2.0)
        mcu.drain()

        if not args.no_home:
            print("\nHoming sliders - keep clear of the rail...")
            mcu.home()
            time.sleep(8.0)

        mcu.free(arm2_id)
        time.sleep(0.5)

        if mcu.position(arm2_id) is None:
            print("\n! the MCU is not answering position queries.")
            print("  Check the port, and that Strikers.ino is the build with the 'g' frame.")
            return 1

        samples = run_arm2(mcu, cfg, mm_to_ticks, ref_notes, ref_mm, source)

    except (KeyboardInterrupt, EOFError):
        print("\ninterrupted")
    finally:
        print("\nRe-engaging arm 2...")
        try:
            mcu.lock(arm2_id)
            time.sleep(0.5)
        finally:
            mcu.close()

    home, fitted_k = report_arm2(samples, cfg, mm_to_ticks, highest_key_mm)
    if home is None:
        return 0

    payload = {
        "arm1_home_mm": home,
        "arm1_home_rounded": int(round(home)),
        "ticks_per_mm_configured": mm_to_ticks,
        "ticks_per_mm_fitted": fitted_k,
        "def_h_arm1_home_mm": -cfg["kSliderB"][1],
        "samples": [
            {"note": n, "rail_mm": r, "ticks": t, "implied_home_mm": r - t / mm_to_ticks}
            for r, t, n in samples
        ],
    }

    write_json(out_path, payload)
    return 0


def write_json(path: str, payload: dict) -> None:
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\nWrote {path}")


if __name__ == "__main__":
    sys.exit(main())
