//
// Shaia (mini-Shimon) controller definitions.
// Derived from ShimonController-choreo/Include/Def.h, re-targeted to Shaia's
// hardware: 2 sliders + 4 strikers on one MCU serial link, and a 5 axis
// Dynamixel head.
//
// Geometry / timing constants are lifted from the Python runtime
// (shaia_targetUpdated/defs.py + arms.py), which is the calibrated source of
// truth for the rig.
//

#ifndef SHAIACONTROLLER_DEF_H
#define SHAIACONTROLLER_DEF_H

#include "TypeDef.h"

//#define SIMULATE

#define USE_ARMS
#define USE_HEAD

#define MASTER_HOST "127.0.0.1"
#define MASTER_PORT 9001

#define PORT 9000

// Constant latency from an incoming note to the audible strike. Every slider
// move is sized to take this full duration, so short and long moves share the
// same gentle acceleration profile (see arms.py:179).
#define DLY 375 /* ms */

// Fallback slider move duration when a move is issued outside the planner.
#define SLIDER_MOVE_DURATION 450 /* ms */

// The time taken between the signal sent and the sound heard from striking.
#define STRIKE_TIME 50 /* ms */

// A repeat of the same input note inside this window reuses the previous arm
// and skips the slider command - the slider is already in position.
#define NOTE_BUFFER_WINDOW 500 /* ms */

#define ARM_CMD_OSC_ROUTE       "/arm"
#define ARM_HOME_OSC_ROUTE      "/arm_home"
#define ARM_RESET_OSC_ROUTE     "/arm_reset"
#define STRIKER_CMD_OSC_ROUTE   "/striker"
#define SHIMON_OSC_ROUTE        "/shimon"
#define HEAD_CMD_OSC_ROUTE      "/head-commands"
#define HEAD_OSC_ROUTE          "/head"
#define HEAD_STOP_OSC_ROUTE     "/head_stop"
#define HEAD_LOOK_OSC_ROUTE     "/head_look"

// Single serial link to the OpenCR board running Strikers/Strikers.ino.
// It carries both slider and striker frames.
#define MCU_PORT "/dev/tty.usbmodemFFFFFFFEFFFF1"
#define MCU_BAUDRATE 115200

#define NUM_ARMS 2
#define NUM_STRIKERS 4
#define NUM_OCTAVES_TO_TRY 3

// F3 (53) .. B5 (83)
#define MIN_NOTE 53
#define MAX_NOTE 83
#define NUM_KEYS 31

// mm: B5 sits at 676 mm, + 6 mm clearance to arm 1's home.
#define SLIDER_LIMIT 682

// Lowest note F3 sits 6 mm from arm 0's home. Uniform 20 mm chromatic spacing
// with a +35 mm extra gap at each B->C octave boundary. Indexed by
// (midiNote - MIN_NOTE).
const int kNotePositionTable[NUM_KEYS] = {
      6,  26,  46,  66,  86, 106, 126, 181, 201, 221,
    241, 261, 281, 301, 321, 341, 361, 381, 401, 456,
    476, 496, 516, 536, 556, 576, 596, 616, 636, 656,
    676
};

// Per arm {left exclusion, right exclusion} in mm. With 2 arms only the inner
// sides need clearance from the neighbour.
const int kBoundaries[NUM_ARMS][2] = { {0, 40}, {40, 0} };

// Each arm's encoder zero, in rail mm - the rail position at which its drive
// reads 0 ticks.
//
// These are NOT free parameters. The firmware seeds every move from its own
// last commanded value, not from the encoder:
//
//     void prepareMove(int32_t targetTicks, int duration) {
//         float q0 = m_commandedTarget;      // <- not getActualPosition()
//
// m_commandedTarget is 0 at boot and after homing, so the host's idea of where
// an arm sits must agree with the firmware's zero exactly. If it does not, the
// very first move is a jump of the whole disagreement, at whatever acceleration
// that implies - the host's own limit check passes, because the host is
// modelling the small move it meant to make.
//
// The invariant below is what enforces that agreement; it is asserted at the
// bottom of this block so a mismatched pair cannot compile.
// Measured on the rig: after homing, "g0" reports 0 ticks for both drives,
// with node 2 sitting at the low end of the marimba and node 1 at the high end.
#define ARM0_HOME_MM 0             // node 2 reads 0 ticks at the LOW end
#define ARM1_HOME_MM SLIDER_LIMIT  // node 1 reads 0 ticks at the HIGH end

// Where each arm physically parks, in rail mm - and, by the invariant, the
// point at which its drive reads 0 ticks.
constexpr int kInitialPosition[NUM_ARMS] = { ARM0_HOME_MM, ARM1_HOME_MM };

// Rail mm -> per arm home relative mm: armMm = kSliderW[id]*railMm + kSliderB[id].
//
// Each arm counts ticks away from its own stop: arm 0 up the rail from the low
// end, arm 1 down the rail from the high end. Both therefore stay positive
// across the whole key range, which is what the unsigned wire format needs.
//
// This only works with inverted=1 in the firmware's Slider::init - the drives
// home by driving positive INTO their stops, so "away" is negative at the drive
// and PDO_setPosition's m_iDirMultiplier does the flip.
constexpr int kSliderW[NUM_ARMS] = { 1, -1 };
constexpr int kSliderB[NUM_ARMS] = { -ARM0_HOME_MM, ARM1_HOME_MM };

// (1) Each arm must read 0 ticks where it parks, or its first move is a slam of
// the difference. ARM0_HOME_MM = SLIDER_LIMIT once put arm 0's parked position
// at 52378 ticks while the firmware had it at 0, turning a 221 mm move into a
// 461 mm one at 2.1 g - under the host's own 3 g limit, so nothing warned.
//
// As parameterised above this holds by construction (kSliderB is derived from
// the same macro as kInitialPosition), so it cannot fail today. It is here to
// catch a hand-edit that breaks the pairing - flipping a kSliderW sign, or
// writing a literal offset into one array but not the other.
static_assert(kSliderW[0] * kInitialPosition[0] + kSliderB[0] == 0,
              "arm 0 must read 0 ticks at its parked position - see prepareMove above");
static_assert(kSliderW[1] * kInitialPosition[1] + kSliderB[1] == 0,
              "arm 1 must read 0 ticks at its parked position - see prepareMove above");

// (2) Both ends of the rail must map to an arm-relative position that is still
// on the rail. This is the one with teeth: it fails on any sign or offset that
// would send the arm past a hard stop, or demand a negative tick count the
// unsigned wire format cannot carry.
static_assert(kSliderB[0] >= 0 && kSliderB[0] <= SLIDER_LIMIT
              && kSliderW[0] * SLIDER_LIMIT + kSliderB[0] >= 0
              && kSliderW[0] * SLIDER_LIMIT + kSliderB[0] <= SLIDER_LIMIT,
              "arm 0's transform maps the rail off the rail - check kSliderW/kSliderB/ARM0_HOME_MM");
static_assert(kSliderB[1] >= 0 && kSliderB[1] <= SLIDER_LIMIT
              && kSliderW[1] * SLIDER_LIMIT + kSliderB[1] >= 0
              && kSliderW[1] * SLIDER_LIMIT + kSliderB[1] <= SLIDER_LIMIT,
              "arm 1's transform maps the rail off the rail - check kSliderW/kSliderB/ARM1_HOME_MM");

// Arm index -> the 1-indexed slider id on the wire, which the firmware uses
// directly as the drive's CAN node id. The two drives are addressed the other
// way round from the arm order: arm 0 (the low end of the rail) is CAN node 2,
// arm 1 (the high end) is node 1. Everything above the wire - the planner, the
// boundaries, kSliderW/kSliderB - stays in arm order; this table is the only
// place the physical node numbering is allowed to matter.
//
// calibrate_keys.py parses this out of here, so keep it a plain literal list.
const int kArmToSliderId[NUM_ARMS] = { 2, 1 };

// Slider encoder calibration. Nameplate is 1024 counts/rev over 40 mm/rev, but
// commanding arm 0 to 261 mm at that ratio landed it at ~87 mm, so the real
// ratio is 3x that - likely an unaccounted gear/quadrature factor on the EC45.
#define SLIDER_TICKS_PER_REV 3072
#define SLIDER_MM_PER_REV 40
const float kMmToTicks = (float)SLIDER_TICKS_PER_REV / (float)SLIDER_MM_PER_REV;   // 76.8

const int kBlackKeys[5] = { 1, 3, 6, 8, 10 };
const int kOctavesToTry[NUM_OCTAVES_TO_TRY] = { 0, 1, -1 };

// Slider velocity/acceleration ceilings, used only for the trajectory the host
// interpolates for position feedback - the MCU owns the real motion profile.
#define ACC_LIMIT 3 // g
#define VELOCITY_LIMIT 2.5f // m/s

// Head
#define HM_CONFIG_FILE "../Config/HeadMotorConfig.yaml"
#define DANCE_MODES_FILE "../Config/danceModes.json"

#define DXL_PORT "/dev/tty.usbserial-FT62AP2P"
#define DXL_BAUDRATE 57600
#define DXL_PROTOCOL_VERSION 2.0

#define NUM_HEAD_AXES 5

// Gesture engine
#define GESTURE_TICK 50         /* ms */
#define GESTURE_TRIGGER_WINDOW 70 /* ms */
#define DEFAULT_BPM 60.f

namespace Port {
    enum class Head {
        Dxl = 0,    // All 5 head axes live on one Dynamixel bus
        Gesture,    // Gesture generated primitives

        kNumPorts
    };

    enum class Arm {
        Slider = 0, // Slider move frames to the MCU
        Striker,    // Striker frames to the MCU

        kNumPorts [[maybe_unused]]
    };

    const int kNumPorts = 2;
}

#define CMD_BUFFER_SIZE 1024
#define POSITION_BUFFER_SIZE 1000
#define HEAD_THREAD_TIME_PERIOD 15 /* ms */

#define ARM_MIDI_CHANNEL 1
#define HEAD_MIDI_CHANNEL 2

#endif //SHAIACONTROLLER_DEF_H
