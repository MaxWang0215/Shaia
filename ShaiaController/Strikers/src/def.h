//
// Shaia striker/slider firmware definitions.
//
// Shaia's OpenCR board drives six EPOS4 nodes on one CAN bus:
//   node 1     arm 1's slider  (the high end of the rail)
//   node 2     arm 0's slider  (the low end of the rail)
//   node 3, 4  arm 0's strikers   (3 = black keys, 4 = white keys)
//   node 5, 6  arm 1's strikers   (5 = black keys, 6 = white keys)
//
// The two slider drives are addressed the other way round from the arm order -
// arm 0 is node 2, not node 1. The host owns that mapping (kArmToSliderId in
// ShaiaController/Include/Def.h) and sends the CAN node id on the wire, so
// everything below here is indexed by node id and never sees arm numbers.
//
// The host addresses strikers by bitmask (1 << (node - 1)) and sliders by
// 1-indexed id, matching ShaiaController/.../McuTransport.h.
//

#ifndef SHAIA_DEF_H
#define SHAIA_DEF_H

#define NUM_SLIDERS 2
#define NUM_STRIKERS 4
#define NUM_NODES (NUM_SLIDERS + NUM_STRIKERS)

#define SLIDER_NODE_START 1
#define STRIKER_NODE_START (SLIDER_NODE_START + NUM_SLIDERS)

// Index 0 is a dummy so the arrays can be indexed by CAN node id.
// 0 is normal, 1 is flipped.
//
// The slider inversion belongs to the arm's role, not to the drive: arm 1 homes
// at the high end of the rail and counts positive ticks inward, the opposite
// physical direction from arm 0. Arm 1 is node 1 here, so node 1 is the flipped
// one - if the two drives are ever re-addressed, these two entries move with
// kArmToSliderId on the host.
const int kNodeDirection[NUM_NODES + 1] = { 0, 1, 0, 1, 1, 0, 0 };
const int kEncoderResolution[NUM_NODES + 1] = { 1024, 1024, 1024, 1024, 1024, 1024, 1024 };

// ---------------------------------------------------------------- Strikers --
#define HOME_POSITION 25 // Deg

#define MAX_TRAJ_POINTS 5000
#define NUM_POINTS_IN_TRAJ_FOR_HIT 50  // Make sure Hit > up
#define NUM_POINTS_IN_TRAJ_FOR_UP 15

#define CLEAR_FAULT_TIMER_INTERVAL 100   // ms

#define MAX_STRIKER_ANGLE_DEG 180

#define MAX_PARAM_1 255
#define MAX_PARAM_2 255

// ----------------------------------------------------------------- Sliders --
// Rail travel in encoder ticks. The host converts mm -> ticks with the
// empirically calibrated 76.8 ticks/mm (defs.py), so 682 mm of rail is
// ~52377 ticks. Commands beyond this are clamped here as a backstop.
#define SLIDER_MAX_TRAVEL_TICKS 52377

// Default move duration if a slider frame asks for 0 ms.
#define SLIDER_DEFAULT_MOVE_MS 450

// Fraction of the move spent ramping at each end. 0.1 matches the host side
// trapezoid (Arm::computeMotionParam: d = v * 0.9 * T).
#define SLIDER_BLEND 0.1f

// Homing runs into the hard stop and trips on motor current.
#define SLIDER_HOMING_CURRENT_THRESHOLD 1000
#define SLIDER_HOMING_TIMEOUT_MS 15000

// How long homing may sit in "not started" before we call it a failure. The
// drive reports "interrupted" and "not started" with the same bits, so this is
// the window in which that value is read as the benign one.
#define SLIDER_HOMING_START_GRACE_MS 1000

// How often homing logs position and current while it runs.
#define SLIDER_HOMING_REPORT_MS 500

// How often loop() sweeps for nodes that have latched an error. A faulted node
// is not being fed setpoints, so the sweep's SDO traffic cannot collide with
// the RPDO ISR; a healthy node costs one local comparison.
#define FAULT_RECOVERY_INTERVAL_MS 250

#endif // SHAIA_DEF_H
