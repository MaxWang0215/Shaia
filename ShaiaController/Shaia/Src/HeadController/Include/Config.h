//
// Per axis config, parsed from Config/HeadMotorConfig.yaml.
//
// Shaia's head has no Copley/HD axes, so the choreo build's MotorType split is
// gone - every axis is a protocol 2.0 Dynamixel. What varies per axis is the
// degree range that normalized [0, 1] maps onto and, for NeckTurn, the
// operating mode (extended position, because its range crosses 0 deg).
//

#ifndef SHAIACONTROLLER_CONFIG_H
#define SHAIACONTROLLER_CONFIG_H

#include "HeadCommands.h"
#include "DxlController.h"

struct MotorConfig_t {
    HeadCommand name = HeadCommand::Unknown;
    int id = 0;
    // minDeg may be greater than maxDeg - several axes are wired so that
    // normalized 0 is the higher degree value (see Shaiahead.py).
    float minDeg = 0;
    float maxDeg = 0;
    float defaultPosition = 0.5f;   // normalized [0, 1]
    float defaultVelocity = 0.02f;  // normalized [0, 1]
    int operatingMode = OP_MODE_POSITION;
};

#endif //SHAIACONTROLLER_CONFIG_H
