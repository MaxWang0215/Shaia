//
// Created by Raghavasimhan Sankaranarayanan on 11/26/21.
// 

#include "epos4.h"

int Epos4::init(int iNodeID, MotorSpec spec, int encoderResolution, bool inverted, unsigned long timeout_ms) {
    m_uiNodeID = iNodeID;
    m_motorSpec = spec;
    m_ulTimeout_ms = timeout_ms;
    m_iEncoderResolution = encoderResolution;

    if (inverted) m_iDirMultiplier = -1;
    m_currentNMTState = PreOperational;

    int err;
    err = setControlWord(0x87);
    if (err < 0) {
        LOG_ERROR("control word send");
        return err;
    }

    err = shutdown();

    switch (spec) {
    case EC45:
        err = configEC45();
        break;

    case EC60:
        err = configEC60();
        break;
    }

    if (err < 0) {
        LOG_ERROR("Motor Config Error");
        return err;
    }

    err = setFollowErrorWindow(MAX_FOLLOW_ERROR);
    if (err < 0) {
        LOG_ERROR("set Follow Error Window Error");
        return err;
    }

    err = PDO_config();
    if (err < 0) {
        LOG_ERROR("PDO Config Error");
        return err;
    }

    err = setNMTState(m_currentNMTState);
    if (err != 0) {
        LOG_ERROR("setNMTState");
        return err;
    }

    _WORD stat = 0x0;
    err = readStatusWord(&stat);
    if (err < 0) {
        LOG_ERROR("status word err");
        return err;
    }
    LOG_LOG("Status (%i): %h", (int) m_uiNodeID, (int) stat);

    // auto fw = firmwareVersion();
    // LOG_LOG("node %i fw version: %h", m_uiNodeID, fw);
    return 0;
}

void Epos4::reset() {
    setNMTState(NMTState::PreOperational);
    setEnable(false);
}

int Epos4::configEC45() {
    int err;
    // LOG_LOG("Configuring EC45");

    err = setNominalCurrent(3210);
    if (err != 0) {
        LOG_ERROR("setNominalCurrent");
        return err;
    }

    err = setOutputCurrentLimit(5000);
    if (err != 0) {
        LOG_ERROR("setOutputCurrentLimit");
        return err;
    }

    err = setMotorTorqueConstant(36900); // uNm/A
    if (err != 0) {
        LOG_ERROR("setMotorTorqueConstant");
        return err;
    }

    err = setThermalTimeConstantWinding(296); // x 0.1 s
    if (err != 0) {
        LOG_ERROR("setThermalTimeConstantWinding");
        return err;
    }

    err = setNumPolePairs(8);
    if (err != 0) {
        LOG_ERROR("setNumPolePairs");
        return err;
    }

    // Encoder
    err = setEncoderNumPulses(m_iEncoderResolution);
    if (err != 0) {
        LOG_ERROR("setEncoderNumPulses");
        return err;
    }
    // refer FW Spec 6.2.56.2 for type
    err = setEncoderType(0x0);
    if (err != 0) {
        LOG_ERROR("setEncoderType");
        return err;
    }

    err = setCurrentControlParameters();
    if (err != 0) {
        LOG_ERROR("setCurrentControlParameters");
        return err;
    }

    err = setPositionControlParameters();
    if (err != 0) {
        LOG_ERROR("setPositionControlParameters");
        return err;
    }

    return 0;
}

int Epos4::configEC60() {
    int err;
    // LOG_LOG("Configuring EC60");

    err = setNominalCurrent(5470);
    if (err != 0) {
        LOG_ERROR("setNominalCurrent");
        return err;
    }

    err = setOutputCurrentLimit(30000);
    if (err != 0) {
        LOG_ERROR("setOutputCurrentLimit");
        return err;
    }

    err = setMotorTorqueConstant(0); // 52500 uNm/A
    if (err != 0) {
        LOG_ERROR("setMotorTorqueConstant");
        return err;
    }

    err = setThermalTimeConstantWinding(414); // x 0.1 s
    if (err != 0) {
        LOG_ERROR("setThermalTimeConstantWinding");
        return err;
    }

    err = setNumPolePairs(7);
    if (err != 0) {
        LOG_ERROR("setNumPolePairs");
        return err;
    }

    // Encoder
    err = setEncoderNumPulses(m_iEncoderResolution);
    if (err != 0) {
        LOG_ERROR("setEncoderNumPulses");
        return err;
    }
    // refer FW Spec 6.2.56.2 for type
    err = setEncoderType(0x0);
    if (err != 0) {
        LOG_ERROR("setEncoderType");
        return err;
    }

    err = setCurrentControlParameters_EC60();
    if (err != 0) {
        LOG_ERROR("setCurrentControlParameters");
        return err;
    }

    err = setPositionControlParameters_EC60();
    if (err != 0) {
        LOG_ERROR("setPositionControlParameters");
        return err;
    }

    return 0;
}

// Give up on the in-flight SDO transaction. Clearing m_bSDOBusy makes
// setRxMsg discard a reply that arrives after the deadline instead of handing
// it to whichever request runs next, and zeroing m_rxMsg keeps a stale payload
// from being read back as this transaction's answer.
void Epos4::abortSDO(const char* caller, _WORD index, _BYTE subIndex) {
    m_bSDOBusy = false;
    memset(&m_rxMsg, 0, sizeof(can_message_t));
    m_uiError = 0;
    LOG_ERROR("%s timed out after %i ms on node %i, object %h:%h",
              caller, (int) m_ulTimeout_ms, m_uiNodeID, index, subIndex);
}

int Epos4::readObj(_WORD index, _BYTE subIndex, _DWORD* answer) {
    // block SDO
    m_bSDOBusy = true;

    // EPOS4 Communication Guide (2021-03) -> 3.4
    // EPOS4 Application Notes (2021-03) -> 5.4
    m_txMsg.id = COB_ID_SDO + m_uiNodeID;
    m_txMsg.format = CAN_STD_FORMAT;
    m_txMsg.length = 8;
    m_txMsg.data[0] = 0x40;
    m_txMsg.data[1] = index & 0xFF;
    m_txMsg.data[2] = (index & 0xFF00) >> 8;
    m_txMsg.data[3] = subIndex;
    m_txMsg.data[4] = 0x00;
    m_txMsg.data[5] = 0x00;
    m_txMsg.data[6] = 0x00;
    m_txMsg.data[7] = 0x00;

    if (CanBus.writeMessage(&m_txMsg) != m_txMsg.length) {
        LOG_ERROR("CanBus WriteMessage failed");
        m_bSDOBusy = false;
        return -1;
    }

    auto waitTime = millis();
    while (m_bSDOBusy) {
        // LOG_LOG("sdo busy");
        delay(1);
        if (m_ulTimeout_ms > 0 && ((millis() - waitTime) > m_ulTimeout_ms)) {
            abortSDO("readObj", index, subIndex);
            return -2;
        }
    }

    /* check for error code */
    if (m_rxMsg.data[0] == 0x80) {
        m_uiError = (m_rxMsg.data[7] << 24) + (m_rxMsg.data[6] << 16) + (m_rxMsg.data[5] << 8) + m_rxMsg.data[4];
        return -1;
    }

    *answer = (((int32_t) (m_rxMsg.data[7])) << 24) + (((int32_t) (m_rxMsg.data[6])) << 16) + (((int32_t) (m_rxMsg.data[5])) << 8) + (int32_t) (m_rxMsg.data[4]);

    return 0;
}

int Epos4::writeObj(_WORD index, _BYTE subIndex, _DWORD param) {
    // EPOS4 Communication Guide (2021-03) -> 3.4
    // EPOS4 Application Notes (2021-03) -> 5.4
    m_bSDOBusy = true;
    m_txMsg.id = COB_ID_SDO + m_uiNodeID;
    m_txMsg.format = CAN_STD_FORMAT;
    m_txMsg.length = 8;
    m_txMsg.data[0] = 0x22;
    m_txMsg.data[1] = index & 0xFF;
    m_txMsg.data[2] = (index & 0xFF00) >> 8;
    m_txMsg.data[3] = subIndex;
    m_txMsg.data[4] = param & 0xFF;
    m_txMsg.data[5] = (param & 0xFF00) >> 8;
    m_txMsg.data[6] = (param & 0xFF0000) >> 16;
    m_txMsg.data[7] = (param & 0xFF000000) >> 24;

    int n = CanBus.writeMessage(&m_txMsg);
    if (n != 8) {
        LOG_ERROR("CanBus WriteMessage failed. Wrote %i bytes instead of %i bytes", n, 8);
        m_bSDOBusy = false;
        return -1;
    }

    auto waitTime = millis();
    while (m_bSDOBusy) {
        // LOG_LOG("sdo busy");
        delay(1);
        if (m_ulTimeout_ms > 0 && ((millis() - waitTime) > m_ulTimeout_ms)) {
            abortSDO("writeObj", index, subIndex);
            return -2;
        }
    }

    /* check for error code */
    if (m_rxMsg.data[0] == 0x80) {
        m_uiError = (m_rxMsg.data[7] << 24) + (m_rxMsg.data[6] << 16) + (m_rxMsg.data[5] << 8) + (m_rxMsg.data[4]);
        LOG_ERROR("Device Error: %h", m_uiError);
        return -1;
    }

    return 0;
}

_WORD Epos4::firmwareVersion() {
    _DWORD answer;

    int n = 0;
    if ((n = readObj(0x1018, 0x03, &answer)) != 0) {
        LOG_ERROR("readObj in readSWversion. Error code: %h", m_uiError);
        return n;
    }

    n = answer >> 0x10;
    return n;
}

// Whether each drive sees its power stage rail. The EPOS4 does not implement
// the CiA 402 DC link voltage object (0x6079) - it aborts the read - so the
// reading we get is statusword bit 4, "voltage enabled". Costs one SDO
// transaction, so callers must make sure the RPDO stream is not running: both
// share m_txMsg.
void Epos4::reportPowerStatus() {
    _WORD status = 0;
    if (readStatusWord(&status) != 0) {
        LOG_ERROR("Node %i: status word read failed", m_uiNodeID);
        return;
    }

    LOG_LOG("Node %i: status %h, voltage enabled %i, fault %i, last error (%h): %s",
            m_uiNodeID, (int) status,
            (int) ((status & (1 << 4)) != 0),
            (int) ((status & (1 << 3)) != 0),
            m_uiError, getDeviceError(m_uiError).c_str());
}

int Epos4::readStatusWord(_WORD* status) {
    _DWORD ret;

    if (readObj(0x6041, 0x00, &ret) != 0) {
        LOG_ERROR("Read status word. ");
        return -1;
    }

    *status = ret & 0xFFFF;
    return 0;
}

int Epos4::readITP(uint8_t* time, int8_t* unit) {
    _DWORD val;
    int n = readObj(INTERP_TIME_ADDR, 0x01, &val);
    if (n != 0) {
        LOG_ERROR("Read Obj failed. Error code: ", m_uiError);
        return -1;
    }

    *time = val;

    n = readObj(INTERP_TIME_ADDR, 0x02, &val);
    if (n != 0) {
        LOG_ERROR("Read Obj failed. Error code: ", m_uiError);
        return -1;
    }

    *unit = val;
}

int Epos4::setITP(uint8_t value, int8_t index) {
    _BYTE idx = index & 0xFF;
    int n = writeObj(INTERP_TIME_ADDR, 0x01, value);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    n = writeObj(INTERP_TIME_ADDR, 0x02, idx);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    value = 0;
    index = 0;
    readITP(&value, &index);

    return 0;
}

int32_t Epos4::angle2Pos(float fAngle, bool bRadian) {
    if (!bRadian)
        fAngle = fAngle * (M_PI / 180.f);

    return round((fAngle / (2.f * M_PI)) * (m_iEncoderResolution));
}

// Torque actual value (object 0x6077), in per mille of rated torque. Stands in
// for motor current: what matters here is whether the drive is pushing at all.
// Not every EPOS4 firmware carries it - 0x6078 aborts with 0x06020000, object
// does not exist - so a rejection disables the read rather than repeating it.
int Epos4::getActualTorque(int16_t* piTorque) {
    if (m_bTorqueReadUnsupported) return -1;

    _DWORD ret;
    if (readObj(0x6077, 0x00, &ret) != 0) {
        if (m_uiError == 0x06020000) {
            LOG_WARN("Node %i: torque actual value not supported, position only from here", m_uiNodeID);
            m_bTorqueReadUnsupported = true;
        } else {
            LOG_ERROR("Get torque failed. Error: %h", m_uiError);
        }
        return -1;
    }

    *piTorque = (int16_t) (ret & 0xFFFF);
    return 0;
}

// Position and current together, for watching whether an axis is actually
// turning. Homing runs pre-operational, so there is no TPDO3 feedback and
// these have to be read over SDO.
// Read back the limits that decide whether homing can move at all. Homing
// speeds and the velocity ceilings are never written by this firmware, so they
// hold whatever is in each drive's NVM - and a zero in any of them shows up as
// "homing in progress" with status word bit 11, internal limit active, and no
// motion. Objects the firmware rejects are reported, not retried.
void Epos4::reportHomingLimits() {
    struct Limit {
        _WORD index;
        _BYTE subIndex;
        const char* name;
    };

    static const Limit limits[] = {
        { 0x6099, 0x01, "speed for switch search" },
        { 0x6099, 0x02, "speed for zero search" },
        { 0x609A, 0x00, "homing acceleration" },
        { 0x607F, 0x00, "max profile velocity" },
        { 0x6080, 0x00, "max motor speed" },
        { 0x30B2, 0x00, "homing current threshold" },
        // Written as (uint8_t)-4 = 0xFC = 252 for CurrentThresholdNegative.
        { 0x6098, 0x00, "homing method (252 == -4)" },
        // Software position limits. A min limit at or above the current
        // position clamps a negative move to nothing, which reads as internal
        // limit active with no torque and no motion.
        { 0x607D, 0x01, "min position limit" },
        { 0x607D, 0x02, "max position limit" },
    };

    for (unsigned i = 0; i < sizeof(limits) / sizeof(limits[0]); ++i) {
        _DWORD value = 0;
        if (readObj(limits[i].index, limits[i].subIndex, &value) != 0) {
            LOG_WARN("Node %i: %h:%h (%s) unreadable. Error: %h",
                     m_uiNodeID, limits[i].index, limits[i].subIndex, limits[i].name, m_uiError);
            continue;
        }

        LOG_LOG("Node %i limit: %s = %i", m_uiNodeID, limits[i].name, (int) value);
    }
}

void Epos4::reportMotion(const char* tag) {
    int32_t pos = 0;
    if (getActualPosition(&pos) != 0) return;

    // Velocity demand separates "the drive is not asking for motion" from "the
    // drive is asking and the motor is not delivering".
    _DWORD vel = 0;
    int iVel = (readObj(0x606B, 0x00, &vel) == 0) ? (int) (int32_t) vel : 0;

    // Position is the answer to "did it move"; torque is a bonus if available.
    int16_t torque = 0;
    if (getActualTorque(&torque) == 0)
        LOG_LOG("%s node %i: pos %i, vel demand %i, torque %i per mille, status %h",
                tag, m_uiNodeID, (int) pos, iVel, (int) torque, (int) m_uiCurrentStatusWord);
    else
        LOG_LOG("%s node %i: pos %i, vel demand %i, status %h",
                tag, m_uiNodeID, (int) pos, iVel, (int) m_uiCurrentStatusWord);
}

int Epos4::getActualPosition(int32_t* piPos) {
    _DWORD ret;

    if (readObj(0x6064, 0x00, &ret) != 0) {
        LOG_ERROR("Get positon failed. Error: %h", m_uiError);
        return -1;
    }

    LOG_TRACE("Position: %i", (int) ret);
    *piPos = ret;

    return 0;
}

HomingStatus Epos4::getHomingStatus() {
    if (m_iCurrentMode != Homing) {
        return Error;
    }

    _WORD stat;
    int err = readStatusWord(&stat);
    if (err != 0) {
        LOG_ERROR("readStatusWord");
        return Error;
    }
    m_uiCurrentStatusWord = stat;

    if (stat & (1 << 13)) {
        LOG_ERROR("Homing error on node %i. status: %h", m_uiNodeID, stat);
        return Error;
    }

    if ((stat & (1 << 12)) && (stat & (1 << 15))) {
        return Completed;
    }

    // (bit13, bit12, bit10) == (0,0,1) is "homing interrupted OR not started".
    // The drive cannot tell those apart, so a caller polling straight after
    // startHoming() sees this before homing has begun. Deciding which it is
    // belongs to the caller, which knows whether it ever saw progress.
    return stat & (1 << 10) ? Interrupted : InProgress;
}

int Epos4::getOpMode(OpMode* opMode, char* sOpMode) {
    _DWORD ret;

    if (readObj(MODE_OF_OPERATION_DISP_ADDR, 0x00, &ret) != 0) {
        LOG_ERROR("Get op mode failed. Error: %h", m_uiError);
        return -1;
    }

    m_iCurrentMode = static_cast<OpMode>(ret & 0xFF);
    if (opMode) *opMode = m_iCurrentMode;

    if (sOpMode) {
        const char* m = getOpModeString(m_iCurrentMode);
        strcpy(sOpMode, m);
    }
    return 0;
}

char* Epos4::getOpModeString(OpMode mode) const {
    switch (mode) {
    case OpMode::ProfilePosition:
        return "Profile Position Mode";
        break;

    case OpMode::ProfileVelocity:
        return "Profile Velocity Mode";
        break;
    case OpMode::Homing:
        return "Homing Mode";
        break;
    case OpMode::CyclicSyncPosition:
        return "Cyclic Synchronous Position Mode";
        break;
    case OpMode::CyclicSyncVelocity:
        return "Cyclic Synchronous Velocity Mode";
        break;
    case OpMode::CyclicSyncTorque:
        return "Cyclic Synchronous Torque Mode";
        break;
    default:
        break;
    }
    return "";
}

int Epos4::setOpMode(OpMode opMode, uint8_t uiInterpolationTime, int8_t iInterpolationIndex, HomingMethod homingMethod) {
    int n;
    n = writeObj(MODE_OF_OPERATION_ADDR, 0x0, (uint8_t) opMode);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }
    m_iCurrentMode = opMode;

    switch (opMode) {
    case ProfilePosition:
        n = setProfile();
        if (n != 0) {
            LOG_ERROR("setProfile");
            return -1;
        }
        break;

    case ProfileVelocity:
        n = setProfile();
        if (n != 0) {
            LOG_ERROR("setProfile");
            return -1;
        }
        break;

    case Homing:
        n = setHomingMethod(homingMethod);
        if (n != 0) {
            LOG_ERROR("setHomingMethod");
            return -1;
        }

        n = setHomingCurrentThreshold(2000);
        if (n != 0) {
            LOG_ERROR("setHomingCurrentThreshold");
            return -1;
        }
        break;

    case CyclicSyncPosition:
        n = setITP(uiInterpolationTime, iInterpolationIndex);
        if (n != 0) return -1;

        n = setProfile();
        if (n != 0) {
            LOG_ERROR("setProfile");
            return -1;
        }
        break;

    case CyclicSyncVelocity:
        break;

    case CyclicSyncTorque:
        break;
    }

    // Refer App notes -> 7.6
    // control word Shutdown (quickstop and enable voltage)
    int err = shutdown();

    _DWORD mooDisplay;
    n = readObj(0x6061, 0x0, &mooDisplay);
    if (n != 0) {
        LOG_ERROR("read Modes of operation display");
        return -1;
    }

    // LOG_LOG("Mode display (%i): %h", m_uiNodeID, mooDisplay);

    if (mooDisplay != opMode) {
        LOG_WARN("Mode not set properly");
    }
    return err;
}

int Epos4::setNominalCurrent(_DWORD current) {
    if (current < 0 || current > 6000) {    // For EPOS4 50/5
        LOG_ERROR("Current Not in range");
        return -1;
    }

    int n;
    n = writeObj(MOTOR_DATA_ADDR, NOMINAL_CURRENT, current);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }
    return 0;
}

int Epos4::setOutputCurrentLimit(_DWORD currentLimit) {
    if (currentLimit < 0 || currentLimit > 30000) {
        LOG_ERROR("Current Not in range");
        return -1;
    }

    int n;
    n = writeObj(MOTOR_DATA_ADDR, OUTPUT_CURRENT_LIMIT, currentLimit);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }
    return 0;
}

int Epos4::setNumPolePairs(_BYTE polePairs) {
    if (polePairs < 1 || polePairs > 255) {    // For EPOS4 50/5
        LOG_ERROR("Current Not in range");
        return -1;
    }

    int n;
    n = writeObj(MOTOR_DATA_ADDR, POLE_PAIRS, polePairs);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }
    return 0;
}

int Epos4::setThermalTimeConstantWinding(_WORD ttconstWinding) {
    if (ttconstWinding < 1 || ttconstWinding > 10000) {
        LOG_ERROR("torque Constant Not in range. Refer FW spec 6.2.53.4");
        return -1;
    }
    int n;
    n = writeObj(MOTOR_DATA_ADDR, THERMAL_TIME_CONST_WINDING, ttconstWinding);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }
    return 0;

}

int Epos4::setMotorTorqueConstant(_DWORD torqueConst) {
    if (torqueConst < 0 || torqueConst > 10000000) {
        LOG_ERROR("torque Constant Not in range. Refer FW spec 6.2.53.5");
        return -1;
    }

    int n;
    n = writeObj(MOTOR_DATA_ADDR, TORQUE_CONST, torqueConst);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }
    return 0;
}

int Epos4::setEncoderNumPulses(_DWORD pulses) {
    if (pulses < 16 || pulses > 2500000) {
        LOG_ERROR("Pulses Not in range. Refer FW spec 6.2.56.1");
        return -1;
    }

    return writeObj(0x3010, 0x01, pulses);
}

int Epos4::setEncoderType(_WORD type) {
    return writeObj(0x3010, 0x02, type);
}

int Epos4::setControlWord(_WORD cw) {
    int n = writeObj(CTRL_WORD_ADDR, 0x00, cw);
    // LOG_LOG("%i", n);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }
    m_uiCurrentCtrlWord = cw;
    delay(5);
    return 0;
}

int Epos4::shutdown() {
    return setControlWord(0x0006);
}

int Epos4::setEnable(bool bEnable) {
    // control word switch on and enable
    // Refer FW spec -> 6.2.94
    // bit 0 -> Switched On
    // bit 1 -> Enable Voltage
    // bit 2 -> Quick Stop
    // bit 3 -> Enable Operation
    _WORD cw = 0x000F;
    // bit 4 -> new setpoint (PPM) | Homing start (HMM)
    // bit 5 -> Change immediately (PPM)
    // bit 6 -> Abs (0) / rel (1) (PPM)
    // bit 7 -> Fault Reset
    // bit 8 -> Halt (Profiled modes)
    // bit 15 -> Endless Move (PPM)

    // if (m_iCurrentMode == ProfilePosition) cw = 0xBB;

    if (!bEnable) cw = 0x0004;

    return setControlWord(cw);
}

int Epos4::setProfile(_DWORD vel, _DWORD acc) {
    int err;
    // Profile Velocity
    err = writeObj(0x6081, 0x0, vel);
    if (err != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    // Profile Acc
    err = writeObj(0x6083, 0x0, acc);
    if (err != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    // Profile Deceleration
    err = writeObj(0x6084, 0x0, acc);
    if (err != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    // Profile Quick stop deceleration
    err = writeObj(0x6085, 0x0, 10000);
    if (err != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    // Shutdown
    err = setControlWord(0x0006);
    if (err != 0) {
        LOG_ERROR("Shutdown Failed");
        return -1;
    }

    // Switch on & Enable
    err = setControlWord(0x000F);
    if (err != 0) {
        LOG_ERROR("Switch on & Enable Failed");
        return -1;
    }
}

int Epos4::setHomingMethod(HomingMethod method) {
    return writeObj(0x6098, 0x00, (uint8_t) method);
}

int Epos4::setHomingCurrentThreshold(_WORD currentThreshold) {
    return writeObj(0x30B2, 0x0, currentThreshold);
}

int Epos4::moveToPosition(int32_t pos, bool bWait) {
    pos *= m_iDirMultiplier;

    LOG_LOG("position: %i", pos);
    int err;
    // Target position
    err = writeObj(0x607A, 0x0, pos);
    if (err != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    // Abs Pos, start immediately
    err = setControlWord(0x003F);
    if (err != 0) {
        LOG_ERROR("Abs Pos, start immediately");
        return -1;
    }

    _WORD stat = 0x0;
    err = readStatusWord(&stat);
    if (err != 0) {
        LOG_ERROR("readStatusWord");
        return -1;
    }
    LOG_LOG("stat (%i): %h", m_uiNodeID, stat);

    if (bWait) {
        // refer FW spec table 7-64 -> 
        while (!(stat & 0x400)) {
            err = readStatusWord(&stat);
            // LOG_LOG("wait stat: %h", stat);
            if (err != 0) {
                LOG_ERROR("readStatusWord");
                return -1;
            }

            if ((stat) & 0x8) {
                LOG_ERROR("Fault state");
                return -1;
            }
            err = setControlWord(0x000F);
            if (err != 0) {
                LOG_ERROR("Abs Pos, start immediately");
                return -1;
            }

        }
        delay(1000);
    }

    m_iEncoderPosition = pos;
    return 0;
}

int Epos4::rotate(float fAngle, bool bRadian, bool bWait) {
    int32_t pos = angle2Pos(fAngle, bRadian);
    return moveToPosition(pos, bWait);
}

int Epos4::moveWithVelocity(int32_t velocity) {
    int err;
    velocity *= m_iDirMultiplier;
    // Target velocity
    err = writeObj(0x60FF, 0x0, velocity);
    if (err != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }
    delay(10);
    // Start Move
    err = setControlWord(0x000F);
    if (err != 0) {
        LOG_ERROR("Start Move failed");
        return -1;
    }
    return 0;
}

int Epos4::setFollowErrorWindow(_DWORD errWindow) {
    int err;
    err = writeObj(0x6065, 0x00, errWindow);
    if (err != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    return 0;
}

int Epos4::quickStop() {
    return setControlWord(0x000B);
}

int Epos4::startHoming() {
    // Controlword bit 4 starts homing on a rising edge, and the drive only
    // acts on that edge once it is already in Operation Enabled. Writing 0x1F
    // straight from Ready To Switch On enables the drive with the bit already
    // high, so there is no edge left to see and homing never starts - the
    // status word then reads (bit13, bit12, bit10) = (0, 0, 1), which the spec
    // calls "interrupted or not started". Enable first with the bit clear.
    int err = setControlWord(0x000F);
    if (err != 0) {
        LOG_ERROR("Node %i: enable before homing failed", m_uiNodeID);
        return err;
    }

    // Wait for Operation Enabled (status word bit 2) before making the edge.
    unsigned long start = millis();
    _WORD stat = 0;
    while ((millis() - start) < HOMING_ENABLE_TIMEOUT_MS) {
        if (readStatusWord(&stat) == 0 && (stat & (1 << 2))) break;
        delay(5);
    }

    m_uiCurrentStatusWord = stat;
    if (!(stat & (1 << 2))) {
        LOG_ERROR("Node %i did not reach Operation Enabled before homing. status: %h", m_uiNodeID, (int) stat);
        return -1;
    }

    // This delay is needed for homing to work
    delay(50);  // Can be as low as 10ms
    return setControlWord(0x001F);
}

int Epos4::SetHomePosition(int32_t iPos, int32_t iOffset) {
    int n;

    iPos *= m_iDirMultiplier;

    n = writeObj(0x30B0, 0x00, iPos);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }
    m_iEncoderPosition = iPos;

    // Set homing offset
    n = writeObj(0x30B1, 0x00, iOffset);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    // Set Position Offset
    n = writeObj(0x60B0, 0x00, iOffset);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    return 0;
}

int Epos4::clearFault() {
    // Bit 7 is the fault reset, and it lands the drive in Switch On Disabled -
    // from which setEnable(true)'s 0x000F is not a legal transition. Follow it
    // with a shutdown so the drive sits in Ready To Switch On, which is where
    // init() leaves it and what every enable path here assumes.
    int err = setControlWord(0x80);
    if (err != 0) return err;

    err = shutdown();
    if (err != 0) return err;

    m_uiError = 0;
    return 0;
}

int Epos4::setCurrentControlParameters() {
    int n;
    n = writeObj(CURRENT_CTRL_PARAM_ADDR, CC_P_GAIN, 781067);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    n = writeObj(CURRENT_CTRL_PARAM_ADDR, CC_I_GAIN, 1386330);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    return 0;
}

int Epos4::setCurrentControlParameters_EC60() {
    int n;
    n = writeObj(CURRENT_CTRL_PARAM_ADDR, CC_P_GAIN, 569005);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    n = writeObj(CURRENT_CTRL_PARAM_ADDR, CC_I_GAIN, 3403363);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    return 0;
}

int Epos4::setPositionControlParameters() {
    int n;
    n = writeObj(POS_CTRL_PARAM_ADDR, PC_P_GAIN, 3303872);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    _DWORD ans;
    n = readObj(POS_CTRL_PARAM_ADDR, PC_P_GAIN, &ans);
    if (n != 0) {
        LOG_ERROR("Read Obj failed. Error code: ", m_uiError);
        return -1;
    }

    n = writeObj(POS_CTRL_PARAM_ADDR, PC_I_GAIN, 11508462);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    n = writeObj(POS_CTRL_PARAM_ADDR, PC_D_GAIN, 79040);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    n = writeObj(POS_CTRL_PARAM_ADDR, PC_FF_V_GAIN, 0);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    n = writeObj(POS_CTRL_PARAM_ADDR, PC_FF_A_GAIN, 630);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    return 0;
}

int Epos4::setPositionControlParameters_EC60() {
    int n;
    n = writeObj(POS_CTRL_PARAM_ADDR, PC_P_GAIN, 11758200); // 11758200 // 15758200
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    _DWORD ans;
    n = readObj(POS_CTRL_PARAM_ADDR, PC_P_GAIN, &ans);
    if (n != 0) {
        LOG_ERROR("Read Obj failed. Error code: ", m_uiError);
        return -1;
    }

    n = writeObj(POS_CTRL_PARAM_ADDR, PC_I_GAIN, 22040510);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    n = writeObj(POS_CTRL_PARAM_ADDR, PC_D_GAIN, 222732);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    n = writeObj(POS_CTRL_PARAM_ADDR, PC_FF_V_GAIN, 0);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    n = writeObj(POS_CTRL_PARAM_ADDR, PC_FF_A_GAIN, 7746);
    if (n != 0) {
        LOG_ERROR("Write Obj failed. Error code: %h", m_uiError);
        return -1;
    }

    return 0;
}

int Epos4::setNMTState(NMTState nmtState) {
    m_txMsg.id = 0x0;
    m_txMsg.length = 2;
    m_txMsg.format = CAN_STD_FORMAT;
    m_txMsg.data[0] = (uint8_t) nmtState;
    m_txMsg.data[1] = m_uiNodeID;

    int n = CanBus.writeMessage(&m_txMsg);
    if (n != 2) {
        LOG_ERROR("CanBus write msg failed. wrote %i units instead of %i units", n, m_txMsg.length);
        return -1;
    }

    m_currentNMTState = nmtState;
    return 0;
}

int Epos4::PDO_config() {
    int err;

    /********** TPDO 3 stuffs **********/

    // The COB-ID entry may only be changed while the PDO is invalid, so mark
    // it invalid (bit 31), set the parameters, then write the COB-ID back with
    // bit 31 clear to actually enable the PDO. Bit 30 stays set - we never poll
    // TPDO3 with a remote frame.
    err = writeObj(0x1802, 0x01, COB_ID_PDO_DISABLED | COB_ID_PDO_NO_RTR | (COB_ID_TPDO3 + m_uiNodeID));
    if (err != 0) {
        LOG_ERROR("Disable TPDO3 failed. Error code: %h", m_uiError);
    }

    // Set TPDO3 transmission type to Asynchronous
    err = writeObj(0x1802, 0x02, 255);
    if (err != 0) {
        LOG_ERROR("Set transmission type failed. Error code: %h", m_uiError);
    }

    // Set TPDO3 Inhibition time
    err = writeObj(0x1802, 0x03, TPDO3_INHIBIT_TIME_100US);
    if (err != 0) {
        LOG_ERROR("Set inhibit time failed. Error code: %h", m_uiError);
    }

    // Set COB ID for TPDO3 and mark it valid so the drive starts sending it.
    err = writeObj(0x1802, 0x01, COB_ID_PDO_NO_RTR | (COB_ID_TPDO3 + m_uiNodeID));
    if (err != 0) {
        LOG_ERROR("Set COB ID Failed. Error code: %h", m_uiError);
    }

    /********** RPDO stuffs **********/
    err = PDO_configRPDO3();
    if (err != 0) {
        LOG_ERROR("Failed to configure RPDO3. Error code: %h", m_uiError);
    }

    return kNoError;
    // return PDO_configRPDO4();
}

int Epos4::PDO_configRPDO3() {
    int err;

    // Set COB ID for RPDO3
    err = writeObj(0x1402, 0x01, COB_ID_RPDO3 + m_uiNodeID);
    if (err != 0) {
        LOG_ERROR("Set COB ID Failed for RPDO3. Error code: %h", m_uiError);
        return err;
    }

    // Set RPDO3 transmission type to Asynchronous
    err = writeObj(0x1402, 0x02, 255);
    if (err != 0) {
        LOG_ERROR("Set transmission type failed for RPDO3. Error code: %h", m_uiError);
        return err;
    }

    /*
    To be able to change the PDO mapping, the following procedure must be performed:
    a) Write the value “0” (zero) to subindex 0x00 (disable PDO).
    b) Modify the desired objects in subindex 0x01…0x0n.
    c) Write the desired number of mapped objects to subindex 0x00.
    */

    // // Write the value “0” (zero) to subindex 0x00 (disable PDO).
    // err = writeObj(0x1602, 0x00, 0);
    // if (err != 0) {
    //     LOG_ERROR("write zero to 0x00 failed for RPDO3. Error code: %h", m_uiError);
    //     return err;
    // }

    // // Modify the desired objects in subindex 0x01…0x0n.
    // err = writeObj(0x1602, 0x01, 0x60400010);   // Controlword
    // if (err != 0) {
    //     LOG_ERROR("write to 0x01 failed for RPDO3. Error code: %h", m_uiError);
    //     return err;
    // }

    // err = writeObj(0x1602, 0x02, 0x607A0020);   // Position
    // if (err < 0) {
    //     LOG_ERROR("write to 0x02 failed for RPDO3. Error code: %h", m_uiError);
    //     return err;
    // }

    // // Write the desired number of mapped objects to subindex 0x00.
    // err = writeObj(0x1602, 0x00, 2);
    // if (err != 0) {
    //     LOG_ERROR("write length failed for RPDO3. Error code: %h", m_uiError);
    //     return err;
    // }

    return 0;
}

int Epos4::PDO_configRPDO4() {
    int err;

    // Set COB ID for RPDO4
    err = writeObj(0x1403, 0x01, COB_ID_RPDO4 + m_uiNodeID);
    if (err != 0) {
        LOG_ERROR("Set COB ID Failed for RPDO4. Error code: %h", m_uiError);
        return err;
    }

    // Set RPDO3 transmission type to Asynchronous
    err = writeObj(0x1403, 0x02, 255);
    if (err != 0) {
        LOG_ERROR("Set transmission type failed for RPDO4. Error code: %h", m_uiError);
        return err;
    }

    /*
    To be able to change the PDO mapping, the following procedure must be performed:
    a) Write the value “0” (zero) to subindex 0x00 (disable PDO).
    b) Modify the desired objects in subindex 0x01…0x0n.
    c) Write the desired number of mapped objects to subindex 0x00.
    */

    // // Write the value “0” (zero) to subindex 0x00 (disable PDO).
    // err = writeObj(0x1603, 0x00, 0);
    // if (err != 0) {
    //     LOG_ERROR("write zero to 0x00 failed for RPDO4. Error code: %h", m_uiError);
    //     return err;
    // }

    // // Modify the desired objects in subindex 0x01…0x0n.
    // err = writeObj(0x1603, 0x01, 0x60810020);   // Velocity
    // if (err != 0) {
    //     LOG_ERROR("write to 0x01 failed for RPDO4. Error code: %h", m_uiError);
    //     return err;
    // }

    // err = writeObj(0x1603, 0x02, 0x60830020);   // Acceleration
    // if (err != 0) {
    //     LOG_ERROR("write to 0x02 failed for RPDO4. Error code: %h", m_uiError);
    //     return err;
    // }

    // // Write the desired number of mapped objects to subindex 0x00.
    // err = writeObj(0x1603, 0x00, 2);
    // if (err != 0) {
    //     LOG_ERROR("write length failed for RPDO4. Error code: %h", m_uiError);
    //     return err;
    // }

    return 0;
}

int Epos4::PDO_setControlWord(_WORD cw) {
    m_txMsg.id = COB_ID_RPDO1 + m_uiNodeID;
    m_txMsg.format = CAN_STD_FORMAT;
    m_txMsg.length = 2;
    m_txMsg.data[0] = cw & 0xFF;
    m_txMsg.data[1] = (cw >> 8) & 0xFF;
    int n = CanBus.writeMessage(&m_txMsg);

    if (n != 2) {
        LOG_ERROR("CanBus write msg failed.");
        return -1;
    }

    m_uiCurrentCtrlWord = cw;
    return 0;
}

int Epos4::PDO_quickStop() {
    return PDO_setControlWord(0x000B);
}

int Epos4::PDO_processMsg(can_message_t& msg) {
    m_uiCurrentStatusWord = ((msg.data[1] & 0xFF) << 8) + msg.data[0];
    static int32_t callbackEncPos = 0;
    m_iEncoderPosition = (((msg.data[5] & 0xFF) << 24) + ((msg.data[4] & 0xFF) << 16) + ((msg.data[3] & 0xFF) << 8) + msg.data[2]) * m_iDirMultiplier;
    // Only bytes 6-7 carry the error code, and only if the drive's TPDO3
    // mapping actually includes 0x603F. Nothing here configures 0x1A02, so a
    // shorter default mapping would otherwise zero out the code that
    // handleEMCYMsg just recorded.
    if (msg.length >= 8)
        m_uiError = ((msg.data[7] & 0xFF) << 8) + msg.data[6];
    // Callback only when the encoder value changes more than a threshold
    if (abs(m_iEncoderPosition - callbackEncPos) > CALLBACK_ENC_THRESHOLD) {
        callbackEncPos = m_iEncoderPosition;
        // LOG_LOG("Encoder position: %i", m_iEncoderPosition);
    }

    m_bFault = (m_uiCurrentStatusWord & (1 << 3));

    // This runs in the CAN rx interrupt and TPDO3 arrives continuously, so
    // logging every faulted frame means a blocking Serial write per frame from
    // ISR context. A latched fault then saturates the link and stalls whatever
    // is behind it - which is how SDO transactions in init end up timing out.
    // Report edges only: a fault that stays put has already been reported.
    if (m_bFault != m_bLastReportedFault || m_uiError != m_uiLastReportedError) {
        m_bLastReportedFault = m_bFault;
        m_uiLastReportedError = m_uiError;

        if (m_bFault || m_uiError != 0x0000)
            LOG_WARN("status: %h - Fault detected in node %i. Error (%h): %s", m_uiCurrentStatusWord, m_uiNodeID, m_uiError, getDeviceError(m_uiError).c_str());
        else
            LOG_LOG("Node %i fault cleared", m_uiNodeID);
    }

    return 0;
}

int Epos4::PDO_setPosition(int32_t position) {
    // position *= m_iDirMultiplier;
    // LOG_LOG("%i", position);

    uint8_t LSB = 0x0F;
    if (m_bFault)
        LSB = 0x8F;

    m_txMsg.id = COB_ID_RPDO3 + m_uiNodeID;
    m_txMsg.format = CAN_STD_FORMAT;
    m_txMsg.length = 6;
    m_txMsg.data[0] = LSB;  // cw set to enable and switch on. Clear fault if present
    m_txMsg.data[1] = 0x0;
    m_txMsg.data[2] = (uint8_t) (position & 0xFF);
    m_txMsg.data[3] = (uint8_t) ((position >> 8) & 0xFF);
    m_txMsg.data[4] = (uint8_t) ((position >> 16) & 0xFF);
    m_txMsg.data[5] = (uint8_t) ((position >> 24) & 0xFF);
    int n = CanBus.writeMessage(&m_txMsg);

    if (n != m_txMsg.length) {
        LOG_ERROR("CanBus write msg failed.");
        return -1;
    }

    m_iEncoderPosition = position;
    return 0;
}

int Epos4::PDO_rotate(float fAngle, bool bRadian) {
    int32_t pos = angle2Pos(fAngle, bRadian);
    return PDO_setPosition(pos);
}

int Epos4::PDO_setVelocity(int32_t iVelocity) {
    iVelocity = iVelocity * m_iDirMultiplier;
    uint8_t LSB = 0x0F;
    if (m_bFault)
        LSB = 0x8F;

    m_txMsg.id = COB_ID_RPDO4 + m_uiNodeID;
    m_txMsg.format = CAN_STD_FORMAT;
    m_txMsg.length = 6;
    m_txMsg.data[0] = LSB;  // cw set to enable and switch on. Clear fault if present
    m_txMsg.data[1] = 0x0;
    m_txMsg.data[2] = (uint8_t) (iVelocity & 0xFF);
    m_txMsg.data[3] = (uint8_t) ((iVelocity >> 8) & 0xFF);
    m_txMsg.data[4] = (uint8_t) ((iVelocity >> 16) & 0xFF);
    m_txMsg.data[5] = (uint8_t) ((iVelocity >> 24) & 0xFF);
    int n = CanBus.writeMessage(&m_txMsg);

    if (n != m_txMsg.length) {
        LOG_ERROR("CanBus write msg failed.");
        return -1;
    }

    return 0;
}

int Epos4::setRxMsg(can_message_t& msg) {
    if (!m_bSDOBusy) {
        LOG_ERROR("No one requested CAN message. Not copying...");
        return -1;
    }
    memcpy(&m_rxMsg, &msg, sizeof(can_message_t));
    m_bSDOBusy = false;
    return 0;
}

void Epos4::handleEMCYMsg(can_message_t& msg) {
    m_uiError = (msg.data[1] << 8) + msg.data[0];
    if (m_uiError != 0)
        LOG_ERROR("Device Error for node %i: %h", m_uiNodeID, m_uiError);
}
