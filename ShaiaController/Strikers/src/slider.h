//
// One slider axis (an arm on the shared rail), driven as an EPOS4 node in
// Cyclic Synchronous Position mode.
//
// New for Shaia - the choreo firmware only knew about strikers, because on the
// big Shimon the arms hang off IAI actuators the host drives over Modbus. Here
// the host just says "arm 1 to 12345 ticks over 450 ms" and this class runs
// the profile locally at the 1 ms PDO rate.
//
// The profile is a linear segment blended with parabolic ramps at both ends,
// same shape as Util::interpWithBlend, but evaluated per tick so we don't have
// to keep a multi-thousand point array per axis in RAM.
//

#ifndef SLIDER_H
#define SLIDER_H

#include "def.h"
#include "epos4/epos4.h"
#include "util.h"
#include "ErrorDef.h"

class Slider {
public:
    ~Slider() { reset(); }

    Error_t init(int iNodeId, MotorSpec spec) {
        int err = epos.init(iNodeId, spec, kEncoderResolution[iNodeId], kNodeDirection[iNodeId], 2000);
        if (err != 0) {
            LOG_ERROR("Epos init failed for slider node id: %i", iNodeId);
            return kSetValueError;
        }

        m_bMoving = false;
        m_bInitialized = true;
        return kNoError;
    }

    void reset() {
        stopMove();
        epos.reset();
        m_bInitialized = false;
    }

    uint8_t getNodeId() const {
        if (!m_bInitialized) return 0;
        return epos.getNodeId();
    }

    bool isInitialized() const { return m_bInitialized; }

    // Drive's own view of its power stage - see the 'v' command.
    void reportPowerStatus() { epos.reportPowerStatus(); }

    // Runs the drive into its hard stop and zeroes the encoder there. Blocking:
    // only called from setup()/the 'h' command, never from the PDO ISR.
    //
    // m_bHoming keeps update() - which runs in the RPDO timer interrupt - from
    // touching Epos4::m_txMsg while the blocking SDO traffic below is using it.
    Error_t home() {
        m_bHoming = true;
        Error_t err = homeAxis();
        m_bHoming = false;
        return err;
    }

    Error_t homeAxis() {
        if (!m_bInitialized) return kNotInitializedError;

        stopMove();
        m_bReleased = false;

        int err = epos.setOpMode(OpMode::Homing, PDO_RATE, -3, HomingMethod::CurrentThresholdNegative);
        if (err != 0) return kSetValueError;

        epos.reportHomingLimits();

        err = epos.setHomingCurrentThreshold(SLIDER_HOMING_CURRENT_THRESHOLD);
        if (err != 0) return kSetValueError;

        err = epos.startHoming();
        if (err != 0) return kSetValueError;

        unsigned long start = millis();
        unsigned long lastReport = 0;
        bool bStarted = false;
        while (millis() - start < SLIDER_HOMING_TIMEOUT_MS) {
            // A current threshold home that trips without the axis turning
            // looks identical in the status word to a real one, so watch the
            // encoder and the current rather than trusting "homed".
            if ((millis() - lastReport) > SLIDER_HOMING_REPORT_MS) {
                lastReport = millis();
                epos.reportMotion("homing");
            }

            HomingStatus status = epos.getHomingStatus();
            if (status == HomingStatus::Completed) {
                epos.reportMotion("homed");
                LOG_LOG("Slider %i homed", epos.getNodeId());
                m_iCurrentTarget = 0;
                return kNoError;
            }

            if (status == HomingStatus::InProgress) bStarted = true;

            if (status == HomingStatus::Error) {
                LOG_ERROR("Slider %i homing failed. status: %h", epos.getNodeId(), epos.getStatusWord());
                return kSetValueError;
            }

            // Interrupted also reads as "not started", so it only counts as a
            // failure once homing has actually been seen running.
            if (status == HomingStatus::Interrupted && bStarted) {
                LOG_ERROR("Slider %i homing interrupted. status: %h", epos.getNodeId(), epos.getStatusWord());
                return kSetValueError;
            }

            if (!bStarted && (millis() - start) > SLIDER_HOMING_START_GRACE_MS) {
                LOG_ERROR("Slider %i homing never started - drive did not enter Operation Enabled. status: %h",
                          epos.getNodeId(), epos.getStatusWord());
                return kSetValueError;
            }

            delay(20);
        }

        epos.reportMotion("timeout");
        LOG_ERROR("Slider %i homing timed out", epos.getNodeId());
        return kTimeoutError;
    }

    Error_t enablePDO(bool bEnable) {
        int err;
        if (bEnable) {
            err = epos.setOpMode(OpMode::CyclicSyncPosition);
            if (err != 0) return kSetValueError;
            err = epos.setNMTState(NMTState::Operational);
        } else {
            err = epos.setNMTState(NMTState::PreOperational);
        }

        if (err != 0) return kSetValueError;
        return kNoError;
    }

    Error_t enable(bool bEnable) {
        int err = epos.setEnable(bEnable);
        if (err != 0) return kSetValueError;
        return kNoError;
    }

    // Drop a latched error so update() will publish setpoints again.
    //
    // The drive's error code only ever reaches us through EMCY - TPDO3 keeps
    // the default 6 byte statusword+position mapping, so PDO_processMsg never
    // refreshes it - which means a one-off fault stays latched forever unless
    // something clears it here. Blocking SDO: callers must have the PDO timer
    // stopped, or be outside the ISR.
    Error_t clearFault() {
        if (epos.clearFault() != 0) return kSetValueError;
        return kNoError;
    }

    // Cheap when healthy (no CAN traffic at all), expensive only when the node
    // has actually faulted. Mirrors Striker::checkAndRecover.
    void checkAndRecover() {
        if (!m_bInitialized || m_bReleased || m_bHoming || m_bRecovering) return;
        if (epos.isDeviceReady()) return;

        LOG_LOG("Recovering slider %i from error %h", epos.getNodeId(), epos.getDeviceError());

        // clearFault() zeroes m_uiError, which would let update() start writing
        // Epos4::m_txMsg from the ISR while the SDO traffic below still owns it.
        m_bRecovering = true;
        recoverAxis();
        m_bRecovering = false;
    }

    void recoverAxis() {
        if (enablePDO(false) != kNoError) {
            LOG_ERROR("Slider %i: cannot disable PDO", epos.getNodeId());
            return;
        }

        if (clearFault() != kNoError) {
            LOG_ERROR("Slider %i: cannot clear fault", epos.getNodeId());
            return;
        }

        if (enablePDO(true) != kNoError) {
            LOG_ERROR("Slider %i: cannot enable PDO", epos.getNodeId());
            return;
        }

        // Resume from where the axis actually is - an interrupted move must not
        // snap back to its old commanded target.
        int32_t ticks = 0;
        if (readPosition(ticks) == kNoError) {
            stopMove();
            m_iCurrentTarget = ticks;
            m_iQf = ticks;
        }

        if (enable(true) != kNoError)
            LOG_ERROR("Slider %i: cannot re-enable device", epos.getNodeId());
    }

    // Start a timed move to an absolute encoder position.
    Error_t moveTo(int32_t targetTicks, uint16_t duration_ms) {
        if (!m_bInitialized) return kNotInitializedError;

        targetTicks = constrain(targetTicks, 0, SLIDER_MAX_TRAVEL_TICKS);
        if (duration_ms == 0) duration_ms = SLIDER_DEFAULT_MOVE_MS;

        int32_t n = duration_ms / PDO_RATE;
        if (n > MAX_TRAJ_POINTS) n = MAX_TRAJ_POINTS;
        if (n < 2) n = 2;

        // Start from where the drive actually is, so an interrupted move does
        // not jump.
        m_iQ0 = epos.getCurrentPosition_ticks();
        m_iQf = targetTicks;
        m_iN = n;
        m_iNb = (int32_t)(SLIDER_BLEND * (float)n);
        if (m_iNb < 1) m_iNb = 1;
        if (m_iNb > n / 2) m_iNb = n / 2;

        // Parabolic blend coefficient: q(i) = q0 + a2 * i^2 over the ramp.
        m_fA2 = 0.5f * (float)(m_iQf - m_iQ0) / ((float)m_iNb * (float)(m_iN - m_iNb));

        m_iIdx = 0;
        m_bMoving = true;
        m_iCurrentTarget = targetTicks;
        return kNoError;
    }

    void stopMove() {
        m_bMoving = false;
        m_iIdx = 0;
    }

    bool isMoving() const { return m_bMoving; }
    bool isReleased() const { return m_bReleased; }

    // Read the drive's own encoder over SDO. Works in any state, including
    // while the axis is released, which is what makes hand calibration
    // possible.
    Error_t readPosition(int32_t& ticks) {
        if (!m_bInitialized) return kNotInitializedError;
        int32_t pos = 0;
        if (epos.getActualPosition(&pos) != 0) return kGetValueError;
        ticks = pos;
        return kNoError;
    }

    // Drop out of position control so the arm can be pushed by hand. update()
    // stops publishing setpoints, otherwise the drive would fight the operator
    // and then snap back the moment it is re-enabled.
    Error_t release() {
        if (!m_bInitialized) return kNotInitializedError;

        stopMove();
        m_bReleased = true;

        Error_t e = enablePDO(false);
        if (e != kNoError) return e;

        return enable(false);
    }

    // Re-engage, holding wherever the arm was left rather than wherever it was
    // last commanded to - without this the axis jumps across the rail.
    Error_t hold() {
        if (!m_bInitialized) return kNotInitializedError;

        int32_t ticks = 0;
        Error_t e = readPosition(ticks);
        if (e != kNoError) {
            LOG_ERROR("Slider %i: cannot read position to hold at", epos.getNodeId());
            return e;
        }

        m_iCurrentTarget = ticks;
        m_iQf = ticks;
        stopMove();

        e = enablePDO(true);
        if (e != kNoError) return e;

        e = enable(true);
        if (e != kNoError) return e;

        m_bReleased = false;
        LOG_LOG("Slider %i holding at %i ticks", epos.getNodeId(), (int) ticks);
        return kNoError;
    }

    // Called every PDO_RATE ms from the timer ISR.
    void update() {
        if (!m_bInitialized || m_bReleased || m_bHoming || m_bRecovering) return;

        if (!epos.isDeviceReady()) {
            stopMove();
            return;
        }

        if (!m_bMoving) {
            // Keep publishing the last position so the drive does not trip a
            // 0x8250 RPDO timeout.
            epos.PDO_setPosition(m_iCurrentTarget);
            return;
        }

        if (m_iIdx >= m_iN) {
            m_bMoving = false;
            m_iCurrentTarget = m_iQf;
            epos.PDO_setPosition(m_iQf);
            return;
        }

        epos.PDO_setPosition(positionAt(m_iIdx));
        ++m_iIdx;
    }

    Error_t setRxMsg(can_message_t& msg) {
        int err = epos.setRxMsg(msg);
        if (err != 0) return kSetValueError;
        return kNoError;
    }

    Error_t PDO_processMsg(can_message_t& msg) {
        int err = epos.PDO_processMsg(msg);
        if (err != 0) return kSetValueError;
        return kNoError;
    }

    void handleEMCYMsg(can_message_t& msg) {
        epos.handleEMCYMsg(msg);
    }

private:
    Epos4 epos;
    bool m_bInitialized = false;
    volatile bool m_bHoming = false;
    volatile bool m_bRecovering = false;
    volatile bool m_bMoving = false;
    volatile bool m_bReleased = false;

    int32_t m_iQ0 = 0;
    int32_t m_iQf = 0;
    int32_t m_iN = 2;
    int32_t m_iNb = 1;
    float m_fA2 = 0;
    volatile int32_t m_iIdx = 0;
    int32_t m_iCurrentTarget = 0;

    // Position at trajectory index i: parabolic ramp up, linear cruise,
    // parabolic ramp down.
    int32_t positionAt(int32_t i) const {
        if (i < m_iNb) {
            return m_iQ0 + (int32_t)(m_fA2 * (float)i * (float)i);
        }

        if (i >= m_iN - m_iNb) {
            float j = (float)(m_iN - 1 - i);
            return m_iQf - (int32_t)(m_fA2 * j * j);
        }

        float blendOffset = m_fA2 * (float)m_iNb * (float)m_iNb;
        float qa = (float)m_iQ0 + blendOffset;
        float qb = (float)m_iQf - blendOffset;
        float alpha = (float)(i - m_iNb) / (float)(m_iN - (2 * m_iNb));
        return (int32_t)(qa + alpha * (qb - qa));
    }
};

#endif // SLIDER_H
