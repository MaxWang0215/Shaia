//
// One head axis. Wraps the shared Dynamixel bus and owns the normalized
// position -> degree mapping.
//
// This is the C++ equivalent of Shaia's `moveHeadTurn`/`moveNeckTilt`/... OSC
// handlers (Shaia/GestureInput/Shaiahead.py):
//     goal = pos * (max - min) + min
//

#ifndef SHAIACONTROLLER_HEADMOTOR_H
#define SHAIACONTROLLER_HEADMOTOR_H

#include <iostream>
#include <algorithm>
#include <optional>

#include "Config.h"
#include "DxlController.h"
#include "Def.h"
#include "ErrorDef.h"
#include "Logger.h"

class HeadMotor {
public:
    HeadMotor() : m_controller(DxlController::getInstance()) {}
    HeadMotor(const HeadMotor&) = delete;
    HeadMotor& operator=(const HeadMotor&) = delete;

    Error_t init(MotorConfig_t&& mc) {
        m_config = std::move(mc);

        if (m_config.operatingMode != OP_MODE_POSITION) {
            Error_t e = m_controller.setOperatingMode(m_config.id, m_config.operatingMode);
            ERROR_CHECK(e, e);
        }

        return enable();
    }

    Error_t reset() {
        return disable();
    }

    [[nodiscard]] std::string getName() const { return HeadCommandUtil::getCommand(m_config.name); }
    [[nodiscard]] int getId() const { return m_config.id; }

    void setVelocity(float fVelocity) { m_config.defaultVelocity = fVelocity; }

    Error_t set(std::optional<float> fVel) {
        if (fVel) setVelocity(*fVel);
        return kNoError;
    }

    Error_t enable() { return m_controller.servoOn(m_config.id); }
    Error_t disable() { return m_controller.servoOff(m_config.id); }

    // Move to the axis' default pose.
    Error_t zero() {
        return goTo(m_config.defaultPosition, m_config.defaultVelocity);
    }

    // normalizedPos is clamped to [0, 1] then mapped onto [minDeg, maxDeg].
    Error_t goTo(float normalizedPos, std::optional<float> vel) {
        normalizedPos = std::max(0.f, std::min(1.f, normalizedPos));
        float velocity = vel.value_or(m_config.defaultVelocity);

        Error_t e = m_controller.setProfileVelocity(m_config.id, velocity);
        ERROR_CHECK(e, e);

        float goalDeg = normalizedPos * (m_config.maxDeg - m_config.minDeg) + m_config.minDeg;
        LOG_TRACE("{} -> {} ({} deg) @ v={}", getName(), normalizedPos, goalDeg, velocity);
        return m_controller.setGoalPositionDeg(m_config.id, goalDeg);
    }

private:
    DxlController& m_controller;
    MotorConfig_t m_config;
};

#endif //SHAIACONTROLLER_HEADMOTOR_H
