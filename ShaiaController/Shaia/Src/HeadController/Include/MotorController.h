//
// Owns the head axes and dispatches HeadCmdPacket_t to them.
// Adapted from ShimonController-choreo's MotorController: same YAML driven
// setup, but there is only one motor type and no homing pass (Dynamixels come
// up knowing their absolute position).
//

#ifndef SHAIACONTROLLER_MOTORCONTROLLER_H
#define SHAIACONTROLLER_MOTORCONTROLLER_H

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "yaml-cpp/yaml.h"

#include "Def.h"
#include "Util.h"
#include "HeadCommands.h"
#include "Config.h"
#include "DxlController.h"
#include "HeadMotor.h"
#include "ErrorDef.h"
#include "Logger.h"

class MotorController {
public:
    explicit MotorController(std::string configFile) : m_configFile(std::move(configFile)) {}

    Error_t init() {
        Error_t e = DxlController::getInstance().init(DXL_PORT, DXL_BAUDRATE);
        ERROR_CHECK(e, e);

        return parseConfig(m_configFile);
    }

    Error_t reset() {
        for (auto& element: m_motors) {
            if (element.second) element.second->reset();
        }
        m_motors.clear();
        return DxlController::getInstance().reset();
    }

    // Move every axis to its configured default pose.
    Error_t zero() {
        LOG_INFO("Moving head to its default pose");
        for (auto& element: m_motors) {
            Error_t e = element.second->zero();
            ERROR_CHECK(e, e);
        }
        return kNoError;
    }

    Error_t servosOn(bool turnOn) {
        LOG_INFO(turnOn ? "Head servos on" : "Head servos off");
        for (auto& element: m_motors) {
            Error_t e = turnOn ? element.second->enable() : element.second->disable();
            ERROR_CHECK(e, e);
        }
        return kNoError;
    }

    Error_t send(const HeadCmdPacket_t& cmdPacket) {
        auto cmd = cmdPacket.cmd;
        auto it = m_motors.find(cmd);
        if (it == m_motors.end() || !it->second) {
            LOG_WARN("No head motor configured for {}", HeadCommandUtil::getCommand(cmd));
            return kInvalidArgsError;
        }

        switch (cmdPacket.cmdType) {
            case HeadCommandType::Dummy:
                return kNoError;
            case HeadCommandType::Control:
                return it->second->set(cmdPacket.fVel);
            case HeadCommandType::LowLevel:
                if (!cmdPacket.fPos) return kInvalidArgsError;
                return it->second->goTo(*cmdPacket.fPos, cmdPacket.fVel);
            case HeadCommandType::Behavioral:
                LOG_WARN("Behavioral commands are handled by HeadController, not here");
                return kNotImplementedError;
        }

        return kNoError;
    }

private:
    std::unordered_map<HeadCommand, std::unique_ptr<HeadMotor>> m_motors;
    std::string m_configFile;

    Error_t parseConfig(const std::string& configFile) {
        try {
            auto root = YAML::LoadFile(configFile);

            for (auto&& config: root) {
                auto cmd = HeadCommandUtil::getCommand(config["name"].as<std::string>());
                if (cmd == HeadCommand::Unknown) {
                    LOG_ERROR("Unknown axis name '{}' in {}. It must match HeadCommands.h",
                              config["name"].as<std::string>(), configFile);
                    return kUnknownCaseError;
                }

                if (m_motors.find(cmd) != m_motors.end()) {
                    LOG_ERROR("Duplicate axis name '{}' in {}", config["name"].as<std::string>(), configFile);
                    return kNamingError;
                }

                MotorConfig_t motorConfig {
                        .name = cmd,
                        .id = config["id"].as<int>(),
                        .minDeg = config["min"].as<float>(),
                        .maxDeg = config["max"].as<float>(),
                        .defaultPosition = config["default"] ? config["default"].as<float>() : 0.5f,
                        .defaultVelocity = config["defaultVel"] ? config["defaultVel"].as<float>() : 0.02f,
                        .operatingMode = config["operatingMode"]
                                         ? config["operatingMode"].as<int>()
                                         : OP_MODE_POSITION
                };

                auto m = std::make_unique<HeadMotor>();
                Error_t e = m->init(std::move(motorConfig));
                ERROR_CHECK(e, e);
                m_motors[cmd] = std::move(m);
            }
        } catch (YAML::BadFile&) {
            LOG_ERROR("{} not found. Run the binary from build/ or fix HM_CONFIG_FILE in Def.h", configFile);
            return kFileParseError;
        } catch (YAML::Exception& e) {
            LOG_ERROR("{}", e.msg);
            return kFileParseError;
        }

        if (m_motors.empty()) {
            LOG_ERROR("No head axes defined in {}", configFile);
            return kFileParseError;
        }

        LOG_INFO("Head configured with {} axes", m_motors.size());
        return kNoError;
    }
};

#endif //SHAIACONTROLLER_MOTORCONTROLLER_H
