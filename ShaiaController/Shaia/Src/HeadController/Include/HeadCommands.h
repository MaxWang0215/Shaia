//
// Head command vocabulary. Adapted from ShimonController-choreo, retargeted to
// Shaia's 5 Dynamixel axes (Shaia/GestureInput/Shaiahead.py) and its gesture
// set (Shaia/Dance/danceModes.json).
//
// Low level positions are normalized [0, 1] and map onto the per axis degree
// range in Config/HeadMotorConfig.yaml. Velocities are the same normalized
// [0, 1] the dance modes use (scaled to Dynamixel profile velocity units).
//

#ifndef SHAIACONTROLLER_HEADCOMMANDS_H
#define SHAIACONTROLLER_HEADCOMMANDS_H

#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>
#include "Def.h"
#include "Util.h"

// Order matters! getPortFor() and isHighLevelCommand() rely on it.
enum class HeadCommand {
    Unknown = -1,
    /****** Dynamixel axes *******/
    HeadTurn = 0,
    HeadTilt,
    Mouth,
    NeckTilt,
    NeckTurn,
    /****** High level commands *******/
    Dance = 5,      // Play a gesture from danceModes.json
    StopGesture,
    Look,
    Breathe,
    StopBreath,
    HeadBang
};

enum class HeadCommandType {
    Dummy = 0,
    Control,
    LowLevel,
    Behavioral
};

class HeadCmdPacket_t {
public:
    HeadCmdPacket_t() = default;
    HeadCmdPacket_t(HeadCommand command, std::optional<float> fPosition)
            : cmd(command), cmdType(HeadCommandType::LowLevel), fPos(fPosition) {}
    HeadCmdPacket_t(HeadCommand command, std::optional<float> fPosition, std::optional<float> fVelocity)
            : cmd(command), cmdType(HeadCommandType::LowLevel), fPos(fPosition), fVel(fVelocity) {}
    explicit HeadCmdPacket_t(HeadCommand command)
            : cmd(command), cmdType(HeadCommandType::Behavioral) {}

    HeadCommand cmd = HeadCommand::Unknown;
    HeadCommandType cmdType = HeadCommandType::Dummy;

    std::optional<float> fPos;
    std::optional<float> fVel;

    HeadCmdPacket_t(const HeadCmdPacket_t&) = delete;
    HeadCmdPacket_t& operator=(const HeadCmdPacket_t&) = delete;

    HeadCmdPacket_t(HeadCmdPacket_t&& pkt) noexcept
            : cmd(pkt.cmd), cmdType(pkt.cmdType), fPos(pkt.fPos), fVel(pkt.fVel) {}

    HeadCmdPacket_t& operator=(HeadCmdPacket_t&& other) noexcept {
        cmd = other.cmd;
        cmdType = other.cmdType;
        fPos = other.fPos;
        fVel = other.fVel;
        return *this;
    }
};

class HeadCommandUtil {
public:
    static void populateStrCmdMap() {
        for (auto& element: kCmdStrMap) {
            strCmdMap[element.second] = element.first;
        }
    }

    static std::string getCommand(HeadCommand c) {
        if (kCmdStrMap.find(c) == kCmdStrMap.end()) return "UNKNOWN";
        return kCmdStrMap.at(c);
    }

    static HeadCommand getCommand(const std::string& c) {
        auto data = Util::toUpperCase(c);
        if (strCmdMap.find(data) == strCmdMap.end()) return HeadCommand::Unknown;
        return strCmdMap[data];
    }

    static Port::Head getPortFor(const std::string& c) {
        return getPortFor(getCommand(c));
    }

    static Port::Head getPortFor(HeadCommand cmd) {
        if (cmd == HeadCommand::Unknown) return Port::Head::kNumPorts;
        if (cmd < HeadCommand::Dance) return Port::Head::Dxl;
        return Port::Head::kNumPorts;
    }

    static bool isHighLevelCommand(HeadCommand cmd) {
        return cmd >= HeadCommand::Dance;
    }

    static inline const std::unordered_map<HeadCommand, std::string> kCmdStrMap = {
            { HeadCommand::Unknown,     "UNKNOWN"    },
            { HeadCommand::HeadTurn,    "HEADTURN"   },
            { HeadCommand::HeadTilt,    "HEADTILT"   },
            { HeadCommand::Mouth,       "MOUTH"      },
            { HeadCommand::NeckTilt,    "NECKTILT"   },
            { HeadCommand::NeckTurn,    "NECKTURN"   },
            { HeadCommand::Dance,       "DANCE"      },
            { HeadCommand::StopGesture, "STOP"       },
            { HeadCommand::Look,        "LOOK"       },
            { HeadCommand::Breathe,     "BREATHING"  },
            { HeadCommand::StopBreath,  "STOPBREATH" },
            { HeadCommand::HeadBang,    "BEAT"       }
    };

    static inline std::unordered_map<std::string, HeadCommand> strCmdMap;
};

#endif //SHAIACONTROLLER_HEADCOMMANDS_H
