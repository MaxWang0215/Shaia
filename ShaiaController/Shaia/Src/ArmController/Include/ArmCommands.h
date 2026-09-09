//
// String <-> enum mapping for the single argument /arm commands.
// Adapted from ShimonController-choreo. Shaia's MCU only implements HOME and
// RESTART; the servo on/off/alarm commands are accepted for API compatibility
// with the choreo controller and logged as unsupported.
//

#ifndef SHAIACONTROLLER_ARMCOMMANDS_H
#define SHAIACONTROLLER_ARMCOMMANDS_H

#include <iostream>
#include <string>
#include <unordered_map>
#include "Util.h"

enum class ArmCommand {
    Unknown = -1,
    Off = 0,
    On = 1,
    Home = 2,
    AlarmClear = 3,
    Reset = 4
};

class ArmCommandUtil {
public:
    static void populateStrCmdMap() {
        for (auto& element: kCmdStrMap) {
            strCmdMap[element.second] = element.first;
        }
    }

    static std::string getCommand(ArmCommand c) {
        if (kCmdStrMap.find(c) == kCmdStrMap.end()) return "UNKNOWN";
        return kCmdStrMap.at(c);
    }

    static ArmCommand getCommand(const std::string& c) {
        auto data = Util::toUpperCase(c);
        if (strCmdMap.find(data) == strCmdMap.end()) return ArmCommand::Unknown;
        return strCmdMap[data];
    }

    static inline const std::unordered_map<ArmCommand, std::string> kCmdStrMap = {
            { ArmCommand::Unknown,    "UNKNOWN" },
            { ArmCommand::Off,        "OFF"     },
            { ArmCommand::On,         "ON"      },
            { ArmCommand::Home,       "HOME"    },
            { ArmCommand::AlarmClear, "CLEAR"   },
            { ArmCommand::Reset,      "RESTART" }
    };

    static inline std::unordered_map<std::string, ArmCommand> strCmdMap;
};

#endif //SHAIACONTROLLER_ARMCOMMANDS_H
