//
// Gesture base class.
//
// A gesture never touches the motors directly - it pushes low level
// HeadCmdPacket_t primitives onto the head command queue, which the
// HeadController thread drains. Same contract as the choreo build.
//

#ifndef SHAIACONTROLLER_GESTURE_H
#define SHAIACONTROLLER_GESTURE_H

#include <iostream>
#include <optional>

#include "CommandManager.h"
#include "HeadCommands.h"

class Gesture {
public:
    explicit Gesture(CommandManager<HeadCmdPacket_t, Port::Head>& cmdManager) : m_cmdManager(cmdManager) {}
    virtual ~Gesture() = default;

    virtual void start() {}
    virtual void stop() {}

protected:
    CommandManager<HeadCmdPacket_t, Port::Head>& m_cmdManager;

    void move(HeadCommand axis, float normalizedPos, float velocity) {
        HeadCmdPacket_t pkt(axis, normalizedPos, velocity);
        m_cmdManager.push(Port::Head::Gesture, std::move(pkt));
    }
};

#endif //SHAIACONTROLLER_GESTURE_H
