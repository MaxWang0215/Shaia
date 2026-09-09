//
// One shot "look at the audience" pose: lift the neck, face forward, tilt the
// head slightly down, close the mouth.
//
// Port of shaia_targetUpdated/head.py's GestureRunner.look(). Velocities match
// the lq_sway dance mode so the move reads as part of the same vocabulary.
//

#ifndef SHAIACONTROLLER_LOOKGESTURE_H
#define SHAIACONTROLLER_LOOKGESTURE_H

#include "Gesture.h"
#include "Logger.h"

class LookGesture : public Gesture {
public:
    explicit LookGesture(CommandManager<HeadCmdPacket_t, Port::Head>& cmdManager) : Gesture(cmdManager) {}

    void start() override {
        move(HeadCommand::NeckTilt, 0.8f, 0.010f);
        move(HeadCommand::NeckTurn, 0.5f, 0.015f);
        move(HeadCommand::HeadTurn, 0.5f, 0.020f);
        move(HeadCommand::HeadTilt, 0.3f, 0.020f);
        move(HeadCommand::Mouth,    0.0f, 0.020f);
        LOG_INFO("Look pose");
    }
};

#endif //SHAIACONTROLLER_LOOKGESTURE_H
