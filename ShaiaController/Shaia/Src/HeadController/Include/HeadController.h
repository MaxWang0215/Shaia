//
// Head subsystem: 5 Dynamixel axes + the gesture engine.
//
// Same structure as ShimonController-choreo's HeadController - OSC callbacks
// enqueue command packets, one worker thread drains the queue and talks to the
// motors - with the Copley/HD half removed and Shaia's danceModes.json gesture
// runner added.
//

#ifndef SHAIACONTROLLER_HEADCONTROLLER_H
#define SHAIACONTROLLER_HEADCONTROLLER_H

#include <iostream>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <condition_variable>

#include "MotorController.h"
#include "HeadCommands.h"
#include "CommandManager.h"
#include "OscListener.h"

#include "Def.h"
#include "Util.h"
#include "ErrorDef.h"
#include "Logger.h"

// Gestures
#include "Gestures/DanceGesture.h"
#include "Gestures/BreatheGesture.h"
#include "Gestures/HeadBangGesture.h"
#include "Gestures/LookGesture.h"

class HeadController {
public:
    HeadController(OscListener& oscListener,
                   const std::string& configFile,
                   const std::string& danceModesFile,
                   tp programStartTime);
    ~HeadController();

    Error_t init();
    Error_t reset();

    Error_t start();
    Error_t stop();

private:
    volatile std::atomic<bool> m_bRunning = false;
    bool m_bInitialized = false;
    std::unique_ptr<std::thread> m_pThread = nullptr;
    std::mutex m_mtx;
    std::condition_variable m_cv;
    const tp m_kProgramStartTime;
    const std::string m_danceModesFile;

    OscListener& m_oscListener;
    MotorController m_motorController;
    CommandManager<HeadCmdPacket_t, Port::Head> m_cmdManager;

    DanceGesture m_danceGesture;
    BreatheGesture m_breatheGesture;
    HeadBangGesture m_headBangGesture;
    LookGesture m_lookGesture;

    void headCallback(const char* cmd, std::optional<float> val1, std::optional<float> val2);
    void gestureCallback(const char* name, std::optional<float> bpm);
    void stopGestureCallback();
    void lookCallback();

    void threadHandler();
    void handleHighLevelCommands(HeadCommand cmd, std::optional<float> val);
    void stopAllGestures();
};

#endif //SHAIACONTROLLER_HEADCONTROLLER_H
