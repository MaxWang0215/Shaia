//
// Top level orchestrator. Owns the OSC listener/transmitter and the two
// subsystems, exactly like ShimonController-choreo's Shimon class.
//

#ifndef SHAIACONTROLLER_SHAIA_H
#define SHAIACONTROLLER_SHAIA_H

#include <iostream>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <condition_variable>

#include "ArmController.h"
#include "HeadController.h"
#include "OscListener.h"
#include "OscTransmitter.h"
#include "CircularQueue.h"

#include "Def.h"
#include "ErrorDef.h"
#include "StatusDef.h"
#include "Logger.h"

class Shaia {
public:
    Shaia(int port,
          const std::string& hmConfigFile,
          const std::string& danceModesFile,
          tp programStartTime = std::chrono::steady_clock::now());

    Error_t init(const std::string& masterHostName, int masterPort);
    void start();
    void stop();

private:
    std::atomic<bool> m_bRunning = false;
    const tp m_kProgramStartTime;
    OscTransmitter m_oscTransmitter;
    OscListener m_oscListener;
    HeadController m_headController;
    ArmController m_armController;

    std::unique_ptr<std::thread> m_pMasterTransmitThread = nullptr;
    std::mutex m_mtx;
    std::condition_variable m_cv;
    CircularQueue<std::array<int, NUM_ARMS>> m_positionTransmitQueue;

    void sysMsgCallback(const char* msg);
    void armStatusCallback(Status_t status);
    void armPositionCallback(std::array<int, NUM_ARMS> position);

    Error_t initArms();
    Error_t initHead();
    Error_t initMasterTransmitter(const std::string& host, int port);
    void masterTransmitHandler();
};

#endif //SHAIACONTROLLER_SHAIA_H
