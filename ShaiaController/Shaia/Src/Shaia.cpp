//
// Top level orchestrator. See Shaia.h.
//

#include <cstring>
#include "Shaia.h"

Shaia::Shaia(int port,
             const std::string& hmConfigFile,
             const std::string& danceModesFile,
             tp programStartTime)
        : m_kProgramStartTime(programStartTime)
        , m_oscListener(port)
        , m_headController(m_oscListener, hmConfigFile, danceModesFile, programStartTime)
        , m_armController(m_oscListener, programStartTime)
        , m_positionTransmitQueue(POSITION_BUFFER_SIZE) {
    m_oscListener.setSystemMsgCallback([this](auto&& PH) {
        sysMsgCallback(std::forward<decltype(PH)>(PH));
    });
}

Error_t Shaia::init(const std::string& masterHostName, int masterPort) {
    Error_t e;

#ifdef USE_ARMS
    e = initArms();
    if (e != kNoError) {
        LOG_ERROR("Arm Init failed. Error Code {}", (int)e);
        return e;
    }

    m_armController.setStatusCallback([this](auto&& PH1) {
        armStatusCallback(std::forward<decltype(PH1)>(PH1));
    });

    m_armController.setPositionCallback([this](auto&& PH1) {
        armPositionCallback(std::forward<decltype(PH1)>(PH1));
    });
#else
    LOG_INFO("Not using Arms");
#endif

#ifdef USE_HEAD
    e = initHead();
    if (e != kNoError) {
        LOG_ERROR("Head Init failed. Error Code {}", (int)e);
        return e;
    }
#else
    LOG_INFO("Not using Head");
#endif

    e = initMasterTransmitter(masterHostName, masterPort);
    if (e != kNoError) {
        LOG_ERROR("master transmitter Init failed. Error Code {}", (int)e);
        return e;
    }

    return kNoError;
}

void Shaia::start() {
#ifdef USE_ARMS
    m_armController.start();
#endif

#ifdef USE_HEAD
    m_headController.start();
#endif

    m_bRunning = true;
    m_pMasterTransmitThread = std::make_unique<std::thread>([this] { masterTransmitHandler(); });

    // Start listening to OSC messages. Call this last - it blocks.
    LOG_INFO("Listening for OSC on port {}", PORT);
    m_oscListener.start();
}

void Shaia::stop() {
    m_oscListener.stop();

#ifdef USE_HEAD
    m_headController.stop();
#endif

#ifdef USE_ARMS
    m_armController.stop();
#endif

    m_bRunning = false;
    m_cv.notify_all();
    if (m_pMasterTransmitThread && m_pMasterTransmitThread->joinable()) m_pMasterTransmitThread->join();
    m_pMasterTransmitThread = nullptr;
    LOG_TRACE("Shaia Stopped...");
}

Error_t Shaia::initArms() {
    LOG_INFO("Initializing Arms");
    return m_armController.init();
}

Error_t Shaia::initHead() {
    LOG_INFO("Initializing Head...");
    return m_headController.init();
}

Error_t Shaia::initMasterTransmitter(const std::string& host, int port) {
    return m_oscTransmitter.init(host, port);
}

void Shaia::masterTransmitHandler() {
    while (m_bRunning) {
        bool success = false;
        std::array<int, NUM_ARMS> position{};
        {
            std::unique_lock<std::mutex> lk(m_mtx);
            m_cv.wait(lk, [this] { return !m_positionTransmitQueue.isEmpty() || !m_bRunning; });
            if (!m_bRunning) break;
            success = m_positionTransmitQueue.pop(position);
        }

        if (success) m_oscTransmitter.send("/arm", position);
    }
}

void Shaia::sysMsgCallback(const char* msg) {
    if (strcmp(msg, "quit") == 0 || strcmp(msg, "close") == 0 || strcmp(msg, "exit") == 0) {
        stop();
    } else if (strcmp(msg, "setHost") == 0 || strcmp(msg, "SetHost") == 0 || strcmp(msg, "sethost") == 0) {
        m_oscTransmitter.setHost(m_oscListener.getHost(), MASTER_PORT);
        LOG_INFO("Host and port set to {}, {}", m_oscTransmitter.getHost(), MASTER_PORT);
    } else {
        LOG_WARN("Unknown system message: {}", msg);
    }
}

void Shaia::armStatusCallback(Status_t status) {
    m_oscTransmitter.send("/arm-status", status);
}

void Shaia::armPositionCallback(std::array<int, NUM_ARMS> position) {
    m_positionTransmitQueue.push(position);
    m_cv.notify_all();
}
