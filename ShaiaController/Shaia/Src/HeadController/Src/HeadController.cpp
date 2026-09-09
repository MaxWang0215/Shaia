//
// Head subsystem. See HeadController.h.
//

#include "HeadController.h"

HeadController::HeadController(OscListener& oscListener,
                               const std::string& configFile,
                               const std::string& danceModesFile,
                               tp programStartTime)
        : m_kProgramStartTime(programStartTime)
        , m_danceModesFile(danceModesFile)
        , m_oscListener(oscListener)
        , m_motorController(configFile)
        , m_cmdManager(m_cv)
        , m_danceGesture(m_cmdManager)
        , m_breatheGesture(m_cmdManager)
        , m_headBangGesture(m_cmdManager)
        , m_lookGesture(m_cmdManager) {
    HeadCommandUtil::populateStrCmdMap();
}

HeadController::~HeadController() {
    reset();
}

Error_t HeadController::init() {
    m_oscListener.setHeadCallback([this](auto&& PH1, auto&& PH2, auto&& PH3) {
        headCallback(std::forward<decltype(PH1)>(PH1),
                     std::forward<decltype(PH2)>(PH2),
                     std::forward<decltype(PH3)>(PH3));
    });

    m_oscListener.setHeadGestureCallback([this](auto&& PH1, auto&& PH2) {
        gestureCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    });

    m_oscListener.setHeadStopCallback([this] { stopGestureCallback(); });
    m_oscListener.setHeadLookCallback([this] { lookCallback(); });

    Error_t e = m_motorController.init();
    ERROR_CHECK(e, e);

    e = m_danceGesture.load(m_danceModesFile);
    ERROR_CHECK(e, e);

    m_bInitialized = true;
    LOG_INFO("Head Initialized");
    return kNoError;
}

Error_t HeadController::reset() {
    if (!m_bInitialized) return kNoError;

    stopAllGestures();
    m_danceGesture.stopRunner();
    m_motorController.reset();

    m_bInitialized = false;
    return kNoError;
}

Error_t HeadController::start() {
    if (!m_bInitialized) return kNotInitializedError;

    Error_t e = m_motorController.zero();
    ERROR_CHECK(e, e);

    m_bRunning = true;
    m_danceGesture.start();
    m_pThread = std::make_unique<std::thread>([this] { threadHandler(); });
    return kNoError;
}

Error_t HeadController::stop() {
    if (!m_bInitialized) return kNotInitializedError;

    stopAllGestures();
    m_danceGesture.stopRunner();

    m_bRunning = false;
    m_cv.notify_all();
    if (m_pThread && m_pThread->joinable()) m_pThread->join();
    m_pThread = nullptr;

    m_motorController.reset();

    // Mark uninitialized so the destructor's reset() is a no-op - by then the
    // Dynamixel singleton may already have been destroyed.
    m_bInitialized = false;
    return kNoError;
}

void HeadController::headCallback(const char* sCmd, std::optional<float> val1, std::optional<float> val2) {
    HeadCommand cmd = HeadCommandUtil::getCommand(sCmd);

    if (cmd == HeadCommand::Unknown) {
        LOG_WARN("Unknown head command: {}", sCmd);
        return;
    }

    if (HeadCommandUtil::isHighLevelCommand(cmd)) {
        handleHighLevelCommands(cmd, val1);
        return;
    }

    Port::Head port = HeadCommandUtil::getPortFor(cmd);
    if (port == Port::Head::kNumPorts) return;

    if (!val1) {
        LOG_WARN("{} needs a position argument", sCmd);
        return;
    }

    HeadCmdPacket_t cmdPkt(cmd, val1, val2);
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (!m_cmdManager.push(port, std::move(cmdPkt))) LOG_WARN("Warning: Cannot push: {} ", sCmd);
    }
}

void HeadController::gestureCallback(const char* name, std::optional<float> bpm) {
    // A gesture takes over the head, so cancel the one shot behaviours first.
    m_breatheGesture.stop();
    m_danceGesture.play(name, bpm.value_or(DEFAULT_BPM));
}

void HeadController::stopGestureCallback() {
    stopAllGestures();
}

void HeadController::lookCallback() {
    stopAllGestures();
    m_lookGesture.start();
}

void HeadController::handleHighLevelCommands(HeadCommand cmd, std::optional<float> val) {
    switch (cmd) {
        case HeadCommand::Dance:
            // /head-commands DANCE <bpm> replays whatever gesture was last
            // named on /head; use /head <name> [bpm] to pick one.
            LOG_WARN("Use '{} <gestureName> [bpm]' to start a gesture", HEAD_OSC_ROUTE);
            break;
        case HeadCommand::StopGesture:
            stopAllGestures();
            break;
        case HeadCommand::Look:
            stopAllGestures();
            m_lookGesture.start();
            break;
        case HeadCommand::Breathe:
            m_danceGesture.stop();
            m_breatheGesture.start();
            break;
        case HeadCommand::StopBreath:
            m_breatheGesture.stop();
            break;
        case HeadCommand::HeadBang:
            m_headBangGesture.beat((int)val.value_or(960.f));
            break;
        default:
            LOG_ERROR("Unknown HighLevel Command: {}", HeadCommandUtil::getCommand(cmd));
            break;
    }
}

void HeadController::stopAllGestures() {
    m_danceGesture.stop();
    m_breatheGesture.stop();
    m_headBangGesture.stop();
}

void HeadController::threadHandler() {
    while (true) {
        auto loopTime = std::chrono::steady_clock::now();
        {
            std::unique_lock<std::mutex> lk(m_mtx);
            LOG_TRACE("Waiting for Head Command...");
            m_cv.wait(lk, [this] { return !m_cmdManager.isEmpty() || !m_bRunning; });

            if (!m_bRunning) break;
        }

        // Drain both queues rather than one packet each - a dance mode can fire
        // several axes on the same beat and they should land together.
        HeadCmdPacket_t cmdPkt;
        while (m_cmdManager.pop(Port::Head::Dxl, cmdPkt, false))
            m_motorController.send(cmdPkt);   // Ignore any error

        while (m_cmdManager.pop(Port::Head::Gesture, cmdPkt, false))
            m_motorController.send(cmdPkt);

        std::this_thread::sleep_until(loopTime + std::chrono::milliseconds(HEAD_THREAD_TIME_PERIOD));
    }
}
