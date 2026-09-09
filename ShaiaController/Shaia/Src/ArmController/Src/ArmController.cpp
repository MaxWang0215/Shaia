//
// Slider path planning + striker scheduling. See ArmController.h.
//

#include "ArmController.h"
#include "Util.h"

ArmController::ArmController(OscListener& oscListener, tp programStartTime)
        : m_oscListener(oscListener)
        , m_transport(McuTransport::getInstance())
        , m_strikerController(programStartTime)
        , kProgramStartTime(programStartTime) {
    for (int i = 0; i < NUM_ARMS; ++i) {
        m_pArms[i] = new Arm(i, kInitialPosition[i], m_transport);
    }

    ArmCommandUtil::populateStrCmdMap();
}

ArmController::~ArmController() {
    reset();

    for (int i = 0; i < NUM_ARMS; ++i) {
        delete m_pArms[i];
        m_pArms[i] = nullptr;
    }
}

Error_t ArmController::init() {
    Error_t e;

    e = m_transport.init(MCU_PORT, MCU_BAUDRATE);
    ERROR_CHECK(e, e);

    LOG_TRACE("Setting setArmServoCallback");
    m_oscListener.setArmServoCallback([this](auto&& PH1) {
        armServoCallback(std::forward<decltype(PH1)>(PH1));
    });

    LOG_TRACE("Setting setArmMidiCallback");
    m_oscListener.setArmMidiCallback([this](auto&& PH1, auto&& PH2, auto&& PH3) {
        armMidiCallback(std::forward<decltype(PH1)>(PH1),
                        std::forward<decltype(PH2)>(PH2),
                        std::forward<decltype(PH3)>(PH3));
    });

    LOG_TRACE("Setting setArmRawCallback");
    m_oscListener.setArmRawCallback([this](auto&& PH1, auto&& PH2, auto&& PH3, auto&& PH4) {
        armCallback(std::forward<decltype(PH1)>(PH1),
                    std::forward<decltype(PH2)>(PH2),
                    std::forward<decltype(PH3)>(PH3),
                    std::forward<decltype(PH4)>(PH4));
    });

    m_oscListener.setArmHomeCallback([this] { armHomeCallback(); });
    m_oscListener.setArmResetCallback([this] { armResetCallback(); });

    m_oscListener.setStrikerChoreoCallback([this](auto&& PH1, auto&& PH2, auto&& PH3) {
        strikerChoreoCallback(std::forward<decltype(PH1)>(PH1),
                              std::forward<decltype(PH2)>(PH2),
                              std::forward<decltype(PH3)>(PH3));
    });

    m_oscListener.setStrikerMidiCallback([this](auto&& PH1, auto&& PH2, auto&& PH3, auto&& PH4) {
        strikerMidiCallback(std::forward<decltype(PH1)>(PH1),
                            std::forward<decltype(PH2)>(PH2),
                            std::forward<decltype(PH3)>(PH3),
                            std::forward<decltype(PH4)>(PH4));
    });

    e = m_strikerController.init();
    ERROR_CHECK(e, e);

    m_iNoteCounter = 0;
    m_bHasLastNote = false;
    m_bInitialized = true;

    return kNoError;
}

Error_t ArmController::reset() {
    // stop() already tore everything down; the destructor calling us again
    // must not touch the MCU singleton, which may be gone by then.
    if (!m_bInitialized) return kNoError;

    Error_t e = resetArms();
    ERROR_CHECK(e, e);

    m_iNoteCounter = 0;
    m_bHasLastNote = false;
    m_bInitialized = false;
    return m_strikerController.reset();
}

Error_t ArmController::resetArms() {
    Error_t e = kNoError;
    for (auto* pArm: m_pArms) {
        auto err = pArm->reset();
        if (err != kNoError) e = err;
    }
    return e;
}

Error_t ArmController::start() {
    if (!m_bInitialized) return kNotInitializedError;

    m_bRunning = true;
    m_pStatusQueryThread = std::make_unique<std::thread>([this] { statusQueryHandler(); });

    for (auto* pArm: m_pArms)
        pArm->start();

    return m_strikerController.start();
}

Error_t ArmController::stop() {
    if (!m_bInitialized) return kNotInitializedError;

    m_bRunning = false;
    if (m_pStatusQueryThread && m_pStatusQueryThread->joinable()) m_pStatusQueryThread->join();
    m_pStatusQueryThread = nullptr;

    m_strikerController.stop();

    for (auto* pArm: m_pArms)
        pArm->stop();

    m_bInitialized = false;
    return kNoError;
}

void ArmController::armMidiCallback(bool bNoteON, int note, int velocity) {
    if (!bNoteON) return;

    std::lock_guard<std::mutex> lk(m_oscMtx);
    auto timeNow = steady_clock::now();

    // Same note buffer: the slider is already at (or arriving at) the right
    // rail position from the previous strike on this note, so send the striker
    // on its own and leave the arm alone.
    bool bSameRecent = m_bHasLastNote
                       && m_iLastInputNote == note
                       && duration_cast<milliseconds>(timeNow - m_lastNoteTime).count() < NOTE_BUFFER_WINDOW;

    if (bSameRecent) {
        Error_t e = m_strikerController.scheduleStrike(m_iLastPlannedNote, m_iLastArmId, velocity,
                                                       timeNow + milliseconds(DLY),
                                                       StrikerController::Mode::Strike);
        if (e != kNoError) LOG_ERROR("Error Scheduling Strike: {}", (int)e);
        m_lastNoteTime = timeNow;
        return;
    }

    Arm::Message_t msg{};
    msg.midiNote = note;
    msg.midiVelocity = velocity;

    m_iNoteCounter++;
    Error_t e = planPath(msg);
    if (e != kNoError) {
        LOG_WARN("No reachable arm for note {}", msg.midiNote);
        return;
    }

    e = m_pArms[msg.arm_id]->addMessageToQueue(msg);
    if (e != kNoError) {
        LOG_ERROR("Failed adding message to queue for arm {}. Error {}", msg.arm_id, (int)e);
        return;
    }

    e = m_strikerController.scheduleStrike(msg.midiNote, msg.arm_id, msg.midiVelocity,
                                           msg.arrivalTime, StrikerController::Mode::Strike);
    if (e != kNoError) LOG_ERROR("Error Scheduling Strike: {}", (int)e);

    m_iLastInputNote = note;
    m_iLastPlannedNote = msg.midiNote;
    m_iLastArmId = msg.arm_id;
    m_lastNoteTime = timeNow;
    m_bHasLastNote = true;
}

Error_t ArmController::planPath(Arm::Message_t& msg) {
    auto timeNow = steady_clock::now();
    auto originalNote = msg.midiNote;
    if (originalNote <= 0) return kInvalidArgsError;

    auto e = kImpossibleError;

    for (int o = 0; o < NUM_OCTAVES_TO_TRY; ++o) {
        int shiftedNote = Util::transpose(originalNote, kOctavesToTry[o] * 12);
        int target = Util::midiToPosition(shiftedNote);

        if (target < 0 || target > SLIDER_LIMIT) {
            e = kImpossibleError;
            continue;
        }

        std::list<Arm::Message_t> candidates;

        for (auto* pArm: m_pArms) {
            int id = pArm->getID();

            // The arm needs the full DLY window to reach the new target. If it
            // is still finishing a previous move this note cannot land on the
            // beat, so this arm is out for this octave.
            if (pArm->getMostRecentArrivalTime() > timeNow) {
                e = kAlreadyThereError;
                continue;
            }

            int mostRecentTarget = pArm->getMostRecentTarget();
            int dir = (target == mostRecentTarget) ? 0 : ((target > mostRecentTarget) ? 1 : -1);

            if (checkInterference(id, target, dir) != 0) {
                e = kInterferenceError;
                continue;
            }

            Arm::Message_t m = {
                    .arm_id = id,
                    .target = target,
                    .midiNote = shiftedNote,
                    .midiVelocity = msg.midiVelocity,
                    .time_ms = DLY,
                    .arrivalTime = timeNow + milliseconds(DLY),
                    .msgTime = timeNow
            };

            // Only fills in the trapezoid we model locally for position
            // feedback - it never rejects a candidate.
            pArm->computeMotionParam(m);
            candidates.push_back(m);
        }

        if (candidates.empty()) {
            LOG_DEBUG("{} octave not possible for note {} at position {}. Reason code: {}",
                      kOctavesToTry[o], originalNote, target, (int)e);
            continue;
        }

        // Tiebreaker: the arm that has to travel the least from its most
        // recent target.
        candidates.sort([this](const Arm::Message_t& m1, const Arm::Message_t& m2) {
            return std::abs(m1.target - m_pArms[m1.arm_id]->getMostRecentTarget())
                   < std::abs(m2.target - m_pArms[m2.arm_id]->getMostRecentTarget());
        });

        msg = candidates.front();
        return kNoError;
    }

    msg.midiNote = originalNote;    // Restore the original message
    return e;
}

int ArmController::checkInterference(int armId, int target, int direction) {
    if (armId < 0 || armId >= NUM_ARMS) return 0;

    if (direction > 0) {
        int r_idx = armId + 1;
        if (r_idx >= NUM_ARMS) return 0;
        if (target >= m_pArms[r_idx]->getLeftBoundary()) return 1;
    } else if (direction < 0) {
        int l_idx = armId - 1;
        if (l_idx < 0) return 0;
        if (target <= m_pArms[l_idx]->getRightBoundary()) return -1;
    }

    return 0;
}

void ArmController::armCallback(int armId, int position, float acceleration, float v_max) {
    if (armId < 0 || armId >= NUM_ARMS) {
        LOG_WARN("Raw arm command for unknown arm {}", armId);
        return;
    }

    Arm::Message_t msg{};
    msg.arm_id = armId;
    msg.target = std::max(0, std::min(SLIDER_LIMIT, position));
    msg.acceleration = acceleration;
    msg.v_max = v_max;
    msg.time_ms = DLY;
    msg.msgTime = steady_clock::now();
    msg.arrivalTime = msg.msgTime + milliseconds(DLY);

    m_pArms[armId]->move(msg);
}

void ArmController::armServoCallback(const char* sCmd) {
    auto cmd = ArmCommandUtil::getCommand(sCmd);

    Error_t e = kNoError;
    switch (cmd) {
        case ArmCommand::Home:
            e = home();
            m_iNoteCounter = 0;
            m_bHasLastNote = false;
            break;
        case ArmCommand::Reset:
            e = restart();
            m_iNoteCounter = 0;
            m_bHasLastNote = false;
            break;
        case ArmCommand::On:
        case ArmCommand::Off:
        case ArmCommand::AlarmClear:
            // The MCU owns servo enable and fault handling on Shaia; there is
            // no host side equivalent. Kept so choreo era clients still parse.
            LOG_WARN("'{}' is not supported on Shaia - the MCU manages the drives", sCmd);
            break;
        default:
            LOG_WARN("Unknown Servo Command : {}", sCmd);
            e = kUnknownCaseError;
            break;
    }

    if (e != kNoError) LOG_ERROR("Error Code: {}", (int)e);
}

void ArmController::armHomeCallback() {
    Error_t e = home();
    if (e != kNoError) LOG_ERROR("Home failed. Error Code: {}", (int)e);
}

void ArmController::armResetCallback() {
    Error_t e = restart();
    if (e != kNoError) LOG_ERROR("Restart failed. Error Code: {}", (int)e);
}

void ArmController::strikerChoreoCallback(uint8_t idCode, int target, int time_ms) {
    std::lock_guard<std::mutex> lk(m_oscMtx);
    Error_t e = m_strikerController.scheduleStrike(idCode, target, time_ms);
    if (e != kNoError) LOG_ERROR("Error Scheduling Strike: {}", (int)e);
}

void ArmController::strikerMidiCallback(char mode, uint8_t idCode, uint8_t midiVelocity, uint8_t channel) {
    (void)channel;
    std::lock_guard<std::mutex> lk(m_oscMtx);
    Error_t e = m_strikerController.strike(idCode, midiVelocity, StrikerController::getMode(mode));
    if (e != kNoError) LOG_ERROR("Error striking. Error Code: {}", (int)e);
}

void ArmController::statusQueryHandler() {
    const int interval_ms = 1;
    tp timeNow = steady_clock::now();
    while (m_bRunning) {
        updateArmPositionsToMaster(interval_ms);
        timeNow += milliseconds(interval_ms);
        std::this_thread::sleep_until(timeNow);
    }
}

Error_t ArmController::updateArmPositionsToMaster(int interval_ms) {
    std::array<int, NUM_ARMS> position{};
    bool bDidChange = false;

    for (auto* pArm: m_pArms) {
        if (interval_ms > 0)
            bDidChange = pArm->updatePositionFromTrajectory(interval_ms) || bDidChange;
        position[pArm->getID()] = pArm->getPosition();
    }

    if (interval_ms <= 0) bDidChange = true;

    if (m_positionCallback && bDidChange) m_positionCallback(position);

    return kNoError;
}

Error_t ArmController::home() {
    LOG_INFO("Homing...");
    Error_t e = m_transport.sendHome();
    ERROR_CHECK(e, e);

    e = resetArms();
    ERROR_CHECK(e, e);

    if (m_statusCallback) m_statusCallback(Status_t::HomingComplete);
    updateArmPositionsToMaster();
    return kNoError;
}

Error_t ArmController::restart() {
    LOG_INFO("Restarting MCU controller...");
    Error_t e = m_strikerController.restart();
    ERROR_CHECK(e, e);

    for (auto* pArm: m_pArms)
        pArm->resetBuffer();

    return kNoError;
}
