//
// Slider path planning + striker scheduling.
//
// Ported from ShimonController-choreo's ArmController, with the planner
// rewritten to match Shaia's Python runtime (shaia_targetUpdated/arms.py):
//   * every move takes the full DLY window, so short and long moves share one
//     gentle acceleration profile,
//   * an arm that is still travelling (mostRecentArrival > now) is skipped
//     rather than re-timed,
//   * candidates are ranked by travel distance, not by acceleration,
//   * a repeat of the same input note inside NOTE_BUFFER_WINDOW reuses the
//     previous arm and skips the slider command entirely.
//

#ifndef SHAIACONTROLLER_ARMCONTROLLER_H
#define SHAIACONTROLLER_ARMCONTROLLER_H

#include <iostream>
#include <array>
#include <atomic>
#include <chrono>
#include <climits>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <thread>

#include "Arm.h"
#include "McuTransport.h"
#include "StrikerController.h"
#include "OscListener.h"
#include "SliderModel.h"
#include "ArmCommands.h"
#include "CommandManager.h"

#include "Def.h"
#include "ErrorDef.h"
#include "StatusDef.h"
#include "Logger.h"

using namespace std::chrono;

class ArmController {
public:
    explicit ArmController(OscListener& oscListener, tp programStartTime = steady_clock::now());
    ~ArmController();

    Error_t init();
    Error_t reset();
    Error_t resetArms();
    Error_t start();
    Error_t stop();

    Error_t home();
    Error_t restart();

    void setStatusCallback(std::function<void(Status_t)> callback_fn) { m_statusCallback = std::move(callback_fn); }
    void setPositionCallback(std::function<void(std::array<int, NUM_ARMS>)> callback_fn) {
        m_positionCallback = std::move(callback_fn);
    }

private:
    bool m_bInitialized = false;
    volatile std::atomic<bool> m_bRunning = false;
    std::array<Arm*, NUM_ARMS> m_pArms{};

    long long m_iNoteCounter = 0;

    OscListener& m_oscListener;
    McuTransport& m_transport;
    StrikerController m_strikerController;

    // Same note buffer (arms.py:207). A rapid repeat of the same incoming note
    // reuses the previous arm so the busy-arm check cannot bounce it across the
    // rail, and we skip a redundant slider command.
    int m_iLastInputNote = -1;
    int m_iLastPlannedNote = -1;
    int m_iLastArmId = -1;
    tp m_lastNoteTime{};
    bool m_bHasLastNote = false;

    std::function<void(Status_t)> m_statusCallback = nullptr;
    std::function<void(std::array<int, NUM_ARMS>)> m_positionCallback = nullptr;

    std::unique_ptr<std::thread> m_pStatusQueryThread = nullptr;

    std::mutex m_oscMtx;

    tp kProgramStartTime;

    void armMidiCallback(bool bNoteON, int note, int velocity);
    void armServoCallback(const char* cmd);
    void armCallback(int armId, int position, float acceleration, float v_max);
    void armHomeCallback();
    void armResetCallback();

    void strikerChoreoCallback(uint8_t idCode, int target, int time_ms);
    void strikerMidiCallback(char mode, uint8_t idCode, uint8_t midiVelocity, uint8_t channel);

    Error_t updateArmPositionsToMaster(int interval_ms = -1);
    void statusQueryHandler();

    /*
     * Check for interference for a given arm.
     * Returns 0 if the position is reachable, 1 if the right neighbour is in
     * the way and -1 if the left neighbour is.
     */
    int checkInterference(int armId, int position, int direction);

    Error_t planPath(Arm::Message_t& msg);
};

#endif //SHAIACONTROLLER_ARMCONTROLLER_H
