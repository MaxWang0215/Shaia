//
// Schedules striker frames onto the shared MCU link.
//
// Shaia has 4 strikers, numbered after the 2 sliders on the MCU's CAN bus:
//   arm 0 -> { black: 3, white: 4 },  arm 1 -> { black: 5, white: 6 }
// The firmware reads the id byte as a bitmask, so a striker id n goes out as
// 1 << (n - 1) (see shaia_targetUpdated/transport.py).
//

#ifndef SHAIACONTROLLER_STRIKERCONTROLLER_H
#define SHAIACONTROLLER_STRIKERCONTROLLER_H

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <condition_variable>

#include "Def.h"
#include "Util.h"
#include "CommandManager.h"
#include "McuTransport.h"
#include "ErrorDef.h"
#include "Logger.h"

using namespace std::chrono;

class StrikerController {
public:
    enum class Mode {
        None,
        Strike,
        Choreo,
        Tremolo,
        TremoloStop,
        Restart
    };

    struct StrikerMessage_t {
        uint8_t strikerId = 0;      // already a bitmask
        uint8_t midiVelocity = 0;
        int position = 0;
        int time_ms = 0;
        Mode mode = Mode::None;
        tp strikeTime{};
    };

    explicit StrikerController(tp programStartTime)
            : m_transport(McuTransport::getInstance())
            , m_cmdManager(m_cv)
            , kProgramStartTime(programStartTime) {}

    ~StrikerController() { reset(); }

    Error_t init() {
        if (!m_transport.isInitialized()) return kNotInitializedError;
        m_bInitialized = true;

        // Deliberately *not* restart(): the MCU has just booted, homed and
        // enabled all six drives by the time we open the port. Restarting tears
        // the power stage back down and reconfigures every node over blocking
        // SDO with the RPDO stream stopped, which is long enough for the drives
        // to trip a 0x8250 RPDO timeout - and the firmware latches that error,
        // so the sliders never rejoin the cyclic stream. Restart stays
        // available explicitly via /arm_reset and the RESTART servo command.
        setMode(Mode::Strike);
        return resetBuffer();
    }

    Error_t start() {
        if (!m_bInitialized) return kNotInitializedError;
        m_bRunning = true;
        m_pStrikerThread = std::make_unique<std::thread>([this] { strikerHandler(); });
        return kNoError;
    }

    void stop() {
        m_bRunning = false;
        m_cv.notify_all();
        if (m_pStrikerThread && m_pStrikerThread->joinable()) m_pStrikerThread->join();
        m_pStrikerThread = nullptr;
    }

    Error_t reset(bool stopThread = true) {
        if (!m_bInitialized) return kNoError;
        LOG_TRACE("Resetting Strikers");
        if (stopThread) stop();
        m_bInitialized = false;
        return kNoError;
    }

    Error_t restart() {
        LOG_DEBUG("Restarting Strikers");
        Error_t e = m_transport.sendRestart();
        ERROR_CHECK(e, e);

        setMode(Mode::Strike);
        return resetBuffer();
    }

    void setMode(Mode mode) {
        if (mode != Mode::None) m_currentMode = mode;
    }

    // Striker id (3..6) for the arm/note pair, matching arms.py:112.
    static int getStrikerId(int note, int armId) {
        return (NUM_ARMS + 1) + (armId * 2) + (Util::isWhiteKey(note) ? 1 : 0);
    }

    static uint8_t getStrikerBitmask(int strikerId) {
        if (strikerId < 1) return 0;
        return (uint8_t)((1u << (strikerId - 1)) & 0xFF);
    }

    Error_t scheduleStrike(int note, int armId, int midiVelocity, tp strikeTime, Mode mode = Mode::None) {
        if (!m_bInitialized) return kNotInitializedError;

        std::lock_guard<std::mutex> lk(m_mtx);
        setMode(mode);

        auto uiVelocity = (uint8_t)std::max(0, std::min(127, midiVelocity));
        uint8_t bitmask = getStrikerBitmask(getStrikerId(note, armId));

        StrikerMessage_t msg = { .strikerId = bitmask,
                                 .midiVelocity = uiVelocity,
                                 .mode = m_currentMode,
                                 .strikeTime = strikeTime };

        if (!m_cmdManager.push(Port::Arm::Striker, msg)) return kBufferWriteError;
        return kNoError;
    }

    // Choreo: move the given strikers to `target` degrees over `time_ms`.
    Error_t scheduleStrike(uint8_t idCode, int target, int time_ms) {
        if (!m_bInitialized) return kNotInitializedError;

        std::lock_guard<std::mutex> lk(m_mtx);
        setMode(Mode::Choreo);

        StrikerMessage_t msg = { .strikerId = idCode,
                                 .position = target,
                                 .time_ms = time_ms,
                                 .mode = m_currentMode,
                                 .strikeTime = steady_clock::now() + milliseconds(DLY) };

        if (!m_cmdManager.push(Port::Arm::Striker, msg)) return kBufferWriteError;
        return kNoError;
    }

    Error_t strike(int note, int armId, int midiVelocity, Mode mode = Mode::None) {
        if (!m_bInitialized) return kNotInitializedError;
        midiVelocity = std::max(0, std::min(127, midiVelocity));
        return strike(getStrikerBitmask(getStrikerId(note, armId)), midiVelocity, mode);
    }

    Error_t strike(uint8_t bitmask, int midiVelocity, Mode mode = Mode::None) {
        if (mode != Mode::None) setMode(mode);
        return m_transport.sendStriker(bitmask, (uint8_t)midiVelocity, getMode(m_currentMode));
    }

    Error_t choreo(uint8_t bitmask, int position, int time_ms) {
        return m_transport.sendChoreo(bitmask, (uint16_t)position, (uint16_t)time_ms);
    }

    static char getMode(Mode mode) {
        switch (mode) {
            case Mode::Strike:      return 's';
            case Mode::Tremolo:     return 't';
            case Mode::Choreo:      return 'c';
            case Mode::TremoloStop: return 'p';
            case Mode::Restart:     return 'r';
            default:                return 'u';   // Unknown
        }
    }

    static Mode getMode(char mode) {
        switch (mode) {
            case 's': return Mode::Strike;
            case 't': return Mode::Tremolo;
            case 'c': return Mode::Choreo;
            case 'p': return Mode::TremoloStop;
            case 'r': return Mode::Restart;
            default:  return Mode::None;
        }
    }

    Error_t resetBuffer() {
        if (m_cmdManager.reset(Port::Arm::Striker)) return kNoError;
        return kBufferWriteError;
    }

private:
    McuTransport& m_transport;
    Mode m_currentMode = Mode::Strike;
    bool m_bInitialized = false;

    std::mutex m_mtx;
    std::condition_variable m_cv;
    volatile std::atomic<bool> m_bRunning = false;

    std::unique_ptr<std::thread> m_pStrikerThread = nullptr;

    CommandManager<StrikerMessage_t, Port::Arm> m_cmdManager;
    tp kProgramStartTime;

    void strikerHandler() {
        while (m_bRunning) {
            StrikerMessage_t msg{};
            bool success;
            {
                std::unique_lock<std::mutex> lk(m_mtx);
                m_cv.wait(lk, [this] { return !m_bRunning || !m_cmdManager.isEmpty(); });
                if (!m_bRunning) break;
                success = m_cmdManager.pop(Port::Arm::Striker, msg);
            }

            if (!success) {
                LOG_ERROR("Cannot get Striker msg. Error Code: {}", (int)kBufferReadError);
                continue;
            }

            Error_t e;
            if (msg.mode == Mode::Choreo) {
                e = choreo(msg.strikerId, msg.position, msg.time_ms);
            } else {
                // Fire STRIKE_TIME early so the mechanical impact lands on strikeTime.
                std::this_thread::sleep_until(msg.strikeTime - milliseconds(STRIKE_TIME));
                e = strike(msg.strikerId, msg.midiVelocity, msg.mode);
            }
            if (e != kNoError) LOG_ERROR("Error striking. Error Code: {}", (int)e);
        }
    }
};

#endif //SHAIACONTROLLER_STRIKERCONTROLLER_H
