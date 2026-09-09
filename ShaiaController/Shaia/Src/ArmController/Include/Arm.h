//
// One slider arm: its own command queue, worker thread and collision
// boundaries. Adapted from ShimonController-choreo's Arm.h - the IAI/Modbus
// controller is replaced by a slider frame on the shared MCU link, and the
// motion profile is owned by the MCU (we only mirror it locally so we can
// report position back to the master).
//

#ifndef SHAIACONTROLLER_ARM_H
#define SHAIACONTROLLER_ARM_H

#include <iostream>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "CommandManager.h"
#include "McuTransport.h"
#include "SliderModel.h"
#include "Def.h"
#include "Util.h"
#include "ErrorDef.h"
#include "Logger.h"

using namespace std::chrono;

class Arm {
public:
    typedef steady_clock::time_point _tp;

    struct Message_t {
        int arm_id{};
        int target{};           // mm along the shared rail
        float acceleration{};   // g   - local trajectory model only
        float v_max{};          // mm/s - local trajectory model only
        int midiNote{};
        int midiVelocity{};
        int time_ms{DLY};       // move duration handed to the MCU
        _tp arrivalTime;
        _tp msgTime;

        void pprint() const {
            std::cout << "Id: " << arm_id
                      << "\t target: " << target
                      << "\t acc: " << acceleration
                      << "\t vmax: " << v_max
                      << "\t midi vel: " << midiVelocity
                      << "\t time (ms): " << time_ms
                      << std::endl;
        }
    };

    struct Boundary {
        int left = 0;
        int right = 0;
    };

    Arm(int id, int iHomePosition, McuTransport& transport)
            : m_id(id)
            , kHomePosition(iHomePosition)
            , m_transport(transport)
            , m_cmdManager(m_cmdCv) {
        reset();
    }

    Error_t reset() {
        m_bMoving = false;
        m_iLastReportedPos = -1;
        _setPosition(kHomePosition);

        Message_t msg{};
        msg.arm_id = m_id;
        msg.target = kHomePosition;
        msg.midiNote = MIN_NOTE;
        msg.time_ms = DLY;
        msg.msgTime = steady_clock::now() - milliseconds(DLY);
        msg.arrivalTime = steady_clock::now();
        setMessage(msg);

        m_mostRecentMsg = msg;
        return kNoError;
    }

    Error_t start() {
        m_bRunning = true;
        kProgramStartTime = steady_clock::now();
        m_pThread = std::make_unique<std::thread>([this] { threadHandler(); });
        return kNoError;
    }

    void stop() {
        m_bRunning = false;
        m_bMoving = false;
        m_cmdCv.notify_all();
        if (m_pThread && m_pThread->joinable()) m_pThread->join();
        m_pThread = nullptr;
    }

    Error_t addMessageToQueue(const Message_t& msg) {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (!m_cmdManager.push(Port::Arm::Slider, msg)) {
            LOG_ERROR("Unable to push msg for arm {}", m_id);
            return kSetValueError;
        }
        m_mostRecentMsg = msg;
        return kNoError;
    }

    Error_t setMessage(const Message_t& msg) {
        m_lastMsg = m_msg;
        m_msg = msg;
        _updateTargetBoundaries();
        _updateTrajectory();
        return kNoError;
    }

    Error_t move(const Message_t& msg) {
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            setMessage(msg);
        }
        return move();
    }

    Error_t move() {
        int ticks = Util::railToArmTicks(m_id, m_msg.target);
        Error_t e = m_transport.sendSlider(m_id, ticks, m_msg.time_ms);
        if (e != kNoError) {
            LOG_ERROR("Slider send error for arm {}: {}", m_id, e);
            return e;
        }

        LOG_INFO("Note: {} \t Arm: {} \t time: {} ms \t pos: {} \t target: {} mm ({} ticks) \t acc: {} \t v_max: {} \t msgTime: {} \t arrTime: {}",
                 m_msg.midiNote,
                 m_id,
                 m_msg.time_ms,
                 m_lastMsg.target,
                 m_msg.target,
                 ticks,
                 m_msg.acceleration,
                 (int)std::round(m_msg.v_max),
                 duration_cast<milliseconds>(m_msg.msgTime - kProgramStartTime).count(),
                 duration_cast<milliseconds>(m_msg.arrivalTime - kProgramStartTime).count());

        m_iTrajIdx = 0;
        m_bMoving = true;
        return kNoError;
    }

    [[nodiscard]] int getID() const { return m_id; }
    [[nodiscard]] int getPosition() const { return m_iPosition; }

    void updatePosition(int iPos) { _setPosition(iPos); }

    // Walks the locally modelled trajectory so the master gets position
    // feedback even though the MCU owns the real motion.
    bool updatePositionFromTrajectory(int interval_ms = 1) {
        std::lock_guard<std::mutex> lk(m_mtx);

        m_iTrajIdx = std::max(m_iTrajIdx + interval_ms, 0);
        if (m_iTrajIdx > m_iEndTimeIdx) {
            m_iTrajIdx = m_iEndTimeIdx;
            m_bMoving = false;
        }

        if (!m_bMoving) return false;

        int pos = m_aiTraj[m_iTrajIdx];
        if (pos != m_iLastReportedPos) {
            updatePosition(pos);
            m_iLastReportedPos = m_iPosition;
            return true;
        }

        return false;
    }

    [[nodiscard]] int getLeftBoundary() const {
        return std::min(m_boundary.left, std::min(m_tBoundary.left, getInstantaneousLeftBoundary()));
    }

    [[nodiscard]] int getRightBoundary() const {
        return std::max(m_boundary.right, std::max(m_tBoundary.right, getInstantaneousRightBoundary()));
    }

    [[nodiscard]] int getInstantaneousLeftBoundary() const {
        return m_iPosition - kBoundaries[m_id][0];
    }

    [[nodiscard]] int getInstantaneousRightBoundary() const {
        return m_iPosition + kBoundaries[m_id][1];
    }

    // State after every queued command has run - not the state right now.
    [[nodiscard]] const _tp& getMostRecentMsgTime() const { return m_mostRecentMsg.msgTime; }
    [[nodiscard]] const _tp& getMostRecentArrivalTime() const { return m_mostRecentMsg.arrivalTime; }
    [[nodiscard]] int getMostRecentTarget() const { return m_mostRecentMsg.target; }

    /*
     * Fills in the trapezoid the MCU will run, for our own position feedback.
     * Every move takes the full msg.time_ms, so peak velocity and acceleration
     * scale with distance rather than distance/time^2 (see arms.py:179).
     */
    Error_t computeMotionParam(Message_t& msg) const {
        auto dPos = std::abs(msg.target - getMostRecentTarget());
        if (dPos == 0) {
            msg.acceleration = 0;
            msg.v_max = 0;
            return kNoError;
        }

        if (msg.time_ms < 1) return kArgLimitError;

        auto fTime = (float)msg.time_ms / 1000.f;            // s
        // Trapezoid with a 10% ramp at each end: d = v * 0.9 * T.
        msg.v_max = (float)dPos / (0.9f * fTime);            // mm/s
        msg.acceleration = (msg.v_max / 1000.f) / (0.1f * fTime) / 9.8f;   // g

        if (msg.v_max > VELOCITY_LIMIT * 1000.f)
            LOG_WARN("Arm {} peak velocity {} mm/s exceeds the {} mm/s limit",
                     m_id, (int)std::round(msg.v_max), (int)(VELOCITY_LIMIT * 1000.f));
        if (msg.acceleration > ACC_LIMIT)
            LOG_WARN("Arm {} acceleration {} g exceeds the {} g limit", m_id, msg.acceleration, ACC_LIMIT);

        return kNoError;
    }

    Error_t resetBuffer() {
        if (m_cmdManager.reset(Port::Arm::Slider)) return kNoError;
        return kBufferWriteError;
    }

private:
    int m_id = -1;
    int m_iPosition = -1;
    int m_iLastReportedPos = -1;
    const int kHomePosition;
    std::atomic<bool> m_bRunning = false;
    std::atomic<bool> m_bMoving = false;
    McuTransport& m_transport;
    Boundary m_boundary{};
    Boundary m_tBoundary{};

    tp kProgramStartTime = steady_clock::now();

    Message_t m_msg, m_lastMsg;
    Message_t m_mostRecentMsg;

    CommandManager<Message_t, Port::Arm> m_cmdManager;

    std::unique_ptr<std::thread> m_pThread = nullptr;

    std::mutex m_mtx;
    std::condition_variable m_cmdCv;

    int m_aiTraj[DLY]{};
    int m_iEndTimeIdx = DLY - 1;
    volatile std::atomic<int> m_iTrajIdx = 0;

    Error_t _updateBoundaries() {
        m_boundary.left = m_iPosition - kBoundaries[m_id][0];
        m_boundary.right = m_iPosition + kBoundaries[m_id][1];
        return kNoError;
    }

    Error_t _updateTargetBoundaries() {
        m_tBoundary.left = m_msg.target - kBoundaries[m_id][0];
        m_tBoundary.right = m_msg.target + kBoundaries[m_id][1];
        return kNoError;
    }

    void _updateTrajectory() {
        int time_ms = std::max(1, std::min(DLY, m_msg.time_ms));
        SliderModel::compute(time_ms, m_lastMsg.target, m_msg.target,
                             m_msg.acceleration, m_msg.v_max, m_aiTraj);
        m_iEndTimeIdx = time_ms - 1;
    }

    Error_t _setPosition(int position) {
        m_iPosition = position;
        return _updateBoundaries();
    }

    void threadHandler() {
        while (m_bRunning) {
            Message_t msg{};
            bool success;
            {
                std::unique_lock<std::mutex> lk(m_mtx);
                m_cmdCv.wait(lk, [this] { return !m_cmdManager.isEmpty() || !m_bRunning; });
                if (!m_bRunning) break;
                success = m_cmdManager.pop(Port::Arm::Slider, msg);
            }

            if (!success) continue;

            std::this_thread::sleep_until(msg.msgTime);
            Error_t e = move(msg);
            if (e != kNoError && e != kAlreadyThereError) LOG_ERROR("Move Error Code: {}", e);
        }
    }
};

#endif //SHAIACONTROLLER_ARM_H
