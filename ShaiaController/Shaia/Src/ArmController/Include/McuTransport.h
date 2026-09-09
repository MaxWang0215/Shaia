//
// Single serial link to the OpenCR board running Strikers/Strikers.ino.
// It carries slider moves, striker strikes and board level commands, so every
// frame goes through one mutex protected SerialDevice.
//
// Replaces the choreo build's IAIController (Modbus over RS-485) +
// StrikerHandler pair - on Shaia both sliders and strikers are EPOS4 nodes
// hanging off the MCU's CAN bus, and the host only speaks to the MCU.
//
// Wire format (matches shaia_targetUpdated/transport.py). Every frame is
// newline terminated; the firmware delimits on '\n'.
//
//   slider   'm' sliderId  tickHi tickLo durHi durLo '\n'   (7 B)
//   striker  mode bitmask  midiVelocity                '\n' (4 B)
//   choreo   'c' bitmask   posHi  posLo  msHi  msLo   '\n'  (7 B)
//   home     'h' 0 0                                   '\n' (4 B)
//   restart  'r' 0 0                                   '\n' (4 B)
//
// sliderId is 1 indexed and is the drive's CAN node id. It is not armId + 1 -
// the two slider drives are wired the other way round from the arm order, so
// the mapping goes through kArmToSliderId (Def.h): arm 0 -> 2, arm 1 -> 1.
// Striker frames carry a bitmask, not an index: bitmask = 1 << (strikerId - 1),
// striker ids 3..6.
//

#ifndef SHAIACONTROLLER_MCUTRANSPORT_H
#define SHAIACONTROLLER_MCUTRANSPORT_H

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

#include "SerialDevice.h"
#include "Def.h"
#include "ErrorDef.h"
#include "Logger.h"

class McuTransport {
public:
    // Singleton: the sliders and the strikers share one physical port.
    McuTransport(const McuTransport&) = delete;
    McuTransport& operator=(const McuTransport&) = delete;

    static McuTransport& getInstance() {
        static McuTransport instance;
        return instance;
    }

    ~McuTransport() { reset(); }

    Error_t init(const std::string& device, int baudrate) {
        if (m_bInitialized) return kNoError;

        LOG_TRACE("Opening MCU link on {} @ {}", device, baudrate);
        Error_t e = m_serial.openPort(device);
        ERROR_CHECK(e, e);

        e = m_serial.init(baudrate);
        ERROR_CHECK(e, e);

        m_bRunning = true;
        m_pRecvThread = std::make_unique<std::thread>([this] { recvHandler(); });

        m_bInitialized = true;
        return kNoError;
    }

    Error_t reset() {
        m_bRunning = false;
        if (m_pRecvThread && m_pRecvThread->joinable()) m_pRecvThread->join();
        m_pRecvThread = nullptr;
        m_serial.reset();
        m_bInitialized = false;
        return kNoError;
    }

    [[nodiscard]] bool isInitialized() const { return m_bInitialized; }

    // armId is 0 indexed; positionTicks is the arm's own home relative ticks.
    Error_t sendSlider(int armId, int positionTicks, int duration_ms) {
        if (!m_bInitialized) return kNotInitializedError;
        if (armId < 0 || armId >= NUM_ARMS) return kInvalidArgsError;

        positionTicks = std::max(0, std::min(0xFFFF, positionTicks));
        duration_ms = std::max(0, std::min(0xFFFF, duration_ms));

        uint8_t buf[7] = {
                'm',
                (uint8_t)kArmToSliderId[armId],
                (uint8_t)((positionTicks >> 8) & 0xFF),
                (uint8_t)(positionTicks & 0xFF),
                (uint8_t)((duration_ms >> 8) & 0xFF),
                (uint8_t)(duration_ms & 0xFF),
                '\n'
        };

        LOG_DEBUG("arm {} (slider {}) -> {} ticks in {} ms",
                  armId, kArmToSliderId[armId], positionTicks, duration_ms);
        std::lock_guard<std::mutex> lk(m_mtx);
        return m_serial.write(buf, sizeof(buf));
    }

    Error_t sendStriker(uint8_t strikerBitmask, uint8_t midiVelocity, char mode) {
        if (!m_bInitialized) return kNotInitializedError;

        uint8_t buf[4] = { (uint8_t)mode, strikerBitmask, midiVelocity, '\n' };

        LOG_DEBUG("striker mask: 0x{:02x}, velocity: {}, mode: {}", strikerBitmask, midiVelocity, mode);
        std::lock_guard<std::mutex> lk(m_mtx);
        return m_serial.write(buf, sizeof(buf));
    }

    Error_t sendChoreo(uint8_t strikerBitmask, uint16_t position, uint16_t time_ms) {
        if (!m_bInitialized) return kNotInitializedError;

        uint8_t buf[7] = {
                'c',
                strikerBitmask,
                (uint8_t)((position >> 8) & 0xFF),
                (uint8_t)(position & 0xFF),
                (uint8_t)((time_ms >> 8) & 0xFF),
                (uint8_t)(time_ms & 0xFF),
                '\n'
        };

        LOG_DEBUG("choreo mask: 0x{:02x}, position: {}, time: {} ms", strikerBitmask, position, time_ms);
        std::lock_guard<std::mutex> lk(m_mtx);
        return m_serial.write(buf, sizeof(buf));
    }

    // Runs the MCU side homing routine for every slider.
    Error_t sendHome() {
        if (!m_bInitialized) return kNotInitializedError;
        uint8_t buf[4] = { 'h', 0, 0, '\n' };
        LOG_INFO("Homing sliders");
        std::lock_guard<std::mutex> lk(m_mtx);
        return m_serial.write(buf, sizeof(buf));
    }

    // Re-inits the MCU's controller (sliders + strikers).
    Error_t sendRestart() {
        if (!m_bInitialized) return kNotInitializedError;
        uint8_t buf[4] = { 'r', 0, 0, '\n' };
        LOG_INFO("Restarting MCU controller");
        std::lock_guard<std::mutex> lk(m_mtx);
        return m_serial.write(buf, sizeof(buf));
    }

private:
    SerialDevice m_serial;
    std::mutex m_mtx;
    std::atomic<bool> m_bRunning = false;
    std::atomic<bool> m_bInitialized = false;
    std::unique_ptr<std::thread> m_pRecvThread = nullptr;

    McuTransport() = default;

    // The firmware logs over the same link. Mirror those lines into our logger.
    void recvHandler() {
        m_serial.flush();
        while (m_bRunning) {
            char msg[128] = {};
            int n = m_serial.readline(msg, '\n', sizeof(msg) - 1, 10);
#ifndef SIMULATE
            if (n > 0) handleLog(msg);
#else
            (void)n;
#endif
            if (!m_bRunning) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    static void handleLog(const std::string& msg) {
        if (msg.rfind("INFO", 0) == 0) {
            LOG_INFO("[mcu] {}", msg.substr(6));
        } else if (msg.rfind("ERROR", 0) == 0) {
            LOG_ERROR("[mcu] {}", msg.substr(7));
        } else if (msg.rfind("TRACE", 0) == 0) {
            LOG_TRACE("[mcu] {}", msg.substr(7));
        } else if (msg.rfind("WARN", 0) == 0) {
            LOG_WARN("[mcu] {}", msg.substr(6));
        } else {
            LOG_WARN("[mcu] {}", msg);
        }
    }
};

#endif //SHAIACONTROLLER_MCUTRANSPORT_H
