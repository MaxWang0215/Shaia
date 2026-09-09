//
// Dynamixel protocol 2.0 bus for Shaia's head.
//
// The choreo build talked protocol 1.0 to an MX mouth/eyebrow pair while the
// four big axes ran on Copley drives. Shaia has no Copley: all five axes are
// protocol 2.0 X-series servos on one FTDI port, so this is a direct port of
// Shaia/utils/Dynamixelutils.py's register map.
//

#ifndef SHAIACONTROLLER_DXLCONTROLLER_H
#define SHAIACONTROLLER_DXLCONTROLLER_H

#include <iostream>
#include <string>
#include <mutex>
#include <cmath>
#include <algorithm>

#include "dynamixel_sdk.h"
#include "Def.h"
#include "ErrorDef.h"
#include "Logger.h"

// Control table (X-series, protocol 2.0)
#define ADDR_OPERATING_MODE     11
#define ADDR_TORQUE_ENABLE      64
#define ADDR_POSITION_P_GAIN    84
#define ADDR_GOAL_CURRENT       102
#define ADDR_PROFILE_VELOCITY   112
#define ADDR_GOAL_POSITION      116
#define ADDR_PRESENT_POSITION   132

#define TORQUE_ENABLE   1
#define TORQUE_DISABLE  0

// Operating modes
#define OP_MODE_POSITION                3
#define OP_MODE_EXTENDED_POSITION       4
#define OP_MODE_CURRENT_BASED_POSITION  5

#define DXL_TICKS_PER_REV       4095
#define DXL_MAX_PROFILE_VELOCITY 2047

#ifndef COMM_SUCCESS
#define COMM_SUCCESS 0
#endif

class DxlController {
public:
    // Singleton: every head axis shares one half duplex bus.
    DxlController(const DxlController&) = delete;
    DxlController(DxlController&&) = delete;
    DxlController& operator=(const DxlController&) = delete;
    DxlController& operator=(DxlController&&) = delete;

    static DxlController& getInstance() {
        static DxlController instance;
        return instance;
    }

    static int degToTick(float deg) {
        return (int)std::lround(deg * (float)DXL_TICKS_PER_REV / 360.f);
    }

    static float tickToDeg(int tick) {
        return (float)tick * 360.f / (float)DXL_TICKS_PER_REV;
    }

    Error_t init(const std::string& port, int baudrate = DXL_BAUDRATE) {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (m_bInitialized) return kNoError;

#ifndef SIMULATE
        m_pPortHandler = dynamixel::PortHandler::getPortHandler(port.c_str());
        if (!m_pPortHandler) return kFileOpenError;

        if (!m_pPortHandler->openPort()) {
            LOG_ERROR("Failed to open dynamixel port {}", port);
            return kFileOpenError;
        }

        if (!m_pPortHandler->setBaudRate(baudrate)) {
            LOG_ERROR("Failed to set baudrate {} on {}", baudrate, port);
            return kSetValueError;
        }
#endif
        LOG_INFO("Head dynamixel bus online ({} @ {})", port, baudrate);
        m_bInitialized = true;
        return kNoError;
    }

    Error_t reset() {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (!m_bInitialized) return kNoError;
#ifndef SIMULATE
        if (m_pPortHandler) m_pPortHandler->closePort();
#endif
        m_bInitialized = false;
        return kNoError;
    }

    [[nodiscard]] bool isInitialized() const { return m_bInitialized; }

    Error_t servoOn(int id) { return write1(id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE); }
    Error_t servoOff(int id) { return write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE); }

    // Changing the operating mode requires torque off; restore it afterwards.
    Error_t setOperatingMode(int id, int mode) {
        Error_t e = servoOff(id);
        ERROR_CHECK(e, e);
        e = write1(id, ADDR_OPERATING_MODE, (uint8_t)mode);
        ERROR_CHECK(e, e);
        return servoOn(id);
    }

    // velocity is normalized [0, 1] the way danceModes.json expresses it.
    Error_t setProfileVelocity(int id, float velocity) {
        velocity = std::max(0.f, std::min(1.f, velocity));
        auto scaled = (uint32_t)std::lround(velocity * (float)DXL_MAX_PROFILE_VELOCITY);
        return write4(id, ADDR_PROFILE_VELOCITY, scaled);
    }

    Error_t setGoalPositionTicks(int id, int ticks) {
        return write4(id, ADDR_GOAL_POSITION, (uint32_t)ticks);
    }

    Error_t setGoalPositionDeg(int id, float deg) {
        return setGoalPositionTicks(id, degToTick(deg));
    }

    Error_t setPGain(int id, int p) { return write2(id, ADDR_POSITION_P_GAIN, (uint16_t)p); }
    Error_t setGoalCurrent(int id, int current) { return write2(id, ADDR_GOAL_CURRENT, (uint16_t)current); }

    Error_t getPresentPositionTicks(int id, int& ticks) {
        ticks = 0;
#ifndef SIMULATE
        if (!m_bInitialized) return kNotInitializedError;
        std::lock_guard<std::mutex> lk(m_mtx);
        uint32_t value = 0;
        uint8_t dxlError = 0;
        int result = m_pPacketHandler->read4ByteTxRx(m_pPortHandler, (uint8_t)id, ADDR_PRESENT_POSITION, &value, &dxlError);
        if (!checkError(id, result, dxlError)) return kReadError;
        ticks = (int)value;
#endif
        return kNoError;
    }

private:
    std::atomic<bool> m_bInitialized = false;
    std::mutex m_mtx;
    dynamixel::PortHandler* m_pPortHandler = nullptr;
    dynamixel::PacketHandler* m_pPacketHandler;

    DxlController() : m_pPacketHandler(dynamixel::PacketHandler::getPacketHandler(DXL_PROTOCOL_VERSION)) {}

    ~DxlController() { reset(); }

    Error_t write1(int id, uint16_t addr, uint8_t value) {
#ifndef SIMULATE
        if (!m_bInitialized) return kNotInitializedError;
        std::lock_guard<std::mutex> lk(m_mtx);
        uint8_t dxlError = 0;
        int result = m_pPacketHandler->write1ByteTxRx(m_pPortHandler, (uint8_t)id, addr, value, &dxlError);
        if (!checkError(id, result, dxlError)) return kWriteError;
#else
        (void)id; (void)addr; (void)value;
#endif
        return kNoError;
    }

    Error_t write2(int id, uint16_t addr, uint16_t value) {
#ifndef SIMULATE
        if (!m_bInitialized) return kNotInitializedError;
        std::lock_guard<std::mutex> lk(m_mtx);
        uint8_t dxlError = 0;
        int result = m_pPacketHandler->write2ByteTxRx(m_pPortHandler, (uint8_t)id, addr, value, &dxlError);
        if (!checkError(id, result, dxlError)) return kWriteError;
#else
        (void)id; (void)addr; (void)value;
#endif
        return kNoError;
    }

    Error_t write4(int id, uint16_t addr, uint32_t value) {
#ifndef SIMULATE
        if (!m_bInitialized) return kNotInitializedError;
        std::lock_guard<std::mutex> lk(m_mtx);
        uint8_t dxlError = 0;
        int result = m_pPacketHandler->write4ByteTxRx(m_pPortHandler, (uint8_t)id, addr, value, &dxlError);
        if (!checkError(id, result, dxlError)) return kWriteError;
#else
        (void)id; (void)addr; (void)value;
#endif
        return kNoError;
    }

    bool checkError(int id, int result, uint8_t err) {
#ifndef SIMULATE
        if (result != COMM_SUCCESS) {
            LOG_ERROR("dxl {}: {}", id, m_pPacketHandler->getTxRxResult(result));
            return false;
        }
        if (err != 0) {
            LOG_ERROR("dxl {}: {}", id, m_pPacketHandler->getRxPacketError(err));
            return false;
        }
#else
        (void)id; (void)result; (void)err;
#endif
        return true;
    }
};

#endif //SHAIACONTROLLER_DXLCONTROLLER_H
