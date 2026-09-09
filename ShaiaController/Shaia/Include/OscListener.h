//
// UDP OSC listener. Adapted from ShimonController-choreo, with the routes the
// Shaia Python runtime already speaks (/arm_home, /arm_reset, /head,
// /head_stop, /head_look) added alongside the choreo era ones.
//
// Routes:
//   /arm  <cmd>                          servo command string (HOME/RESTART/...)
//   /arm  <note> <velocity>              midi note trigger
//   /arm  <armId> <pos> <acc> <v_max>    raw move
//   /arm_home                            home the sliders
//   /arm_reset                           restart the MCU controller
//   /striker <ids> <position> <time_ms>  choreo
//   /striker <mode> <ids> <vel> <ch>     midi
//   /head-commands <axis> [pos] [vel]    low level head move (normalized pos)
//   /head <gesture> [bpm]                start a looping gesture
//   /head_stop                           stop the current gesture
//   /head_look                           one shot 'look at the audience' pose
//   /shimon <msg>                        system message
//

#ifndef SHAIACONTROLLER_OSCLISTENER_H
#define SHAIACONTROLLER_OSCLISTENER_H

#include <iostream>
#include <cstdint>
#include <cstring>
#include <atomic>
#include <functional>
#include <optional>

#include "osc/OscReceivedElements.h"
#include "osc/OscPacketListener.h"
#include "ip/UdpSocket.h"

#include "Def.h"
#include "Logger.h"

class OscListener : public osc::OscPacketListener {
public:
    explicit OscListener(int port) : m_socket(IpEndpointName(IpEndpointName::ANY_ADDRESS, port), this) {}

    void setSystemMsgCallback(std::function<void(const char*)> callback_fn) {
        m_sysMsgCallback = std::move(callback_fn);
    }

    void setArmMidiCallback(std::function<void(bool, int, int)> callback_fn) {
        m_armMidiCallback = std::move(callback_fn);
    }

    void setArmServoCallback(std::function<void(const char*)> callback_fn) {
        m_armServoCallback = std::move(callback_fn);
    }

    void setArmRawCallback(std::function<void(int, int, float, float)> callback_fn) {
        m_armCallback = std::move(callback_fn);
    }

    void setArmHomeCallback(std::function<void()> callback_fn) {
        m_armHomeCallback = std::move(callback_fn);
    }

    void setArmResetCallback(std::function<void()> callback_fn) {
        m_armResetCallback = std::move(callback_fn);
    }

    void setHeadCallback(std::function<void(const char*, std::optional<float>, std::optional<float>)> callback_fn) {
        m_headCallback = std::move(callback_fn);
    }

    void setHeadGestureCallback(std::function<void(const char*, std::optional<float>)> callback_fn) {
        m_headGestureCallback = std::move(callback_fn);
    }

    void setHeadStopCallback(std::function<void()> callback_fn) {
        m_headStopCallback = std::move(callback_fn);
    }

    void setHeadLookCallback(std::function<void()> callback_fn) {
        m_headLookCallback = std::move(callback_fn);
    }

    void setStrikerRawCallback(std::function<void(char, uint8_t, int, int, int)> callback_fn) {
        m_strikerCallback = std::move(callback_fn);
    }

    void setStrikerChoreoCallback(std::function<void(uint8_t, int, int)> callback_fn) {
        m_strikerChoreoCallback = std::move(callback_fn);
    }

    void setStrikerMidiCallback(std::function<void(char, uint8_t, uint8_t, uint8_t)> callback_fn) {
        m_strikerMidiCallback = std::move(callback_fn);
    }

    void start() {
        m_bRunning = true;
        m_socket.Run();
    }

    void stop() {
        if (!m_bRunning) return;
        m_socket.AsynchronousBreak();
        m_bRunning = false;
    }

    [[nodiscard]] unsigned long getHost() const { return m_ulHostAddr; }
    [[nodiscard]] int getPort() const { return m_iPort; }

    // oscpack builds ReceivedMessage (and throws on malformed packets) inside
    // ProcessPacket, i.e. outside ProcessMessage's try block. Without this a
    // single junk UDP datagram takes the robot down.
    void ProcessPacket(const char* data, int size, const IpEndpointName& remoteEndpoint) override {
        try {
            osc::OscPacketListener::ProcessPacket(data, size, remoteEndpoint);
        } catch (osc::Exception& e) {
            LOG_WARN("Dropping malformed OSC packet ({} bytes): {}", size, e.what());
        }
    }

protected:
    void ProcessMessage(const osc::ReceivedMessage& m, const IpEndpointName& remoteEndpoint) override {
        m_ulHostAddr = remoteEndpoint.address;
        m_iPort = remoteEndpoint.port;
        try {
            osc::uint32 count = m.ArgumentCount();
            osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
            const char* typeTags = m.TypeTags();
            const char* addr = m.AddressPattern();

            if (std::strcmp(addr, ARM_CMD_OSC_ROUTE) == 0) {
                processArmMessage(args, count);
            } else if (std::strcmp(addr, ARM_HOME_OSC_ROUTE) == 0) {
                if (m_armHomeCallback) m_armHomeCallback();
            } else if (std::strcmp(addr, ARM_RESET_OSC_ROUTE) == 0) {
                if (m_armResetCallback) m_armResetCallback();
            } else if (std::strcmp(addr, STRIKER_CMD_OSC_ROUTE) == 0) {
                processStrikerMessage(args, count);
            } else if (std::strcmp(addr, HEAD_CMD_OSC_ROUTE) == 0) {
                processHeadMessage(args, typeTags);
            } else if (std::strcmp(addr, HEAD_OSC_ROUTE) == 0) {
                processGestureMessage(args, count, typeTags);
            } else if (std::strcmp(addr, HEAD_STOP_OSC_ROUTE) == 0) {
                if (m_headStopCallback) m_headStopCallback();
            } else if (std::strcmp(addr, HEAD_LOOK_OSC_ROUTE) == 0) {
                if (m_headLookCallback) m_headLookCallback();
            } else if (std::strcmp(addr, SHIMON_OSC_ROUTE) == 0) {
                processSysMessage(args, typeTags);
            }

        } catch (osc::Exception& e) {
            // Parsing errors - unexpected argument types, missing arguments -
            // arrive here as exceptions.
            LOG_WARN("error while parsing message: {}: {}", m.AddressPattern(), e.what());
        }
    }

private:
    UdpListeningReceiveSocket m_socket;
    unsigned long m_ulHostAddr = 0;
    int m_iPort = 0;
    std::atomic<bool> m_bRunning = false;

    std::function<void(bool, int, int)> m_armMidiCallback = nullptr;
    std::function<void(int, int, float, float)> m_armCallback = nullptr;
    std::function<void(const char*)> m_armServoCallback = nullptr;
    std::function<void()> m_armHomeCallback = nullptr;
    std::function<void()> m_armResetCallback = nullptr;
    std::function<void(const char*)> m_sysMsgCallback = nullptr;

    std::function<void(const char*, std::optional<float>, std::optional<float>)> m_headCallback = nullptr;
    std::function<void(const char*, std::optional<float>)> m_headGestureCallback = nullptr;
    std::function<void()> m_headStopCallback = nullptr;
    std::function<void()> m_headLookCallback = nullptr;

    std::function<void(char, uint8_t, uint8_t, uint8_t)> m_strikerMidiCallback = nullptr;
    std::function<void(uint8_t, int, int)> m_strikerChoreoCallback = nullptr;
    std::function<void(char, uint8_t, int, int, int)> m_strikerCallback = nullptr;

    void processArmMessage(osc::ReceivedMessageArgumentStream& args, uint32_t count) {
        if (count == 1) {           // Servo command
            const char* cmd;
            args >> cmd >> osc::EndMessage;
            if (m_armServoCallback) m_armServoCallback(cmd);
        } else if (count == 2) {    // Midi format
            osc::int32 note, velocity;
            args >> note >> velocity >> osc::EndMessage;
            if (m_armMidiCallback) m_armMidiCallback(velocity != 0, (int)note, (int)velocity);
        } else if (count == 4) {    // Raw format
            osc::int32 armId, position, v_max;
            float acc;
            args >> armId >> position >> acc >> v_max >> osc::EndMessage;
            if (m_armCallback) m_armCallback((int)armId - 1, (int)position, acc, (float)v_max);
        } else {
            LOG_WARN("Unknown /arm command format. cmd count: {}", count);
        }
    }

    void processStrikerMessage(osc::ReceivedMessageArgumentStream& args, uint32_t count) {
        osc::int32 strikerIds;
        char type;

        if (count == 3) {           // Choreo format
            osc::int32 position, time_ms;
            args >> strikerIds >> position >> time_ms >> osc::EndMessage;
            if (strikerIds < 0 || strikerIds > 0xFF) return;
            if (m_strikerChoreoCallback) m_strikerChoreoCallback((uint8_t)strikerIds, (int)position, (int)time_ms);
        } else if (count == 4) {    // Midi format
            osc::int32 midiVelocity, channel;
            args >> type >> strikerIds >> midiVelocity >> channel >> osc::EndMessage;
            if (strikerIds < 0 || strikerIds > 0xFF) return;
            if (m_strikerMidiCallback)
                m_strikerMidiCallback(type, (uint8_t)strikerIds, (uint8_t)midiVelocity, (uint8_t)channel);
        } else if (count == 5) {    // Raw format
            osc::int32 dummy, position, acceleration;
            args >> type >> strikerIds >> dummy >> position >> acceleration >> osc::EndMessage;
            if (strikerIds < 0 || strikerIds > 0xFF) return;
            if (m_strikerCallback)
                m_strikerCallback(type, (uint8_t)strikerIds, (int)dummy, (int)position, (int)acceleration);
        } else {
            LOG_WARN("Unknown /striker command format. cmd count: {}", count);
        }
    }

    void processHeadMessage(osc::ReceivedMessageArgumentStream& args, const char* typeTags) {
        const char* cmd;
        if (strcmp(typeTags, "sii") == 0) {
            osc::int32 val1, val2;
            args >> cmd >> val1 >> val2 >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, (float)val1, (float)val2);
        } else if (strcmp(typeTags, "sif") == 0) {
            osc::int32 val1;
            float val2;
            args >> cmd >> val1 >> val2 >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, (float)val1, val2);
        } else if (strcmp(typeTags, "sfi") == 0) {
            float val1;
            osc::int32 val2;
            args >> cmd >> val1 >> val2 >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, val1, (float)val2);
        } else if (strcmp(typeTags, "sff") == 0) {
            float val1, val2;
            args >> cmd >> val1 >> val2 >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, val1, val2);
        } else if (strcmp(typeTags, "sf") == 0) {
            float val1;
            args >> cmd >> val1 >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, val1, std::optional<float>());
        } else if (strcmp(typeTags, "si") == 0) {
            osc::int32 val1;
            args >> cmd >> val1 >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, (float)val1, std::optional<float>());
        } else if (strcmp(typeTags, "s") == 0) {
            args >> cmd >> osc::EndMessage;
            if (m_headCallback) m_headCallback(cmd, std::optional<float>(), std::optional<float>());
        } else {
            LOG_WARN("unknown typeTag: {}", typeTags);
        }
    }

    // /head <gestureName> [bpm]
    void processGestureMessage(osc::ReceivedMessageArgumentStream& args, uint32_t count, const char* typeTags) {
        const char* name;
        if (count == 0) {
            LOG_WARN("Ignoring /head with 0 args");
            return;
        }

        if (strcmp(typeTags, "s") == 0) {
            args >> name >> osc::EndMessage;
            if (m_headGestureCallback) m_headGestureCallback(name, std::optional<float>());
        } else if (strcmp(typeTags, "sf") == 0) {
            float bpm;
            args >> name >> bpm >> osc::EndMessage;
            if (m_headGestureCallback) m_headGestureCallback(name, bpm);
        } else if (strcmp(typeTags, "si") == 0) {
            osc::int32 bpm;
            args >> name >> bpm >> osc::EndMessage;
            if (m_headGestureCallback) m_headGestureCallback(name, (float)bpm);
        } else {
            LOG_WARN("Unknown /head typeTag: {}", typeTags);
        }
    }

    void processSysMessage(osc::ReceivedMessageArgumentStream& args, const char* typeTags) {
        const char* cmd;
        if (strcmp(typeTags, "s") == 0) {
            args >> cmd >> osc::EndMessage;
            if (m_sysMsgCallback) m_sysMsgCallback(cmd);
        }
    }
};

#endif //SHAIACONTROLLER_OSCLISTENER_H
