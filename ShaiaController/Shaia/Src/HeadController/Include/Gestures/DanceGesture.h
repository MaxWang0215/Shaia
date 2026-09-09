//
// Looping, beat scheduled head gesture driven by Config/danceModes.json.
//
// C++ port of shaia_targetUpdated/head.py's GestureRunner. One gesture plays at
// a time; play() atomically swaps to a new one and bumps a generation counter
// so the worker switches immediately instead of finishing the current period.
//
// A dance mode is a list of moves:
//   { "motor": "HeadTilt", "startBeat": 0.5, "periodBeat": 1,
//     "position": 0.8, "velocity": 0.03 }
// startBeat/periodBeat are in beats and get scaled by 60/bpm; position and
// velocity are the normalized [0, 1] values the head axes take directly.
//
// danceModes.json is parsed with yaml-cpp - JSON is a subset of YAML 1.2, so
// no extra dependency is needed.
//

#ifndef SHAIACONTROLLER_DANCEGESTURE_H
#define SHAIACONTROLLER_DANCEGESTURE_H

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>

#include "yaml-cpp/yaml.h"

#include "Gesture.h"
#include "Def.h"
#include "Util.h"
#include "ErrorDef.h"
#include "Logger.h"

using namespace std::chrono;

class DanceGesture : public Gesture {
public:
    struct Move_t {
        HeadCommand motor = HeadCommand::Unknown;
        float startBeat = 0;
        float periodBeat = 1;
        float position = 0.5f;
        float velocity = 0.02f;
    };

    struct Mode_t {
        std::string name;
        float energy = 0;
        float bpmMin = 30;
        float bpmMax = 120;
        std::vector<Move_t> moves;
    };

    explicit DanceGesture(CommandManager<HeadCmdPacket_t, Port::Head>& cmdManager) : Gesture(cmdManager) {}

    ~DanceGesture() override { stopRunner(); }

    Error_t load(const std::string& danceModesFile) {
        try {
            auto root = YAML::LoadFile(danceModesFile);

            for (auto it = root.begin(); it != root.end(); ++it) {
                Mode_t mode;
                mode.name = it->first.as<std::string>();
                auto node = it->second;

                if (node["energy"]) mode.energy = node["energy"].as<float>();
                if (node["bpm_min"]) mode.bpmMin = node["bpm_min"].as<float>();
                if (node["bpm_max"]) mode.bpmMax = node["bpm_max"].as<float>();

                for (auto&& m: node["moves"]) {
                    Move_t move;
                    move.motor = HeadCommandUtil::getCommand(m["motor"].as<std::string>());
                    if (move.motor == HeadCommand::Unknown) {
                        LOG_WARN("Gesture '{}' references unknown motor '{}' - skipping that move",
                                 mode.name, m["motor"].as<std::string>());
                        continue;
                    }
                    move.startBeat = m["startBeat"].as<float>();
                    move.periodBeat = m["periodBeat"].as<float>();
                    move.position = m["position"].as<float>();
                    move.velocity = m["velocity"].as<float>();

                    if (move.periodBeat <= 0) {
                        LOG_WARN("Gesture '{}' has a move with periodBeat {} - skipping it",
                                 mode.name, move.periodBeat);
                        continue;
                    }

                    mode.moves.push_back(move);
                }

                m_modes[mode.name] = std::move(mode);
            }
        } catch (YAML::BadFile&) {
            LOG_ERROR("{} not found. Run the binary from build/ or fix DANCE_MODES_FILE in Def.h", danceModesFile);
            return kFileParseError;
        } catch (YAML::Exception& e) {
            LOG_ERROR("Error parsing {}: {}", danceModesFile, e.msg);
            return kFileParseError;
        }

        if (m_modes.empty()) {
            LOG_ERROR("No gestures found in {}", danceModesFile);
            return kFileParseError;
        }

        LOG_INFO("Gestures available: {}", listGestures());
        return kNoError;
    }

    std::string listGestures() const {
        std::string out;
        for (auto& element: m_modes) {
            if (!out.empty()) out += ", ";
            out += element.first;
        }
        return out;
    }

    bool hasGesture(const std::string& name) const { return m_modes.find(name) != m_modes.end(); }

    void start() override {
        m_bRunning = true;
        m_pThread = std::make_unique<std::thread>([this] { threadHandler(); });
    }

    void stopRunner() {
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            m_bRunning = false;
        }
        m_cv.notify_all();
        if (m_pThread && m_pThread->joinable()) m_pThread->join();
        m_pThread = nullptr;
    }

    void play(const std::string& name, float bpm = DEFAULT_BPM) {
        auto it = m_modes.find(name);
        if (it == m_modes.end()) {
            LOG_WARN("Unknown gesture: {} (available: {})", name, listGestures());
            return;
        }

        if (bpm <= 0) {
            LOG_WARN("Bad BPM {}, falling back to {}", bpm, DEFAULT_BPM);
            bpm = DEFAULT_BPM;
        }

        const auto& mode = it->second;
        // Fold the incoming tempo by factors of 2 until it sits in the range
        // this mode was choreographed for (dance.py:104).
        float danceBpm = normalizeBpm(bpm, mode.bpmMin, mode.bpmMax);
        float beatToSec = 60.f / danceBpm;

        std::vector<Event_t> events;
        events.reserve(mode.moves.size());
        for (const auto& move: mode.moves) {
            events.push_back({ .motor = move.motor,
                               .offset_s = move.startBeat * beatToSec,
                               .period_s = move.periodBeat * beatToSec,
                               .position = move.position,
                               .velocity = move.velocity });
        }

        {
            std::lock_guard<std::mutex> lk(m_mtx);
            m_currentName = name;
            m_events = std::move(events);
            m_startTime = steady_clock::now();
            m_iGeneration++;
        }
        m_cv.notify_all();

        LOG_INFO("Playing gesture {} @ {} BPM (requested {}, {} events)",
                 name, danceBpm, bpm, mode.moves.size());
    }

    void stop() override {
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            if (m_currentName.empty()) return;
            LOG_INFO("Stopping gesture {}", m_currentName);
            m_currentName.clear();
            m_events.clear();
            m_iGeneration++;
        }
        m_cv.notify_all();
    }

    [[nodiscard]] bool isPlaying() {
        std::lock_guard<std::mutex> lk(m_mtx);
        return !m_currentName.empty();
    }

    // Fold by factors of 2 until the tempo fits in [bpmMin, bpmMax].
    static float normalizeBpm(float bpm, float bpmMin, float bpmMax) {
        if (bpm <= 0 || bpmMin <= 0 || bpmMax < bpmMin) return bpm;
        while (bpm > bpmMax) bpm /= 2.f;
        while (bpm < bpmMin) bpm *= 2.f;
        return bpm;
    }

private:
    struct Event_t {
        HeadCommand motor;
        float offset_s;
        float period_s;
        float position;
        float velocity;
    };

    std::unordered_map<std::string, Mode_t> m_modes;

    std::mutex m_mtx;
    std::condition_variable m_cv;
    std::atomic<bool> m_bRunning = false;
    std::unique_ptr<std::thread> m_pThread = nullptr;

    std::string m_currentName;
    std::vector<Event_t> m_events;
    tp m_startTime;
    long long m_iGeneration = 0;

    void threadHandler() {
        std::unordered_map<size_t, float> lastFired;
        long long generationSeen = -1;

        while (true) {
            std::string name;
            std::vector<Event_t> events;
            tp startTime;
            long long generation;

            {
                std::unique_lock<std::mutex> lk(m_mtx);
                if (!m_bRunning) return;
                if (m_currentName.empty()) {
                    // Idle - block until play() or stopRunner() wakes us.
                    m_cv.wait(lk, [this] { return !m_bRunning || !m_currentName.empty(); });
                    if (!m_bRunning) return;
                }
                name = m_currentName;
                events = m_events;
                startTime = m_startTime;
                generation = m_iGeneration;
            }

            if (generation != generationSeen) {
                lastFired.clear();
                generationSeen = generation;
            }

            float t = (float)duration_cast<microseconds>(steady_clock::now() - startTime).count() / 1e6f;
            const float triggerWindow = (float)GESTURE_TRIGGER_WINDOW / 1000.f;

            for (size_t idx = 0; idx < events.size(); ++idx) {
                const auto& e = events[idx];
                if (t < e.offset_s) continue;

                auto n = (float)(int)((t - e.offset_s) / e.period_s);
                float nextTrigger = e.offset_s + n * e.period_s;

                if (t - nextTrigger < 0 || t - nextTrigger >= triggerWindow) continue;

                auto it = lastFired.find(idx);
                if (it != lastFired.end() && it->second == nextTrigger) continue;

                lastFired[idx] = nextTrigger;
                move(e.motor, e.position, e.velocity);
            }

            // Sleep on the cv so play()/stop() take effect immediately.
            std::unique_lock<std::mutex> lk(m_mtx);
            if (m_bRunning && !m_currentName.empty() && generation == m_iGeneration)
                m_cv.wait_for(lk, milliseconds(GESTURE_TICK));
        }
    }
};

#endif //SHAIACONTROLLER_DANCEGESTURE_H
