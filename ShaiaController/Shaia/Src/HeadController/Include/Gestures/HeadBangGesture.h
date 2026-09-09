//
// One head bang on the beat. `beat(length_ms)` is the inter-beat interval, so
// the master can drive it straight from its tempo tracker.
//
// Same staging as the choreo build's HeadBangGesture (neck lifts, head snaps
// up, then both come down) on Shaia's NeckTilt/HeadTilt axes.
//

#ifndef SHAIACONTROLLER_HEADBANGGESTURE_H
#define SHAIACONTROLLER_HEADBANGGESTURE_H

#include <array>
#include <future>

#include "Gesture.h"
#include "Scheduler.h"

class HeadBangGesture : public Gesture {
public:
    explicit HeadBangGesture(CommandManager<HeadCmdPacket_t, Port::Head>& cmdManager) : Gesture(cmdManager) {}

    ~HeadBangGesture() override { stop(); }

    void beat(int length_ms) {
        // Below ~500 ms the mechanics cannot follow, so bang on every other
        // beat (or every fourth) instead.
        while (length_ms > 0 && length_ms < kMinBeat_ms) length_ms *= 2;
        if (length_ms <= 0) length_ms = kMinBeat_ms;

        m_scheduler.stop();

        // Lift into the bang.
        move(HeadCommand::NeckTilt, kNeckUp, 0.02f);

        // Head snaps up.
        m_futures[0] = m_scheduler.schedule([this] {
            move(HeadCommand::HeadTilt, kHeadUp, 0.05f);
        }, Scheduler::Delay_ms(length_ms / 5));

        // Neck drops on the beat.
        m_futures[1] = m_scheduler.schedule([this] {
            move(HeadCommand::NeckTilt, kNeckDown, 0.05f);
        }, Scheduler::Delay_ms(length_ms / 2));

        // Head follows through.
        m_futures[2] = m_scheduler.schedule([this] {
            move(HeadCommand::HeadTilt, kHeadDown, 0.06f);
        }, Scheduler::Delay_ms((length_ms / 2) + (length_ms / 5)));
    }

    void stop() override {
        m_scheduler.stop();
        move(HeadCommand::NeckTilt, kNeckNeutral, 0.015f);
    }

private:
    static constexpr int kMinBeat_ms = 500;

    static constexpr float kNeckNeutral = 0.5f;
    static constexpr float kNeckUp = 0.7f;
    static constexpr float kNeckDown = 0.3f;
    static constexpr float kHeadUp = 0.9f;
    static constexpr float kHeadDown = 0.1f;

    Scheduler m_scheduler;
    std::array<std::future<void>, 3> m_futures;
};

#endif //SHAIACONTROLLER_HEADBANGGESTURE_H
