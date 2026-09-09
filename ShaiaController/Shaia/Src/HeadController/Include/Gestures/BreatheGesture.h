//
// Idle breathing: a slow neck rise/fall with a smaller head tilt riding on top.
//
// Same shape as the choreo build's BreatheGesture, retargeted from the Copley
// neck/headTilt axes to Shaia's NeckTilt/HeadTilt Dynamixels, so positions are
// normalized [0, 1] instead of radians.
//

#ifndef SHAIACONTROLLER_BREATHEGESTURE_H
#define SHAIACONTROLLER_BREATHEGESTURE_H

#include <array>
#include <atomic>
#include <future>

#include "Gesture.h"
#include "Scheduler.h"

class BreatheGesture : public Gesture {
public:
    explicit BreatheGesture(CommandManager<HeadCmdPacket_t, Port::Head>& cmdManager) : Gesture(cmdManager) {}

    ~BreatheGesture() override { stop(); }

    void start() override {
        stop();

        m_bNeckUp = false;
        m_bHeadUp = false;

        // Settle onto the neutral pose first.
        move(HeadCommand::NeckTilt, kNeckLow, kNeckVelocity);
        move(HeadCommand::HeadTilt, kHeadLow, kHeadVelocity);

        m_futures[0] = m_scheduler.schedule([this] {
            move(HeadCommand::NeckTilt, m_bNeckUp ? kNeckLow : kNeckHigh, kNeckVelocity);
            m_bNeckUp = !m_bNeckUp;
        }, Scheduler::Delay_ms((int)(0.8 * kPeriod_ms)), Scheduler::Period_ms(kPeriod_ms));

        m_futures[1] = m_scheduler.schedule([this] {
            move(HeadCommand::HeadTilt, m_bHeadUp ? kHeadLow : kHeadHigh, kHeadVelocity);
            m_bHeadUp = !m_bHeadUp;
        }, Scheduler::Delay_ms(0), Scheduler::Period_ms(kPeriod_ms));
    }

    void stop() override {
        m_scheduler.stop();
    }

private:
    // A breath in/out cycle. Each axis toggles once per period.
    static constexpr int kPeriod_ms = 2000;

    static constexpr float kNeckLow = 0.45f;
    static constexpr float kNeckHigh = 0.65f;
    static constexpr float kNeckVelocity = 0.008f;

    static constexpr float kHeadLow = 0.45f;
    static constexpr float kHeadHigh = 0.58f;
    static constexpr float kHeadVelocity = 0.012f;

    std::atomic<bool> m_bNeckUp = false;
    std::atomic<bool> m_bHeadUp = false;

    Scheduler m_scheduler;
    std::array<std::future<void>, 2> m_futures;
};

#endif //SHAIACONTROLLER_BREATHEGESTURE_H
