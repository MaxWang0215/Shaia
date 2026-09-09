//
// Firmware side controller: owns the CAN bus, the 2 sliders and the 4
// strikers, and the 1 ms PDO timer that steps every node's trajectory.
//
// Adapted from ShimonController-choreo/Strikers/src/strikerController.h. The
// difference is that node ids 1..2 are sliders rather than strikers, so CAN
// receive routing and the PDO tick have to fan out to two different classes.
//

#ifndef SHAIACONTROLLER_H
#define SHAIACONTROLLER_H

#include "def.h"
#include "slider.h"
#include "striker.h"
#include <HardwareTimer.h>

class ShaiaController {
public:
    static ShaiaController* createInstance() {
        pInstance = new ShaiaController;
        return pInstance;
    }

    static void destroyInstance() {
        delete pInstance;
        pInstance = nullptr;
    }

    Error_t init(MotorSpec spec, bool bInitCAN = true) {
        if (bInitCAN) {
            if (!CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT)) {
                LOG_ERROR("CAN open failed");
                return kFileOpenError;
            }

            CanBus.attachRxInterrupt(canRxHandle);
        }

        RPDOTimer.setPeriod(PDO_RATE * 1000);
        RPDOTimer.attachInterrupt(RPDOTimerIRQHandler);

        m_motorSpec = spec;

        Error_t err = initSliders(spec);
        if (err != kNoError) return err;

        return initStrikers(spec);
    }

    Error_t initSliders(MotorSpec spec) {
        for (int i = SLIDER_NODE_START; i < SLIDER_NODE_START + NUM_SLIDERS; ++i) {
            Error_t err = sliderAt(i).init(i, spec);
            if (err != kNoError) LOG_ERROR("Cannot initialize slider with id %i. Error: %i", i, err);
        }
        return kNoError;
    }

    Error_t initStrikers(MotorSpec spec) {
        for (int i = STRIKER_NODE_START; i < STRIKER_NODE_START + NUM_STRIKERS; ++i) {
            Error_t err = strikerAt(i).init(i, spec);
            if (err != kNoError) LOG_ERROR("Cannot initialize striker with id %i. Error: %i", i, err);
        }
        return kNoError;
    }

    void reset(bool bTerminateCAN = true) {
        m_bPlaying = false;
        RPDOTimer.stop();
        enablePDO(false);

        for (int i = 0; i < NUM_SLIDERS; ++i)
            m_slider[i].reset();

        for (int i = 0; i < NUM_STRIKERS; ++i)
            m_striker[i].reset();

        if (bTerminateCAN) CanBus.end();
    }

    Error_t restart() {
        auto spec = m_motorSpec;
        reset(false);
        Error_t err = init(spec, false);
        if (err != kNoError) {
            LOG_ERROR("Cannot init controller");
            return err;
        }

        start();
        return kNoError;
    }

    static Striker::Command getStrikerCmd(char cmd) {
        switch (cmd) {
        case 's':
            return Striker::Command::Normal;
        case 't':
            return Striker::Command::Tremolo;
        case 'p':
            return Striker::Command::StopTremolo;
        case 'r':
            return Striker::Command::Restart;
        case 'q':
            return Striker::Command::Quit;
        case 'c':
            return Striker::Command::Choreo;
        default:
            LOG_ERROR("unknown command : %c", cmd);
            return Striker::Command::Normal;
        }
    }

    void start() {
        // A restart stops the RPDO stream for as long as it takes to walk six
        // nodes' worth of blocking SDO config, which is comfortably past the
        // drives' 0x8250 RPDO timeout. The resulting EMCY latches in
        // Epos4::m_uiError - TPDO3 keeps the default 6 byte mapping, so nothing
        // refreshes it - and Slider/Striker::update() then refuse to publish
        // setpoints for a node that would have recovered the moment it got one.
        // Clear whatever is latched before enabling, or the axes come back up
        // dead.
        clearFaults();

        Error_t err = enablePDO();
        if (err != kNoError) {
            LOG_ERROR("cannot enable PDO");
            return;
        }

        err = enableAll();
        if (err != kNoError) {
            LOG_ERROR("cannot enable motors");
            return;
        }

        m_bPlaying = true;
        RPDOTimer.start();
    }

    void stop() {
        Error_t err = enablePDO(false);
        if (err != kNoError) {
            LOG_ERROR("cannot disable PDO");
            return;
        }

        err = enableAll(false);
        if (err != kNoError) {
            LOG_ERROR("cannot disable motors");
            return;
        }

        m_bPlaying = false;
        RPDOTimer.stop();
    }

    Error_t enablePDO(bool bEnable = true) {
        for (int i = SLIDER_NODE_START; i < SLIDER_NODE_START + NUM_SLIDERS; ++i) {
            Error_t err = sliderAt(i).enablePDO(bEnable);
            if (err != kNoError) {
                LOG_ERROR("Slider %i EnablePDO failed. Error Code %i", i, err);
                return err;
            }
        }

        for (int i = STRIKER_NODE_START; i < STRIKER_NODE_START + NUM_STRIKERS; ++i) {
            Error_t err = strikerAt(i).enablePDO(bEnable);
            if (err != kNoError) {
                LOG_ERROR("Striker %i EnablePDO failed. Error Code %i", i, err);
                return err;
            }
        }

        return kNoError;
    }

    Error_t clearFaults() {
        for (int i = SLIDER_NODE_START; i < SLIDER_NODE_START + NUM_SLIDERS; ++i)
            sliderAt(i).clearFault();

        for (int i = STRIKER_NODE_START; i < STRIKER_NODE_START + NUM_STRIKERS; ++i)
            strikerAt(i).clearFault();

        return kNoError;
    }

    // Bring back any node that has latched an error since the last sweep. Both
    // update() paths bail out for a faulted node, so the SDO traffic this does
    // cannot collide with the ISR's use of that node's tx buffer; healthy nodes
    // cost a local comparison and no CAN traffic at all. Call from loop().
    void checkAndRecover() {
        if (!m_bPlaying) return;

        for (int i = 0; i < NUM_SLIDERS; ++i)
            m_slider[i].checkAndRecover();

        for (int i = 0; i < NUM_STRIKERS; ++i)
            m_striker[i].checkAndRecover();
    }

    Error_t enableAll(bool bEnable = true) {
        for (int i = SLIDER_NODE_START; i < SLIDER_NODE_START + NUM_SLIDERS; ++i) {
            Error_t err = sliderAt(i).enable(bEnable);
            if (err != kNoError) {
                LOG_ERROR("Slider %i enable failed", i);
                return err;
            }
        }

        for (int i = STRIKER_NODE_START; i < STRIKER_NODE_START + NUM_STRIKERS; ++i) {
            Error_t err = strikerAt(i).enable(bEnable);
            if (err != kNoError) {
                LOG_ERROR("Striker %i enable failed", i);
                return err;
            }
        }

        return kNoError;
    }

    // ------------------------------------------------------------ Sliders --

    // sliderId is 1-indexed, as it arrives on the wire.
    Error_t moveSlider(uint8_t sliderId, int32_t targetTicks, uint16_t duration_ms) {
        if (sliderId < SLIDER_NODE_START || sliderId >= SLIDER_NODE_START + NUM_SLIDERS) {
            LOG_ERROR("Unknown slider id %i", sliderId);
            return kInvalidArgsError;
        }

        return sliderAt(sliderId).moveTo(targetTicks, duration_ms);
    }

    // --- hand calibration support -------------------------------------------
    // sliderId 0 means "all sliders" for the three calls below.

    // Release a slider so it can be pushed by hand.
    Error_t freeSlider(uint8_t sliderId) {
        // The PDO timer keeps running: Slider::update() suppresses setpoints
        // for a released axis on its own, and the strikers still need their
        // stream or their drives trip an RPDO timeout.
        if (sliderId == 0) {
            Error_t ret = kNoError;
            for (int i = SLIDER_NODE_START; i < SLIDER_NODE_START + NUM_SLIDERS; ++i) {
                Error_t err = sliderAt(i).release();
                if (err != kNoError) ret = err;
            }
            return ret;
        }

        if (!isSliderNode(sliderId)) {
            LOG_ERROR("Unknown slider id %i", sliderId);
            return kInvalidArgsError;
        }

        return sliderAt(sliderId).release();
    }

    // Re-engage a slider, holding wherever it was left.
    Error_t lockSlider(uint8_t sliderId) {
        Error_t ret = kNoError;

        if (sliderId == 0) {
            for (int i = SLIDER_NODE_START; i < SLIDER_NODE_START + NUM_SLIDERS; ++i) {
                Error_t err = sliderAt(i).hold();
                if (err != kNoError) ret = err;
            }
        } else if (isSliderNode(sliderId)) {
            ret = sliderAt(sliderId).hold();
        } else {
            LOG_ERROR("Unknown slider id %i", sliderId);
            return kInvalidArgsError;
        }

        return ret;
    }

    // Report positions as "POS <id> <ticks>" - a bare line, not a log line, so
    // the host can parse it without stripping the logger's prefix.
    // Ask every drive what it sees on its power stage rail. The RPDO ISR and
    // the SDO path share Epos4::m_txMsg, so the cyclic stream is paused for the
    // handful of ms this takes.
    Error_t reportPowerStatus() {
        bool bWasPlaying = m_bPlaying;
        if (bWasPlaying) RPDOTimer.stop();

        for (int i = SLIDER_NODE_START; i < SLIDER_NODE_START + NUM_SLIDERS; ++i)
            sliderAt(i).reportPowerStatus();

        for (int i = STRIKER_NODE_START; i < STRIKER_NODE_START + NUM_STRIKERS; ++i)
            strikerAt(i).reportPowerStatus();

        if (bWasPlaying) RPDOTimer.start();
        return kNoError;
    }

    Error_t reportSliderPosition(uint8_t sliderId) {
        int first = SLIDER_NODE_START;
        int last = SLIDER_NODE_START + NUM_SLIDERS;

        if (sliderId != 0) {
            if (!isSliderNode(sliderId)) {
                LOG_ERROR("Unknown slider id %i", sliderId);
                return kInvalidArgsError;
            }
            first = sliderId;
            last = sliderId + 1;
        }

        Error_t ret = kNoError;
        for (int i = first; i < last; ++i) {
            int32_t ticks = 0;
            Error_t err = sliderAt(i).readPosition(ticks);
            if (err != kNoError) {
                LOG_ERROR("Cannot read position of slider %i", i);
                ret = err;
                continue;
            }

            Serial.print("POS ");
            Serial.print(i);
            Serial.print(" ");
            Serial.println((long) ticks);
        }

        return ret;
    }

    // sliderId 0 homes every slider, otherwise just that one - handy when the
    // two axes are behaving differently and you want to retry one of them.
    Error_t homeSliders(uint8_t sliderId = 0) {
        int first = SLIDER_NODE_START;
        int last = SLIDER_NODE_START + NUM_SLIDERS;

        if (sliderId != 0) {
            if (!isSliderNode(sliderId)) {
                LOG_ERROR("Unknown slider id %i", sliderId);
                return kInvalidArgsError;
            }
            first = sliderId;
            last = sliderId + 1;
        }

        // Homing is blocking and drives the axis open loop, so the axis being
        // homed has to come out of the cyclic stream - but only that axis.
        // Calling stop() here would disable every node, which drops the
        // strikers limp for the duration.
        Error_t ret = kNoError;
        for (int i = first; i < last; ++i) {
            if (m_bPlaying) sliderAt(i).enablePDO(false);

            Error_t err = sliderAt(i).home();
            if (err != kNoError) ret = err;

            if (m_bPlaying) {
                Error_t back = sliderAt(i).enablePDO(true);
                if (back == kNoError) back = sliderAt(i).enable(true);
                if (back != kNoError) {
                    LOG_ERROR("Slider %i did not rejoin the PDO stream. Error: %i", i, back);
                    ret = back;
                }
            }
        }

        return ret;
    }

    // ----------------------------------------------------------- Strikers --

    uint8_t prepare(uint8_t idCode, Striker::Command cmd, uint16_t param1, uint16_t param2) {
        uint8_t uiStrike = 0;

        if (cmd == Striker::Command::Restart) {
            restart();
            return 0;
        }

        // Tremolo with param1 == 0 means "stop tremolo".
        if (cmd == Striker::Command::Tremolo && param1 == 0) cmd = Striker::Command::StopTremolo;

        for (int i = STRIKER_NODE_START; i < STRIKER_NODE_START + NUM_STRIKERS; ++i) {
            if (idCode & (1 << (i - 1))) {
                bool bStrike = strikerAt(i).prepare(cmd, param1, param2);
                uiStrike |= bStrike << (i - 1);
            }
        }

        return uiStrike;
    }

    void executeCommand(uint8_t idCode, Striker::Command cmd, uint16_t param1, uint16_t param2) {
        uint8_t uiStrike = prepare(idCode, cmd, param1, param2);
        strike(uiStrike);
    }

    void strike(uint8_t idCode) {
        for (int i = STRIKER_NODE_START; i < STRIKER_NODE_START + NUM_STRIKERS; ++i) {
            if (idCode & (1 << (i - 1))) strikerAt(i).strike();
        }
    }

    Error_t homeStrikers() {
        for (int i = STRIKER_NODE_START; i < STRIKER_NODE_START + NUM_STRIKERS; ++i)
            strikerAt(i).home();
        return kNoError;
    }

private:
    // Packed, not indexed by node id - each Striker owns a 20 KB trajectory
    // buffer, so the dummy slots the choreo firmware kept would cost ~60 KB of
    // the OpenCR's SRAM for nothing. sliderAt()/strikerAt() do the mapping.
    Slider m_slider[NUM_SLIDERS];
    Striker m_striker[NUM_STRIKERS];

    Slider& sliderAt(int nodeId) { return m_slider[nodeId - SLIDER_NODE_START]; }
    Striker& strikerAt(int nodeId) { return m_striker[nodeId - STRIKER_NODE_START]; }

    static ShaiaController* pInstance;
    volatile bool m_bPlaying = false;
    MotorSpec m_motorSpec = EC45;

    HardwareTimer RPDOTimer;

    ShaiaController() : RPDOTimer(TIMER_CH1) {}

    ~ShaiaController() {
        reset();
    }

    static bool isSliderNode(int id) {
        return id >= SLIDER_NODE_START && id < SLIDER_NODE_START + NUM_SLIDERS;
    }

    static bool isStrikerNode(int id) {
        return id >= STRIKER_NODE_START && id < STRIKER_NODE_START + NUM_STRIKERS;
    }

    static void canRxHandle(can_message_t* arg) {
        if (!pInstance) return;

        auto id = (int)(arg->id - COB_ID_SDO_SC);
        if (isSliderNode(id)) pInstance->sliderAt(id).setRxMsg(*arg);
        else if (isStrikerNode(id)) pInstance->strikerAt(id).setRxMsg(*arg);

        id = (int)(arg->id - COB_ID_TPDO3);
        if (isSliderNode(id)) pInstance->sliderAt(id).PDO_processMsg(*arg);
        else if (isStrikerNode(id)) pInstance->strikerAt(id).PDO_processMsg(*arg);

        id = (int)(arg->id - COB_ID_EMCY);
        if (isSliderNode(id)) pInstance->sliderAt(id).handleEMCYMsg(*arg);
        else if (isStrikerNode(id)) pInstance->strikerAt(id).handleEMCYMsg(*arg);
    }

    static void RPDOTimerIRQHandler() {
        if (!pInstance) return;

        for (int i = 0; i < NUM_SLIDERS; ++i)
            pInstance->m_slider[i].update();

        for (int i = 0; i < NUM_STRIKERS; ++i)
            pInstance->m_striker[i].update();
    }
};

ShaiaController* ShaiaController::pInstance = nullptr;
#endif // SHAIACONTROLLER_H
