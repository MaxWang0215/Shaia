//
// Shaia striker + slider firmware for the Robotis OpenCR 1.0.
//
// Six EPOS4 nodes on one CAN bus: nodes 1-2 are the sliders on the shared
// rail, nodes 3-6 the strikers (3/4 for arm 0, 5/6 for arm 1).
//
// The host (ShaiaController) speaks a newline delimited ASCII protocol on USB
// serial. Frames are dispatched on their first byte; the length tells the two
// 7-byte forms apart from the 4-byte ones.
//
//   slider   'm' sliderId  tickHi tickLo durHi durLo '\n'  (7 B)
//            sliderId is 1-indexed, ticks are that arm's home relative
//            encoder position, dur is the move time in ms.
//
//   striker  mode bitmask  midiVelocity                '\n' (4 B)
//            mode: 's' strike, 't' tremolo, 'p' tremolo stop.
//            bitmask has bit (node-1) set for every striker addressed.
//
//   choreo   'c' bitmask   posHi posLo  msHi msLo     '\n'  (7 B)
//            move the addressed strikers to posDeg over ms.
//
//   power    'v'                                      '\n'  (diagnostic)
//            logs each drive's DC link voltage, status word and last error.
//
//   home     'h' sliderId 0                          '\n'  (4 B)
//            sliderId 0 homes every slider. Also typable in a serial
//            monitor as "h" (all) or "h2" (that slider only).
//   restart  'r' 0 0                                  '\n'  (4 B)
//
// Hand calibration (see calibrate_keys.py). sliderId 0 means "all":
//   free     'f' sliderId 0                           '\n'  (4 B)
//   lock     'l' sliderId 0                           '\n'  (4 B)
//   get pos  'g' sliderId 0                           '\n'  (4 B)
//            replies "POS <sliderId> <ticks>" per slider.
//
// This replaces ShimonController-choreo/Strikers/Strikers.ino, which had no
// slider support (the big Shimon's arms are IAI actuators the host drives
// directly over Modbus) and used 16-bit striker params only.
//

#include "src/shaiaController.h"
#include "src/logger.h"

ShaiaController* pController = nullptr;

String inputString = "";        // Holds the incoming serial frame
bool stringComplete = false;    // Set once a full '\n' terminated frame is in

enum class FrameType {
    Unknown,
    Slider,
    Striker,
    Home,
    Restart,
    Free,
    Lock,
    GetPosition,
    PowerStatus
};

void setup() {
    // USB CDC re-enumerates on every reset and the host's serial monitor takes
    // a second or two to reattach. Without this pause the entire init log -
    // node status words, config failures, homing - is gone before anyone is
    // listening, which is not something you can scroll back to.
    Serial.begin(115200);
    delay(2500);

    LOG_LOG("Initializing Shaia's sliders and strikers...");
    inputString.reserve(16);

    pController = ShaiaController::createInstance();
    Error_t err = pController->init(MotorSpec::EC45);
    if (err != kNoError) {
        LOG_ERROR("Controller Init failed");
        return;
    }

    delay(100);

    // Strikers park at HOME_POSITION; the sliders run into their hard stops.
    pController->homeStrikers();
    err = pController->homeSliders();
    if (err != kNoError) LOG_ERROR("Slider homing failed - positions will be relative to power on");

    LOG_LOG("Successfully Initialized! Controller Starting...");
    delay(50);
    pController->start();
    delay(75);
    LOG_LOG("Listening for commands...");
}

void loop() {
    recoverFaultedNodes();

    if (!stringComplete) return;

    String frame = inputString;
    inputString = "";
    stringComplete = false;

    if (!pController) return;

    uint8_t idCode = 0;
    uint16_t param1 = 0;
    uint16_t param2 = 0;
    char mode = 0;
    FrameType type = FrameType::Unknown;

    Error_t err = parseCommand(frame, type, mode, idCode, param1, param2);
    if (err != kNoError) return;

    switch (type) {
    case FrameType::Slider:
        LOG_LOG("slider %i -> %i ticks in %i ms", idCode, param1, param2);
        pController->moveSlider(idCode, (int32_t) param1, param2);
        break;

    case FrameType::Striker: {
        Striker::Command cmd = ShaiaController::getStrikerCmd(mode);
        LOG_LOG("cmd %c, idCode: %i, param1: %i, param2: %i", mode, idCode, param1, param2);
        pController->executeCommand(idCode, cmd, param1, param2);
        break;
    }

    case FrameType::Home:
        if (idCode) LOG_LOG("homing slider %i", idCode);
        else LOG_LOG("homing all sliders");
        pController->homeSliders(idCode);
        break;

    case FrameType::Restart:
        LOG_LOG("restarting controller");
        pController->restart();
        break;

    case FrameType::Free:
        LOG_LOG("releasing slider %i for hand positioning", idCode);
        pController->freeSlider(idCode);
        break;

    case FrameType::Lock:
        LOG_LOG("re-engaging slider %i", idCode);
        pController->lockSlider(idCode);
        break;

    case FrameType::GetPosition:
        pController->reportSliderPosition(idCode);
        break;

    case FrameType::PowerStatus:
        pController->reportPowerStatus();
        break;

    default:
        break;
    }
}

// A drive that latches an error stops being fed setpoints, and a node that is
// not being fed never clears its fault on its own - so without this sweep a
// single transient (an RPDO timeout during a restart, say) takes an axis out
// until the next power cycle.
void recoverFaultedNodes() {
    static unsigned long lastCheck = 0;
    if (!pController) return;
    if (millis() - lastCheck < FAULT_RECOVERY_INTERVAL_MS) return;
    lastCheck = millis();
    pController->checkAndRecover();
}

void serialEvent() {
    while (Serial.available()) {
        char inChar = (char) Serial.read();
        inputString += inChar;
        if (inChar == '\n') {
            stringComplete = true;
            return;     // Leave the rest in the buffer for the next frame
        }
    }
}

Error_t parseCommand(const String& rawData, FrameType& type, char& mode,
                     uint8_t& idCode, uint16_t& param1, uint16_t& param2) {
    if (rawData.length() > 0 && (rawData[0] == 'v' || rawData[0] == 'h')) {
        // Typed by hand from the serial monitor, so shorter than the 4 byte
        // floor every host frame meets. 'h2' homes slider 2; a bare 'h', and
        // the host's binary 'h' frame, home all of them.
        mode = rawData[0];
        idCode = 0;
        if (rawData.length() > 1 && rawData[1] >= '1' && rawData[1] <= '9')
            idCode = (uint8_t) (rawData[1] - '0');

        type = (mode == 'v') ? FrameType::PowerStatus : FrameType::Home;
        return kNoError;
    }

    if (rawData.length() < 4) return kCorruptedDataError;

    mode = rawData[0];
    idCode = (uint8_t) rawData[1];
    param1 = 0;
    param2 = 0;

    switch (mode) {
    case 'm':
        // 'm' sliderId tickHi tickLo durHi durLo '\n'
        if (rawData.length() < 7) return kCorruptedDataError;
        param1 = ((uint16_t) (uint8_t) rawData[2] << 8) | (uint8_t) rawData[3];   // ticks
        param2 = ((uint16_t) (uint8_t) rawData[4] << 8) | (uint8_t) rawData[5];   // duration ms
        type = FrameType::Slider;
        return kNoError;

    case 'c':
        // 'c' bitmask posHi posLo msHi msLo '\n'
        if (rawData.length() < 7) return kCorruptedDataError;
        param1 = ((uint16_t) (uint8_t) rawData[2] << 8) | (uint8_t) rawData[3];   // position deg
        param2 = ((uint16_t) (uint8_t) rawData[4] << 8) | (uint8_t) rawData[5];   // time ms
        type = FrameType::Striker;
        return kNoError;

    case 'r':
        type = FrameType::Restart;
        return kNoError;

    case 'f':
        type = FrameType::Free;
        return kNoError;

    case 'l':
        type = FrameType::Lock;
        return kNoError;

    case 'g':
        type = FrameType::GetPosition;
        return kNoError;

    case 's':
    case 't':
    case 'p':
    case 'q':
        // mode bitmask midiVelocity '\n'
        param1 = (uint8_t) rawData[2];   // midi velocity
        type = FrameType::Striker;
        return kNoError;

    default:
        LOG_WARN("Unknown frame mode: %c", mode);
        type = FrameType::Unknown;
        return kCorruptedDataError;
    }
}
