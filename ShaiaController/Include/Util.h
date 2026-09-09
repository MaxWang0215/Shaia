//
// Shared helpers. Adapted from ShimonController-choreo/Include/Util.h.
//

#ifndef SHAIACONTROLLER_UTIL_H
#define SHAIACONTROLLER_UTIL_H

#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>
#include <cstdio>
#include <array>
#include <list>

#include "Def.h"

class Util {
public:
    static void sleep_us(int micro) {
        std::this_thread::sleep_for(std::chrono::microseconds(micro));
    }

    static void sleep_ms(int millis) {
        std::this_thread::sleep_for(std::chrono::milliseconds(millis));
    }

    static void sleep_s(int seconds) {
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
    }

    static bool valueInArray(int value, const int* pArr, int len) {
        for (int i = 0; i < len; ++i) {
            if (value == pArr[i]) return true;
        }
        return false;
    }

    static bool isWhiteKey(int midiNote) {
        return !valueInArray(((midiNote % 12) + 12) % 12, kBlackKeys, 5);
    }

    // Plain octave transposition. The caller range checks via midiToPosition.
    static int transpose(int midiNote, int semitones) {
        return midiNote + semitones;
    }

    // mm along the shared rail for a midi note, or -1 if the note is off the bars.
    static int midiToPosition(int midiNote) {
        if (midiNote < MIN_NOTE || midiNote > MAX_NOTE) return -1;
        return kNotePositionTable[midiNote - MIN_NOTE];
    }

    // Rail mm -> the arm's own home relative encoder ticks.
    static int railToArmTicks(int armId, int railMm) {
        railMm = std::max(0, std::min(SLIDER_LIMIT, railMm));
        int armMm = (kSliderW[armId] * railMm) + kSliderB[armId];
        int ticks = (int)std::lround(armMm * kMmToTicks);
        return std::max(0, std::min(0xFFFF, ticks));
    }

    static std::string toLowerCase(const std::string& data) {
        auto out = data;
        std::transform(out.begin(), out.end(), out.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        return out;
    }

    static std::string toUpperCase(const std::string& data) {
        auto out = data;
        std::transform(out.begin(), out.end(), out.begin(),
                       [](unsigned char c) { return std::toupper(c); });
        return out;
    }

    static int hex2int(char c) {
        if (isdigit(c)) return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    }

    static int strtoi(const char* c, int n, int base = 16) {
        int num = 0;
        int m = 0;
        for (int i = n - 1; i > -1; --i) {
            int temp = hex2int(c[i]);
            if (temp < 0) break;
            num += (int)std::pow(base, m) * temp;
            ++m;
        }
        return num;
    }

    static bool arePositionsDifferent(std::array<int, NUM_ARMS> a, std::array<int, NUM_ARMS> b) {
        for (int i = 0; i < NUM_ARMS; ++i) {
            if (a[i] != b[i]) return true;
        }
        return false;
    }
};

#endif //SHAIACONTROLLER_UTIL_H
