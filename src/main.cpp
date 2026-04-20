/*
 * Velostat 3x3 Drum Pad — USB MPE Controller
 * Hardware: Electrosmith Daisy Seed (STM32H750 @ 480 MHz)
 * * [V4 FINAL MERGE] 
 * 1. 12-bit ADC alignment (All Arduino thresholds x4).
 * 2. 300us Settling delay for RC charging curve.
 * 3. High-Z Row switching logic (Matches Arduino pinMode(INPUT)).
 * 4. Original MPE MIDI Protocol & LPF.
 * 04/19/2026 by Peijie Liu
 */

#include "daisy_seed.h"
#include <cmath>
#include <cstring>

using namespace daisy;
using namespace seed;

static DaisySeed      hw;
static MidiUsbHandler midi;

static const int numRows = 3;
static const int numCols = 3;
static const Pin rowPinDefs[numRows] = {D3, D2, D1};
static GPIO      rowGpio[numRows];
static const Pin colPinDefs[numCols] = {A0, A1, A2};

// ── MPE Configuration ─────────────────────────────────────────────────────────
static const uint8_t MPE_CHANNEL = 1;
static const uint8_t MPE_NOTE = 60;
static const uint32_t MPE_MSG_INTERVAL_MS = 20;
static const int PB_DEADBAND = 32;
static const int CC74_DEADBAND = 2;

// ── Thresholds (Scaled to 12-bit: Arduino values * 4) ─────────────────────────
static int noiseThreshold   = 360; // (90 * 4)
static int pressThreshold   = 480; // (120 * 4)
static int totalThreshold   = 720; // (180 * 4)
static int releaseThreshold = 320; // (80 * 4)

// ── Calibration Output Variables ──────────────────────────────────────────────
static int calibMinX = 10;
static int calibMaxX = 116;
static int calibMinY = 10;
static int calibMaxY = 116;
static int velocityMax = 4800; // (1200 * 4)

static float baselineValues[numRows][numCols];
static int   pressureValues[numRows][numCols];
static float lpfOut[numRows][numCols];

static const float LPF_BETA = 0.55f;
static const int   settlingDelay    = 300;    // Verified 300us for RC settling
static const float recoveryRateFast = 0.30f;
static const float recoveryRateSlow = 0.05f;

static const float    coordAlpha    = 0.35f;
static const float    velocityAlpha = 0.30f;
static const uint32_t holdTimeMs    = 40;

static uint32_t lastPressTime  = 0;
static bool     wasPressed     = false;
static float    smoothX        = 63.0f;
static float    smoothY        = 63.0f;
static float    smoothVelocity =  0.0f;

static bool     noteActive    = false;
static int      lastPitchBend =  0;
static int      lastCC74      =  0;
static uint32_t lastMsgTime   =  0;

static inline int clamp(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline int mapRange(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void delayUs(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) {}
}

// ── MPE Helpers ───────────────────────────────────────────────────────────────
static void mpeNoteOn(uint8_t note, uint8_t vel) {
    uint8_t msg[3] = {(uint8_t)(0x90 | (MPE_CHANNEL & 0x0F)), note, vel};
    midi.SendMessage(msg, 3);
}

static void mpeNoteOff(uint8_t note) {
    uint8_t msg[3] = {(uint8_t)(0x80 | (MPE_CHANNEL & 0x0F)), note, 0};
    midi.SendMessage(msg, 3);
}

static void mpePressure(uint8_t pressure) {
    uint8_t msg[2] = {(uint8_t)(0xD0 | (MPE_CHANNEL & 0x0F)), pressure};
    midi.SendMessage(msg, 2);
}

static void mpeCC74(uint8_t value) {
    uint8_t msg[3] = {(uint8_t)(0xB0 | (MPE_CHANNEL & 0x0F)), 74, value};
    midi.SendMessage(msg, 3);
}

static void mpePitchBend(int value) {
    value = clamp(value/2, -8192, 8191);
    uint16_t raw14 = (uint16_t)(value + 8192);
    uint8_t msg[3] = {(uint8_t)(0xE0 | (MPE_CHANNEL & 0x0F)), (uint8_t)(raw14 & 0x7F), (uint8_t)((raw14 >> 7) & 0x7F)};
    midi.SendMessage(msg, 3);
}

static int xToPitchBend(int x) {
    float norm = (float)(x - 63) / 63.0f;
    return (int)(norm * 8191.0f);
}

static int yToCC74(int y) {
    return clamp(y, 0, 127);
}

// ── Hardware Read (High-Z Row switching) ──────────────────────────────────────
static void readRow(int r, uint16_t raw[]) {
    // 1. Set row to OUTPUT and drive HIGH
    rowGpio[r].Init(rowPinDefs[r], GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);
    rowGpio[r].Write(true);
    
    // 2. Wait for RC charging curve to peak
    delayUs(settlingDelay); 
    
    for (int c = 0; c < numCols; c++) {
        raw[c] = hw.adc.Get(c) >> 4; // 16-bit to 12-bit
    }
    
    // 3. Set row back to INPUT (High-Z) to prevent cross-talk
    rowGpio[r].Init(rowPinDefs[r], GPIO::Mode::INPUT, GPIO::Pull::NOPULL);
    delayUs(20);
}

// ── Calibration Phase ─────────────────────────────────────────────────────────
static void captureStaticBaseline() {
    System::Delay(1000); 
    for (int r = 0; r < numRows; r++) {
        uint16_t raw[numCols];
        readRow(r, raw);
        for (int c = 0; c < numCols; c++) {
            baselineValues[r][c] = (float)raw[c];
            lpfOut[r][c]         = (float)raw[c];
        }
    }
    hw.SetLed(true);
    System::Delay(1000);
    hw.SetLed(false);
}

static void scanMatrixRaw() {
    for (int r = 0; r < numRows; r++) {
        uint16_t raw[numCols];
        readRow(r, raw);
        for (int c = 0; c < numCols; c++) {
            int diff = (int)raw[c] - (int)baselineValues[r][c];
            pressureValues[r][c] = (diff > 0) ? diff : 0;
        }
    }
}

static void interactiveCalibration() {
    bool zoneCalibrated[5] = {0};
    int  zoneMaxForce[5] = {0}, zonePeakX[5] = {0}, zonePeakY[5] = {0};
    int  calibratedCount = 0;
    bool isPressing = false;
    int  currentPeak = 0, peakCoordX = -1, peakCoordY = -1;
    
    const int CALIB_TRIGGER_THRESH = 100; // Minimal 12-bit detection threshold

    while (calibratedCount < 5) {
        scanMatrixRaw();
        int maxCellVal = 0;
        for (int r = 0; r < numRows; r++) {
            for (int c = 0; c < numCols; c++) {
                if (pressureValues[r][c] > maxCellVal) maxCellVal = pressureValues[r][c];
            }
        }

        if (maxCellVal > CALIB_TRIGGER_THRESH) {
            isPressing = true;
            hw.SetLed(true);
            if (maxCellVal > currentPeak) {
                currentPeak = maxCellVal;
                long wX = 0, wY = 0, tW = 0;
                for (int r = 0; r < numRows; r++) {
                    for (int c = 0; c < numCols; c++) {
                        int w = pressureValues[r][c];
                        wX += (long)w * (c * 63); wY += (long)w * (r * 63); tW += w;
                    }
                }
                if (tW > 0) { peakCoordX = (int)(wX / tW); peakCoordY = (int)(wY / tW); }
            }
        } else if (isPressing && maxCellVal < (CALIB_TRIGGER_THRESH / 2)) {
            isPressing = false;
            hw.SetLed(false);
            int zoneIndex = -1;
            if (peakCoordX >= 40 && peakCoordX <= 86 && peakCoordY >= 40 && peakCoordY <= 86) zoneIndex = 2;
            else {
                if (peakCoordY < 63) zoneIndex = (peakCoordX < 63) ? 0 : 1;
                else                 zoneIndex = (peakCoordX < 63) ? 3 : 4;
            }
            if (!zoneCalibrated[zoneIndex]) {
                zoneCalibrated[zoneIndex] = true;
                zoneMaxForce[zoneIndex] = currentPeak; zonePeakX[zoneIndex] = peakCoordX; zonePeakY[zoneIndex] = peakCoordY;
                calibratedCount++;
            }
            currentPeak = 0; peakCoordX = -1; peakCoordY = -1;
            System::Delay(400); 
        }
        System::Delay(10);
    }

    long sumMax = 0;
    for (int i = 0; i < 5; i++) sumMax += zoneMaxForce[i];
    velocityMax = (int)((sumMax / 5) * 0.9f); 

    // Auto-calculate thresholds relative to calibrated range
    pressThreshold   = (int)(velocityMax * 0.5f);
    releaseThreshold = (int)(pressThreshold * 0.6f);
    noiseThreshold   = (int)(pressThreshold * 0.30f);
    totalThreshold   = (int)(pressThreshold * 2.0f);

    calibMinX = (zonePeakX[0] + zonePeakX[3]) / 2 + 2; calibMaxX = (zonePeakX[1] + zonePeakX[4]) / 2 - 2;
    calibMinY = (zonePeakY[0] + zonePeakY[1]) / 2 + 2; calibMaxY = (zonePeakY[3] + zonePeakY[4]) / 2 - 2;

    System::Delay(500); hw.SetLed(true); System::Delay(1000); hw.SetLed(false);
}

// ── Main Execution ────────────────────────────────────────────────────────────
static void scanMatrix() {
    for (int r = 0; r < numRows; r++) {
        uint16_t raw[numCols];
        readRow(r, raw);
        for (int c = 0; c < numCols; c++) {
            float xn = (float)raw[c];
            lpfOut[r][c] = LPF_BETA * xn + (1.0f - LPF_BETA) * lpfOut[r][c];
            int diff = (int)lpfOut[r][c] - (int)baselineValues[r][c];
            if (diff >= noiseThreshold) pressureValues[r][c] = diff;
            else {
                pressureValues[r][c] = 0;
                if (xn < baselineValues[r][c]) baselineValues[r][c] = baselineValues[r][c] * (1.0f - recoveryRateFast) + xn * recoveryRateFast;
                else baselineValues[r][c] = baselineValues[r][c] * (1.0f - recoveryRateSlow) + xn * recoveryRateSlow;
            }
        }
    }
}

static bool findWeightedPressLocation(int &x, int &y, int &velocity) {
    long wX = 0, wY = 0, tW = 0;
    int maxC = 0;
    for (int r = 0; r < numRows; r++) {
        for (int c = 0; c < numCols; c++) {
            int w = pressureValues[r][c];
            if (w > maxC) maxC = w;
            wX += (long)w * ((c * 126) / (numCols - 1));
            wY += (long)w * ((r * 126) / (numRows - 1));
            tW += w;
        }
    }
    int aPress = wasPressed ? releaseThreshold : pressThreshold;
    int aTotal = wasPressed ? (totalThreshold / 2) : totalThreshold;
    if (maxC < aPress || tW < aTotal) return false;

    x = clamp(mapRange((int)(wX / tW), calibMinX, calibMaxX, 0, 126), 0, 126);
    y = clamp(mapRange((int)(wY / tW), calibMinY, calibMaxY, 0, 127), 0, 127);
    velocity = clamp(mapRange(clamp(maxC, aPress, velocityMax), aPress, velocityMax, 1, 127), 1, 127);
    return true;
}

int main(void) {
    hw.Init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CYCCNT = 0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    for (int r = 0; r < numRows; r++) rowGpio[r].Init(rowPinDefs[r], GPIO::Mode::INPUT, GPIO::Pull::NOPULL);

    AdcChannelConfig adc_cfg[numCols];
    for (int c = 0; c < numCols; c++) adc_cfg[c].InitSingle(colPinDefs[c]);
    hw.adc.Init(adc_cfg, numCols);
    hw.adc.Start();

    MidiUsbHandler::Config midi_cfg; midi.Init(midi_cfg);
    System::Delay(200);

    captureStaticBaseline();
    interactiveCalibration();

    while (true) {
        midi.Listen();
        scanMatrix();
        int x, y, velocity;
        bool pressed = findWeightedPressLocation(x, y, velocity);
        uint32_t now = System::GetNow();

        if (pressed) {
            smoothX = coordAlpha * x + (1.0f - coordAlpha) * smoothX;
            smoothY = coordAlpha * y + (1.0f - coordAlpha) * smoothY;
            smoothVelocity = velocityAlpha * velocity + (1.0f - velocityAlpha) * smoothVelocity;
            int sx = (int)(smoothX + 0.5f), sy = (int)(smoothY + 0.5f), vel = clamp((int)(smoothVelocity + 0.5f), 1, 127);

            if (!wasPressed) { if (noteActive) mpeNoteOff(MPE_NOTE); mpeNoteOn(MPE_NOTE, (uint8_t)vel); noteActive = true; }
            int pbRaw = xToPitchBend(sx) + 8192;
            if (abs(pbRaw - lastPitchBend) >= PB_DEADBAND) { mpePitchBend(pbRaw - 8192); lastPitchBend = pbRaw; }
            if (now - lastMsgTime >= MPE_MSG_INTERVAL_MS) {
                int c74 = yToCC74(sy);
                if (abs(c74 - lastCC74) >= CC74_DEADBAND) { mpeCC74((uint8_t)c74); lastCC74 = c74; }
                mpePressure((uint8_t)vel); lastMsgTime = now;
            }
            lastPressTime = now; wasPressed = true;
        } else {
            if (wasPressed && (now - lastPressTime < holdTimeMs)) {
                if (now - lastMsgTime >= MPE_MSG_INTERVAL_MS) { mpePressure((uint8_t)clamp((int)(smoothVelocity + 0.5f), 0, 127)); lastMsgTime = now; }
            } else if (wasPressed) {
                if (noteActive) { mpeNoteOff(MPE_NOTE); noteActive = false; }
                mpePressure(0); mpePitchBend(0); mpeCC74(0); lastPitchBend = 8192; lastCC74 = 0;
                wasPressed = false; smoothX = 63.0f; smoothY = 63.0f; smoothVelocity = 0.0f;
            }
        }
        System::Delay(15);
    }
}