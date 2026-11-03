// Headless MLX90640 adapter. The calibration/math implementation lives in vendor/.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

void thermal_i2c_close();
void thermal_i2c_pause();
static paramsMLX90640 calibration;
static bool initialized = false;
static bool first_frame_pending = true;
static float sensor_emissivity = .95f;
static const uint8_t address = 0x33;
using Clock = std::chrono::steady_clock;

// The supplied MLX90640_GetFrameData busy-waits indefinitely for data-ready.
// This equivalent acquisition sequence has a deadline and sleeps while waiting;
// the vendored calibration and temperature functions remain unchanged.
static int bounded_subpage(uint16_t *frame, Clock::time_point deadline) {
    for (int attempt = 0; attempt < 5; ++attempt) {
        uint16_t status = 0;
        do {
            if (Clock::now() >= deadline) return -20;
            if (MLX90640_I2CRead(address, 0x8000, 1, &status)) return -21;
            if (!(status & 8)) thermal_i2c_pause();
        } while (!(status & 8));
        if (MLX90640_I2CWrite(address, 0x8000, 0x0030)) return -21;
        if (MLX90640_I2CRead(address, 0x0400, 832, frame)) return -21;
        if (MLX90640_I2CRead(address, 0x8000, 1, &status)) return -21;
        if (status & 8) continue;  // A subpage changed during the RAM transfer.
        if (MLX90640_I2CRead(address, 0x800D, 1, frame + 832)) return -21;
        frame[833] = status & 1;
        return frame[833];
    }
    return -22;
}

extern "C" int thermal_open(float emissivity) {
    initialized = false;
    first_frame_pending = true;
    thermal_i2c_close();
    if (!std::isfinite(emissivity) || emissivity < .1f || emissivity > 1.f) return -23;
    uint16_t eeprom[832];
    MLX90640_I2CInit();
    if (MLX90640_DumpEE(address, eeprom)) return -21;
    const int status = MLX90640_ExtractParameters(eeprom, &calibration);
    if (status) return status;
    if (MLX90640_SetChessMode(address) || MLX90640_SetResolution(address, 2) ||
            MLX90640_SetRefreshRate(address, 4)) return -21;  // 8 subpages/second
    sensor_emissivity = emissivity;
    initialized = true;
    return 0;
}

extern "C" int thermal_read_frame(float *temperatures) {
    if (!initialized || !temperatures) return -24;
    uint16_t frame[834];
    std::fill(temperatures, temperatures + 768, std::numeric_limits<float>::quiet_NaN());
    const auto deadline = Clock::now() + std::chrono::milliseconds(900);
    // The powered sensor may still hold a ready subpage from before arrival.
    // Clear that notification, discard the in-flight conversion, then collect
    // both chess subpages below. No pre-task half-frame can reach the alarm.
    if (first_frame_pending) {
        if (MLX90640_I2CWrite(address, 0x8000, 0x0030)) return -21;
        const int discarded = bounded_subpage(frame, deadline);
        if (discarded < 0) return discarded;
    }
    unsigned pages = 0;
    // A complete 32x24 image requires two distinct chess subpages. Never mix
    // a previous frame with a new half-frame, including immediately after boot.
    for (int attempt = 0; attempt < 8 && pages != 3; ++attempt) {
        const int page = bounded_subpage(frame, deadline);
        if (page < 0) return page;
        const float ambient = MLX90640_GetTa(frame, &calibration);
        if (!std::isfinite(ambient)) return -25;
        MLX90640_CalculateTo(frame, &calibration, sensor_emissivity, ambient - 8.f, temperatures);
        pages |= 1u << page;
    }
    if (pages != 3) return -20;
    MLX90640_BadPixelsCorrection(calibration.brokenPixels, temperatures, 1, &calibration);
    MLX90640_BadPixelsCorrection(calibration.outlierPixels, temperatures, 1, &calibration);
    for (unsigned pixel = 0; pixel < 768; ++pixel) {
        const float temperature = temperatures[pixel];
        if (!std::isfinite(temperature) || temperature < -40.f || temperature > 300.f) return -25;
    }
    first_frame_pending = false;
    return 0;
}

extern "C" int thermal_read_max(float *maximum) {
    if (!maximum) return -24;
    float temperatures[768];
    const int result = thermal_read_frame(temperatures);
    if (result) return result;
    *maximum = *std::max_element(temperatures, temperatures + 768);
    return 0;
}

extern "C" void thermal_close() {
    initialized = false;
    thermal_i2c_close();
}
