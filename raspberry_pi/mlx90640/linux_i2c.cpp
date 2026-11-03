// Linux I2C1 backend for the unmodified Melexis API (Apache-2.0).
// Uses a repeated START, unsigned byte decoding and checked open/ioctl calls.
#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <unistd.h>
#include "MLX90640_I2C_Driver.h"

static int sensor_fd = -1;

static int ensure_open() {
    if (sensor_fd >= 0) return 0;
    sensor_fd = open("/dev/i2c-1", O_RDWR | O_CLOEXEC);
    if (sensor_fd < 0) return -1;
    // The task monitor and standalone viewer must never interleave subpages.
    if (flock(sensor_fd, LOCK_EX | LOCK_NB) != 0) {
        const int error = errno;
        close(sensor_fd);
        sensor_fd = -1;
        errno = error;
        return -1;
    }
    // Some adapters ignore these hints. The Python supervisor also has a
    // process-level timeout, covering adapters stuck inside a kernel ioctl.
    ioctl(sensor_fd, I2C_TIMEOUT, 10);  // 10 * 10 ms
    ioctl(sensor_fd, I2C_RETRIES, 1);
    return 0;
}

void MLX90640_I2CInit() { ensure_open(); }
void MLX90640_I2CFreqSet(int) {}  // Set dtparam=i2c_arm_baudrate in Pi boot config.
void thermal_i2c_pause() { usleep(2000); }

void thermal_i2c_close() {
    if (sensor_fd >= 0) close(sensor_fd);
    sensor_fd = -1;
}

int MLX90640_I2CRead(uint8_t address, uint16_t start, uint16_t words, uint16_t *data) {
    if (!data || !words || words > 832 || ensure_open() != 0) return -1;
    uint8_t command[2] = {uint8_t(start >> 8), uint8_t(start)};
    uint8_t bytes[1664];
    i2c_msg messages[2] = {
        {address, 0, 2, command},
        {address, I2C_M_RD, uint16_t(words * 2), bytes}
    };
    i2c_rdwr_ioctl_data transaction = {messages, 2};
    if (ioctl(sensor_fd, I2C_RDWR, &transaction) != 2) return -1;
    for (unsigned i = 0; i < words; ++i)
        data[i] = (uint16_t(bytes[2 * i]) << 8) | bytes[2 * i + 1];
    return 0;
}

int MLX90640_I2CWrite(uint8_t address, uint16_t target, uint16_t value) {
    if (ensure_open() != 0) return -1;
    uint8_t bytes[4] = {uint8_t(target >> 8), uint8_t(target),
                        uint8_t(value >> 8), uint8_t(value)};
    i2c_msg message = {address, 0, 4, bytes};
    i2c_rdwr_ioctl_data transaction = {&message, 1};
    return ioctl(sensor_fd, I2C_RDWR, &transaction) == 1 ? 0 : -1;
}
