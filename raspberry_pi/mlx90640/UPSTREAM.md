# MLX90640 temperature acquisition

`vendor/MLX90640_API.cpp`, `vendor/MLX90640_API.h`,
`vendor/MLX90640_I2C_Driver.h` and `vendor/LICENSE` are copied unchanged from
the supplied `MLX90640资料包/MLX90640官方库` (Melexis, Apache-2.0).

`linux_i2c.cpp` replaces the demo transport with checked Linux I2C transfers,
unsigned byte decoding, a repeated START, and `/dev/i2c-1`. `thermal_sensor.cpp`
adds deadline-limited acquisition of both chess subpages, bad-pixel correction,
and validation of all 768 temperatures before returning their maximum.
The demo's architecture-specific `libmlx90640.so` is deliberately not used.

Default I2C address: `0x33`. Refresh register: `4` (8 subpages/s, about 4 full
images/s). Emissivity: `0.95`; reflected temperature: ambient minus 8°C, following
the example's reflected-temperature convention. Mount the sensor facing the same
direction as the visible camera. The alarm uses the whole thermal field of view;
it does not assume that a visible detection box is thermally registered.

Build on the target Raspberry Pi using `bash install_thermal.sh` from the parent
directory. No SDL, display server, Python package or precompiled ARM binary is
needed. The temperature reader runs in a supervised subprocess so a failing I2C
adapter cannot block task execution or the 5 Hz UDP status heartbeat.

`thermal_read_frame` exports all 768 validated, corrected temperatures for the
browser heatmap. `thermal_read_max` preserves its original ABI and uses that
same acquisition path. The Linux adapter holds an advisory device lock so a
standalone preview and a task monitor cannot interleave sensor subpages.
