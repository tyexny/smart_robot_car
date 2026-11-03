CXX ?= g++
CXXFLAGS ?= -O2 -std=c++11 -Wall -Wextra -fPIC

all: libthermal_mlx90640.so

libthermal_mlx90640.so: thermal_sensor.cpp fpga_i2c.cpp vendor/MLX90640_API.cpp vendor/MLX90640_API.h vendor/MLX90640_I2C_Driver.h
	$(CXX) $(CXXFLAGS) -shared -Ivendor thermal_sensor.cpp fpga_i2c.cpp vendor/MLX90640_API.cpp -o $@ -lm -pthread

.PHONY: all
