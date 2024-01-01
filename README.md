# STM32 Driver For BMI088
This is a simple SPI driver for the [Bosch BMI088](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/) IMU. It was originally created for the ongoing [ModuLog Project](https://github.com/Aldicodi/ModuLog-project).

Features include:
* Support for data readout on both accelerometer and gyro including unit conversion.
* Support for first-in, first-out (FIFO) readout and configuration for both sensors.
* Support for modifying most settings, including modifying builtin low-pass filters.
* Ability to perform builtin self-tests for both sensors.

Missing features include:
* Interrupt configuration support.
* Parsing of interrupt data in FIFO streams.
* I2C support.

## How to use
1. Modify lines 9 & 10 of `Accel.h` and `Gyro.h` to refelt your GPIO chip select ports/pins.
```c
// Set port and pin for accelerometer here
#define ACCEL_CS_PORT YOUR_PORT_HERE
#define ACCEL_CS_PIN YOUR_PIN_HERE
```

2. Pass your SPI handler struct into `IMU_INIT`, if you are using both sensors, or into `ACCEL/GYRO_INIT` if you are just using one.

3. Call `IMU_ENABLE_ALL` or respective gyro/accel enabling functions (see internals of `IMU_ENABLE_ALL` in `IMU.c`).

4. (Optional) perform self-tests.

5. Enjoy!