#ifndef __BMI088_ACCEL 
#define __BMI088_ACCEL 

#include "main.h"
#include "Vectors.h"

// Set port and pin for Gyro here
#define ACCEL_CS_PORT CSA_GPIO_Port
#define ACCEL_CS_PIN CSA_Pin

typedef struct accelErrors
{
    uint8_t isFatal;
    uint8_t errorCode;
} AccelError;

typedef struct accelDataBuffer
{
    uint8_t skipped; // Number of frames skipped between end of last FIFO access and start of this one
    uint8_t len; // How many data frames there are
    Vector3* array; // Acceleration data in m/s^2
} AccelDataBuffer;

// Constants
// Power modes for entire chip
#define ACCEL_PWR_SUSPEND 0x03
#define ACCEL_PWR_ACTIVE 0x00
// Enable/disable just accelerometer
#define ACCEL_ACCEL_ENABLED 0x04
#define ACCEL_ACCEL_DISABLED 0x00
// Oversampling rates for accelerometer
#define ACCEL_OSR_NORMAL 0x0A // No oversampling
#define ACCEL_OSR_2 0x09 // Two-fold oversampling
#define ACCEL_OSR_4 0x09 // Four-fold oversampling
// Output data rates for accelerometer. All in Hz
#define ACCEL_ODR_12p5 0x05
#define ACCEL_ODR_25 0x06
#define ACCEL_ODR_50 0x07
#define ACCEL_ODR_100 0x08
#define ACCEL_ODR_200 0x09
#define ACCEL_ODR_400 0x0A
#define ACCEL_ODR_800 0x0B
#define ACCEL_ODR_1600 0x0C
// Accelerometer ranges
#define ACCEL_RANGE_3G 0x00
#define ACCEL_RANGE_6G 0x01
#define ACCEL_RANGE_12G 0x02
#define ACCEL_RANGE_24G 0x03
// FIFO
#define ACCEL_FIFO_ENABLED 0b01010000
#define ACCEL_FIFO_DISABLED 0b00010000
#define ACCEL_FIFO_MODE_STREAM 0x2
#define ACCEL_FIFO_MODE_STOP_AT_FULL 0x3
#define ACCEL_FIFO_DOWNSAMP_NONE 0x80
#define ACCEL_FIFO_DOWNSAMP_2x   0x90
#define ACCEL_FIFO_DOWNSAMP_4x   0xA0
#define ACCEL_FIFO_DOWNSAMP_8x   0xB0
#define ACCEL_FIFO_DOWNSAMP_16x  0xC0
#define ACCEL_FIFO_DOWNSAMP_32x  0xD0
#define ACCEL_FIFO_DOWNSAMP_64x  0xE0
#define ACCEL_FIFO_DOWNSAMP_128x 0xF0

// Swap for units to be m/s^2 or ft/s^2
#define GRAV 9.80665
// #define GRAV 32.1740

// All functions

// Basic initialization. Performs nescesarry dummy read
void ACCEL_INIT(SPI_HandleTypeDef* spiHandler);
void ACCEL_GOOD_SETTINGS();
// Performs the self-test procedure. Takes > 150ms
//  returns 1 for sucess, 0, for failure
uint8_t ACCEL_SELF_TEST();

// Reads all settings from accelerometer into memory
void ACCEL_RELOAD_SETTINGS();

//    Read functions
uint8_t ACCEL_READ_ID();

Vector3 ACCEL_READ_ACCELERATION();

float ACCEL_READ_TEMPERATURE();
uint32_t ACCEL_READ_SENSORTIME();

AccelError ACCEL_READ_ERROR_STATUS();

uint8_t ACCEL_READ_DATA_READY();

uint8_t ACCEL_READ_PWR_MODE();
uint8_t ACCEL_READ_ACCEL_ENABLED();


//     Write functions


void ACCEL_SET_CONFIG(uint8_t oversamplingRate, uint8_t outputDataRate);
void ACCEL_SET_RANGE(uint8_t range);

void ACCEL_WRITE_PWR_ACTIVATE();
void ACCEL_WRITE_PWR_SUSPEND();
void ACCEL_WRITE_ACCEL_ENABLE();
void ACCEL_WRITE_ACCEL_DISABLE();

// First In First Out

uint16_t ACCEL_READ_FIFO_LEN();
AccelDataBuffer ACCEL_READ_FIFO();

void ACCEL_WRITE_FIFO_ENABLED(uint8_t enabled);
void ACCEL_WRITE_FIFO_MODE(uint8_t modeFIFO);
void ACCEL_WRITE_FIFO_DOWNSAMP(uint8_t downsampFIFO);

#endif