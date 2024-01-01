#ifndef __BMI088_GYRO 
#define __BMI088_GYRO 

#include "main.h"
#include "Vectors.h"
#include <stdlib.h>

// Set port and pin for Gyro here
#define GYRO_CS_PORT CSG_GPIO_Port
#define GYRO_CS_PIN CSG_Pin

typedef struct gyroDataBuffer
{
    uint8_t len; // How many data frames there are
    Vector3* array; // Rate data in rad/s
} GyroDataBuffer;

// Constants
// Power mode for gyro
#define GYRO_PWR_DEEP_SUSPND 0x20
#define GYRO_PWR_SUSPND 0x80
#define GYRO_PWR_NORMAL 0x00
// Range values for gyro
#define GYRO_RANGE_DPS_2K  0x00
#define GYRO_RANGE_DPS_1K  0x01
#define GYRO_RANGE_DPS_500 0x02
#define GYRO_RANGE_DPS_250 0x03
#define GYRO_RANGE_DPS_125 0x04
// Gyro output data rates + low-pass filter bandwidths
// All units are in Hz
#define GYRO_ODR_2K__BW_532 0x00
#define GYRO_ODR_2K__BW_230 0x01
#define GYRO_ODR_1K__BW_116 0x02
#define GYRO_ODR_400__BW_47 0x03
#define GYRO_ODR_200__BW_23 0x04
#define GYRO_ODR_100__BW_12 0x05
#define GYRO_ODR_200__BW_64 0x06
#define GYRO_ODR_100__BW_32 0x07
// FIFO
#define GYRO_FIFO_DISABLED 0x00
#define GYRO_FIFO_STOP_AT_FULL 0x40
#define GYRO_FIFO_STREAM 0x80

void GYRO_INIT(SPI_HandleTypeDef* spiHandler);
void GYRO_GOOD_SETTINGS();
// Performs the self-test procedure. Takes > 150ms
//  returns 1 for sucess, 0, for failure
uint8_t GYRO_SELF_TEST();

// Reads all settings from gyroscope into memory
void GYRO_RELOAD_SETTINGS();

//    Read functions
uint8_t GYRO_READ_ID();

Vector3 GYRO_READ_RATES();

GyroDataBuffer GYRO_READ_FIFO();

//     Write functions

void GYRO_SET_POWERMODE(uint8_t gyroPowermode);
void GYRO_SET_RANGE(uint8_t gyroRange);
void GYRO_SET_OUPUT_DATA_RATE(uint8_t gyroODR);
void GYRO_SET_FIFO_MODE(uint8_t gyroFIFOMode);

#endif