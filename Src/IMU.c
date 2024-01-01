#include "IMU.h"

#include "Accel.h"
#include "Gyro.h"


void IMU_INIT(SPI_HandleTypeDef* spiHandle){
    ACCEL_INIT(spiHandle);
    GYRO_INIT(spiHandle);
}

void IMU_SETUP_FOR_LOGGING(){
    ACCEL_GOOD_SETTINGS();
    GYRO_GOOD_SETTINGS();
}

void IMU_ENABLE_ALL(){
    ACCEL_WRITE_PWR_ACTIVATE();
    ACCEL_WRITE_ACCEL_ENABLE();
    GYRO_SET_POWERMODE(GYRO_PWR_NORMAL);
    HAL_Delay(100);
}

int IMU_READY(){
    int ready = 0;
    if(!ACCEL_SELF_TEST()){
        ready = -1;
    }
    if(!GYRO_SELF_TEST()){
        ready -= 2;
    }
    return ready == 0 ? 1 : ready;
}