
#include "Gyro.h"
#include <math.h>


// GPIO connectivity
#define PORT GYRO_CS_PORT
#define PIN GYRO_CS_PIN
#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET

// Address space
#define ADDR_CHIP_ID 0x00
#define ADDR_RATE_X_LSB 0x02
#define ADDR_RATE_X_MSB 0x03
#define ADDR_RATE_Y_LSB 0x04
#define ADDR_RATE_Y_MSB 0x05
#define ADDR_RATE_Z_LSB 0x06
#define ADDR_RATE_Z_MSB 0x07
#define ADDR_INT_STAT_1 0x0A
#define ADDR_FIFO_STATUS 0x0E
#define ADDR_RANGE 0x0F
#define ADDR_BANDWIDTH 0x10
#define ADDR_LPM1 0x11
#define ADDR_SOFTRESET 0x14
#define ADDR_INT_CTRL 0x15
#define ADDR_INT3_INT4_IO_CONF 0x16
#define ADDR_INT3_INT4_IO_MAP 0x18
#define ADDR_FIFO_WM_EN 0x1E
#define ADDR_FIFO_EXT_INT_S 0x34
#define ADDR_GYRO_SELF_TEST 0x3C
#define ADDR_FIFO_CONFIG_0 0x3D
#define ADDR_FIFO_CONFIG_1 0x3E
#define ADDR_FIFO_DATA 0x3F

// Rate stuff
#define MAX_2K_TO_RADS ((M_PI * 2000.0 / 180.0) / 0x7FFF.0p0)
#define MAX_1K_TO_RADS ((M_PI * 1000.0 / 180.0) / 0x7FFF.0p0)
#define MAX_500_TO_RADS ((M_PI * 5000.0 / 180.0) / 0x7FFF.0p0)
#define MAX_250_TO_RADS ((M_PI * 250.0 / 180.0) / 0x7FFF.0p0)
#define MAX_125_TO_RADS ((M_PI * 125.0 / 180.0) / 0x7FFF.0p0)

// FIFO
#define FIFO_FRAME_SIZE 6
#define FIFO_MAX_FRAMES 100
#define FIFO_MAX_BYTES (FIFO_FRAME_SIZE * FIFO_MAX_FRAMES)


// Other logic
#define READ 0x80
#define WRITE 0x00

static SPI_HandleTypeDef* gyro_hspi;
static uint8_t range;
static uint8_t odr;

// Infrastructure declarations
static void select();
static void unselect();
static void readAddr(uint8_t, uint8_t*, int);
static void writeAddr(uint8_t, uint8_t);

static Vector3 parseRawUInts(uint8_t*);

// Forward-facing logic

void GYRO_INIT(SPI_HandleTypeDef* spiHandler){
    unselect();
    gyro_hspi = spiHandler;
    GYRO_RELOAD_SETTINGS();
}

void GYRO_GOOD_SETTINGS(){
    GYRO_SET_RANGE(GYRO_RANGE_DPS_1K);
    GYRO_SET_OUPUT_DATA_RATE(GYRO_ODR_1K__BW_116);
    GYRO_SET_FIFO_MODE(GYRO_FIFO_STREAM);
    GYRO_RELOAD_SETTINGS();
}

// Read functions

uint8_t GYRO_READ_ID(){
    uint8_t id = 0;
    select();

    readAddr(ADDR_CHIP_ID, &id, 1);

    unselect();
    return id;
}

Vector3 GYRO_READ_RATES(){
    uint8_t rawVals[6];
    select();
    readAddr(ADDR_RATE_X_LSB, rawVals, 6);
    unselect();

    return parseRawUInts(rawVals);
}

void GYRO_RELOAD_SETTINGS(){
    uint8_t rawVals[2];
    select();
    readAddr(ADDR_RANGE, rawVals, 2);
    unselect();

    range = rawVals[0];
    odr = rawVals[1] & 0b01111111; // Ignore bit 7
}   

uint8_t GYRO_SELF_TEST(){
    uint8_t result = 0;
    uint8_t count = 0;
    select();
    writeAddr(ADDR_GYRO_SELF_TEST, 0x01);// Set bit 0
    unselect();
    while(!(result & 0b00000010) && count < 10){ // Bit 1 must be 1 to exit
        HAL_Delay(100);
        select();
        readAddr(ADDR_GYRO_SELF_TEST, &result, 1);
        unselect();
        count++;
    }
    // if test completed
    if(result & 0b00000010){
        return !(result & 0b00000100); // Essentail return NOT failed.
        // (bit 2 is 1 if test failed)
    } else { // count reached 10 & test did not complete. Assume fail
        return 0;
    }
}


GyroDataBuffer GYRO_READ_FIFO(){
    uint8_t rawBuff[FIFO_MAX_BYTES] = {0};
    int i = 0;
    GyroDataBuffer out;
    out.len = 0;
    out.array = malloc(sizeof(Vector3)*(FIFO_MAX_FRAMES));

    select();
    readAddr(ADDR_FIFO_DATA, rawBuff, FIFO_MAX_BYTES);
    unselect();    
    
    // Looks ugly but I want the check to be thorough and most of the time it'll short-circut
    while(i < FIFO_MAX_BYTES && !(rawBuff[i]==0 && rawBuff[i+1]==128 
                                && rawBuff[i+2]==0 && rawBuff[i+3]==128 
                                && rawBuff[i+4]==0 && rawBuff[i+5]==128)){
        out.array[out.len] = parseRawUInts(rawBuff + i);
        out.len++;
        i += 6;
    }

    out.array = realloc(out.array, out.len * sizeof(Vector3));
    return out;
}

// Write functions

void GYRO_SET_POWERMODE(uint8_t gyroPowermode){
    select();
    writeAddr(ADDR_LPM1, gyroPowermode);
    unselect();
}
void GYRO_SET_RANGE(uint8_t gyroRange){
    select();
    range = gyroRange;
    writeAddr(ADDR_RANGE, gyroRange);
    unselect();
}
void GYRO_SET_OUPUT_DATA_RATE(uint8_t gyroODR){
    select();
    writeAddr(ADDR_BANDWIDTH, gyroODR);
    unselect();
}
void GYRO_SET_FIFO_MODE(uint8_t gyroFIFOMode){
    select();
    writeAddr(ADDR_FIFO_CONFIG_1, gyroFIFOMode);
    unselect();
}

// Infrastructure definitions
static void select(){
    HAL_GPIO_WritePin(PORT, PIN, LOW);
}

static void unselect(){
    HAL_GPIO_WritePin(PORT, PIN, HIGH);
}

static void readAddr(uint8_t addr, uint8_t* outBuff, int outBytes){
    uint8_t address = READ | addr;

    HAL_SPI_Transmit(gyro_hspi, &address, 1, 100);
    HAL_SPI_Receive(gyro_hspi, outBuff, outBytes, 100);
}

static void writeAddr(uint8_t addr, uint8_t data){
    uint8_t message[] = {WRITE|addr, data};
    HAL_SPI_Transmit(gyro_hspi, message, 2, 100);
}

// Converts raw values to radians per second
static Vector3 parseRawUInts(uint8_t* rawVals){
    Vector3 out;
    // Int casts nescessary for two's complement
    out.x = (int16_t)(rawVals[1]*256 + rawVals[0]);
    out.y = (int16_t)(rawVals[3]*256 + rawVals[2]);
    out.z = (int16_t)(rawVals[5]*256 + rawVals[4]);

    // Possible optimization: bitshift binary representation of 125 (could replace switch. Would need to remove defines)

    // Convert units to rad/s
    switch (range)
    {
    case GYRO_RANGE_DPS_2K:
        V_MUL(out, MAX_2K_TO_RADS);
        break;
    
    case GYRO_RANGE_DPS_1K:
        V_MUL(out, MAX_1K_TO_RADS);
        break;
    
    case GYRO_RANGE_DPS_500:
        V_MUL(out, MAX_500_TO_RADS);
        break;
    
    case GYRO_RANGE_DPS_250:
        V_MUL(out, MAX_250_TO_RADS);
        break;
    
    case GYRO_RANGE_DPS_125:
        V_MUL(out, MAX_125_TO_RADS);
        break;
    
    default:
        out.x *= 0;
        out.y *= 0;
        out.z *= 0;
        break;
    }
    
    return out;
}
