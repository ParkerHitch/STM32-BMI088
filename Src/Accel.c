
#include "Accel.h"
#include <stdlib.h>

// GPIO connectivity
#define PORT ACCEL_CS_PORT
#define PIN ACCEL_CS_PIN

#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET

// Address space
#define ADDR_CHIP_ID 0x00
#define ADDR_ERR_REG 0x02
#define ADDR_STATUS 0x03
#define ADDR_ACC_X_LSB 0x12
#define ADDR_ACC_X_MSB 0x13
#define ADDR_ACC_Y_LSB 0x14
#define ADDR_ACC_Y_MSB 0x15
#define ADDR_ACC_Z_LSB 0x16
#define ADDR_ACC_Z_MSB 0x17
#define ADDR_SENSORTIME_0 0x18
#define ADDR_SENSORTIME_1 0x19
#define ADDR_SENSORTIME_2 0x1A
#define ADDR_INT_STAT_1 0x1D
#define ADDR_TEMP_MSB 0x22
#define ADDR_TEMP_LSB 0x23
#define ADDR_FIFO_LENGTH_0 0x24
#define ADDR_FIFO_LENGTH_1 0x25
#define ADDR_FIFO_DATA 0x26
#define ADDR_ACC_CONF 0x40
#define ADDR_ACC_RANGE 0x41
#define ADDR_FIFO_DOWNS 0x45
#define ADDR_FIFO_WTM_0 0x46
#define ADDR_FIFO_WTM_1 0x47
#define ADDR_FIFO_CONFIG_0 0x48
#define ADDR_FIFO_CONFIG_1 0x49
#define ADDR_INT1_IO_CTRL 0x53
#define ADDR_INT2_IO_CTRL 0x54
#define ADDR_INT_MAP_DATA 0x58
#define ADDR_ACC_SELF_TEST 0x6D
#define ADDR_ACC_PWR_CONF 0x7C
#define ADDR_ACC_PWR_CTRL 0x7D
#define ADDR_ACC_SOFTRESET 0x7E

// Sensor attributes
#define FIFO_MAX_BUFFER_BYTES 1024
#define FIFO_DATA_FRAME_SIZE_BYTES 7

// Other logic
#define READ 0x80
#define WRITE 0x00

static SPI_HandleTypeDef* a_hspi;
static uint8_t a_maxRangeBits;
static double a_maxRangeReal;
static uint8_t a_bwp;
static uint8_t a_odr;

// Infrastructure

// typedef union unionInt16{
//     struct bytes
//     {
//         uint8_t MSB;
//         uint8_t LSB;
//     };
//     bytes[2];
//     uint16_t noSign;
//     int16_t sign;
// } unionInt16;
static void select();
static void unselect();
static void readAddr(uint8_t, uint8_t*, int);
static void writeAddr(uint8_t, uint8_t);
static void setRangeMem(uint8_t);

static Vector3 parseRawUInts(uint8_t*);

// Forward-facing logic

void ACCEL_INIT(SPI_HandleTypeDef* spiHandler){
    unselect();
    a_hspi = spiHandler;
    ACCEL_READ_ID(); // Dummy read to make sure everything else works
    ACCEL_RELOAD_SETTINGS();
}

void ACCEL_GOOD_SETTINGS(){
    ACCEL_SET_RANGE(ACCEL_RANGE_24G);
    ACCEL_SET_CONFIG(ACCEL_OSR_NORMAL, ACCEL_ODR_400);
    ACCEL_WRITE_FIFO_DOWNSAMP(ACCEL_FIFO_DOWNSAMP_NONE);
    ACCEL_WRITE_FIFO_MODE(ACCEL_FIFO_MODE_STREAM);
    ACCEL_WRITE_FIFO_ENABLED(ACCEL_FIFO_ENABLED);
}

uint8_t ACCEL_SELF_TEST(){
    Vector3 positive;
    Vector3 negative;
    Vector3 difference;
    uint8_t isGood = 1;

    // Setup
    ACCEL_SET_RANGE(ACCEL_RANGE_24G);
    ACCEL_SET_CONFIG(ACCEL_OSR_NORMAL, ACCEL_ODR_1600);
    // ACCEL_RELOAD_SETTINGS();
    // positive = ACCEL_READ_ACCELERATION();
    HAL_Delay(5);
    // Positive
    select();
    writeAddr(ADDR_ACC_SELF_TEST, 0x0D);
    unselect();
    HAL_Delay(55);
    positive = ACCEL_READ_ACCELERATION();
    // Negative
    select();
    writeAddr(ADDR_ACC_SELF_TEST, 0x09);
    unselect();
    HAL_Delay(55);
    negative = ACCEL_READ_ACCELERATION();
    // Reset
    select();
    writeAddr(ADDR_ACC_SELF_TEST, 0x00);
    unselect();
    HAL_Delay(55);
    // Calculate results
    difference = vSub(positive, negative);
    isGood = (difference.x >= GRAV) &&
                (difference.y >= GRAV) &&
                (difference.z >= GRAV/2);

    return isGood;
}

void ACCEL_RELOAD_SETTINGS(){
    uint8_t rawData[2];
    select();
    readAddr(ADDR_ACC_CONF, rawData, 2);
    unselect();

    a_bwp = rawData[0] >> 4; // First 4
    a_odr = rawData[0] & 0b00001111; // Last 4

    rawData[1] = rawData[1] & 0b00000011; // Last 2

    setRangeMem(rawData[1]);
}

uint8_t ACCEL_READ_ID(){
    uint8_t id = 0;
    select();

    readAddr(ADDR_CHIP_ID, &id, 1);

    unselect();
    return id;
}

Vector3 ACCEL_READ_ACCELERATION(){
    uint8_t rawVals[6];
    select();
    readAddr(ADDR_ACC_X_LSB, rawVals, 6);
    unselect();

    return parseRawUInts(rawVals);
}

float ACCEL_READ_TEMPERATURE(){
    uint8_t rawVals[2];
    int16_t rawVal;
    select();
    readAddr(ADDR_TEMP_MSB, rawVals, 2);
    unselect();
    rawVal = (rawVals[0] << 3) | (rawVals[1] >> 5);
    // since it's an 11 bit number first bit is the negative twos compliment one
    rawVal = rawVal > 1023 ? rawVal - 2048 : rawVal;
    return 23 + (((float)rawVal) * 0.125);
}

uint32_t ACCEL_READ_SENSORTIME(){
    uint8_t vals[3];
    uint32_t val1, val2, val3;
    uint32_t val;
    select();
    readAddr(ADDR_SENSORTIME_0, vals, 3);
    unselect();
    val1 = vals[0];
    val2 = vals[1]<<8;
    val3 = vals[2]<<16;
    val = val1 | val2 | val3;
    return val;
}

AccelError ACCEL_READ_ERROR(){
    uint8_t val;
    AccelError out;
    select();
    readAddr(ADDR_ERR_REG, &val, 1);
    unselect();
    
    out.isFatal = (val & 1);
    out.errorCode = (val & 0b00011100) >> 2;

    return out;
}

uint8_t ACCEL_READ_PWR_MODE(){
    uint8_t mode;
    select();
    readAddr(ADDR_ACC_PWR_CONF, &mode, 1);
    unselect();

    return mode;
}

uint8_t ACCEL_READ_ACCEL_ENABLED(){
    uint8_t enabled;
    select();
    readAddr(ADDR_ACC_PWR_CTRL, &enabled, 1);
    unselect();
    
    return enabled;
}

// Write functions
void ACCEL_SET_CONFIG(uint8_t oversamplingRate, uint8_t outputDataRate){
    uint8_t message = (oversamplingRate << 4) | outputDataRate;
    select();
    writeAddr(ADDR_ACC_CONF, message);
    unselect();
    a_bwp = oversamplingRate;
    a_odr = outputDataRate;
}
void ACCEL_SET_RANGE(uint8_t range){
    select();
    writeAddr(ADDR_ACC_RANGE, range);
    unselect();
    setRangeMem(range);
}

void ACCEL_WRITE_PWR_ACTIVATE(){
    select();
    writeAddr(ADDR_ACC_PWR_CONF, ACCEL_PWR_ACTIVE);
    unselect();
}
void ACCEL_WRITE_PWR_SUSPEND(){
    select();
    writeAddr(ADDR_ACC_PWR_CONF, ACCEL_PWR_SUSPEND);
    unselect();
}
void ACCEL_WRITE_ACCEL_ENABLE(){
    select();
    writeAddr(ADDR_ACC_PWR_CTRL, ACCEL_ACCEL_ENABLED);
    unselect();
}
void ACCEL_WRITE_ACCEL_DISABLE(){
    select();
    writeAddr(ADDR_ACC_PWR_CTRL, ACCEL_ACCEL_DISABLED);
    unselect();
}

// FIFO

uint16_t ACCEL_READ_FIFO_LEN(){
    uint8_t rawData[2];
    select();
    readAddr(ADDR_FIFO_LENGTH_0, rawData, 2);
    unselect();

    rawData[1] &= 00111111;

    return (rawData[1]<<8) | rawData[0];
}


#define FIFO_FRAME_H_END 0x80
#define FIFO_FRAME_H_DATA 0x84
#define FIFO_FRAME_H_SKIP 0x40
#define FIFO_FRAME_H_SENSORTIME 0x44
#define FIFO_FRAME_H_CONFIG 0x48
#define FIFO_FRAME_H_DROP 0x50
AccelDataBuffer ACCEL_READ_FIFO(){
    uint8_t rawBuff[FIFO_MAX_BUFFER_BYTES] = {0};
    uint8_t frameType;
    int i;
    AccelDataBuffer out;
    out.len = 0;
    out.array = malloc(sizeof(Vector3)*(FIFO_MAX_BUFFER_BYTES/FIFO_DATA_FRAME_SIZE_BYTES));
    
    select();
    readAddr(ADDR_FIFO_DATA, rawBuff, FIFO_MAX_BUFFER_BYTES);
    unselect();

    i = 0;
    frameType = rawBuff[0] & 0xFC; // Ignore last 2 bits

    // A skip frame should only be presented at the begining of the data, so we can check for it here
    if(frameType == FIFO_FRAME_H_SKIP){
        out.skipped = rawBuff[i+1];
        i += 2;
        frameType = rawBuff[i];
    } else {
        out.skipped = 0;
    }

    while(frameType != FIFO_FRAME_H_END){
        switch (frameType)
        {
        case FIFO_FRAME_H_DATA:
            out.array[out.len] = parseRawUInts(rawBuff + i + 1);
            out.len++;
            i += 7;
            break;
        case FIFO_FRAME_H_SENSORTIME:
            i += 4; // We don't really care.
            // This should be at the end of the data so we could technically break out of the while loop here but I don't want to.
            break;
        case FIFO_FRAME_H_CONFIG:
            i += 2; 
            // If this happens we are kinda screwed because we were using the new config to parse all the old data before this frame.
            // I would love to raise some sort of error here but I don't really know how
            break;
        case FIFO_FRAME_H_DROP:
            i += 2;
            // We have dropped a frame, so this could throw off timings
            // Need to communicate this to logic so that it can interpolate
            // Solution: Vector3 struct of NANs.
            out.array[out.len] = (Vector3) VECTOR_NULL;
            out.len++;
            break;
        default:
            // We are fucked...
            // Again I would love to raise an error but idk how.
            goto endLoop;
            break;
        }
        frameType = rawBuff[i] & 0xFC;
    }
    endLoop:
    out.array = realloc(out.array, out.len * sizeof(Vector3)); // Scale down to only nescessary memory
    return out;
}


void ACCEL_WRITE_FIFO_ENABLED(uint8_t enabled){
    select();
    writeAddr(ADDR_FIFO_CONFIG_1, enabled);
    unselect();
}

void ACCEL_WRITE_FIFO_MODE(uint8_t modeFIFO){
    select();
    writeAddr(ADDR_FIFO_CONFIG_0, modeFIFO);
    unselect();
}

void ACCEL_WRITE_FIFO_DOWNSAMP(uint8_t downsampFIFO){
    select();
    writeAddr(ADDR_FIFO_DOWNS, downsampFIFO);
    unselect();
}

// Infrastructure backend
static void select(){
    HAL_GPIO_WritePin(PORT, PIN, LOW);
}

static void unselect(){
    HAL_GPIO_WritePin(PORT, PIN, HIGH);
}

static void readAddr(uint8_t addr, uint8_t* outBuff, int outBytes){
    addr = READ | addr;
    // uint8_t tempBuff[outBytes + 1]; // need to read dummy byte
    
    HAL_SPI_Transmit(a_hspi, &addr, 1, 100);
    HAL_SPI_Receive(a_hspi, outBuff, 1, 100); // read dummy
    HAL_SPI_Receive(a_hspi, outBuff, outBytes, 100);

}

static void writeAddr(uint8_t addr, uint8_t data){
    uint8_t message[] = {WRITE|addr, data};
    HAL_SPI_Transmit(a_hspi, message, 2, 100);
}

static Vector3 parseRawUInts(uint8_t* rawVals){
    int32_t x, y, z;
    Vector3 out;
    // Int casts nescessary for two's complement
    x = ((int16_t)(rawVals[1]*256 + rawVals[0])) * (2 << a_maxRangeBits);
    y = ((int16_t)(rawVals[3]*256 + rawVals[2])) * (2 << a_maxRangeBits);
    z = ((int16_t)(rawVals[5]*256 + rawVals[4])) * (2 << a_maxRangeBits);

    // Convert units to real units (m/s^2 or ft/s^2)
    out.x = (x / 32768.0) * 1.5 * GRAV;
    out.z = (z / 32768.0) * 1.5 * GRAV;
    out.y = (y / 32768.0) * 1.5 * GRAV;
    
    return out;
}

static void setRangeMem(uint8_t range){
    a_maxRangeBits = range;
    switch (range)
    {
    case ACCEL_RANGE_3G:
        a_maxRangeReal = 3 * GRAV;
        break;
    case ACCEL_RANGE_6G:
        a_maxRangeReal = 6 * GRAV;
        break;
    case ACCEL_RANGE_12G:
        a_maxRangeReal = 12 * GRAV;
        break;
    case ACCEL_RANGE_24G:
        a_maxRangeReal = 24 * GRAV;
        break;
    default:
        a_maxRangeReal = 0;
        break;
    }
}