/** 
 * Author	: @makermatty (maker.matejsuchanek.cz)
 * Date		: 15-6-2020
 */

#ifdef ESP32

#ifndef MM_MPU6050_H
#define MM_MPU6050_H

#include <Arduino.h>
#include <Wire.h>

#include "MakerMatty_Curves.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MPU6050_ADDR 0x68
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1a
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_TEMP_H 0x41
#define MPU6050_TEMP_L 0x42

struct mpu6050_data /*__attribute__((packed))*/ {
    union {
        struct {
            union {
                struct {
                    int16_t accX;
                    int16_t accY;
                    int16_t accZ;
                };
                int16_t acc[3];
            };
            int16_t temp;
            union {
                struct {
                    int16_t gyroX;
                    int16_t gyroY;
                    int16_t gyroZ;
                };
                int16_t gyro[3];
            };
        };
        uint8_t raw[14];
    };
};


class MPU6050_Raw {
public:
    MPU6050_Raw(TwoWire& w);

    void begin(const int8_t sda = -1, const int8_t scl = -1, const uint32_t freq = 0U);
    void read();

    int16_t getRawAccX() { return rawData.accX; };
    int16_t getRawAccY() { return rawData.accY; };
    int16_t getRawAccZ() { return rawData.accZ; };

    int16_t* getRawAcc() { return rawData.acc; };

    int16_t getRawTemp() { return rawData.temp; };

    int16_t getRawGyroX() { return rawData.gyroX; };
    int16_t getRawGyroY() { return rawData.gyroY; };
    int16_t getRawGyroZ() { return rawData.gyroZ; };

    int16_t* getRawGyro() { return rawData.gyro; };

protected:
    void writeMPU6050(uint8_t reg, uint8_t data);
    uint8_t readMPU6050(uint8_t reg);

    TwoWire* wire;
    mpu6050_data rawData;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////

class MPU6050 : public MPU6050_Raw {

#define DEFAULT_GYRO_CALIBRATION_PERIOD 1000 // 1000 number of ms to perform offet calculation
#define DEFAULT_GYRO_CALIBRATION_TRESHOLD 100 // 100 max peak minus min peak

    class GyroAxis {

    public:
        GyroAxis(const uint32_t calibPeriod = DEFAULT_GYRO_CALIBRATION_PERIOD);
        void update(const int16_t val, const uint32_t timeDelta_ms);
        int16_t getValue();
        int32_t getAngle();

    private:
        int16_t value;
        int16_t offset;
        int16_t maxVal;
        int16_t minVal;
        uint32_t period;
        uint32_t time;

        int32_t angle;

    // public:
    //     const int16_t& Value;
    //     const int32_t& Angle;
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////

#define DEFAULT_ACC_SCAN_PERIOD 100 // number of averages to calculate acc value
#define DEFAULT_ACC_UPDATE_PERIOD 10 // ms per average update
#define DEFAULT_SHOCK_DURATION 10 // 25 number of averages to calculate acc value

#define DEFAULT_SHOCK_NOISE_TRESHOLD 120

    class AccAxis {
    public:
        AccAxis(const uint16_t accScanPeriod_ms = DEFAULT_ACC_SCAN_PERIOD, const uint16_t accUpdatePeriod_ms = DEFAULT_ACC_UPDATE_PERIOD, const uint16_t shockDutaion_ms = DEFAULT_SHOCK_DURATION);
        void update(const int16_t accRaw, const uint32_t timeDelta_ms);
        int16_t getValue();
        uint16_t getShock();

    private:
        uint32_t accTime_ms;
        uint32_t shockTime_ms;

        MA<int16_t> accAvrg;
        uint16_t accUpdatePeriod_ms;

        MA<uint16_t> shockAvrg;

        int16_t accRaw_last;

    // public:
        // const int16_t& Value;
        // const uint16_t& Shock;
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////

public:
    MPU6050(TwoWire& wire = Wire);

    void begin(const uint8_t sdaPin = SDA, const uint8_t sclPin = SCL, const uint32_t wireFreq = 400000UL);

    void update();

    void getAcc(int16_t buffer[3]);
    int16_t getAccX() { return acc[0].getValue(); }
    int16_t getAccY() { return acc[1].getValue(); }
    int16_t getAccZ() { return acc[2].getValue(); }

    void getGyro(int16_t buffer[3]);
    int16_t getGyroX() { return gyro[0].getValue(); }
    int16_t getGyroY() { return gyro[1].getValue(); }
    int16_t getGyroZ() { return gyro[2].getValue(); }

    //cca 24000000 units per 360°
    //(angle % 12000000 + 12000000) gets range 0 - 24000000
    void getAngle(int32_t buffer[3]);
    uint16_t getAngleX() { return gyro[0].getAngle(); }
    uint16_t getAngleY() { return gyro[1].getAngle(); }
    uint16_t getAngleZ() { return gyro[2].getAngle(); }

    void getShock(uint16_t buffer[3]);
    uint16_t getShockX() { return acc[0].getShock(); }
    uint16_t getShockY() { return acc[1].getShock(); }
    uint16_t getShockZ() { return acc[2].getShock(); }

private:
    GyroAxis gyro[3];
    AccAxis acc[3];

    unsigned long lastUpdated_millis;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif
#endif