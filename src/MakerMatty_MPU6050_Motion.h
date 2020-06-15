/** 
 * Author	: @makermatty (maker.matejsuchanek.cz)
 * Date		: 15-6-2020
 */

#ifdef ESP32

#ifndef _MM_MPU6050_MOTION_h
#define _MM_MPU6050_MOTION_h

#include "MakerMatty_MPU6050.h"
#include <Arduino.h>

class MPU6050_Motion {

public:
    MPU6050_Motion(TwoWire& wire = Wire);

    // @returns semaphore for i2c communication
    SemaphoreHandle_t begin(const uint8_t sdaPin = SDA, const uint8_t sclPin = SCL, const uint32_t wireFreq = 400000UL, const BaseType_t xCoreID = APP_CPU_NUM);
    void end();

    typedef void (*UpdateCallback)(MPU6050* mpu6050, void* cbarg);
    void onUpdate(UpdateCallback cb, void* cbarg = nullptr);

    void getAcc(int16_t buffer[3]);
    void getGyro(int16_t buffer[3]);
    void getAngle(int32_t buffer[3]);
    void getShock(uint16_t buffer[3]);

private:
    TwoWire* wire;
    uint8_t sda;
    uint8_t scl;
    uint32_t freq;

    struct TaskData {
        TaskHandle_t taskHandle;
        SemaphoreHandle_t dataSemaphore;
        SemaphoreHandle_t i2cSemaphore;
        MPU6050_Motion* self;
    };

    TaskData taskData;
    static void task(void* p);

    MPU6050 mpu6050;

    UpdateCallback m_updateCb;
    void* m_updateArg;
};

typedef MPU6050_Motion MakerMatty_MPU6050_Motion;

#endif
#endif