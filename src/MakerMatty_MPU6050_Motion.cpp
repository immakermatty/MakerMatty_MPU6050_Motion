#ifdef ESP32

#include "MakerMatty_MPU6050_Motion.h"

MPU6050_Motion::MPU6050_Motion(TwoWire& w)
    : wire(&w)
    , sda(SDA)
    , scl(SCL)
    , freq(400000UL)
    , taskData {
        .taskHandle = nullptr,
        .dataSemaphore = nullptr,
        .i2cSemaphore = nullptr,
        .self = this,
    }
    , mpu6050(*wire)
    , m_updateCb(nullptr)
    , m_updateArg(nullptr)
{
}

SemaphoreHandle_t MPU6050_Motion::begin(const uint8_t sdaPin, const uint8_t sclPin, const uint32_t wireFreq, const BaseType_t xCoreID)
{
    if (taskData.taskHandle == nullptr) {

        //if (taskData.dataSemaphore == nullptr) {
        taskData.dataSemaphore = xSemaphoreCreateMutex();
        assert(taskData.dataSemaphore);
        //}

        //if (taskData.i2cSemaphore == nullptr) {
        taskData.i2cSemaphore = xSemaphoreCreateMutex();
        assert(taskData.i2cSemaphore);
        //}

        sda = sdaPin;
        scl = sclPin;
        freq = wireFreq;
        BaseType_t result = xTaskCreateUniversal(task, "motion", 4096, &taskData, 1, &taskData.taskHandle, xCoreID);
        assert(result == pdPASS);

        return taskData.i2cSemaphore;
    } else {
        return nullptr;
    }
}

void MPU6050_Motion::task(void* p)
{
    TaskData* data = (TaskData*)p;

    data->self->mpu6050.begin(data->self->sda, data->self->scl, data->self->freq);

    while (true) {

        if (xSemaphoreTake(data->i2cSemaphore, 1)) {
            /* takes around 549 us @ 160MHz */
            data->self->mpu6050.read();
            xSemaphoreGive(data->i2cSemaphore);
        }

        if (xSemaphoreTake(data->dataSemaphore, 1)) {
            /* takes around 25us @ 160MHz */
            data->self->mpu6050.update();

            if (data->self->m_updateCb != nullptr) {
                (*data->self->m_updateCb)(&(data->self->mpu6050), data->self->m_updateArg);
            }

            xSemaphoreGive(data->dataSemaphore);
        }

        delay(1);

        // static uint32_t ups = 0;
        // static int64_t next_micros = 0;
        // ups++;
        // if (esp_timer_get_time() >= next_micros) {
        //     next_micros = esp_timer_get_time() + 1000000;
        //     Serial.printf("ups = %u\n", ups);
        //     ups = 0;
        // }
    }
}

void MPU6050_Motion::end()
{
    if (taskData.taskHandle != nullptr) {
        vTaskDelete(taskData.taskHandle);
        taskData.taskHandle = nullptr;
    }

    if (taskData.dataSemaphore != nullptr) {
        vSemaphoreDelete(taskData.dataSemaphore);
        taskData.dataSemaphore = nullptr;
    }

    if (taskData.i2cSemaphore != nullptr) {
        vSemaphoreDelete(taskData.i2cSemaphore);
        taskData.i2cSemaphore = nullptr;
    }
}

void MPU6050_Motion::onUpdate(UpdateCallback cb, void* cbarg)
{
    m_updateCb = cb;
    m_updateArg = cbarg;
}

void MPU6050_Motion::getAcc(int16_t buffer[3])
{
    if (taskData.dataSemaphore != nullptr && xSemaphoreTake(taskData.dataSemaphore, portMAX_DELAY) == pdTRUE) {
        mpu6050.getAcc(buffer);
        xSemaphoreGive(taskData.dataSemaphore);
    }
}

void MPU6050_Motion::getGyro(int16_t buffer[3])
{
    if (taskData.dataSemaphore != nullptr && xSemaphoreTake(taskData.dataSemaphore, portMAX_DELAY) == pdTRUE) {
        mpu6050.getGyro(buffer);
        xSemaphoreGive(taskData.dataSemaphore);
    }
}

void MPU6050_Motion::getAngle(int32_t buffer[3])
{
    if (taskData.dataSemaphore != nullptr && xSemaphoreTake(taskData.dataSemaphore, portMAX_DELAY) == pdTRUE) {
        mpu6050.getAngle(buffer);
        xSemaphoreGive(taskData.dataSemaphore);
    }
}

void MPU6050_Motion::getShock(uint16_t buffer[3])
{
    if (taskData.dataSemaphore != nullptr && xSemaphoreTake(taskData.dataSemaphore, portMAX_DELAY) == pdTRUE) {
        mpu6050.getShock(buffer);
        xSemaphoreGive(taskData.dataSemaphore);
    }
}

#endif