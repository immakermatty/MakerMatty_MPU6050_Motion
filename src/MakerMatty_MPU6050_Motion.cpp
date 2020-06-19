#ifdef ESP32

#include "MakerMatty_MPU6050_Motion.h"

MPU6050_Motion::MPU6050_Motion(TwoWire& w)
    : wire(&w)
    , sda(SDA)
    , scl(SCL)
    , freq(400000UL)
    , taskData {
        .dataSemaphore = nullptr,
        .pHardwareSemaphore = nullptr,
        .self = this
    }
    , taskHandle(nullptr)
    , mpu6050(*wire)
    , m_updateCb(nullptr)
    , m_updateArg(nullptr)
{
}

bool MPU6050_Motion::begin(const uint8_t sdaPin, const uint8_t sclPin, const uint32_t wireFreq, const BaseType_t xCoreID, SemaphoreHandle_t* hardwareSemaphore)
{
    if (taskHandle == nullptr) {

        if (taskData.dataSemaphore) {
            vSemaphoreDelete(taskData.dataSemaphore);
            taskData.dataSemaphore = nullptr;
        }

        if (taskData.dataSemaphore == nullptr) {
            taskData.dataSemaphore = xSemaphoreCreateMutex();
            if (!taskData.dataSemaphore) {
                assert(taskData.dataSemaphore);
                return false;
            }
        }

        if (hardwareSemaphore) {
            if (*hardwareSemaphore == nullptr) {
                *hardwareSemaphore = xSemaphoreCreateMutex();
                if(!*hardwareSemaphore){
                    assert(*hardwareSemaphore);
                    return false;
                }
            }
            taskData.pHardwareSemaphore = hardwareSemaphore;
        } else {
            taskData.pHardwareSemaphore = nullptr;
        }

        sda = sdaPin;
        scl = sclPin;
        freq = wireFreq;

        BaseType_t result = xTaskCreateUniversal(task, "motion", 4096, &taskData, 1, &taskHandle, xCoreID);

        if (!result) {
            assert(result == pdPASS);
            return false;
        }

        return true;
    } else {
        return true;
    }
}

void MPU6050_Motion::task(void* p)
{
    TaskData* data = (TaskData*)p;

    data->self->mpu6050.begin(data->self->sda, data->self->scl, data->self->freq);

    while (true) {

        // if (data->pHardwareSemaphore) {
        //     if (xSemaphoreTake(*(data->pHardwareSemaphore), 1)) {
        //         /* takes around 549 us @ 160MHz */
        //         data->self->mpu6050.read();
        //         xSemaphoreGive(*(data->pHardwareSemaphore));
        //     }
        // } else {
        //     /* takes around 549 us @ 160MHz */
        //     data->self->mpu6050.read();
        // }

        if (!data->pHardwareSemaphore || xSemaphoreTake(*(data->pHardwareSemaphore), 1)) {
            /* takes around 549 us @ 160MHz */
            data->self->mpu6050.read();
            if (data->pHardwareSemaphore) {
                xSemaphoreGive(*(data->pHardwareSemaphore));
            }
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
    if (taskHandle != nullptr) {
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
    }

    if (taskData.dataSemaphore != nullptr) {
        vSemaphoreDelete(taskData.dataSemaphore);
        taskData.dataSemaphore = nullptr;
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