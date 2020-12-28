#ifdef ESP32

#include "MakerMatty_MPU6050.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////

MPU6050_Raw::MPU6050_Raw(TwoWire& w)
{
    wire = &w;
}

void MPU6050_Raw::begin(const int8_t sda, const int8_t scl, const uint32_t freq)
{
    wire->begin(sda, scl, freq);

    writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
    writeMPU6050(MPU6050_CONFIG, 0x00);
    writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
    writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
    writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
}

void MPU6050_Raw::read()
{
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(0x3B);
    wire->endTransmission(false);

    uint8_t received = wire->requestFrom(MPU6050_ADDR, 14);

    if (received == 14) {
        for (size_t i = 0; i < 14; i += 2) {
            rawData.raw[i + 1] = (uint8_t)wire->read();
            rawData.raw[i] = (uint8_t)wire->read();
        }
    } else {
        //clear rx buffer
        for (size_t i = 0; i < received; i++) {
            wire->read();
        }
    }

    //Serial.printf("%d, %d, %d\period", rawData.gyroX, rawData.gyroY, rawData.gyroZ);

    //rawData.acc[0] = wire->read() << 8 | wire->read();
    //rawData.acc[1] = wire->read() << 8 | wire->read();
    //rawData.acc[2] = wire->read() << 8 | wire->read();
    //rawData.temp = wire->read() << 8 | wire->read();
    //rawData.gyro[0] = wire->read() << 8 | wire->read();
    //rawData.gyro[1] = wire->read() << 8 | wire->read();
    //rawData.gyro[2] = wire->read() << 8 | wire->read();

    //temp = (rawTemp + 12412.0) / 340.0;

    //accX = ((float)rawAccX) / 16384.0;
    //accY = ((float)rawAccY) / 16384.0;
    //accZ = ((float)rawAccZ) / 16384.0;

    //angleAccX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;
    //angleAccY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;

    //gyroX = ((float)rawGyroX) / 65.5;
    //gyroY = ((float)rawGyroY) / 65.5;
    //gyroZ = ((float)rawGyroZ) / 65.5;

    //gyroX -= gyroXoffset;
    //gyroY -= gyroYoffset;
    //gyroZ -= gyroZoffset;

    //unsigned long m = micros();
    //if (m >= micros_last)
    //    interval = (float)((m - micros_last) * 0.000001);
    //micros_last = m;

    //angleGyroX += gyroX * interval;
    //angleGyroY += gyroY * interval;
    //angleGyroZ += gyroZ * interval;

    //angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
    //angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
    //angleZ = angleGyroZ;
}

void MPU6050_Raw::writeMPU6050(byte reg, byte data)
{
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(reg);
    wire->write(data);
    wire->endTransmission();
}

byte MPU6050_Raw::readMPU6050(byte reg)
{
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(reg);
    wire->endTransmission(true);
    wire->requestFrom(MPU6050_ADDR, 1);
    byte data = wire->read();
    return data;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

MPU6050::GyroAxis::GyroAxis(const uint32_t calibPeriod)
    : value(0)
    , maxVal(INT16_MIN)
    , minVal(INT16_MAX)
    , period(calibPeriod)
    , time(0)
    , angle(0)
    , Value(value)
    , Angle(angle)
{
}

void MPU6050::GyroAxis::update(const int16_t val, const uint32_t timeDelta_ms)
{
    //gyro calculation
    value = val;

    time += timeDelta_ms;

    if (time >= period) {
        time = 0;

        if (maxVal - minVal < DEFAULT_GYRO_CALIBRATION_TRESHOLD) {
            offset = (maxVal + minVal) / 2;
        }

        maxVal = INT16_MIN;
        minVal = INT16_MAX;
    }

    if (value > maxVal) {
        maxVal = value;
    }
    if (value < minVal) {
        minVal = value;
    }

    //angle calculation
    angle += (int32_t)value * (int32_t)timeDelta_ms;
}

int16_t MPU6050::GyroAxis::getValue()
{
    int32_t res = (int32_t)value - (int32_t)offset;

    if (res > INT16_MAX)
        return INT16_MAX;

    else if (res < INT16_MIN)
        return INT16_MIN;

    else
        return (int16_t)res;
}

int32_t MPU6050::GyroAxis::getAngle()
{
    return angle;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

MPU6050::AccAxis::AccAxis(const uint16_t accScanPeriod_ms, const uint16_t accUpdatePeriod_ms, const uint16_t shockDutaion_ms)
    : accTime_ms(0)
    , shockTime_ms(0)
    , accAvrg(accScanPeriod_ms / accUpdatePeriod_ms)
    , accUpdatePeriod_ms(accUpdatePeriod_ms)
    , shockAvrg(shockDutaion_ms)
    , accRaw_last(0)
    , Value(accAvrg.Value)
    , Shock(shockAvrg.Value)
{
}

void MPU6050::AccAxis::update(const int16_t accRaw, const uint32_t timeDelta_ms)
{
    accTime_ms += timeDelta_ms;
    shockTime_ms += timeDelta_ms;

    //shock calculation
    int32_t accChange = (int32_t)accRaw - (int32_t)accRaw_last;
    accRaw_last = accRaw;

    accChange = abs(accChange);

    //int16_t overflow
    if (accChange > UINT16_MAX)
        accChange = UINT16_MAX;

    while (shockTime_ms >= 1) {
        shockTime_ms -= 1;
        //shock is being updated "every 1 ms"
        shockAvrg.update((uint16_t)accChange);
    }

    //if the thing is shocking, then skip acc update
    if (shockAvrg.getValue() >= DEFAULT_SHOCK_NOISE_TRESHOLD) {
        accTime_ms = 0;
        return;
    }

    //acc calculation
    while (accTime_ms >= accUpdatePeriod_ms) {
        accTime_ms -= accUpdatePeriod_ms;
        //acc is being updated "every accUpdatePeriod_ms ms"
        accAvrg.update(accRaw);
    }
}

int16_t MPU6050::AccAxis::getValue()
{
    return accAvrg.getValue();
}

uint16_t MPU6050::AccAxis::getShock()
{
    return shockAvrg.getValue();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

MPU6050::MPU6050(TwoWire& wire)
    : MPU6050_Raw(wire)
    , gyro { GyroAxis(), GyroAxis(), GyroAxis() }
    , acc { AccAxis(), AccAxis(), AccAxis() }
    , lastUpdated_millis(0)
{
}

void MPU6050::begin(const uint8_t sdaPin, const uint8_t sclPin, const uint32_t wireFreq)
{
    MPU6050_Raw::begin((int8_t)sdaPin, (int8_t)sclPin, wireFreq);
}

void MPU6050::update()
{
    unsigned long current_millis = millis();
    
    uint32_t delta_ms = (uint32_t)(current_millis - lastUpdated_millis);
    lastUpdated_millis = current_millis;

    for (int i = 0; i < 3; i++) {
        acc[i].update(rawData.acc[i], delta_ms);
        gyro[i].update(rawData.gyro[i], delta_ms);
    }

    // static uint32_t ups = 0;
    // static int64_t next_micros = 0;
    // ups++;
    // if (esp_timer_get_time() >= next_micros) {
    //     next_micros = esp_timer_get_time() + 1000000;
    //     Serial.printf("ups = %u, delta = %u\n", ups, delta_ms);
    //     ups = 0;
    // }

    //detector.update(gyro, acc, delta_ms);
}

void MPU6050::getAcc(int16_t buffer[3])
{
    for (int i = 0; i < 3; i++) {
        buffer[i] = acc[i].Value;
    }
}

void MPU6050::getGyro(int16_t buffer[3])
{
    for (int i = 0; i < 3; i++) {
        buffer[i] = gyro[i].Value;
    }
}

void MPU6050::getAngle(int32_t buffer[3])
{
    for (int i = 0; i < 3; i++) {
        buffer[i] = gyro[i].Angle;
    }
}

void MPU6050::getShock(uint16_t buffer[3])
{
    for (int i = 0; i < 3; i++) {
        buffer[i] = acc[i].Shock;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif