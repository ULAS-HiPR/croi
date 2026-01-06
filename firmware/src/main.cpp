#include <Arduino.h>
#include <IMU/IMU.h>
#include <IMU/MPU6050.h>
#include <Sensor.h>
#include <I2C/I2C_Handler.h>
#include <I2C/I2C_Pico.h>
#include <cstdint>

IMU* imu;


void setup() {
    Serial.begin(115200);
    Wire.begin();

    uint8_t addr = 0x68; // I2C address for MPU6050
    I2C_Handler* i2c = new I2C_Pico(addr);
    IMU* imu = new MPU6050(*i2c);
    imu->init();
}

    
void loop() {
    imu_data imuData;
    imu->update(&imuData);

    Serial.print("Accel: ");
    Serial.print("X: "); Serial.print(imuData.accel.x);
    Serial.print(" Y: "); Serial.print(imuData.accel.y);
    Serial.print(" Z: "); Serial.print(imuData.accel.z);
    delay(100);
}