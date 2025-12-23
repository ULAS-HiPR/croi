#include <Arduino.h>
#include <braiteoiri/include/IMU/IMU.h>
#include <braiteoiri/include/IMU/MPU6050.h>
#include <braiteoiri/include/Sensor.h>
#include <comheadan/include/I2C/I2C_Handler.h>
#include <comheadan/include/I2C/I2C_Pico.h>


uint8_t addr = 0x68; // I2C address for MPU6050
I2C_Handler* i2c = new I2C_Pico(addr);
IMU* imu = new MPU6050(*i2c);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    platform = new PicoIMUPlatform(0x68);
    imu = new MPU6050(*platform);
    imu->init();
}

    
void loop() {
    imu_data imuData;
    imu->read(imuData);

    Serial.print("Accel: ");
    Serial.print("X: "); Serial.print(imuData.accel.x);
    Serial.print(" Y: "); Serial.print(imuData.accel.y);
    Serial.print(" Z: "); Serial.print(imuData.accel.z)
    delay(100);
}