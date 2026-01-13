# Croi

Heart of flight computer. It will run the Kalman filter and flight state machine. It is the core board to determine and send the trigger to the pyro board for  parachutes.

# Set-Up

```bash
git clone git@github.com:ULAS-HiPR/croi.git
cd croi
git submodule update --init --recursive
```

# Libraries used:

## From Braiteoiri:

- Flash: W25N01GV
- Baro: BMP390
- IMU:  LSM6DSO32
- Env: BME680
- LED
- Buzzer

## From Comheadan:

- CAN
- SPI
- I2C

## Other:

- FreeRTOS