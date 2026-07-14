# Croi

Croi is the core flight-computer board in the Ogma stack. It owns the main flight-state logic, sensor/filtering path, flash logging, and stack-level coordination work.

## Role In Ogma

- Runs flight-state logic and Kalman/filtering work.
- Reads onboard IMU and barometer sensors.
- Logs flight and secondary data to external SPI flash.
- Sends/receives stack data over CAN.
- Provides local board-state feedback through the buzzer path.

## Hardware Summary

Current hardware and firmware indicate:

- MCU: STM32F072.
- IMU: LSM6DSO32.
- Barometer: MS5607.
- Extra sensors on hardware: BME680 and ADXL375 are present in the schematic.
- Storage: Macronix MX25L128-class 16 MB SPI flash.
- Comms: TJA1051 CAN transceiver.
- Power: common Ogma 5 V to 3.8 V preregulator to 3.3 V LDO pattern.

## Firmware

Firmware lives in `firmware/`.

Useful targets:

```bash
cd firmware
pio run -e stm32f072c8t6
pio test -e sil
```

## Ogma Console Support

Ogma Console can:

- identify Croi over SWD using `ogma_board_identity`,
- build/flash `stm32f072c8t6`,
- read `croi_status` for IMU, baro, CAN, and logger health,
- read external flash through the SWD mailbox `ogma_flash_mailbox`,
- parse `FlashLogger` records,
- save local CSV/JSON bundles,
- plot parsed altitude data from imported or live-read dumps.

## Host-Visible Symbols

- `ogma_board_identity`
- `croi_status`
- `ogma_flash_mailbox`

## Notes

- CAN is intentionally not modified by the SWD debug/readout path.
- Flash readout is chunked through SRAM so no UART is required. Firmware reports FlashLogger used bytes so host reads do not default to the whole flash device.
- Flash reads require a short SWD bench lease and are rejected while flight states are active.
- Sensor initialization failure halts the MCU. The FSM cannot leave calibration until IMU, barometer, CAN, and logger preflight are healthy.
- Croi emits leased airbrake commands to Lamh from the mission manifest. Deployment and stow are independently timed from entry to `POWERED`; the default manifest keeps airbrakes disabled.
- Croi emits mission-bound, sequenced Pleasc arm/fire commands only in active flight states. Drogue fires on entry to `DROGUE`; main fires on entry to `MAIN` after the configured altitude and minimum delay conditions. Fire commands retry at a bounded rate until Pleasc confirms the channel fired.
- Pleasc status/acknowledgements are logged into version-2 secondary flash records, including mission tag, sequence, channel, result, fault, continuity, and fired masks.
- Croi mission config is compiled into the flight image and reported back with magic/schema/CRC over `croi_status`.
- Mission schema 5 seals the recovery fallback and blackbox policy into that CRC. The guarded main fallback requires elapsed time after apogee, descent speed, an altitude window, and consecutive samples.
- Ogma Console replay compiles and executes the same `FlightPhaseLogic` and `AirbrakeLogic` headers used by this image.
- Hardware watchdog supervision is enabled after FSM, CAN, and logger tasks all report healthy progress.
- Pyro configuration is available in Ogma Console. Live firing still requires the explicit Pleasc Rev1 accepted-risk image and an external RBF/pyro-power disconnect.

## Dependency Lock

Use the exact shared-library pins in `../dependencies.lock.json`:

- `braiteoiri`: `ogma/flight-hardening`
- `comheadan`: `ogma/flight-hardening`

Ogma Console doctor fails a board when these submodule SHAs do not match the lock file.
