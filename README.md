# Croí

Croí is the core flight-computer node in [Ogma](https://sean-osullivan.com/projects/ogma/), the University of Limerick Aeronautics Society's modular avionics stack for high-powered rocketry.

## Role

- Coordinates flight state across the 500 kbit/s CAN bus.
- Samples an LSM6DSO32 IMU and MS5607 barometer.
- Records flight data to SPI NOR flash.
- Runs on an STM32F072 and can operate as the stack's central logger.

## Repository

- `hardware/` - KiCad design and manufacturing outputs.
- `firmware/` - embedded firmware, peripheral probes, and tests.
- `schematics.png` - exported schematic overview.

## Status

Rev 1 firmware and hardware bring-up continue on [`ogma/flight-hardening`](https://github.com/ULAS-HiPR/croi/tree/ogma/flight-hardening). Release-candidate firmware builds and software tests pass, but hardware-in-the-loop validation is still underway. Croí is not yet flight-proven.

## Manufacturing support

Rev 1 PCB fabrication was sponsored through [EasyEDA Education](https://easyeda.com/education) and manufactured by [JLCPCB](https://jlcpcb.com/). The board was designed in KiCad and imported into EasyEDA Pro for the sponsorship and manufacturing workflow.

## More information

See the [Ogma project write-up](https://sean-osullivan.com/projects/ogma/) for the complete stack, bring-up work, and current status.
