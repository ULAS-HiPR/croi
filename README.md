# croi
The heart of the flight computer
### 1. The Core (Flight Controller)

- **Owner:** Kyle
- **Philosophy:** "*Croí* " (heart) Small, dense, critical.
- **Use Case:** Controls L2 recovery (when stacked with Power Sled) OR acts as a standalone data logger for L1 flights.
- **Capabilities:** **Standalone Operation.** Includes a local LDO to run directly off a small LiPo battery without the Power Sled.
- **Components:**
    - **MCU:** **STM32F411CEU6** (High Performance, FPU, Internal CAN Controller).
    - **Memory:** **W25N01GV** (1Gb SPI NAND Flash) - *Vibration proof logging.*
    - **Sensors:** **BMP390** (Altimeter) + **LSM6DSO32** (IMU) + **BME680** (Env).
    - **LDO:** TLV70433DBVT, fixed 3.3v.
    - **Interface:** USB-C (Data dump), Buzzer, RGB LED.
    - **Comms:** **TJA1051 CAN Transceiver**.
    - **Power Logic:**
        - Primary: +5V_LOGIC from stack.
        - Standalone Backup: **2-pin JST connector** (for 3.7V LiPo).
        - *Protection:* Schottky Diodes are required to OR the stack power and JST power, preventing back-flow.
- **Note:** *No Pyros on this board.* We keep the high-current noise away from the processor.
    - Note: Pre-flight Settings configured via commands received by the Teachtaire board.
  

  ![Image of Board Schematics](<schematics.png>)