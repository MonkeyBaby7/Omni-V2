# Omni-Directional Robot with Heading Hold

MicroPython control system for a 4-wheel omni robot using:

- Raspberry Pi Pico
- BNO055 IMU
- DRV8871 motor drivers
- Nextion touchscreen display

The robot supports:

- Compass heading translation
- Heading hold stabilisation
- Rotation commands
- Command scripting
- Live telemetry on Nextion display

---

## Hardware

Controller:
- Raspberry Pi Pico

Sensors:
- BNO055 IMU (I2C)

Motors:
- 4x DC motors
- DRV8871 motor drivers

Display:
- Nextion HMI screen

---

## Wiring

### IMU (BNO055)

| Pin | Pico |
|----|----|
| SDA | GP0 |
| SCL | GP1 |

### Nextion Display

| Pin | Pico |
|----|----|
| TX | GP8 |
| RX | GP9 |

### Motor Drivers

| Motor | Pins |
|----|----|
| M1 Front Right | GP11 GP10 |
| M2 Back Right | GP13 GP12 |
| M3 Back Left | GP16 GP17 |
| M4 Front Left | GP3 GP4 |

---

## Running the Robot

Upload all files to the Pico:
