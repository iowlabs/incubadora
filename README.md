# incubadora

This project consists of a low-cost incubator for biotechnology instrumentation. Its design is easy to integrate and adaptable to different scenarios. It is ideal for remote laboratories, field deployments, school laboratories, and educational kits.

For this project, a PCB was designed that implements the electronics to control the operation of an incubator. It includes a temperature control and a *shaker* for sample agitation.

The PCB is based on a ESP32 so It can implement remote control and monitoring via Wi-Fi. It also has access to a USB communication interface for parameter configuration, curve generation, and data acquisition. It also features a simple interface that uses an LCD display and a rotary encoder as an alternative control method.

## Features.

### General features:

- Temperature Control
- Shaker RPM Control
- Simple User Interface
- WiFi Remote Control Capability
- Temperature profile programming

### Hardware Features

- ESP32 Microcontroller
- PID Temperature Control Loop
	- Heater: Peltier-controlled by a H-bridge (2 separated heater channels)
	- Temperature Sensor: SHT31 ambient temperature and humidity sensor via I²C
	- Fan Assembly
- Shaker Control:
	- Nema 17 Motor
	- DRV8825 Polulu Controller
- Local User Interface
	- Rotary Encoder
	- LCD Display
	- SD Card
- WiFi Remote Control
	- MQTT Commands
- INA219 Current Sensor for Measuring Input Power Consumption

---
# Documentación

- Github repo
- Web

## Licencias

|     |   Licence |
|---- | --------- |
|Hardware | --------- |
|Firmware and Software | --------- |
|Documentation and files | --------- |



---

# Diseño e implementación




---

## PCB
### Schematic


### pcb BOM

---

## Case 3D and structure

---

## List of components


### Assembly steps

---

## Firmware

### Used libraries

### Structure of the code

### Available Features.



---

# Application examples

## Using a portable cooler as incubator.

## Using a large incubator.

## Code to acquire and plot data in real time.

## Controlling via MQTT.
