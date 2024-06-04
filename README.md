# ESP32 Multi-Tasking System with FreeRTOS, OLED Display, Servo Control, and DHT22 Sensor

This repository contains an ESP32 project that leverages FreeRTOS for multitasking and integrates multiple peripherals and sensors. The project demonstrates how to use FreeRTOS tasks and queues to control LEDs, read sensor data, control servos, and update an OLED display. The components used include an Adafruit SSD1306 OLED display, two servos, and a DHT22 temperature and humidity sensor.

![image](https://github.com/hannanpoonawala/ESP32-Multi-Tasking-System-with-FreeRTOS/assets/83177528/fe4a4d63-124c-4862-9456-bf509e08043f)

## Features
- **FreeRTOS Multitasking**: Efficient management of multiple tasks using FreeRTOS.
- **OLED Display**: Real-time display of temperature, humidity, servo angles, and LED states on an Adafruit SSD1306 OLED.
- **Servo Control**: Control two servos via serial commands.
- **DHT22 Sensor Integration**: Read temperature and humidity data from a DHT22 sensor.
- **LED Control**: Control the state of blue and red LEDs via serial commands.

## Components
- **ESP32**: The main microcontroller used in this project.
- **Adafruit SSD1306 OLED Display**: For visual output.
- **Servos**: For demonstrating angle control.
- **DHT22 Sensor**: For measuring temperature and humidity.
- **LEDs**: Blue and red LEDs controlled via GPIO.

## Tasks Implemented
- **Blue LED Task**: Blinks a blue LED based on received commands.
- **Red LED Task**: Blinks a red LED based on received commands.
- **DHT22 Sensor Task**: Reads temperature and humidity data and sends it to the OLED display task.
- **Servo Control Tasks**: Control the angles of two servos based on received commands.
- **OLED Display Task**: Updates the OLED display with sensor data, servo angles, and LED states.
- **Serial Command Task**: Parses serial commands to control LEDs and servos.

## Queues for Inter-task Communication
- **TemperatureQueue**: Queue to hold temperature data.
- **HumidityQueue**: Queue to hold humidity data.
- **ServoAngle1Queue**: Queue to hold the angle for the first servo.
- **ServoAngle2Queue**: Queue to hold the angle for the second servo.
- **BlueLedStateQueue**: Queue to hold the state of the blue LED.
- **RedLedStateQueue**: Queue to hold the state of the red LED.
- **SerialQueue**: Queue to hold serial commands.

## Setup Instructions

### 1. Install Required Libraries
- Adafruit SSD1306
- Adafruit GFX
- ESP32Servo
- DHTesp

### 2. Connect the Components
- Connect the OLED display to the I2C pins of the ESP32.
- Connect the DHT22 sensor to a digital pin.
- Connect servos to PWM-capable pins.
- Connect LEDs to digital output pins.

### 3. Upload the Code
- Open the provided code in the Arduino IDE.
- Select the correct ESP32 board and port.
- Upload the code to the ESP32.

### 4. Interact via Serial Monitor
- Open the Serial Monitor at 115200 baud rate.
- Use commands like `blueOn`, `blueOff`, `redOn`, `redOff`, `servo1.<angle>`, and `servo2.<angle>` to control the LEDs and servos.

This project serves as a comprehensive example of using FreeRTOS with ESP32 for multitasking and managing various peripherals effectively.
