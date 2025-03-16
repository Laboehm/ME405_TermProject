# README
This repository contains the code and report for the ME405 course term project at Cal Poly.

## Driver Files
blah

## Main file
Blah blah

## Support Images/Videos
blah blah blah

# Report

## Introduction
### Abstract
In the following sections, the design, analysis, and results of a programmed Romi bot with a micropython programmed nucleo STM32L476 to complete an obstacle course will be detailed. The base mechatronic system comprises two wheels, two independent motors, two encoders and the general chassis. This overall system is wired through typical pin headers to assign purpose to each of these components and the other later added parts. Together the electrical and mechanical components will intertwine in a micropython code, interpreted through the “Shoe of Brian”. The “Shoe of Brian” is simply an interpreter that allows for the STM32 chip to process micropython commands, as opposed to C programming. Ultimately, all of these components are put together to complete the course with a high degree of repeatability and speed. 

### Problem statement
The primary task is to complete the following track in figure 1 using the Romi bot, with the highest speed and reliability possible. Approach for this task is fully up to the interpretation of the team, with the only requirement being the checkpoints. The bot is able to disregard the lines entirely, but for a systematic approach the lines will be needed in order to hit preceding checkpoints. Based on the structure of the track, it is inevitable that there will be hardcoded portions which will cause the program to be a mix of algorithmic and hard coded.\
![alt text](https://github.com/[Laboehm]/[ME405_TermProject]/blob/[Figures]/Track.jpg?raw=true)\
Figure 1: Romi bot track

## Design
### Mechanical design
This project aims to develop a mechatronic navigation system capable of autonomously traversing, with minimal hard coded portions,  a predefined track. The core components are built upon the Pololu Romi Chassis, with an STM32 Nucleo board, encoders, an IMU (BNO055), bump sensors, and an IR reflectance sensor array (7-sensor line sensor). By integrating all these components, the robot can navigate the track with a mixture of dynamic and hard coded control. Following will be a greater in depth background of the aforementioned components: 

#### **Pololu Romi Chassis**
The Pololu Romi Chassis is a lightweight and versatile robotic platform. It provides:
- Integrated motor mounts for two DC motors and encoders.
- Ball caster support for smooth movement and stability.
- Expandable design, allows for additional components to be directly mounted.

This chassis serves as the foundation for the entirety of the mechanical system.

---

#### **Motor Driver and Power Distribution Board**
The Motor Driver and Power Distribution Board provides basic interfacing for motor control and power management of the Romi. It features:
- Dual-channel H-Bridge Motor Driver for speed and direction control.
- 5V and 3.3V regulators to safely power the sensors and additional controllers.
- PWM control support for smooth motor actuation.
- Battery management with direct interfacing to the six batteries and a dedicated power switch.

This board provides efficient motor control and power distribution to all system components.

---

#### **STM32 Nucleo-L476RG**
The STM32 Nucleo-L476RG is a basic development board that serves as the main processing unit for the Romi. Key features:
- Multiple I/O interfaces including I²C, SPI, and UART.
- ST-Link debugger for programming and real-time debugging.
- Relatively high performance at overall low costs.

The Nucleo enables efficient real-time control of the Romi’s sensors, actuators, and wireless communication.

---
#### **“Shoe of Brian”**
The “Shoe of Brian” is a custom micropython interpreter for the STM32 Nucleo-L476RG to interface all of the pin headers with micropython instead of the language C. Key features:
- Plugs directly into the header pins of the STM32 Nucleo-L476RG
- Compatible with any other development board with the same headings
- Bypasses the Nucleo interpreter to allow for programming in external languages, specifically micropython

The “Shoe of Brian” enables the development board to interpret micropython code.

---

#### **Romi Encoder Pair**
The Romi encoders provide quadrature encoder values for tracking motor movement, ensuring accurate speed and position feedback. Features:
- High-resolution position tracking using quadrature encoding, namely 1440 counts per revolution.
- Essential for improving the accuracy of closed-loop control.
- Real-time operation  to assist in speed regulation and trajectory planning.

These encoders help the robot navigate accurately by monitoring its movement of each wheel independently.

---

#### **BNO055 (IMU)**
The BNO055 is a 9-axis IMU that provides orientation, acceleration, and angular velocity data. Key capabilities:
- Integrated sensor fusion algorithms for real-time orientation tracking.
- Gyroscope-based location data to improve heading accuracy and for bot tracking.
- Critical for clean turn control and maintaining straight-line movement by checking orientation.

This IMU allows for precise turns and heading corrections during navigation.

---

#### **Bump Sensors**
The bump sensor detects physical obstacles by closing an electrical circuit when pressed to pull pin low. Functions:
- Trigger an input when a collision is detected.
- Enable reactive obstacle avoidance to trigger navigation sequences.
- Simply effective sensor that only requires a total of two grounds and two GPIO pins to cover the whole front.

Bump sensors provide a straightforward method of collision detection for course correction. 

---

#### **IR Reflectance Sensors (Line Sensor)**
The IR line sensors or line sensors detect contrasts between surfaces using infrared reflectance over seven total sensors. Features:
- Line-following capability by detecting dark/light surface differences and calculating a centroid.
- Allows for real-time corrections to maintain a smooth path.
- Compatible with PID control to optimize trajectory tracking and smoothness.

These sensors allow the robot to correct its  path by following black lines of varying widths based on PID controller feedback.

---

#### **HC-05 Bluetooth Module**
The HC-05 is a standard bluetooth module that enables wireless communication with external devices, such as the programming device. Key functions:
- Serial (UART) communication for wireless data transfer.
- Remote debugging and monitoring via a bluetooth-connected PC.
- Standard baud rate of about 400k cycles per second, providing high performance.

This module enables remote control, allowing for a more seamless programming and usage experience.

---

These components were chosen to match the specific challenges provided by the track, with the main features being varying lines, blank space, and a wall. The track components challenge the designer to construct the bot and program to handle sharp turns, follow lines, count distances, and detect collisions. Along with the fact that the Romi adjacent components and the STM32 development board were provided by the course as parameters to work within. The external components we had the freedom to use were the IMU, line sensors, and bump sensors. The chassis in particular, hugely simplifies the design by allowing us to ignore any stability issues with its stabilizer balls and light frame. The STM32 Nucleo-L476RG provides a straightforward development board that allows us to actually code the whole project and interface the components. The BNO055 IMU was a recommendation by the professor, and provides a triaxial 14-bit accelerometer, an accurate close-loop triaxial 16-bit gyroscope, a triaxial geomagnetic sensor and a 32-bit microcontroller running the BSX3. All of the parts for the IMU, provide a versatile array of location and orientation recording features for efficient navigation. Romi encoders were already provided and provide a more simple alternative to the navigation of the IMU and provide accurate distance readings. The IR reflectance sensors or line sensors are an essential part of this design, because otherwise we would not be able to detect and follow the line while adjusting for minor deviations. Without the line sensors, we would need to simply hard code the entirety of the algorithm and would require a specific and repeatable starting point. Bump sensors serve as the primary method for detecting collisions and triggering obstacle-avoidance maneuvers to complete the final portion of the course with the wall. The HC-05 Bluetooth module enables wireless debugging and remote monitoring, allowing for real-time performance adjustments without requiring a wire. Together, these components create a robust autonomous system capable of navigating the track efficiently with dynamic controls. The intent with these dynamic controls is to create maximum consistency, while allowing for the bot to theoretically function if placed anywhere on the track. 

### Electrical design



#### Power Connections
| Cable Color | Nucleo Pin | Nucleo Pin Type | Romi PDB Pin | Component | Signal |
|------------|-----------|----------------|-------------|----------|--------|
| Red        | Vin       | Vin            | VSW         | Nucleo Board | ≈7.2V |
| Black      | GND       | GND            | GND         | Nucleo Board | Ground |

#### Encoder Connections
| Cable Color | Nucleo Pin | Nucleo Pin Type | Romi PDB Pin | Component | Signal |
|------------|-----------|----------------|-------------|----------|--------|
| Blue       | PB6       | Timer Channel 1 | ELA/ERA     | Right Encoder | Encoder Ch. A |
| Yellow     | PB7       | Timer Channel 2 | ELB/ERB     | Right Encoder | Encoder Ch. B |
| Blue       | PB4       | Timer Channel 1 | ELA/ERA     | Left Encoder | Encoder Ch. A |
| Yellow     | PB5       | Timer Channel 2 | ELB/ERB     | Left Encoder | Encoder Ch. B |

#### Motor Connections
| Cable Color | Nucleo Pin | Nucleo Pin Type | Romi PDB Pin | Component | Signal |
|------------|-----------|----------------|-------------|----------|--------|
| Green      | PA10      | Any Dig Out    | PWM         | Right Motor | Motor Effort |
| Blue       | PB8       | Any Dig Out    | DIR         | Right Motor | Motor Direction |
| Orange     | PB2       | Any Timer      | SLP         | Right Motor | Motor Enable |
| Green      | PA15      | Left PWM       | PWM         | Left Motor | Motor Effort |
| Blue       | PH0       | Left DIR       | DIR         | Left Motor | Motor Direction |
| Yellow     | PH1       | Left SLP       | SLP         | Left Motor | Motor Enable |

#### BNO055 (IMU) Connections
| Cable Color | Nucleo Pin | Nucleo Pin Type | Romi PDB Pin | Component | Signal |
|------------|-----------|----------------|-------------|----------|--------|
| Red        | -         | -              | -           | BNO055 (IMU) | ≈3.3-5V (Vin) |
| Black      | GND       | GND            | -           | BNO055 (IMU) | Ground (GND) |
| Yellow     | PB14      | Timer          | -           | BNO055 (IMU) | Serial Data Pin (SDA) |
| Green      | PB3       | Timer          | -           | BNO055 (IMU) | Serial Clock Pin (SCL) |
| Blue       | PB13      | -              | -           | BNO055 (IMU) | Reset (RST) |

#### Line Sensor Connections
| Cable Color | Nucleo Pin | Nucleo Pin Type | Romi PDB Pin | Component | Signal |
|------------|-----------|----------------|-------------|----------|--------|
| Red        | CN6       | 5V             | -           | Line sensor | VCC (2.9-5.5V) |
| Black      | CN6       | GND            | -           | Line sensor | GND |
| Black      | PC15      | GPIO           | -           | Line sensor | Even (current for emitter) |
| White      | PC14      | GPIO           | -           | Line sensor | Odd (current for emitter) |
| Brown      | PC3       | Analog         | -           | Line sensor | 1 |
| Green      | PA4       | Analog         | -           | Line sensor | 2 |
| Orange     | PA0       | Analog         | -           | Line sensor | 3 |
| Blue       | PB0       | Analog         | -           | Line sensor | 4 |
| Yellow     | PA1       | Analog         | -           | Line sensor | 5 |
| Purple     | PC1       | Analog         | -           | Line sensor | 6 |
| Gray       | PC0       | Analog         | -           | Line sensor | 7 |

#### Bluetooth Connections
| Cable Color | Nucleo Pin | Nucleo Pin Type | Romi PDB Pin | Component | Signal |
|------------|-----------|----------------|-------------|----------|--------|
| Green      | PC4       | Tx (Nucleo)    | -           | Bluetooth | Rx |
| Yellow     | PC5       | Rx (Nucleo)    | -           | Bluetooth | Tx |
| Blue       | PC8       | GPIO           | -           | Bluetooth | State |
| Brown      | PC6       | GPIO           | -           | Bluetooth | Enable |

#### Bumper Connections
| Cable Color | Nucleo Pin | Nucleo Pin Type | Romi PDB Pin | Component | Signal |
|------------|-----------|----------------|-------------|----------|--------|
| White      | PA8       | GPIO           | -           | Right Bumper | GPIO |
| Purple     | GND       | GND            | -           | Right Bumper | Ground |
| Green      | PC7       | GPIO           | -           | Left Bumper | GPIO |
| Orange     | GND       | GND            | -           | Left Bumper | Ground |


### Code design
| File | High Level Description |
|------|------------|
| `main.py` | Initializes tasks and coordinates state transitions with cotask scheduler logic. |
| `tasks.py` | Contains task logic for movement, turns, and obstacle handling for the scheduler in main. |
| `IMU.py` | Handles BNO055 IMU data acquisition for precise turns and heading correction. |
| `motor.py` | Controls motor effort and direction for movement execution with enable/disable |
| `encoder.py` | Reads encoder values to determine the distance traveled and speed. |
| `bump.py` | Implements bump sensor detection driver for obstacle avoidance. |
| `bluetooth.py` | Manages communication with HC-05 bluetooth module. |
| `controller.py` | Establishes PID based controller logic for use in other modules. |
| `line_sensor.py` | Completes line sensor assignment/calibration, acquires centroid, and provides line detection feedback. |

## Analysis


### Calculations


### Plots


### Demonstration


## Results


## References
[Nucleo Pinout](https://os.mbed.com/platforms/ST-Nucleo-L476RG/)\
[STM32 Datasheet](https://www.st.com/resource/en/datasheet/stm32l476je.pdf)\
[Additional STM32 Datasheet](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf)\
[BNO055 Pinout](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/pinouts)\
[BNO055 DataSheet](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)\
[Bumper Datasheet](https://www.pololu.com/product/3674)\
[ME 405 Support Library Documentation](https://spluttflob.github.io/ME405-Support/annotated.html)\
[ME 405 Support Library](https://github.com/spluttflob/ME405-Support)
