----
----
----
**HAY QUE CAMBIAR LAS URL DE LAS IMÁGENES AL PASAR EL README AL REPOSITORIO PRINCIPAL** 
----
----
----


# VILA-STEM 8’s repository
**This is the Vila-stem 8 repository for the 2026 season WRO Future Engineers challenge**

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/team-photos/team-photo-1.jpeg" width="600">

## TABLE OF CONTENTS

* [1. INTRODUCTION](#1-introduction)

* [2. MOBILITY MANAGEMENT](#2-mobility-management)
    * [2.1. Chassis design](#21-chassis-design)
    * [2.2. How we arrived at this design](#22-how-we-arrived-at-this-design)
    * [2.3. Drive System](#23-drive-system)
    * [2.4. Steering System](#24-steering-system)

* [3. SENSORS AND ELECTRICITY](#3-sensors-and-electricity)
    * [3.1. List of sensors and electrical components](#31-list-of-sensors-and-electrical-components)
    * [3.2. Electrical diagram](#32-electrical-diagram)
    * [3.3. Connection type diagram](#33-connection-type-diagram)
    * [3.4. Power consumption](#34-power-consumption)

* [4. CODE](#4-code)

* [5. PHOTOS](#5-photos)
    * [5.1. Robot photos](#51-robot-photos)
    * [5.2. Team photos](#52-team-photos)

* [6. VIDEOS](#6-videos)

* [7. LIST OF COMPONENTS](#7-list-of-components)

* [8. ENGINEERING JOURNAL](#9-engineering-journal)

* [9. LICENSE](#8-license)

## 1. INTRODUCTION

Vila-stem 8 is the Future Engineers team of the Vila-Stem educational robotics cultural association, located in Vila-real, Spain. Some members of this team have already participated in the WRO in different categories in previous years, including this year, while for others it is their first time competing, but we are all equally excited to participate in the Future Engineers category of WRO 2026.

Our first competition this season was the regional WRO in Dénia, where we won first place and qualified for the national final.

Now we are heading to the national final of WRO Spain 2026 in Andorra, eager to present our project.

Throughout this document, you will find an explanation of our robot and links to other documents that explain the different sections in more detail.

To comply with the competition rules, we have also included a [**Engineering Journal**](/engineering-journal.pdf) in PDF format. This document is a compilation of the information explained throughout this repository.

## 2. MOBILITY MANAGEMENT

### 2.1. Chassis design

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-left.jpeg" width="600">
<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-front.jpeg" width="600">

The lower chassis base comes from a development kit we acquired, so we simply had to assemble it. This kit provided the wheels, motor, and steering system, which simplified assembly. We designed a top platform on top of this kit, where the components are installed, as well as another set of parts to mount the components.

Purchasing this kit allowed us to have a metal chassis that better withstands the robot's stresses. This freed us up to design, using plastic and 3D printing, those parts that wouldn't be subjected to heavy loads.

### 2.2. How we arrived at this design

Initially, the idea was to mount all the components on the lower chassis, but the lack of space and the desire for a more aesthetically pleasing design led us to install a top platform. This platform saves space and also provides easier access to the connections. We ruled out increasing the robot's height because this keeps the center of gravity low and maintains its compact size.

This year we also aimed for a more aesthetically pleasing robot, which is why the design includes, for example, a decorative top cover and insulated cable covers, among other improvements.


### 2.3. Drive System

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/motor-mechanism-photo.jpeg" width="600">

In our robot, the rear wheels are the drive wheels. There is a single axle connecting both wheels, and a gear connects them directly to the motor, without using any differential mechanism. This causes both wheels to always rotate at the same speed.

Although, theoretically, this mechanism could cause the wheels to slip on curves, since the path of travel is different for each wheel in this case, we chose this system because it seemed sufficient for our robot's needs and is easier to install and maintain. A more complex mechanism would have added an extra difficulty that, in the repeated tests we conducted, we didn't find necessary to implement. Any slippage does not affect the robot's proper functioning.

During assembly, we encountered some problems. Some gears and parts didn't fit together, and it took us a while to get them all aligned, but we eventually managed.

### 2.4. Steering System

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/servo-mechanism-photo.jpeg" width="600">

The steering system is the typical pivot axle (like a wagon): the entire front axle rotates on a central pin moved by the servo. It's the most common system, but it works just as well and is inexpensive, so it was the logical choice for us.

Here, too, we had problems, especially during assembly: some axle parts broke, and we had to replace them with identical ones. There were also parts that were difficult to fit, some because they weren't very well made and others because we got a little confused during assembly.

Physically, there isn't a stop that directly limits the servo's movement. However, the code itself includes limitations that would prevent the wheels from over-rotating and hitting the sides of the chassis.

## 3. SENSORS AND ELECTRICITY

The robot requires components for both control and power, as well as for obtaining the necessary information from the environment. This section will cover these components, why of their selection, and connection.

### 3.1. LIST OF SENSORS AND ELECTRICAL COMPONENTS

#### ESP32 S3

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/components-photos/esp32-s3-photo.jfif" width="300">

| | |
| -------- | ------- |
| **WHY:** | We chose an ESP32 over options like an Arduino because of the versatility of its GPIO pins, which allow us to freely configure almost all of them for functions such as I2C or UART communication, or for use as PWM. Furthermore, the ESP32 is already a fairly mainstream microcontroller, so there is ample information and libraries available online, simplifying our work. Of the different ESP32 variations for our main microcontroller, we chose the ESP32 S3 because it is newer, has extra pins, and more RAM, SRAM, and ROM capacity. However, a standard ESP32 could have also served our purpose with minor adjustments. |
| **LOCATION:** | The microcontroller is located on the top platform, under the upper trim cover. It is connected to an expansion board, which facilitates connecting and replacing components. |
| **VOLTAGE:** | 3.3V (can be powered via a 5V pin connected to a linear regulator) |

----

#### HUSKYLENS 2

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/components-photos/huskylens-2-photo.jpg" width="300">

| | |
| -------- | ------- |
| **WHY:** | For much of the robot's development, we tried using an ESP32 S3 CAM with a special lens. We originally chose this option because we thought it would be simpler than using a Huskylens 2, and because our experience with the Huskylens 1 had already been complicated. However, as development progressed, we realized it was actually much more complex. With the ESP32 S3 CAM, we had to manually calibrate several values ​​per code for it to detect colors, and the detection algorithms weren't very effective. In contrast, after some tests with the Huskylens 2, we saw that its ability to be calibrated using the built-in screen and button, along with its improved color detection algorithms, led us to choose it. We had to adapt part of the obstacle code to this new sensor. |
| **LOCATION:** | It is located on the front of the robot, it has an special stand that keeps it above the robot and looking with a small angle down.|
| **VOLTAGE:** | 5V (can also be powered via 3.3V) |
| **CONNECTION TYPE:** | I2C |

----

#### L298N MOTOR DRIVER

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/components-photos/l298n-motor-driver-photo.jfif" width="300">

| | |
| -------- | ------- |
| **WHY:** | We chose this motor driver because we already had it in the classroom, it generally delivers good results, and it's inexpensive and readily available should we need replacements. In the future, we would like to test the TB6612FNG, as we know it also offers good or even better results and takes up less space. In any case, the L298 motor has not presented any problems in all the tests performed and is easy to operate. |
| **LOCATION:** | It is located on the underside of the robot's upper platform. |
| **MOTOR VOLTAGE:** | 12V |
| **LOGIC VOLTAGE:** | 5V (In our case, it is obtained through a linear regulator from the 12V motor voltage) |
| **CONNECTION TYPE:** | PWM (speed control), 2 pins (to control the motor's direction or brake) |

---

#### MOTOR

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/components-photos/12v-motor-photo.jpg" width="300">

| | |
| -------- | ------- |
| **WHY:** | This motor is a 12V DC motor. Initially, we tested it because it was the one we already had in class, and we wanted to avoid buying new motors if the one we already had was working. It is true that this motor is somewhat bulky compared to other alternatives, but in previous, heavier projects, it never presented any torque or power problems, and the robot was already planned to be somewhat bulky, so we thought it would be a good option. The robot's speed isn't a major concern for us, as the time taken to complete the challenges is low on the tie-breaking priority list in the rules. |
| **LOCATION:** | On the back of the robot's lower platform. |
| **VOLTAGE:** | 12V |

---

#### TF-LUNA

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/components-photos/tf-luna-photo.jpg" width="300">

| | |
| -------- | ------- |
| **WHY:** | To complete the missions, it's essential to know the robot's position relative to the walls. This requires considering the robot's distance from the walls. There are several types of distance sensors: On the one hand, ultrasonic sensors are quite simple, inexpensive, and easy to find, but they offer low accuracy and vary significantly over long distances, which caused us considerable problems in this same challenge last year. Therefore, we quickly ruled them out. Another option is to use Time-of-Flight (ToF) sensors. These sensors emit a beam of light and calculate the time between its emission, reflection, and return, allowing for precise determination of the distance to the reflected object, since the speed of light is constant. We noticed that many WRO projects use a rotating ToF sensor called LiDAR. After conducting some tests with a LiDAR we had and reading reviews from others who had worked with this sensor, we found it difficult to use, so we ruled out LiDAR. Finally, we selected fixed ToF sensors, the TF-Luna. The TF-Luna is relatively inexpensive, and after our initial testing, we found it to be quite accurate. Specifically, we tested it by measuring multiple distances between 5 cm and 3 m (dimensions designed for the Future Engineers challenge), and in every case, the distance obtained with the TF-Luna perfectly matched the physical measurement taken with a meter stick. |
| **LOCATION:** | We have three sensors, one on each front side and one at the front. The side sensors are angled slightly forward. These sensors are mounted on a specially designed bracket and anchored to the lower platform. |
| **VOLTAGE:** | 5V |
| **CONNECTION TYPE:** | I2C |

---

#### ADAFRUIT TCS34725

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/components-photos/adafruit-tcs34725-photo.jpg" width="300">

| | |
| -------- | ------- |
| **WHY:** | To determine the robot's position on the board and the number of sections traversed, we use orange lines conveniently placed at the corners of the board. We measure the robot's movement over these lines using this color sensor. We chose this sensor after finding positive online reviews and because it offered an I2C connection. Before using it, we calibrated it, and after testing, we found it to be reliable and accurate, so we kept it. |
| **LOCATION:** | The lower platform has a hole facing the floor for the color sensor. |
| **VOLTAGE:** | 3.3V |
| **CONNECTION TYPE:** | I2C |

---

#### MG995 SERVOMOTOR

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/components-photos/mg995-servomotor-photo.png" width="300">

| | |
| -------- | ------- |
| **WHY:** | A servomotor is necessary to control the steering of the front wheels. This servomotor was chosen because it offers higher torque (9.4 kg/cm) compared to other popular options like the SG90 (1.8 kg/cm), while maintaining a relatively low price and good availability. Price is a particularly important factor in this case, as our experience competing in this category shows that servomotors tend to wear out considerably during training and competition, requiring frequent replacement. Therefore, we prioritized a good price. |
| **LOCATION:** | On the lower platform. |
| **VOLTAGE:** | 5V |
| **CONNECTION TYPE:** | I2C |

---

#### LM2596 CONVERTERS

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/components-photos/lm2596-converter-photo.jpg" width="300">

| | |
| -------- | ------- |
| **WHY:** | The 12V we get from the battery needs to be converted to 5V and 3.3V for the other components to function. This requires a converter. There are mainly two types: linear and switched-mode. Linear converters, like the LM7805, have the problem of being energy inefficient and generating a lot of heat when converting excess voltage. On the other hand, switched-mode converters offer higher efficiency and generate less heat. Therefore, we selected a switched-mode voltage converter. The advantage of choosing this particular version of the LM2596 is that it is adjustable, which allows us to have several identical converters in stock to meet the robot's needs. It also has a display that shows the output voltage in real time, useful for monitoring the power status without needing a multimeter. We have 2 12V to 5V converters because after installing the Huskylens 2 we saw that amount of current it used exceded the limits and made the voltage drop to the point that all the components connected to the only 5V converter stopped working, so we added one 12V to 5V only for the Huskylens 2 and another for all the rest of components.|
| **LOCATION:** | 2 are located on the underside of the robot's upper platform and the Huskylens 2 5V converter is located on top of the robot's upper platform. |

---

#### SWITCH

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/components-photos/power-switch-photo.jpg" width="300">

| | |
| -------- | ------- |
| **WHY:** | According to regulations, a switch is required to cut off or activate the robot's power. |
| **LOCATION:** | On the side of the robot's upper platform. |

---

#### BUTTON

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/components-photos/button-photo.jfif" width="300">

| | |
| -------- | ------- |
| **WHY:** | According to regulations, a separate button is required to activate the robot's execution of the challenge. |
| **LOCATION:** | On the robot's upper platform. |

### 3.2. ELECTRICAL DIAGRAM

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/diagrams/electrical-diagram.jpg" width="1000">

### 3.3. CONNECTION TYPE DIAGRAM

This diagram shows the connection type of each component to the main microcontroller. The pink squares represent the microcontroller. The blue octagons are the sensors that send information to the main microcontroller. The yellow circles are the actuators.

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/diagrams/connection-type-diagram.png" width="500">

### 3.4. POWER CONSUMPTION

| COMPONENT | VOLTAGE (V) | CURRENT (mA) |
| -------- | -------: | -------: |
| 3 * LM2596 Converters | 12 | 50 |
| Servomotor MG995 | 5 | ≈ 200 - 1200 |
| Adafruit TCS34725 | 3.3 | 10 |
| Motor | 12 | ≈ 300 |
| 3 * TF-Luna | 5 | 70 |
| L298N Motor driver | Vm.: 12; Vlog.: 5 | 36 |
| Huskylens 2 | 5 | ≈ 300 - 600 |
| ESP32 S3 | 3.3 | ≈ 50 |


## 4. CODE

We can find a description of the operation and algorithms of the different codes by clicking on the following links.

- [**Open Challenge**](/code/open-challenge/): ESP32 main code for the Open Challenge
- [**Obstacle Challenge**](/code/obstacle-challenge/): ESP32 main code for the Obstacle Challenge

This diagram shows what code needs to be uploaded to the microcontroller.

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/diagrams/code-diagram.png" width="350">

## 5. PHOTOS

### 5.1. ROBOT PHOTOS

| | |
| --- | --- |
| <img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-front.jpeg" width="500"> | <img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-back.jpeg" width="500"> |
| <img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-left.jpeg" width="500"> | <img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-right.jpeg" width="500"> |
| <img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-top.jpeg" width="500"> | <img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-bottom.jpeg" width="500"> |

### 5.2. TEAM PHOTOS

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/team-photos/team-photo-1.jpeg" width="600">

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/team-photos/team-photo-2.jpg" width="600">

## 6. VIDEOS

We have uploaded two videos showing our robot completing each challenge. You can access them via the following links:
* [Open Challenge](https://youtu.be/gIDQgGI8u58)
* [Obstacle Challenge](example.com)


## 7. LIST OF COMPONENTS

* **3 * LM2596 Converters**
* **1 * Servomotor MG995** 
* **1 * Adafruit TCS34725** 
* **1 * 12V DC Motor** 
* **3 * TF-Luna** 
* **1 * LM2596 Converters**
* **1 * Huskylens 2**
* **1 * OV5640**
* **1 * ESP32 S3**

## 8. ENGINEERING JOURNAL

As we mentionad on the introduction, to comply with the competition rules, we have also included a [**Engineering Journal**](/engineering-journal.pdf) in PDF format. This document is a compilation of the information explained throughout this repository.

We highly recommend checking out this document, as it is a better looking version of what can be found here at the repository and easier to use.

## 9. LICENSE

The code in this repository is licensed under the GNU General Public License v3.0.

Everything not under the GNU General Public License v3.0 in this repository (including photographs, diagrams, etc) is under the Creative Commons Attribution Share Alike 4.0 International license

A copy of each license can be found in the [LICENSE](LICENSE.txt) file.


