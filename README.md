----
----
----
**HAY QUE CAMBIAR LAS URL DE LAS IMÁGENES AL PASAR EL README AL REPOSITORIO PRINCIPAL** 
----
----
----


# VILA-STEM 8’s repository
**This is the Vila-stem 8 repository for the 2026 season WRO Future Engineers challenge**

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/team-photos/team-photos-1.png" width="600">

## TABLE OF CONTENTS

## 1. INTRODUCTION

## 2. MOBILITY MANAGEMENT

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

#### ESP32 S3 CAM AND OV5640

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/components-photos/esp32-s3-cam-photo.jpg" width="300">

| | |
| -------- | ------- |
| **WHY:** | We chose a variant of the ESP32 S3 that includes a camera connection port as the camera data processor. This microcontroller was selected because, being the same type as the main microcontroller, we could work more efficiently with both, only needing to become familiar with one variant. We have two microcontrollers because analyzing camera data is a very resource-intensive process, and therefore it was better to dedicate a microcontroller to it. We selected the OV5640 from several cameras already available in the classroom because it provided the best view of the track after testing all of them. |
| **LOCATION:** | The microcontroller is located on the upper platform, beneath the top trim cover. The camera is mounted on a front bracket and connected to the microcontroller by a cable. |
| **VOLTAGE:** | 3.3V (can be powered via a 5V pin connected to a linear regulator) |
| **CONNECTION TYPE:** | UART communication |

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
| **WHY:** | The 12V we get from the battery needs to be converted to 5V and 3.3V for the other components to function. This requires a converter. There are mainly two types: linear and switched-mode. Linear converters, like the LM7805, have the problem of being energy inefficient and generating a lot of heat when converting excess voltage. On the other hand, switched-mode converters offer higher efficiency and generate less heat. Therefore, we selected a switched-mode voltage converter. The advantage of choosing this particular version of the LM2596 is that it is adjustable, which allows us to have several identical converters in stock to meet the robot's needs. It also has a display that shows the output voltage in real time, useful for monitoring the power status without needing a multimeter. |
| **LOCATION:** | Both are located on the underside of the robot's upper platform. |

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

This diagram shows the connection type of each component to the main microcontroller. The pink squares represent the microcontrollers. The blue octagons are the sensors that send information to the main microcontroller. The yellow circles are the actuators.

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/diagrams/connection-type-diagram.png" width="500">

### 3.4. POWER CONSUMPTION

| COMPONENT | VOLTAGE (V) | CURRENT (mA) |
| -------- | -------: | -------: |
| 2 * LM2596 Converters | 12 | 50 |
| Servomotor MG995 | 5 | ≈ 200 - 1200 |
| Adafruit TCS34725 | 3.3 | 10 |
| Motor | 12 | ≈ 300 |
| 3 * TF-Luna | 5 | 70 |
| L298N Motor driver | Vm.: 12; Vlog.: 5 | 36 |
| ESP32 S3 CAM and OV5640 | 3.3 | ≈ 100 |
| ESP32 S3 | 3.3 | ≈ 50 |

## 4. OBSTACLE MANAGEMENT
### 4.1. OPEN CHALLENGE
#### 4.1.1. Strategy
#### 4.1.2. Failures and improvement
### 4.2. OBSTACLE CHALLENGE
#### 4.2.1. Strategy
#### 4.2.2. Failures and improvement

## 5. CODE

We can find a description of the operation and algorithms of the different codes by clicking on the following links.

- [**Open Challenge**](/code/open-challenge/): ESP32 main code for the Open Challenge
- [**Obstacle Challenge**](/code/obstacle-challenge/): ESP32 main code for the Obstacle Challenge
- [**Camera Code**](/code/camera-code/): ESP32 camera code for the Obstacle Challenge

This diagram shows what code needs to be uploaded to each microcontroller.

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/diagrams/code-diagram.png" width="500">

## 6. PHOTOS

### 6.1. ROBOT PHOTOS

| | |
| --- | --- |
| <img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-front.png" width="300"> | <img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-back.png" width="300"> |
| <img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-left.png" width="300"> | <img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-right.png" width="300"> |
| <img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-top.png" width="300"> | <img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/robot-photos/robot-photo-bottom.png" width="300"> |

### 6.2. TEAM PHOTOS

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/team-photos/team-photos-1.png" width="400">

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/team-photos/team-photos-2.png" width="400">

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/photos/team-photos/team-photos-3.png" width="400">

## 7. LIST OF COMPONENTS

* **2 * LM2596 Converters**
* **1 * Servomotor MG995** 
* **1 * Adafruit TCS34725** 
* **1 * 12V DC Motor** 
* **3 * TF-Luna** 
* **1 * LM2596 Converters**
* **1 * ESP32 S3 CAM**
* **1 * OV5640**
* **1 * ESP32 S3**


## 8. LIST OF 3D DESIGNS

## 9. LICENSE
