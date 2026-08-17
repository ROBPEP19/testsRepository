# OPEN_PRUEBA/OPEN_PRUEBA.ino

# Introduction to the code

Although at first glance the code for the Open Challenge may seem trivial due to its apparent simplicity compared to the Obstacle Challenge, our goal with this code was not only to create something simple to quickly overcome this challenge, but also to create a foundation upon which to build the Obstacle Challenge code. With this idea in mind, our algorithm design and code structure have been designed to facilitate this transition.

On the other hand, we understand that the Open Challenge should demonstrate the minimum the robot is capable of, which is why we have also prioritized creating robust yet simple code.

# Code structure

All the code for this challenge is contained in a single .ino file, named OPEN_PRUEBA.ino. This code is executed in the robot's ESP32 main.

There are no other programs for this challenge.

We have not created any additional libraries or files as required.

# Algorithm design

A flowchart of the code's operation can be seen below.

<img src = "https://github.com/ROBPEP19/testsRepository/blob/main/code/Open%20Challenge/Open_Challenge_Flowchart.png" width="500">

As can be seen, as soon as the robot is powered on, it enters standby mode until the Start button is pressed. Meanwhile, as observed in the source code, the robot's various systems and peripherals are initialized (I2C communication, color sensor, servomotor, etc.).

Once the Start button is pressed, the robot enters the loop. We can divide the loop's operation into two parts:
1. Robot Movement
2. Limit Case Checking

## Robot movement

The strategy here is simple: keep the robot centered between the side walls at all times.

To achieve this, the Luna ToF sensors located on the sides of the robot measure the distance to the right and left. Then, using the difference between these distances, the servomotor is instructed to turn the wheels in the corresponding direction. For example, if the distance to the right is greater than to the left, the robot will turn its wheels to the right, and if the distance to the left is greater than to the right, the wheels will turn to the left. The benefit of using the difference in distances is that it allows the servomotor to turn only slightly if the difference is small, while if the difference is very large, it turns much more, allowing for better cornering.

With this, after testing, we have ensured that regardless of the direction of rotation, the starting position, or the size of the central square, the robot always remains centered.

## Check of limit cases

The loop also performs certain checks at all times on the following things, as shown in the flowchart:
- If the color sensor facing the ground detects the blue line, then the internal counter corner_count, which measures how many corners the robot has passed through, is incremented by one. This tells us how many sections the robot has traversed.
- If the front Luna ToF sensor measures a distance less than 350mm, the motor speed is reduced to avoid a collision. This can sometimes happen when getting too close to walls.
- If the front Luna ToF sensor measures a distance less than 150mm, a collision is practically imminent, and the reversing process is activated. This temporarily disables the robot's movement and makes it reverse to avoid getting stuck.
- If the corner_counter is equal to 12, the robot has already completed three laps and must stop in the final section.

# Bugs and errors found in the code

Throughout the development of this code, we encountered various problems and challenges that we had to resolve. This, in turn, allowed us to create a more robust code than we originally envisioned.

Initially, some minor problems we had included incorrectly assigning the I2C address to each Luna ToF sensor and missing resistors here and there. But above all, we would highlight the following problems we encountered, as their solution has improved the code's resilience, and we believe other users might encounter similar issues.

- One problem we found was that the Luna ToF sensors, when measuring very long distances (which we considered to be infinity), returned a measurement of 0 mm. On the other hand, when very close to a wall, since they weren't designed to measure such short distances, they also returned some erroneous values, highly variable, and in many cases, also 0 mm. This presented the problem of how to distinguish between 0 at a very long distance and 0 at a very short distance. For this, we used the "strength" data provided by the Luna ToF sensors themselves. After several tests, we verified that the strength was minimal when it was 0 at infinity and very high when it was 0 at very close range. Therefore, every time we obtained a value of 0, we performed a second check with the strength to determine what type of 0 it was. If it was 0 at infinity, we set it to 1500 mm.
- Sometimes, when the robot had to make very tight turns, it would pick up a lot of speed and crash into the walls. We originally designed a reversing mechanism that should have solved this problem. However, the frequency and manner of the collisions caused it to take a long time to complete the circuit, and in many cases, the reversing maneuver was ineffective. This was resolved by having the robot reduce its speed when it detects that it is approaching the walls. In many cases, this avoids the collision, and in those where it doesn't, the slower collision usually leaves the robot in a better position to more easily complete the circuit after reversing.

# Source code description

## Functions

|                | Name           |
| -------------- | -------------- |
| void IRAM_ATTR | **[OdometerInterrupt](#function-odometerinterrupt)**() |
| uint16_t | **[LunaRead16](#function-lunaread16)**(uint8_t addr, uint8_t reg) |
| uint16_t | **[LunaDistance](#function-lunadistance)**(uint8_t addr) |
| void | **[Drive](#function-drive)**() |
| void | **[ServoTurning](#function-servoturning)**(signed int _distance_right, signed int _distance_left) |
| void | **[setup](#function-setup)**() |
| void | **[loop](#function-loop)**() |

## Variables

|                | Name           |
| -------------- | -------------- |
| const uint8_t | **LUNA_COUNT**  |
| const uint8_t[LUNA_COUNT] | **LUNA_ADDR**  |
| const uint16_t | **LUNA_STRENGHT_MIN**  |
| const uint16_t | **LUNA_CLOSE_STRENGTH**  |
| const uint16_t | **LUNA_CLOSE_MM**  |
| const uint16_t | **FRONT_BRAKE_MM**  |
| const uint16_t | **FRONT_REAR_MM**  |
| const int | **MOTOR_B_PIN**  |
| const int | **MOTOR_A_PIN**  |
| const int | **ENA_PIN**  |
| const int | **BUTTON_PIN**  |
| const int | **ODOMETER_PIN**  |
| const int | **SDA_PIN**  |
| const int | **SCL_PIN**  |
| const int | **SDA_PIN_COLOR**  |
| const int | **SCL_PIN_COLOR**  |
| const int | **SERVO_PIN**  |
| const int | **BASE_SPEED**  |
| Adafruit_TCS34725 | **tcs** <br>Color sensor TCS34725 object.  |
| Servo | **servo** <br>Servo object.  |
| int | **button_state** <br>START button state.  |
| int | **corner_count** <br>Counts the number of corners the robot has passed. It should go from 0 to 12.  |
| long | **delay_line** <br>NOT IN USE.  |
| int | **travel_sense** <br>Controls travel sense. 0 = BRAKE, 1 = FORWARD, 2 = BACKWARD.  |
| byte | **rear_phase** <br>Controls the rear phases. 0 = NORMAL, 1 = STOP, 2 = REVERSE, 3 = RESUME.  |
| long | **end_motor_millis** <br>Variable to control the additional time added from entering the last section until the motor stops.  |
| long | **prev_millis_line_detection** <br>Variable to control time and avoid multiple detections of the same line.  |
| long | **prev_millis_front** <br>Variable to control time and avoid multiple detections of the wall from the front sensor.  |
| unsigned long | **rear_phase_until** <br>Variable to control time for the reverse system.  |
| long | **odo_filter** <br>Variable to control time and avoid multiple detections of the same step of the odometer.  |
| int | **odo_integral** <br>Odometer step count.  |
| int | **d_servo** <br>How much the servo needs to move.  |


## Functions Documentation

### function OdometerInterrupt

```cpp
void IRAM_ATTR OdometerInterrupt()
```


**Parameters**: 

  * **none** 


**Return**: void 

Odometer interrupt. It uses the ESP32 interrupt management system so that when the odometer moves, this code is executed.

--------------

### function LunaRead16

```cpp
uint16_t LunaRead16(uint8_t addr,uint8_t reg)
```


**Parameters**: 

  * **addr** I2C address of the ToF Luna that you wish to request 
  * **reg** Value to be requested. 0x00 for distance. 0x02 for strength 


**See**: [LunaDistance()](#function-lunadistance)

**Return**: Desired value of the ToF Luna in question 

Request the demanded value of the indicated ToF Luna via I2C connectivity. 

--------------

### function LunaDistance

```cpp
uint16_t LunaDistance(uint8_t addr)
```


**Parameters**: 

  * **addr** I2C address of the ToF Luna that you wish to request 


**See**: [LunaRead16()](#function-lunaread16)

**Return**: 

  * Distance in mm measured 
  * 0xFFFF if failed to measure 


This function returns the distance in mm measured by the requested Luna ToF and also ensures that this measurement makes sense using the strength of the Luna ToF, thus giving consistent results in borderline cases.

--------------

### function Drive

```cpp
void Drive()
```


**Parameters**: 

  * **none** 


**See**: 

  * [LunaDistance()](#function-lunadistance)
  * [ServoTurning()](#function-servoturning)


**Return**: void 

The driving and collision avoidance algorith. 

--------------

### function ServoTurning

```cpp
void ServoTurning(signed int _distance_right,signed int _distance_left)
```


**Parameters**: 

  * **_distance_right** The right distance measured by the ToF Luna sensor 
  * **_distance_left** The left distance measured by the ToF Luna sensor 


**See**: [Drive()](#function-drive)

**Return**: void 

The servo turning algorith. 

--------------

### function setup

```cpp
void setup()
```


**Parameters**: 

  * **none** 


**Return**: void 

This code only runs once when the ESP32 starts up. Initialize all systems and peripherals and wait for the start button to be pressed to start the program. 

--------------

### function loop

```cpp
void loop()
```


**Parameters**: 

  * **none** 


**Return**: void 

This code only runs all the time. 


## Source code

```cpp
/***********************************************************************
 *  FILE NAME:  OPEN_PRUEBA
 *
 *
 *  PURPOSE:    This program is the version of the main 
 *              robot code for the Open Challenge of the 
 *              Future Engineers category of the World
 *              Robot Olympiad.
 *  
 *
 *  HOW IT WORKS: The program uses side time-of-flight 
 *                distance sensors to keep the robot 
 *                constantly aligned with the center line 
 *                between the two side walls. Additionally, 
 *                in the event of a frontal collision or 
 *                when approaching a collision, the front 
 *                sensor is used to move backward or reduce 
 *                speed, respectively.
 *
 *
 *  LIBRARIES USED:
 *  Name                        Description
 *  ----                        -----------
 *  ESP32Servo                  Functions required to control a 
 *                               servomotor with the ESP32
 *  Wire                        Allows I2C communication
 *  Adafruit_TCS34725           Functions required to use the 
 *                               Adafruit TCS34725 RGB Color Sensor
 *
 *
 *  OTHER PROGRAMS NEEDED TO WORK
 *  Name                        Description
 *  ----                        -----------
 *  none
 *
 *
 *  CREATED BY: Vila-Stem 8
 *
 *                                                
 *  LAST MODIFIED: August 16th 2026                                     
 *                                                                      
 *                                                                      
 *  REPOSITORY: https://github.com/Vila-Stem/WRO_FUTURE_ENGINEERS_2026  
 *
 ***********************************************************************
*/


/* INCLUDE LIBRARIES */
#include <ESP32Servo.h>                                                   
#include <Wire.h>
#include <Adafruit_TCS34725.h>


/* CONSTANT VALUES */
/* Related to the Luna time of flight (ToF) sensor */
const uint8_t LUNA_COUNT = 3;                               // Number of Luna time of flight sensors
const uint8_t LUNA_ADDR[LUNA_COUNT] = {0x11, 0x12, 0x13};   // I2C address of each ToF sensor. Right, Left, Front
const uint16_t LUNA_STRENGHT_MIN = 20;                      // Minimum strenght needed from the ToF sensor to accept a measurement
const uint16_t LUNA_CLOSE_STRENGTH = 150;                   // Minumum strength needed to consider a 0 distance measurement too close
const uint16_t LUNA_CLOSE_MM = 150;                         // Value (mm) used in case of a too close measurement

/* Collision avoidance threshold distances */
const uint16_t FRONT_BRAKE_MM = 350;                        // Progressive braking start distance (mm) threshold
const uint16_t FRONT_REAR_MM = 150;                         // Reverse gear start distance (mm) threshold

/* Pins */
const int MOTOR_B_PIN = 20;
const int MOTOR_A_PIN = 10;
const int ENA_PIN = 16;                                     // Enabled motor pin. Used with PWM for speed control
const int BUTTON_PIN = 13;                                  // START button pin
const int ODOMETER_PIN = 15;                                // Odometer pin
const int SDA_PIN = 11;
const int SCL_PIN = 12;
const int SDA_PIN_COLOR = 40;
const int SCL_PIN_COLOR = 41;
const int SERVO_PIN = 9;

const int BASE_SPEED = 120;


/* CREATE OBJECTS */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);    ///< Color sensor TCS34725 object
Servo servo;                                                                                    ///< Servo object


/* VARIABLES */
int button_state = HIGH;              ///< START button state
int corner_count = 0;                 ///< Counts the number of corners the robot has passed. It should go from 0 to 12.
long delay_line = 0;                  ///< NOT IN USE
int travel_sense = 1;                 ///< Controls travel sense. 0 = BRAKE, 1 = FORWARD, 2 = BACKWARD
byte rear_phase = 0;                  ///< Controls the rear phases. 0 = NORMAL, 1 = STOP, 2 = REVERSE, 3 = RESUME
long end_motor_millis = 0;            ///< Variable to control the additional time added from entering the last section until the motor stops
long prev_millis_line_detection = 0;  ///< Variable to control time and avoid multiple detections of the same line
long prev_millis_front = 0;           ///< Variable to control time and avoid multiple detections of the wall from the front sensor
unsigned long rear_phase_until = 0;   ///< Variable to control time for the reverse system
long odo_filter = 0;                  ///< Variable to control time and avoid multiple detections of the same step of the odometer
int odo_integral = 0;                 ///< Odometer step count
int d_servo = 0;                      ///< How much the servo needs to move


/* FUNCTIONS */
/**
 * Odometer interrupt. It uses the ESP32 interrupt management 
 * system so that when the odometer moves, this code is executed.
 * @param none
 * @return void
 */
void IRAM_ATTR OdometerInterrupt(){
  if ((micros()-odo_filter)>1000)   // Avoids double detecting the same step
  {
    switch (travel_sense)
    {
      case 0:
        odo_integral++;
        break;
      case 1:
        odo_integral++;
        break;
      case 2:
        odo_integral--;
        break;
    }
    odo_filter = micros();
  }
}


/**
 * Request the demanded value of the indicated ToF Luna via I2C connectivity.
 * @param addr I2C address of the ToF Luna that you wish to request
 * @param reg Value to be requested. 0x00 for distance. 0x02 for strength
 * @return Desired value of the ToF Luna in question
 * @see LunaDistance()
 */
uint16_t LunaRead16(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);                         // Begins I2C communication with the given address 
  Wire.write(reg);                                      // Tells ToF Luna that we desire the given type of information
  Wire.endTransmission(false);                          // Keeps the I2C communication active with the ToF Luna
  Wire.requestFrom(addr, (uint8_t)2);                   // Requests 2 bytes data from the ToF Luna. It releases the I2C bus after the request.
  if (Wire.available() < 2) return 0xFFFF;              // If less than 2 bytes retrieved returns 0xFFFF as failed
  return (uint16_t)(Wire.read() | (Wire.read() << 8));  // Returns the data
}


/**
 * This function returns the distance in mm measured by the requested 
 * Luna ToF and also ensures that this measurement makes sense using 
 * the strength of the Luna ToF, thus giving consistent results in 
 * borderline cases.
 * @param addr I2C address of the ToF Luna that you wish to request
 * @return Distance in mm measured
 * @return 0xFFFF if failed to measure
 * @see LunaRead16()
 */
uint16_t LunaDistance(uint8_t addr) {
  uint16_t d = LunaRead16(addr, 0x00);          // Request ToF Luna distance
  uint16_t strength = LunaRead16(addr, 0x02);   // Request ToF Luna stregth
  if (d == 0xFFFF || strength == 0xFFFF || strength < LUNA_STRENGHT_MIN) return 0xFFFF; // Returns 0xFFFF if failed to measure or the strenght is too low
  if (d == 0)                                   // The distance may be 0 because the sensor is too close to a wall and is saturated, or because it measures infinity
  {
    if (strength >= LUNA_CLOSE_STRENGTH) return LUNA_CLOSE_MM;  // If the strength is very high, the sensor is too close to the wall.
    return 1500;                                                // If the strength is very low, the sensor measures infinity.
  }
  return d*10;  // The ToF Luna returns cm, but the code neeeds mm
}


/**
 * The driving and collision avoidance algorith.
 * @param none
 * @return void
 * @see LunaDistance()
 * @see ServoTurning()
 */
void Drive() 
{  
  // Initialize variable for reading the ToF sensors
  static signed int distance_right,distance_left,distance_front;
  uint16_t d;
  
  // These lines read the ToF Luna sensors and save the distance 
  // in the corresponding variable as long as it hasn't failed reading.
  d = LunaDistance(LUNA_ADDR[0]);                                         // Right distance sensor
  if (d != 0xFFFF) 
  {
    distance_right = d;
    if (distance_right>1500) distance_right=1500;                           // If the distance is greater than 1500mm, it limits it to 1500mm.
    //Serial.print("distance_right: "); Serial.println(distance_right);     // FOR DEBUGGING.
  }
  d = LunaDistance(LUNA_ADDR[1]);                                         // Left distance sensor
  if (d != 0xFFFF) 
  {
    distance_left = d;
    if (distance_left>1500) distance_left=1500;                             // If the distance is greater than 1500mm, it limits it to 1500mm.
    //Serial.print("distance_left: "); Serial.println(distance_left);       // FOR DEBUGGING.
  }
  d = LunaDistance(LUNA_ADDR[2]);                                         // Front distance sensor
  if (d != 0xFFFF) 
  {
    distance_front = d;
    //Serial.print("distance_front: "); Serial.println(distance_front);
  }
  
  // If front sensor detects an obstacle init rear sequence. This sequence is time based
  if (distance_front < FRONT_REAR_MM)
  {
    if (rear_phase == 0)
    {
      if ((prev_millis_front + 1200) <= millis())   // This if statement avoids multiple detection of the same wall
      {
        prev_millis_front = millis();
        rear_phase = 1;
        rear_phase_until = millis() + 250;
      }
    }
  }

  // Rear sequence. This is time based
  if (rear_phase >= 1)
  {
    switch (rear_phase)
    {
      case 1:             // Stops the car for 250 mm (by deafult)
        travel_sense=0;
        if (millis() >= rear_phase_until){ rear_phase=2; rear_phase_until = millis() + 550; }
        break;
      case 2: 
        travel_sense=2;   // Reverse the car for 550 mm (by deafult)
        if (millis() >= rear_phase_until){ rear_phase=3; rear_phase_until = millis() + 50; }
        break;
      case 3:             // The robot continues forward.
        travel_sense=1;
        rear_phase=0;
        break;
    }
  }

  ServoTurning(distance_right, distance_left)     // Servo turning algorithm

  // Speed control
  int base_speed = BASE_SPEED;                    // Base speed of the robot

  if (distance_front < FRONT_REAR_MM)             // Reduces the speed before starting the reverse manouver
  {
    base_speed = 60;
  }
  else if (distance_front < FRONT_BRAKE_MM)       // Reduces the speed when approaching a frontal wall to avoid a collision or having to use the reverse manouver
  {
    base_speed = 75;
  }

  if (d_servo >= 0)
  {
    ledcWrite(ENA_PIN, base_speed + d_servo);     // Sends the motor the speed. It is increased when turning
  }
  else 
  {
    ledcWrite(ENA_PIN, base_speed - d_servo);     // Sends the motor the speed. It is increased when turning
  }
}


/**
 * The servo turning algorith.
 * @param _distance_right The right distance measured by the ToF Luna sensor
 * @param _distance_left The left distance measured by the ToF Luna sensor
 * @return void
 * @see Drive()
 */
void ServoTurning(signed int _distance_right, signed int _distance_left)
{
  // Initialize needed variables
  signed int distance_right = _distance_right;
  signed int distance_left = _distance_left;

  // The servo needs to move a certain amount depending on the 
  // difference of distance measured between the left and 
  // the right ToF Luna sensors
  d_servo = (distance_right - distance_left)/20;  

  //Serial.println(d_servo)                      // FOR DEBUGGING.

  // Limits the amount a servo can move in order to avoid collision of the wheels and the frame
  if (d_servo > 39) d_servo = 39;
  if (d_servo < -39) d_servo = -39;

  // In case of reverse, limited steering to avoid flipping
  if (rear_phase == 2)
  {                                 
    if (d_servo > 12) d_servo = 12;
    if (d_servo < -12) d_servo = -12;
    servo.write(-d_servo + 85);
  }
  else if (travel_sense == 1)
  {
    servo.write(d_servo + 85);
  }
}


/* SETUP AND LOOP FUNCTIONS */
/**
 * This code only runs once when the ESP32 starts up.
 * Initialize all systems and peripherals and wait for the 
 * start button to be pressed to start the program.
 * @param none
 * @return void
 */
void setup() {
  Serial.begin(115200);                             // Starts Serial for debugging reasons
  Wire.begin(SDA_PIN, SCL_PIN);                     // I2C communication for ToF Luna
  Wire.setTimeOut(100);
  Wire.setClock(100000);                            // ToF Luna at 100kHz                                                
  Wire1.begin(SDA_PIN_COLOR, SCL_PIN_COLOR);        // I2C communication for TCS34725 color sensor

  servo.attach(SERVO_PIN);                          // Initialize Servo
  servo.write(90);                                  // 0 postion Servo 
  
  // Pin Modes
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A_PIN, OUTPUT);
  pinMode(MOTOR_B_PIN, OUTPUT);

  // Stop the motor
  digitalWrite(MOTOR_A_PIN, LOW);
  digitalWrite(MOTOR_B_PIN, LOW);

  ledcAttachChannel(ENA_PIN, 1000, 8, 2);           // PWM for the motor. Frequency = 1000, Resolution = 8, Channel = 2
 
  // Checks ToF Luna presence and debug which ones don't work through the serial port.
  for (uint8_t i = 0; i < LUNA_COUNT; i++) 
  {
    Wire.beginTransmission(LUNA_ADDR[i]);
    if (Wire.endTransmission() == 0) 
    {
      Serial.print("TF-Luna ");
      Serial.print(i);
      Serial.print(" OK (0x");
      Serial.print(LUNA_ADDR[i], HEX);
      Serial.println(")");
    } 
    else 
    {
      Serial.print("TF-Luna ");
      Serial.print(i);
      Serial.print(" NOT found (0x");
      Serial.print(LUNA_ADDR[i], HEX);
      Serial.println(")");
    }
  }

  // Initialize color sensor. Debug trought the serial port if not working.
  if (tcs.begin(0x29, &Wire1)) 
  {                                           
    Serial.println("Found sensor");
  } 
  else 
  {
    Serial.println("WARNING: TCS34725 not detected, continuing without color.");
  }
  
  // Initialize odometer
  //attachInterrupt(ODOMETER_PIN, OdometerInterrupt,CHANGE);  // NOT IN USE

  // Waits for START button to be pressed
  int countButton = 0;
  while (countButton <= 10) {
    button_state = digitalRead(BUTTON_PIN);
    if (button_state == LOW) countButton++;
    delay(10);
  }

  Serial.println("START");
}


/**
 * This code only runs all the time.
 * @param none
 * @return void
 */
void loop() {
  // Gets color temp from the Adafruit TCS34725 color sensor
  uint16_t r, g, b, c, colorTemp;                                 // Initialize needed variables
  tcs.getRawData(&r, &g, &b, &c);                                 // Gets the RGB and Clear channels
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);     // Calculates the color temperature from the RGBC channels
  
  //Serial.print("Color Temp: "); Serial.println(colorTemp, DEC); // FOR DEBUGGING. Use it to calibrate the sensor

  // Increases corner count using the lines on the ground and the TCS34725 color sensor.
  if (colorTemp > 3000)
  {
    if ((prev_millis_line_detection + 700) <= millis())
    {
      prev_millis_line_detection = millis();
      corner_count++;
    }
  } 

  // Stops the car when the number of laps is reached (12 corners)
  if (corner_count >= 12) 
  {
    if ((end_motor_millis + 1000) <= millis()) 
    {
      travel_sense=0;    
    }
  } 
  else 
  {    
    end_motor_millis = millis();
  }

  // Sets the travel sense. 0 = BRAKE, 1 = FORWARD, 2 = BACKWARD
  switch(travel_sense)
  {
    case 0: 
      digitalWrite(MOTOR_A_PIN, LOW);
      digitalWrite(MOTOR_B_PIN, LOW);
      break;
    case 1: 
      //digitalWrite(MOTOR_A_PIN, LOW);     // FOR DEBUGGING.
      digitalWrite(MOTOR_A_PIN, HIGH);
      digitalWrite(MOTOR_B_PIN, LOW);
      break;
    case 2: 
      digitalWrite(MOTOR_A_PIN, LOW);
      //digitalWrite(MOTOR_B_PIN, LOW);     // FOR DEBUGGING.
      digitalWrite(MOTOR_B_PIN, HIGH);
      break;
  }
  
  Drive();        // The driving and collision avoidance algorith
}
```


