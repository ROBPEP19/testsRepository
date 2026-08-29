/***********************************************************************
 *  FILE NAME:  obstacle-challenge
 *
 *
 *  PURPOSE:    This program is the version of the main 
 *              robot code for the Obstacle Challenge of the 
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
 *                speed, respectively. It is connected to another
 *                ESP32 S3 CAM that tells if there is an object,
 *                its position and color, so this code makes the 
 *                servo turn enough no pass the obstacle through
 *                the right side.
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
 *  camera-code                 The camera code runs on the ESP32
 *                               S3 CAM. Tells this program the
 *                               color and position of the obstacles
 *
 *
 *  CREATED BY: Vila-Stem 8
 *
 *                                                
 *  LAST MODIFIED: August 29th 2026                                     
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
const int ENA_PIN = 15;                                     // Enabled motor pin. Used with PWM for speed control
const int BUTTON_PIN = 13;                                  // START button pin
const int ODOMETER_PIN = 15;                                // Odometer pin
const int SDA_PIN = 11;
const int SCL_PIN = 12;
const int SDA_PIN_COLOR = 40;
const int SCL_PIN_COLOR = 41;
const int SERVO_PIN = 9;
const int ESP32_CAM_RX = 16;
const int ESP32_CAM_TX = 17;

const int BASE_SPEED = 120;

const int CAM_TIMEOUT_MS = 1000;                            // Time it waits for the ESP32 S3 CAM responds


/* CREATE OBJECTS */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
Servo servo;


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
int d_servo = 0;                      ///< How much the servo needs to move
unsigned long last_cam_request = 0;
int cam_correction = 0;
int cam_count = 0;


/* FUNCTIONS */
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
 * The driving, collision avoidance and traffic light avoidance mechanism
 * algorith.
 * @param none
 * @return void
 * @see LunaDistance()
 * @see ServoTurning()
 */
void Drive() {

  // Initialize variable for reading the ToF sensors
  static signed int distance_right, distance_left, distance_front;
  uint16_t d;

  // Initialize variables for the camera information
  static int count_correction = 0;
  static signed int correction = 0 ;

  // Request camera information every 100 ms
  if (millis() - last_cam_request > 100) 
  {
    last_cam_request = millis();
    RequestCamera();
  }

  // It executes only if a traffic light has been detected
  if (cam_count > 0) {
    count_correction = cam_count;
    correction = cam_correction;
    cam_count = 0;
  }

  // These lines read the ToF Luna sensors and save the distance 
  // in the corresponding variable as long as it hasn't failed reading.
  d = LunaDistance(LUNA_ADDR[0]);                                         // Right distance sensor
  if (d != 0xFFFF) 
  {
    distance_right = d;
    if (distance_right>1500) distance_right=1500;                           // If the distance is greater than 1500mm, it limits it to 1500mm.
  }
  d = LunaDistance(LUNA_ADDR[1]);                                         // Left distance sensor
  if (d != 0xFFFF) 
  {
    distance_left = d;
    if (distance_left>1500) distance_left=1500;                             // If the distance is greater than 1500mm, it limits it to 1500mm.
  }
  d = LunaDistance(LUNA_ADDR[2]);                                         // Front distance sensor
  if (d != 0xFFFF) 
  {
    distance_front = d;
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
        if (millis() >= rear_phase_until){ rear_phase=2; rear_phase_until = millis() + 1500; }
        break;
      case 2: 
        travel_sense=2;   // Reverse the car for 1500 mm (by deafult)
        if (millis() >= rear_phase_until){ rear_phase=3; rear_phase_until = millis() + 50; }
        break;
      case 3:             // The robot continues forward.
        travel_sense=1;
        rear_phase=0;
        break;
    }
  }

  // The servo needs to move a certain amount depending on the 
  // difference of distance measured between the left and 
  // the right ToF Luna sensors
  d_servo = (distance_right - distance_left)/10;  //

  // Limits the amount a servo can move in order to avoid collision of the wheels and the frame
  if (d_servo > 39) d_servo = 39;
  if (d_servo < -39) d_servo = -39;

  if (count_correction > 0) 
  {
    d_servo = correction;
    count_correction--;
  }

  // In case of reverse, limited steering to avoid flipping
  if (rear_phase == 2)
  {                                 
    if (d_servo > 12) d_servo = 12;
    if (d_servo < -12) d_servo = -12;
    servo.write(-d_servo + 90);
  }
  else if (travel_sense == 1)
  {
    servo.write(d_servo + 90);
  }

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

  ledcWrite(ENA_PIN, base_speed);                  // Sends the motor the speed.
}


/**
 * Reads the incoming data from the ESP32 S3 CAM and saves
 * each letter as a position on a char array
 * @param *buf buffer to save the camera reply information
 * @return true if reading was successful
 * @return false if reading failed
 * @see RequestCamera()
 */
bool ReadCameraReply(char *buf) {
  int idx = 0;
  
  unsigned long timeout = millis() + CAM_TIMEOUT_MS;  
  while (millis() < timeout)                      // Checks if there is a reply for a certain amount of time, if not the function returns false.
  {
    while (Serial1.available())                   // Checks if there is information at the serial port to read
    {
      char c = Serial1.read();                    // Reads the first letter of the pile

      if (c == '\n')                              // Checks if it is the end of the line
      {
        buf[idx] = '\0';                          // End symbol at the end of the buffer array
        return true;                              // Stops reading
      }

      if (idx < (int)(48 - 1)) buf[idx++] = c;    // Saves each letter at the buffer array
    }
    delay(1);
  }
  return false;
}


/**
 * It requests the camera data, decodifies its information
 * and calculates the needed correction for the servo.
 * @param none
 * @return void
 * @see ReadCameraReply
 */
void RequestCamera() {
  char buf[48];                                     // Initialize an array to save the camera reply

  for (int attempt = 0; attempt < 2; attempt++)     // Attempts 3 times to communicate with the ESP32 S3 CAM
  {
    // Demanding command that is read by the ESP32 S3 CAM
    Serial1.println("D");                           

    // Checks that the reply was succesful, if it was not it consumes this attempt
    if (!ReadCameraReply(buf)) continue;

    // Pointer set to buf
    char *p = buf;                                  

    // If the first character is "0" it means that it has not seen a traffic light (a.k.a. none). Return with values at 0
    if (*p == '0') 
    {
      cam_correction = 0;
      cam_count = 0;
      return;
    }
    
    // We get from the ESP32 S3 CAM the following information coded as:
    // "Color(0, 1 or 2),CenterX,TotalArea"
    // We want to get the Center X data, which is where the traffic light is located
    int cx = 0;                                 // Center X                                  
    char *token = strtok(p, ",");               // Divides the string into "Color(0, 1 or 2)" and "CenterX,TotalArea"
    if (token) token = strtok(NULL, ",");       // Divides the "CenterX,TotalArea" string into "CenterX" and "TotalArea". We keep the CenterX
    if (token) cx = atoi(token);                // Turns the Center X ASCII value into a integer

    // If Center X is 0 it means a failed measure
    if (cx > 0)
    {
      if (*p == '1')                      // GREEN traffic light
      {
        cam_correction = -(cx / 8);       // Bigger the Center X bigger the correction for the servo
      } 
      else                                // RED traffic light
      {
        cam_correction = (320 - cx) / 8;  // Bigger the Center X bigger the correction for the servo
      }
      cam_count = 15;
    } 
    else                                  
    {
      cam_correction = 0;
      cam_count = 0;
    }
    return;
  }
  
  // Failed measure, reached 3 failed attempts
  cam_correction = 0;
  cam_count = 0;
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
  Serial.begin(115200);                           // Starts Serial for debugging reasons
  Wire.begin(SDA_PIN, SCL_PIN);                   // I2C communication for ToF Luna
  Wire.setTimeOut(100);
  Wire.setClock(100000);                          // ToF Luna at 100kHz   
  Wire1.begin(SDA_PIN_COLOR, SCL_PIN_COLOR);      // I2C communication for TCS34725 color sensor
  Serial1.begin(115200, SERIAL_8N1, ESP32_CAM_RX, ESP32_CAM_TX);      // UART communication with the ESP32 S3 CAM

  servo.attach(SERVO_PIN);                        // Initialize Servo
  servo.write(90);                                // 0 postion Servo 

  // Pin Modes
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A_PIN, OUTPUT);
  pinMode(MOTOR_B_PIN, OUTPUT);

  // Stop the motor
  digitalWrite(MOTOR_A_PIN, LOW);
  digitalWrite(MOTOR_B_PIN, LOW);

  ledcAttachChannel(ENA_PIN, 1000, 8, 2);         // PWM for the motor. Frequency = 1000, Resolution = 8, Channel = 2

  // FOR DEBUGGING
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

  // Initialize color sensor. DEBUG trought the serial port if not working.
  if (tcs.begin(0x29, &Wire1)) 
  {                                           
    Serial.println("Found sensor");
  } 
  else 
  {
    Serial.println("WARNING: TCS34725 not detected, continuing without color.");
  }

  // Waits for START button to be pressed
  int countButton = 0;
  while (countButton <= 10) 
  {
    button_state = digitalRead(BUTTON_PIN);
    if (button_state == LOW) countButton++;
    delay(10);
  }
}


/**
 * This code only runs all the time.
 * @param none
 * @return void
 */
void loop() {
  // Gets color temp from the Adafruit TCS34725 color sensor
  uint16_t r, g, b, c, colorTemp;                             // Initialize needed variables
  tcs.getRawData(&r, &g, &b, &c);                             // Gets the RGB and Clear channels
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c); // Calculates the color temperature from the RGBC channels

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
    if ((end_motor_millis + 1500) <= millis()) 
    {
      travel_sense=0;    
    }
  } 
  else 
  {    
    end_motor_millis = millis();
  }

  // Sets the travel sense. 0 = BRAKE, 1 = FORWARD, 2 = BACKWARD
  switch(travel_sense){
    case 0: digitalWrite(MOTOR_A_PIN, LOW);
            digitalWrite(MOTOR_B_PIN, LOW);
    break;
    case 1: digitalWrite(MOTOR_A_PIN, HIGH);
            digitalWrite(MOTOR_B_PIN, LOW);
            //digitalWrite(MOTOR_A_PIN, LOW);
    break;
    case 2: digitalWrite(MOTOR_A_PIN, LOW);
            digitalWrite(MOTOR_B_PIN, HIGH);
            //digitalWrite(MOTOR_B_PIN, LOW);
    break;
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

