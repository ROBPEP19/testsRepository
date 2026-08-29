//OBSTACLE CHALLENGE CODE - ANDORRA V3
//ESP32-S3 + ESP32-S3-CAM 
//Correccion proporcional segun posicion X del color detectado
//Navegacion por paredes con 3x TF-Luna (I2C Bus 0, pines 11/12)

#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

Servo servo;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);

const int motorA = 10;
const int motorB = 20;
const int ENA    = 15;
const int buttonPin = 13;
#define CAM_TIMEOUT_MS 1000    // el S3-CAM tarda 100-300ms en decodificar y responder; con trazas mas

const uint8_t lunaCount = 3;
const uint8_t lunaAddr[lunaCount] = {0x11, 0x12, 0x13}; // derecha, izquierda, frontal
const uint16_t lunaStrengthMin = 20;
const uint16_t lunaCloseStrength = 150;  // CALIBRAR: si dist==0 y amp>=este -> objeto muy cerca (150mm)
const uint16_t lunaCloseMm = 150;
const uint16_t frontBrakeMm = 350;   // inicio de frenado progresivo
const uint16_t frontRearMm   = 150;  // umbral para iniciar marcha atras

int buttonState = HIGH;
int LapCount = 0;
long DelayLine = 0;
long MotorMillis = 0;
long PrevMillis = 0;
long PrevMillisF = 0;
int TravelSense = 1;
byte rearPhase = 0;                 // 0=normal, 1=stop, 2=reversa, 3=reanudar
unsigned long rearPhaseUntil = 0;

unsigned long lastCamRequest = 0;
int camCorrection = 0;
int camCount = 0;

uint16_t lunaRead16(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)2);
  if (Wire.available() < 2) return 0xFFFF;
  return (uint16_t)(Wire.read() | (Wire.read() << 8));
}

uint16_t lunaDistance(uint8_t addr) {
  uint16_t d = lunaRead16(addr, 0x00);
  uint16_t strength = lunaRead16(addr, 0x02);
  if (d == 0xFFFF || strength == 0xFFFF || strength < lunaStrengthMin) return 0xFFFF;
  if (d == 0) {
    // dist=0 puede ser muy cerca (saturado, amp alta) o lejos/sin objeto (amp baja)
    if (strength >= lunaCloseStrength) return lunaCloseMm;  // reflejo fuerte + dist 0 = muy cerca
    return 1500;                                           // reflejo debil = lejos / sin objeto
  }
  return d*10;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(11,12);
  Wire.setTimeOut(100);                                                // no bloquear si un TF-Luna no responde
  Wire1.begin(40,41);
  Wire.setClock(100000); // TF-Luna a 100kHz

  servo.attach(9);
  servo.write(90);

  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  digitalWrite(motorA, LOW);
  digitalWrite(motorB, LOW);

  ledcAttachChannel(ENA, 1000, 8, 2);

  /*for (uint8_t i = 0; i < lunaCount; i++) {
    Wire.beginTransmission(lunaAddr[i]);
    if (Wire.endTransmission() == 0) {
      Serial.print("TF-Luna ");
      Serial.print(i);
      Serial.print(" OK (0x");
      Serial.print(lunaAddr[i], HEX);
      Serial.println(")");
    } else {
      Serial.print("TF-Luna ");
      Serial.print(i);
      Serial.print(" NO encontrado (0x");
      Serial.print(lunaAddr[i], HEX);
      Serial.println(")");
    }
  }*/

  if (tcs.begin(0x29,&Wire1)) {
    //Serial.println("Found sensor");
  } else {
    //Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  Serial1.begin(115200, SERIAL_8N1, 16, 17);
  //Serial.println("UART con S3-CAM iniciado (RX=35 AZUL, TX=36 VERDE)");

  int countButton = 0;
  while (countButton <= 10) {
    buttonState = digitalRead(buttonPin);
    if (buttonState == LOW) countButton++;
    delay(10);
  }
}

bool readCameraReply(char *buf) {
  int idx = 0;
  unsigned long timeout = millis() + CAM_TIMEOUT_MS;
  while (millis() < timeout) {
    while (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') {
        buf[idx] = '\0';
        return true;
      }
      if (idx < (int)(48 - 1)) buf[idx++] = c;
    }
    delay(1);
  }
  return false;
}

void requestCamera() {
  char buf[48];
  for (int attempt = 0; attempt < 2; attempt++) {    // reintento si el S3-CAM estaba ocupado
    Serial1.println("D");
    if (!readCameraReply(buf)) continue;

    char *p = buf;
    if (*p == '0') {
      //Serial.println("CAM: N");
      camCorrection = 0;
      camCount = 0;
      return;
    }
    int cx = 0;
    char *token = strtok(p, ",");
    if (token) token = strtok(NULL, ",");
    if (token) cx = atoi(token);

    if (cx > 0) {
      if (*p == '1') { // verde
        camCorrection = -(cx / 8);    // verde=izquierda, mas fuerte si esta a la derecha
      } else {
        camCorrection = (320 - cx) / 8; // rojo=derecha, mas fuerte si esta a la izquierda
      }
      camCount = 15;
    } else {
      camCorrection = 0;
      camCount = 0;
    }
    /*Serial.print("CAM: "); Serial.print(buf);
    Serial.print(" -> corr="); Serial.print(camCorrection);
    Serial.print(" frames="); Serial.println(camCount);*/
    return;
  }
  //Serial.println("CAM: (timeout)");
  camCorrection = 0;
  camCount = 0;
}

void loop() {
  uint16_t r, g, b, c, colorTemp;

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);

  //Serial.print("Color Temp: "); Serial.println(colorTemp, DEC);             //INFORMATION FOR COLOR SENSOR CALIBRATION
  if (colorTemp > 3000){                                                  //CALIBRATE BEFORE ROUND
    if ((PrevMillis + 700)<=millis()){
      PrevMillis = millis();
      if (TravelSense >= 2) LapCount--;
      else LapCount++;
    }
  }

  if (LapCount >= 12) { // 12 lineas = 3 vueltas (4 lineas por vuelta)
    if ((MotorMillis + 3500) <= millis()) {
      TravelSense=0;
    }
  } else {
    MotorMillis = millis();
  }

  switch(TravelSense){
    case 0: digitalWrite(motorA, LOW);
            digitalWrite(motorB, LOW);
    break;
    case 1: digitalWrite(motorA, HIGH);
            digitalWrite(motorB, LOW);
            //digitalWrite(motorA, LOW);
    break;
    case 2: digitalWrite(motorA, LOW);
            digitalWrite(motorB, HIGH);
            //digitalWrite(motorB, LOW);
    break;
   }

  drive();
}

void drive() {
  static signed int distanceRight,distanceLeft,distanceFront;
  static int CountCorrection=0;
  static signed int correction=0;

  // LINEAS DEL DRIVE QUE LLAMAN A LA CÁMARA AL EJECUTAR EL LOOP.
  if (millis() - lastCamRequest > 100) {
    lastCamRequest = millis();
    requestCamera();
  }
  if (camCount > 0) {
    CountCorrection = camCount;
    correction = camCorrection;
    camCount = 0;
  }

  uint16_t d;
  d = lunaDistance(lunaAddr[0]);
  if (d != 0xFFFF) {
    distanceRight = d;
    if (distanceRight>1500) distanceRight=1500;
  }

  d = lunaDistance(lunaAddr[1]);
  if (d != 0xFFFF) {
    distanceLeft = d;
    if (distanceLeft>1500) distanceLeft=1500;
  }

  d = lunaDistance(lunaAddr[2]);
  if (d != 0xFFFF) {
    distanceFront = d;
  }

  if (distanceFront < frontRearMm){
    if (rearPhase == 0){
      if ((PrevMillisF + 1200)<=millis()){
        PrevMillisF = millis();
        rearPhase=1;
        rearPhaseUntil = millis() + 250;
      }
    }
  }

  if (rearPhase >= 1){                                                    //REAR SEQUENCE (real time)
    switch (rearPhase){
      case 1: TravelSense=0;
              if (millis() >= rearPhaseUntil){ rearPhase=2; rearPhaseUntil = millis() + 550; }
      break;
      case 2: TravelSense=2;
              if (millis() >= rearPhaseUntil){ rearPhase=3; rearPhaseUntil = millis() + 50; }
      break;
      case 3: TravelSense=1;
              rearPhase=0;
      break;
    }
  }

  signed int dServo = (distanceRight - distanceLeft )/10;
  if (dServo > 39) dServo = 39;
  if (dServo < -39) dServo = -39;
  if (CountCorrection>0) {
    dServo=correction;
    CountCorrection--;
  }
  if (rearPhase == 2){                                 //REVERSE: limited steering to avoid flipping
    if (dServo > 12) dServo = 12;
    if (dServo < -12) dServo = -12;
    servo.write(-dServo + 90);
  }
  else if (TravelSense == 1) servo.write(dServo + 90);

  int baseSpeed = 120;
  if (distanceFront < frontRearMm)  baseSpeed = 60;
  else if (distanceFront < frontBrakeMm) baseSpeed = 75;
  ledcWrite(ENA, baseSpeed);
}
