// ESP32-CAM
// Este ESP32 actuará como cámara
// Enviara datos por Serial al ESP32-MAIN. Que los recibirá y enviará al ordenador.

// Define TX and RX pins for UART (change if needed)
#define TXD1 13
#define RXD1 12

int temperatura = 25;
int humedad = 60;
int presion = 1013;

// Use Serial1 for UART communication
HardwareSerial mySerial(1);

int counter = 0;
int anti_counter = 99999999;

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RXD1, TXD1);  // UART setup
  
  Serial.println("ESP32 UART Transmitter");
}

void loop() {
  
  // printf envía el texto con el formato deseado automáticamente
  // %d se sustituye por las variables int. \n marca el final.
  mySerial.printf("T:%d,H:%d,P:%d\n", temperatura, humedad, presion);
  
  // Simulamos que los datos cambian
  temperatura++;
  humedad++;
  presion++;
  
  delay(1000);
}