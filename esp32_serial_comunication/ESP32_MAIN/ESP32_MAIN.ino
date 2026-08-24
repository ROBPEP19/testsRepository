// ESP32-MAIN

// Define TX and RX pins for UART (change if needed)
#define TXD1 12
#define RXD1 13

// Use Serial1 for UART communication
HardwareSerial mySerial(1);

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RXD1, TXD1);  // UART setup
  
  Serial.println("ESP32 UART Receiver");
}

void loop() {
  if (mySerial.available()) {
    // Leemos todo hasta encontrar el salto de línea
    String trama = Serial1.readStringUntil('\n');
    
    int recTemp, recHum, recPres;
    
    // sscanf busca el formato exacto. 
    // Devuelve el número de variables que logró extraer con éxito.
    int parseados = sscanf(trama.c_str(), "T:%d,H:%d,P:%d", &recTemp, &recHum, &recPres);
    
    if (parseados == 3) {
      Serial.println("¡Datos extraídos correctamente!");
      Serial.printf("Temperatura: %d | Humedad: %d | Presion: %d\n", recTemp, recHum, recPres);
    } else {
      Serial.println("Error: Trama corrupta o incompleta.");
    }
  }
}