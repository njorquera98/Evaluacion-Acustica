#include <Wire.h>

#define I2C_SLAVE_ADDR 0x12
volatile float recibido_dB = 0.0;

void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(recibirDato);
  Serial.println("Arduino I2C esclavo listo");
}

void loop() {
  Serial.print("Valor recibido: ");
  Serial.print(recibido_dB, 1); // Muestra con 1 decimal
  Serial.println(" dB");
  delay(1000);
}

void recibirDato(int cuantos) {
  if (cuantos == 4) {  // Solo si se reciben 4 bytes
    byte buffer[4];
    for (int i = 0; i < 4; i++) {
      buffer[i] = Wire.read();
    }
    memcpy(&recibido_dB, buffer, sizeof(float)); // Convierte bytes a float
  } else {
    while (Wire.available()) Wire.read(); // Limpiar si llegan datos inválidos
  }
}