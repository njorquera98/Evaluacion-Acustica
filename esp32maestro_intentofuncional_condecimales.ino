#include <Wire.h>

// Pines I2C definidos según la Waveshare ESP32S3-Touch-LCD-7
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_SLAVE_ADDR 0x12  // Dirección del Arduino esclavo

// Pines RS485
#define RS485_RX_PIN  15
#define RS485_TX_PIN  16
#define RS485 Serial1

// Parámetros del sensor de ruido (Modbus)
#define SENSOR_ADDRESS      0x01
#define FUNCTION_CODE       0x03
#define REGISTER_START_H    0x00
#define REGISTER_START_L    0x0E
#define REGISTER_LENGTH_H   0x00
#define REGISTER_LENGTH_L   0x01

uint8_t queryFrame[8];
uint8_t responseBuffer[32];
int responseIndex = 0;
unsigned long lastQuery = 0;
const unsigned long QUERY_INTERVAL = 3000; // 3 segundos

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Inicia I2C como maestro
  Serial.println("=== GEMHO Noise Sensor + I2C Master ===");

  RS485.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  while (!RS485) delay(10);

  prepareQueryFrame();
  delay(10);
}

void loop() {
  if (millis() - lastQuery >= QUERY_INTERVAL) {
    sendNoiseQuery();
    lastQuery = millis();
  }

  readSensorResponse();
  delay(10);
}

void prepareQueryFrame() {
  queryFrame[0] = SENSOR_ADDRESS;
  queryFrame[1] = FUNCTION_CODE;
  queryFrame[2] = REGISTER_START_H;
  queryFrame[3] = REGISTER_START_L;
  queryFrame[4] = REGISTER_LENGTH_H;
  queryFrame[5] = REGISTER_LENGTH_L;

  uint16_t crc = calculateCRC16(queryFrame, 6);
  queryFrame[6] = crc & 0xFF;
  queryFrame[7] = (crc >> 8) & 0xFF;
}

void sendNoiseQuery() {
  responseIndex = 0;
  memset(responseBuffer, 0, sizeof(responseBuffer));
  RS485.write(queryFrame, 8);
  Serial.println("Query enviada por RS485");
}

void readSensorResponse() {
  while (RS485.available() && responseIndex < sizeof(responseBuffer)) {
    responseBuffer[responseIndex] = RS485.read();
    responseIndex++;
  }

  if (responseIndex >= 7) {
    if (responseBuffer[0] == SENSOR_ADDRESS &&
        responseBuffer[1] == FUNCTION_CODE &&
        responseBuffer[2] == 0x02) {

      int16_t rawValue = (responseBuffer[3] << 8) | responseBuffer[4];
      float dB = rawValue / 10.0;

      Serial.print("Ruido leído: ");
      Serial.print(dB);
      Serial.println(" dB");

      // Enviar valor por I2C (como entero)
      Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write((uint8_t*)&dB, sizeof(float));
      Wire.endTransmission();

    Serial.println("Enviado por I2C al Arduino (float).");
    }
  }
}

uint16_t calculateCRC16(uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}
