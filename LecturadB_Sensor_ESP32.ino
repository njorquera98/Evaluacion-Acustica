// Define 485 communication pins
#define RS485_RX_PIN  15
#define RS485_TX_PIN  16

// Redefine serial port name
#define RS485 Serial1

// Noise sensor parameters
#define SENSOR_ADDRESS 0x01
#define FUNCTION_CODE 0x03
#define REGISTER_START_H 0x00
#define REGISTER_START_L 0x0E
#define REGISTER_LENGTH_H 0x00
#define REGISTER_LENGTH_L 0x01

// Variables for data processing
uint8_t queryFrame[8];
uint8_t responseBuffer[32];
int responseIndex = 0;
unsigned long lastQuery = 0;
const unsigned long QUERY_INTERVAL = 1000; // 1 segundo

void setup() {
  // Initialize USB Serial for debugging
  Serial.begin(115200);
  Serial.println("=== GEMHO Noise Sensor Reader ===");
  Serial.println("Inicializando comunicacion RS485...");
  
  // Initialize RS485 with parameters from manual
  RS485.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  while (!RS485) {
    delay(10);
  }
  
  // Prepare Modbus query frame
  prepareQueryFrame();
  
  Serial.println("RS485 inicializado correctamente");
  Serial.println("Protocolo: Modbus RTU");
  Serial.println("Baud Rate: 9600 bps");
  Serial.println("Sensor Address: 0x01");
  Serial.println("Rango: 30-120 dB(A)");
  Serial.println("Resolucion: 0.1 dB");
  Serial.println("=============================");
  Serial.println("Iniciando lectura de datos...\n");
  
  delay(2000); // Wait for sensor to be ready
}

void loop() {
  // Send query every QUERY_INTERVAL milliseconds
  if (millis() - lastQuery >= QUERY_INTERVAL) {
    sendNoiseQuery();
    lastQuery = millis();
  }
  
  // Read response from sensor
  readSensorResponse();
  
  delay(10); // Small delay to prevent overwhelming the sensor
}

void prepareQueryFrame() {
  // Prepare Modbus RTU query frame according to manual
  queryFrame[0] = SENSOR_ADDRESS;      // Address code
  queryFrame[1] = FUNCTION_CODE;       // Function code (0x03)
  queryFrame[2] = REGISTER_START_H;    // Register start address high byte
  queryFrame[3] = REGISTER_START_L;    // Register start address low byte (0x0E)
  queryFrame[4] = REGISTER_LENGTH_H;   // Register length high byte
  queryFrame[5] = REGISTER_LENGTH_L;   // Register length low byte (0x01)
  
  // Calculate CRC16 for the frame
  uint16_t crc = calculateCRC16(queryFrame, 6);
  queryFrame[6] = crc & 0xFF;         // CRC Low byte
  queryFrame[7] = (crc >> 8) & 0xFF;  // CRC High byte
}

void sendNoiseQuery() {
  // Clear response buffer
  responseIndex = 0;
  memset(responseBuffer, 0, sizeof(responseBuffer));
  
  // Send Modbus query frame
  RS485.write(queryFrame, 8);
  
  // Debug: Show sent frame
  Serial.print("Enviando query: ");
  for (int i = 0; i < 8; i++) {
    Serial.print("0x");
    if (queryFrame[i] < 16) Serial.print("0");
    Serial.print(queryFrame[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void readSensorResponse() {
  // Read available data from RS485
  while (RS485.available() && responseIndex < sizeof(responseBuffer)) {
    responseBuffer[responseIndex] = RS485.read();
    responseIndex++;
  }
  
  // Check if we have a complete response (minimum 7 bytes for noise data)
  if (responseIndex >= 7) {
    processNoiseResponse();
    responseIndex = 0; // Reset for next reading
  }
}

void processNoiseResponse() {
  // Debug: Show received frame
  Serial.print("Respuesta recibida: ");
  for (int i = 0; i < responseIndex; i++) {
    Serial.print("0x");
    if (responseBuffer[i] < 16) Serial.print("0");
    Serial.print(responseBuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Validate response according to manual format
  // Expected format: [Address][Function][Data Bytes][Noise Value H][Noise Value L][CRC L][CRC H]
  if (responseIndex >= 7 && 
      responseBuffer[0] == SENSOR_ADDRESS && 
      responseBuffer[1] == FUNCTION_CODE &&
      responseBuffer[2] == 0x02) { // Data bytes should be 2
    
    // Extract noise value (16-bit, high byte first)
    uint16_t noiseRaw = (responseBuffer[3] << 8) | responseBuffer[4];
    
    // Convert to dB(A) according to manual: Value * 0.1
    float noiseLevel = noiseRaw * 0.1;
    
    // Validate range (30-120 dB according to manual)
    if (noiseLevel >= 30.0 && noiseLevel <= 120.0) {
      Serial.print(">>> NIVEL DE RUIDO: ");
      Serial.print(noiseLevel, 1);
      Serial.println(" dB(A) <<<");
      
      // Additional info
      Serial.print("    Valor RAW: 0x");
      Serial.print(noiseRaw, HEX);
      Serial.print(" (");
      Serial.print(noiseRaw);
      Serial.println(" decimal)");
      
      // Timestamp
      Serial.print("    Tiempo: ");
      Serial.print(millis() / 1000.0);
      Serial.println(" segundos");
      Serial.println();
    } else {
      Serial.println("ERROR: Valor fuera de rango (30-120 dB)");
    }
  } else {
    Serial.println("ERROR: Respuesta invalida del sensor");
    Serial.print("Direccion esperada: 0x");
    Serial.print(SENSOR_ADDRESS, HEX);
    Serial.print(", recibida: 0x");
    Serial.println(responseBuffer[0], HEX);
  }
}

// CRC16 calculation for Modbus RTU
uint16_t calculateCRC16(uint8_t* data, int length) {
  uint16_t crc = 0xFFFF;
  
  for (int i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
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

// Function to change sensor address if needed
void changeSensorAddress(uint8_t newAddress) {
  Serial.print("Cambiando direccion del sensor a: 0x");
  Serial.println(newAddress, HEX);
  
  uint8_t writeFrame[8];
  writeFrame[0] = SENSOR_ADDRESS;  // Current address
  writeFrame[1] = 0x06;           // Write function code
  writeFrame[2] = 0x00;           // Data address high
  writeFrame[3] = 0x0F;           // Data address low
  writeFrame[4] = 0x00;           // New address high
  writeFrame[5] = newAddress;     // New address low
  
  uint16_t crc = calculateCRC16(writeFrame, 6);
  writeFrame[6] = crc & 0xFF;
  writeFrame[7] = (crc >> 8) & 0xFF;
  
  RS485.write(writeFrame, 8);
  
  Serial.println("Comando de cambio de direccion enviado");
}

