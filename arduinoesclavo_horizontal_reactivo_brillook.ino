/*  arduinoesclavointento_funcionalcondecimales_fixed.ino
    Versión final: conserva la lógica de figuras y recepción I2C
    e incorpora el envío/clock/latch optimizado (del segundo código)
    para mayor brillo y sin parpadeo.
*/

#include <Wire.h>

// Dirección I2C
#define I2C_SLAVE_ADDR 0x12
volatile float recibido_dB = 0.0;
int soundValue = 0;

// Pines matriz 32×64
#define R1   2
#define G1   3
#define B1   4
#define R2   5
#define G2   6
#define B2   7
#define A    8
#define B    9
#define C    10
#define D    14
#define CLK  11
#define LAT  12
#define OE   13  // Output Enable (activo LOW)

// Parámetros de animación
const unsigned long duracionAnimacion = 500;
unsigned long lastFrameChange = 0;
int animFrame = 0;

unsigned long lastChoiceTime = 0;
const unsigned long duracionFrase = 4000;
int choiceBien = 0, choiceCuidado = 0, choiceMuchoRuido = 0;

// Prototipos de auxiliares
void selectRow(int row);
void setPixels(uint8_t top, uint8_t bot);
int determineColor(int val);
bool isWhiteTeeth(int col, int row);
bool isAnimatedFace(int col, int row);
bool isRightRandomPattern(int col, int row);

// --- Forward: agregamos aquí helpers optimizados (pulseClock/latchData) más abajo ---

void recibirDato(int cuantos);

// -------------------------
// setup
// -------------------------
void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(recibirDato);

  // Pines de control como salida
  pinMode(OE, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(LAT, OUTPUT);
  digitalWrite(OE, LOW);

  // Pines de datos en puertos rápidos
  // R1,G1,B1,R2,G2,B2 -> PD2..PD7 / etc. (según tu cableado original)
  // Configura registros de puerto para acceso rápido (tal como en tu código original)
  // A,B,C,D -> multiplex address pins
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);

  // Preparar puertos (tal como en el original)
  // Asegúrate de que las máscaras de puertos correspondan a tu hardware (esto lo tenías en el original)
  DDRD |= 0b11111100;   // PD2..PD7 como salidas para colores
  DDRB |= 0b00111000;   // PB3..PB5 para CLK/LAT/OE
  DDRC |= 0b00000001;   // PC0 (D)

  randomSeed(analogRead(0));
}

// -------------------------
// loop principal
// -------------------------
void loop() {
  // 1) Leer y convertir dB
  noInterrupts();
    float dB = recibido_dB;
  interrupts();
  soundValue = (int)dB;

  // 2) Actualizar animaciones/aleatorios
  unsigned long now = millis();
  if (now - lastFrameChange >= duracionAnimacion) {
    animFrame = (animFrame + 1) % 3;
    lastFrameChange = now;
  }
  if (now - lastChoiceTime >= duracionFrase) {
    choiceBien       = random(0, 3);
    choiceCuidado    = random(0, 3);
    choiceMuchoRuido = random(0, 3);
    lastChoiceTime   = now;
  }

  Serial.print("dB: ");
  Serial.print(dB, 1);
  Serial.print("  soundValue:");
  Serial.println(soundValue);

  // 3) Para cada fila, calculamos en un buffer local de 64 cols
  const int BRIGHTNESS_REPEATS = 2;    // repeticiones por fila (ajusta a + para más brillo)
  const int ON_TIME_US         = 300;  // tiempo "on" por fila (microsegundos)

  for (int row = 0; row < 16; row++) {
    uint8_t bufTop[64], bufBot[64];

    // 3a) Precalcular colores de la fila
    for (int col = 0; col < 64; col++) {
      uint8_t cT = 0, cB = 0;
      if (col < 32) {
        if (isWhiteTeeth(col, row))           cT = 7;
        else if (isAnimatedFace(col, row))    cT = determineColor(soundValue);
        if (isWhiteTeeth(col, row + 16))      cB = 7;
        else if (isAnimatedFace(col, row + 16)) cB = determineColor(soundValue);
      } else {
        int rc = col - 32;
        if (isRightRandomPattern(rc, row))       cT = determineColor(soundValue);
        if (isRightRandomPattern(rc, row + 16))  cB = determineColor(soundValue);
      }
      bufTop[col] = cT;
      bufBot[col] = cB;
    }

    // 3b) Seleccionar fila y enviar bits rápido (optimizado)
    selectRow(row);
    digitalWrite(OE, HIGH);  // apagar mientras shift

    noInterrupts();
    for (int rpt = 0; rpt < BRIGHTNESS_REPEATS; rpt++) {
      for (int col = 0; col < 64; col++) {
        setPixels(bufTop[col], bufBot[col]);
        // reemplazo optimizado: pulso de reloj más rápido
        pulseClock();
      }
    }
    // Latch de fin de fila (optimizado)
    latchData();
    interrupts();

    // 3c) Mostrar fila y controlar brillo (mantener un tiempo constante)
    digitalWrite(OE, LOW);
    delayMicroseconds(ON_TIME_US);
  }

  // 4) Pequeño retardo de ciclo para estabilidad
  delay(1);
}


// ————— Funciones auxiliares —————
int determineColor(int val) {
  if (val <= 50) return 2;
  if (val <= 80) return 3;
  return 1;
}

/* setPixels: mapea bits de color a PORTD (rápido)
   Mantengo el mismo mapeo que usabas en el original para compatibilidad.
*/
void setPixels(uint8_t colorTop, uint8_t colorBottom) {
  byte tv = 0, bv = 0;
  if (colorTop & 1)    tv |= (1<<2);
  if (colorTop & 2)    tv |= (1<<3);
  if (colorTop & 4)    tv |= (1<<4);
  if (colorBottom & 1) bv |= (1<<5);
  if (colorBottom & 2) bv |= (1<<6);
  if (colorBottom & 4) bv |= (1<<7);
  PORTD = (PORTD & 0x03) | tv | bv;
}

/* selectRow: usa los mismos puertos rápidos que tu código original.
   Asegura que A/B/C/D se establezcan via PORTB y PORTC como antes.
*/
void selectRow(int row) {
  // Parte baja de la dirección (bits 0-2)
  PORTB = (PORTB & 0b11111000) | (row & 0x07);
  // Bit 3 de la dirección (pin D/14 -> PC0)
  if (row & 0x08) PORTC |= (1<<PC0);
  else            PORTC &= ~(1<<PC0);
}

/* --- Funciones optimizadas añadidas (del 2do código) --- */
/* pulseClock(): pulso rápido del reloj usando PB3 (digital 11) */
void pulseClock() {
  PORTB |= (1 << PB3);  // CLK HIGH
  PORTB &= ~(1 << PB3); // CLK LOW
}

/* latchData(): latch/OE rápido usando PB4 (pin12) y PB5 (pin13) */
void latchData() {
  PORTB |= (1 << PB5);  // OE HIGH (apaga display)
  PORTB |= (1 << PB4);  // LAT HIGH
  delayMicroseconds(1);  // setup mínimo
  PORTB &= ~(1 << PB4); // LAT LOW
  PORTB &= ~(1 << PB5); // OE LOW (enciende display)
}

// ————— RECEPCIÓN I2C —————
void recibirDato(int cuantos) {
  if (cuantos == 4) {
    byte buf[4];
    for (int i = 0; i < 4; i++) buf[i] = Wire.read();
    memcpy(&recibido_dB, buf, sizeof(float));
  } else {
    while (Wire.available()) Wire.read();
  }
}

// ===========================
// ANIMACIÓN LADO IZQUIERDO
// ===========================
bool isAnimatedFace(int col, int row) {
  if (soundValue <= 100) {
    // Cara feliz (verde)
    switch (animFrame) {
      case 0: return (
                         (col == 1 && row >= 12 && row <= 19) ||
                         (col == 2 && row >= 10 && row <= 21) ||
                         (col == 3 && row >= 8 && row <= 23) ||
                         (col == 4 && row >= 7 && row <= 24) ||
                         (col == 5 && row >= 6 && row <= 25) ||

                         (col == 6 && (row >= 5 && row <= 12 || row >= 16 && row <= 26)) ||
                         (col == 7 && (row >= 4 && row <= 11 || row >= 17 && row <= 27)) ||
                         (col == 8 && (row >= 4 && row <= 11 || row >= 17 && row <= 27)) ||          
                         (col == 9 && (row >= 3 && row <= 11 || row >= 17 && row <= 28)) ||                         
                         (col == 10 && (row >= 3 && row <= 12 || row >= 16 && row <= 28)) ||                   

                         (col == 11 && (row >= 3 && row <= 20 || row >= 23 && row <= 28)) ||
                         (col == 12 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 13 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 14 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 15 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 16 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 17 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 18 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 19 && (row >= 3 && row <= 20 || row >= 23 && row <= 28)) ||

                         (col == 20 && (row >= 3 && row <= 12 || row >= 16 && row <= 28)) ||
                         (col == 21 && (row >= 3 && row <= 11 || row >= 17 && row <= 28)) ||
                         (col == 22 && (row >= 4 && row <= 11 || row >= 17 && row <= 27)) ||
                         (col == 23 && (row >= 4 && row <= 11 || row >= 17 && row <= 27)) ||
                         (col == 24 && (row >= 5 && row <= 12 || row >= 16 && row <= 26)) ||

                         (col == 25 && row >= 6 && row <= 25) ||
                         (col == 26 && row >= 7 && row <= 24) ||
                         (col == 27 && row >= 8 && row <= 23) ||
                         (col == 28 && row >= 10 && row <= 21) ||
                         (col == 29 && row >= 12 && row <= 19) 
                        
                         ) ;

      case 1: return (
                         (col == 1 && row >= 12 && row <= 19) ||
                         (col == 2 && row >= 10 && row <= 21) ||
                         (col == 3 && row >= 8 && row <= 23) ||
                         (col == 4 && row >= 7 && row <= 24) ||

                         (col == 5 && (row >= 6 && row <= 13 || row >= 16 && row <= 25)) ||
                         (col == 6 && (row >= 5 && row <= 12 || row >= 16 && row <= 26)) ||
                         (col == 7 && (row >= 4 && row <= 11 || row >= 16 && row <= 20 || row >= 22 && row <= 27)) ||
                         (col == 8 && (row >= 4 && row <= 11 || row >= 16 && row <= 20 || row >= 23 && row <= 27)) ||
                         (col == 9 && (row >= 3 && row <= 11 || row >= 16 && row <= 20 || row >= 24 && row <= 28)) ||
                         (col == 10 && (row >= 3 && row <= 12 || row >= 16 && row <= 21 || row >= 24 && row <= 28)) ||

                         (col == 11 && (row >= 3 && row <= 13 || row >= 16 && row <= 21 || row >= 24 && row <= 28)) ||
                         (col == 12 && (row >= 2 && row <= 21 || row >= 24 && row <= 29)) ||
                         (col == 13 && (row >= 2 && row <= 21 || row >= 24 && row <= 29)) ||
                         (col == 14 && (row >= 2 && row <= 21 || row >= 24 && row <= 29)) ||
                         (col == 15 && (row >= 2 && row <= 21 || row >= 24 && row <= 29)) ||
                         (col == 16 && (row >= 2 && row <= 21 || row >= 24 && row <= 29)) ||
                         (col == 17 && (row >= 2 && row <= 21 || row >= 24 && row <= 29)) ||
                         (col == 18 && (row >= 2 && row <= 21 || row >= 24 && row <= 29)) ||

                         (col == 19 && (row >= 3 && row <= 13 || row >= 16 && row <= 21 || row >= 24 && row <= 28)) ||
                         (col == 20 && (row >= 3 && row <= 12 || row >= 16 && row <= 21 || row >= 24 && row <= 28)) ||
                         (col == 21 && (row >= 3 && row <= 11 || row >= 16 && row <= 20 || row >= 24 && row <= 28)) ||
                         (col == 22 && (row >= 4 && row <= 11 || row >= 16 && row <= 20 || row >= 23 && row <= 27)) ||
                         (col == 23 && (row >= 4 && row <= 11 || row >= 16 && row <= 20 || row >= 22 && row <= 27)) ||
                         (col == 24 && (row >= 5 && row <= 12 || row >= 16 && row <= 26)) ||
                         (col == 25 && (row >= 6 && row <= 13 || row >= 16 && row <= 25)) ||

                         (col == 26 && row >= 7 && row <= 24) ||
                         (col == 27 && row >= 8 && row <= 23) ||
                         (col == 28 && row >= 10 && row <= 21) ||
                         (col == 29 && row >= 12 && row <= 19)) ;

      case 2: return (
                         (col == 1 && row >= 12 && row <= 19) ||
                         (col == 2 && row >= 10 && row <= 21) ||
                         (col == 3 && row >= 8 && row <= 23) ||
                         (col == 4 && row >= 7 && row <= 24) ||

                         (col == 5 && (row >= 6 && row <= 13 || row >= 18 && row <= 25)) ||
                         (col == 6 && (row >= 5 && row <= 12 || row >= 17 && row <= 19 || row >= 23 && row <= 26)) ||
                         (col == 7 && (row >= 4 && row <= 11 || row >= 16 && row <= 19 || row >= 24 && row <= 27)) ||
                         (col == 8 && (row >= 4 && row <= 11 || row >= 16 && row <= 19 || row >= 25 && row <= 27)) ||
                         (col == 9 && (row >= 3 && row <= 11 || row >= 16 && row <= 19 || row >= 25 && row <= 28)) ||
                         (col == 10 && (row >= 3 && row <= 12 || row >= 17 && row <= 19 || row >= 26 && row <= 28)) ||
                         (col == 11 && (row >= 3 && row <= 13 || row >= 18 && row <= 19 || row >= 26 && row <= 28)) ||

                         (col == 12 && (row >= 2 && row <= 19 || row >= 27 && row <= 29)) ||
                         (col == 13 && (row >= 2 && row <= 19 || row >= 27 && row <= 29)) ||
                         (col == 14 && (row >= 2 && row <= 19 || row >= 27 && row <= 29)) ||
                         (col == 15 && (row >= 2 && row <= 19 || row >= 27 && row <= 29)) ||
                         (col == 16 && (row >= 2 && row <= 19 || row >= 27 && row <= 29)) ||
                         (col == 17 && (row >= 2 && row <= 19 || row >= 27 && row <= 29)) ||
                         (col == 18 && (row >= 2 && row <= 19 || row >= 27 && row <= 29)) ||

                         (col == 19 && (row >= 3 && row <= 13 || row >= 18 && row <= 19 || row >= 26 && row <= 28)) ||
                         (col == 20 && (row >= 3 && row <= 12 || row >= 17 && row <= 19 || row >= 26 && row <= 28)) ||
                         (col == 21 && (row >= 3 && row <= 11 || row >= 16 && row <= 19 || row >= 25 && row <= 28)) ||
                         (col == 22 && (row >= 4 && row <= 11 || row >= 16 && row <= 19 || row >= 25 && row <= 27)) ||
                         (col == 23 && (row >= 4 && row <= 11 || row >= 16 && row <= 19 || row >= 24 && row <= 27)) ||
                         (col == 24 && (row >= 5 && row <= 12 || row >= 17 && row <= 19 || row >= 23 && row <= 26)) ||
                         (col == 25 && (row >= 6 && row <= 13 || row >= 18 && row <= 25)) ||

                         (col == 26 && row >= 7 && row <= 24) ||
                         (col == 27 && row >= 8 && row <= 23) ||
                         (col == 28 && row >= 10 && row <= 21) ||
                         (col == 29 && row >= 12 && row <= 19)
                         ) ;
    }
  } else if (soundValue <= 200) {
    // Cara seria (amarilla)
    switch (animFrame) {
      case 0: return (
                         (col == 1 && row >= 12 && row <= 19) ||
                         (col == 2 && row >= 10 && row <= 21) ||
                         (col == 3 && row >= 8 && row <= 23) ||
                         (col == 4 && row >= 7 && row <= 24) ||
                         (col == 5 && row >= 6 && row <= 25) ||

                         (col == 6 && (row >= 5 && row <= 14 || row >= 18 && row <= 26)) ||
                         (col == 7 && (row >= 4 && row <= 13 || row >= 19 && row <= 27)) ||
                         (col == 8 && (row >= 4 && row <= 10 || row >= 12 && row <= 13 || row >= 19 && row <= 27)) ||
                         (col == 9 && (row >= 3 && row <= 10 || row >= 12 && row <= 13 || row >= 19 && row <= 28)) ||
                         (col == 10 && (row >= 3 && row <= 10 || row >= 12 && row <= 14 || row >= 18 && row <= 28)) ||

                         (col == 11 && (row >= 3 && row <= 20 || row >= 23 && row <= 28)) ||
                         (col == 12 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 13 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 14 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 15 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 16 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 17 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 18 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 19 && (row >= 3 && row <= 20 || row >= 23 && row <= 28)) ||

                         (col == 20 && (row >= 3 && row <= 10 || row >= 12 && row <= 13 || row >= 17 && row <= 28)) ||
                         (col == 21 && (row >= 3 && row <= 10 || row == 12 || row >= 18 && row <= 28)) ||
                         (col == 22 && (row >= 4 && row <= 10 || row == 12 || row >= 18 && row <= 27)) ||
                         (col == 23 && (row >= 4 && row <= 12 || row >= 18 && row <= 27)) ||
                         (col == 24 && (row >= 5 && row <= 13 || row >= 17 && row <= 26)) ||

                         (col == 25 && row >= 6 && row <= 25) ||
                         (col == 26 && row >= 7 && row <= 24) ||
                         (col == 27 && row >= 8 && row <= 23) ||
                         (col == 28 && row >= 10 && row <= 21) ||
                         (col == 29 && row >= 12 && row <= 19)) ;
      case 1: return (
                         (col == 1 && row >= 12 && row <= 19) ||
                         (col == 2 && row >= 10 && row <= 21) ||
                         (col == 3 && row >= 8 && row <= 23) ||
                         (col == 4 && row >= 7 && row <= 24) ||
                         (col == 5 && row >= 6 && row <= 25) ||

                         (col == 6 && (row >= 5 && row <= 13 || row >= 15 && row <= 16 || row >= 19 && row <= 26)) ||
                         (col == 7 && (row >= 4 && row <= 12 || row == 15 || row >= 19 && row <= 27)) ||
                         (col == 8 && (row >= 4 && row <= 11 || row >= 14 && row <= 15 || row >= 20 && row <= 27)) ||
                         (col == 9 && (row >= 3 && row <= 11 || row >= 14 && row <= 15 || row >= 20 && row <= 28)) ||
                         (col == 10 && (row >= 3 && row <= 11 || row >= 14 && row <= 16 || row >= 19 && row <= 28)) ||
                         (col == 11 && (row >= 3 && row <= 12 || row >= 14 && row <= 23 || row >= 25 && row <= 28)) ||

                         (col == 12 && (row >= 2 && row <= 22 || row >= 25 && row <= 29)) ||
                         (col == 13 && (row >= 2 && row <= 22 || row >= 25 && row <= 29)) ||
                         (col == 14 && (row >= 2 && row <= 22 || row >= 25 && row <= 29)) ||
                         (col == 15 && (row >= 2 && row <= 22 || row >= 25 && row <= 29)) ||
                         (col == 16 && (row >= 2 && row <= 22 || row >= 25 && row <= 29)) ||
                         (col == 17 && (row >= 2 && row <= 22 || row >= 25 && row <= 29)) ||
                         (col == 18 && (row >= 2 && row <= 22 || row >= 25 && row <= 29)) ||

                         (col == 19 && (row >= 3 && row <= 12 || row >= 14 && row <= 16 || row >= 19 && row <= 23 || row >= 25 && row <= 28)) ||
                         (col == 20 && (row >= 3 && row <= 11 || row >= 14 && row <= 15 || row >= 19 && row <= 28)) ||
                         (col == 21 && (row >= 3 && row <= 11 || row >= 14 && row <= 15 || row >= 20 && row <= 28)) ||
                         (col == 22 && (row >= 4 && row <= 11 || row >= 14 && row <= 15 || row >= 20 && row <= 27)) ||
                         (col == 23 && (row >= 4 && row <= 12 || row >= 15 && row <= 16 || row >= 19 && row <= 27)) ||
                         (col == 24 && (row >= 5 && row <= 13 || row >= 15 && row <= 26)) ||

                         (col == 25 && row >= 6 && row <= 25) ||
                         (col == 26 && row >= 7 && row <= 24) ||
                         (col == 27 && row >= 8 && row <= 23) ||
                         (col == 28 && row >= 10 && row <= 21) ||
                         (col == 29 && row >= 12 && row <= 19)) ;

      case 2: return (
                         (col == 1 && row >= 12 && row <= 19) ||
                         (col == 2 && row >= 10 && row <= 21) ||
                         (col == 3 && row >= 8 && row <= 23) ||

                         (col == 4 && (row >= 7 && row <= 13 || row >= 15 && row <= 24)) ||
                         (col == 5 && (row >= 6 && row <= 12 || row >= 15 && row <= 17 || row >= 20 && row <= 25)) ||
                         (col == 6 && (row >= 5 && row <= 11 || row >= 14 && row <= 17 || row >= 20 && row <= 26)) ||
                         (col == 7 && (row >= 4 && row <= 11 || row >= 14 && row <= 15 || row >= 18 && row <= 27)) ||
                         (col == 8 && (row >= 4 && row <= 11 || row >= 14 && row <= 15 || row >= 18 && row <= 27)) ||
                         (col == 9 && (row >= 3 && row <= 11 || row >= 14 && row <= 15 || row >= 20 && row <= 28)) ||
                         (col == 10 && (row >= 3 && row <= 11 || row >= 14 && row <= 15 || row >= 20 && row <= 23 || row >= 26 && row <= 28)) ||
                         (col == 11 && (row >= 3 && row <= 11 || row >= 14 && row <= 16 || row >= 20 && row <= 23 || row >= 26 && row <= 28)) ||

                         (col == 12 && (row >= 2 && row <= 22 || row >= 26 && row <= 29)) ||
                         (col == 13 && (row >= 2 && row <= 22 || row >= 25 && row <= 29)) ||
                         (col == 14 && (row >= 2 && row <= 22 || row >= 25 && row <= 29)) ||
                         (col == 15 && (row >= 2 && row <= 22 || row >= 25 && row <= 29)) ||
                         (col == 16 && (row >= 2 && row <= 22 || row >= 25 && row <= 29)) ||
                         (col == 17 && (row >= 2 && row <= 22 || row >= 25 && row <= 29)) ||
                         (col == 18 && (row >= 2 && row <= 22 || row >= 26 && row <= 29)) ||

                         (col == 19 && (row >= 3 && row <= 11 || row >= 14 && row <= 17 || row >= 20 && row <= 23 || row >= 26 && row <= 28)) ||
                         (col == 20 && (row >= 3 && row <= 11 || row >= 14 && row <= 17 || row >= 20 && row <= 23 || row >= 26 && row <= 28)) ||
                         (col == 21 && (row >= 3 && row <= 11 || row >= 14 && row <= 15 || row >= 18 && row <= 28)) ||
                         (col == 22 && (row >= 4 && row <= 11 || row >= 14 && row <= 15 || row >= 18 && row <= 27)) ||
                         (col == 23 && (row >= 4 && row <= 11 || row >= 14 && row <= 15 || row >= 20 && row <= 27)) ||
                         (col == 24 && (row >= 5 && row <= 11 || row >= 14 && row <= 15 || row >= 20 && row <= 26)) ||
                         (col == 25 && (row >= 6 && row <= 12 || row >= 15 && row <= 16 || row >= 20 && row <= 25)) ||
                         (col == 26 && (row >= 7 && row <= 13 || row >= 15 && row <= 24)) ||

                         (col == 27 && row >= 8 && row <= 23) ||
                         (col == 28 && row >= 10 && row <= 21) ||
                         (col == 29 && row >= 12 && row <= 19)) ;
    }
  } else {
    // Cara molesta (roja)
    switch (animFrame) {
      case 0: return (
                         (col == 1 && row >= 12 && row <= 19) ||
                         (col == 2 && row >= 10 && row <= 21) ||
                         (col == 3 && row >= 8 && row <= 23) ||
                         (col == 4 && row >= 7 && row <= 24) ||
                         (col == 5 && row >= 6 && row <= 25) ||

                         (col == 6 && (row >= 5 && row <= 13 || row >= 17 && row <= 26)) ||
                         (col == 7 && (row >= 4 && row <= 12 || row >= 18 && row <= 27)) ||
                         (col == 8 && (row >= 4 && row <= 10 || row == 12 || row >= 18 && row <= 27)) ||
                         (col == 9 && (row >= 3 && row <= 10 || row == 12 || row >= 18 && row <= 28)) ||
                         (col == 10 && (row >= 3 && row <= 10 || row >= 12 && row <= 13 || row >= 17 && row <= 28)) ||

                         (col == 11 && (row >= 3 && row <= 20 || row >= 23 && row <= 28)) ||
                         (col == 12 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 13 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 14 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 15 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 16 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 17 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 18 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 19 && (row >= 3 && row <= 20 || row >= 23 && row <= 28)) ||

                         (col == 20 && (row >= 3 && row <= 10 || row >= 12 && row <= 13 || row >= 17 && row <= 28)) ||
                         (col == 21 && (row >= 3 && row <= 10 || row == 12 || row >= 18 && row <= 28)) ||
                         (col == 22 && (row >= 4 && row <= 10 || row == 12 || row >= 18 && row <= 27)) ||
                         (col == 23 && (row >= 4 && row <= 12 || row >= 18 && row <= 27)) ||
                         (col == 24 && (row >= 5 && row <= 13 || row >= 17 && row <= 26)) ||

                         (col == 25 && row >= 6 && row <= 25) ||
                         (col == 26 && row >= 7 && row <= 24) ||
                         (col == 27 && row >= 8 && row <= 23) ||
                         (col == 28 && row >= 10 && row <= 21) ||
                         (col == 29 && row >= 12 && row <= 19)) ;

      case 1: return (
                         (col == 1 && row >= 12 && row <= 19) ||
                         (col == 2 && row >= 10 && row <= 21) ||
                         (col == 3 && row >= 8 && row <= 23) ||
                         (col == 4 && row >= 7 && row <= 24) ||
                         (col == 5 && row >= 6 && row <= 25) ||
                        
                         (col == 6 && (row >= 5 && row <= 10 || row >= 12 && row <= 13 || row >= 17 && row <= 26)) ||
                         (col == 7 && (row >= 4 && row <= 10 || row == 12 || row >= 18 && row <= 27)) ||
                         (col == 8 && (row >= 4 && row <= 9 || row == 12 || row >= 18 && row <= 27)) ||
                         (col == 9 && (row >= 3 && row <= 9 || row == 12 || row >= 18 && row <= 21 || row >= 24 && row <= 28)) ||
                         (col == 10 && (row >= 3 && row <= 9 || row >= 12 && row <= 13 || row >= 17 && row <= 20 || row >= 24 && row <= 28)) ||

                         (col == 11 && (row >= 3 && row <= 20 || row >= 23 && row <= 28)) ||
                         (col == 12 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 13 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 14 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 15 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 16 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 17 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 18 && (row >= 2 && row <= 20 || row >= 23 && row <= 29)) ||
                         (col == 19 && (row >= 3 && row <= 20 || row >= 23 && row <= 28)) ||
                         (col == 20 && (row >= 3 && row <= 9 ||row >= 12 && row <= 13 || row >= 17 && row <= 20 || row >= 24 && row <= 28)) ||
                         (col == 21 && (row >= 3 && row <= 9 || row == 12 || row >= 18 && row <= 21 || row >= 24 && row <= 28)) ||
                         (col == 22 && (row >= 4 && row <= 9 || row == 12 || row >= 18 && row <= 27)) ||
                         (col == 23 && (row >= 4 && row <= 10 || row == 12 || row >= 18 && row <= 27)) ||
                         (col == 24 && (row >= 5 && row <= 10 || row >= 12 && row <= 13 || row >= 17 && row <= 26)) ||
                         
                         (col == 25 && row >= 6 && row <= 25) ||
                         (col == 26 && row >= 7 && row <= 24) ||
                         (col == 27 && row >= 8 && row <= 23) ||
                         (col == 28 && row >= 10 && row <= 21) ||
                         (col == 29 && row >= 12 && row <= 19) ||
                         (col == 29 && row >= 12 && row <= 19)) ;

      case 2: return (
                         (col == 1 && row >= 12 && row <= 19) ||
                         (col == 2 && row >= 10 && row <= 21) ||
                         (col == 3 && row >= 8 && row <= 23) ||
                         (col == 4 && row >= 7 && row <= 24) ||

                         (col == 5 && (row >= 6 && row <= 8 || row >= 10 && row <= 25)) ||
                         (col == 6 && (row >= 5 && row <= 8 || row >= 11 && row <= 13 || row >= 17 && row <= 26)) ||
                         (col == 7 && (row >= 4 && row <= 9 || row >= 11 && row <= 12 || row >= 18 && row <= 27)) ||
                         (col == 8 && (row >= 4 && row <= 9 || row == 12 || row >= 18 && row <= 27)) ||
                         (col == 9 && (row >= 3 && row <= 10 || row >= 18 && row <= 20 || row >= 24 && row <= 28)) ||
                         (col == 10 && (row >= 3 && row <= 11 || row >= 17 && row <= 19 || row >= 23 && row <= 28)) ||
                         (col == 11 && (row >= 3 && row <= 12 || row >= 14 && row <= 19 || row >= 22 && row <= 28)) ||

                         (col == 12 && (row >= 2 && row <= 19 || row >= 22 && row <= 29)) ||
                         (col == 13 && (row >= 2 && row <= 19 || row >= 22 && row <= 29)) ||
                         (col == 14 && (row >= 2 && row <= 19 || row >= 22 && row <= 29)) ||
                         (col == 15 && (row >= 2 && row <= 19 || row >= 22 && row <= 29)) ||
                         (col == 16 && (row >= 2 && row <= 19 || row >= 22 && row <= 29)) ||
                         (col == 17 && (row >= 2 && row <= 19 || row >= 22 && row <= 29)) ||
                         (col == 18 && (row >= 2 && row <= 19 || row >= 22 && row <= 29)) ||

                         (col == 19 && (row >= 3 && row <= 12 || row >= 14 && row <= 19 || row >= 22 && row <= 28)) ||
                         (col == 20 && (row >= 3 && row <= 11 || row >= 17 && row <= 19 || row >= 23 && row <= 28)) ||
                         (col == 21 && (row >= 3 && row <= 10 || row >= 18 && row <= 20 || row >= 24 && row <= 28)) ||
                         (col == 22 && (row >= 4 && row <= 9 || row == 12 || row >= 18 && row <= 27)) ||
                         (col == 23 && (row >= 4 && row <= 9 || row >= 11 && row <= 12 || row >= 18 && row <= 27)) ||
                         (col == 24 && (row >= 5 && row <= 8 || row >= 11 && row <= 13 || row >= 17 && row <= 26)) ||
                         (col == 25 && (row >= 6 && row <= 8 || row >= 10 && row <= 25)) ||

                         (col == 26 && row >= 7 && row <= 24) ||
                         (col == 27 && row >= 8 && row <= 23) ||
                         (col == 28 && row >= 10 && row <= 21) ||
                         (col == 29 && row >= 12 && row <= 19)
                         ) ;
    }
  }
  return false;
}

// ===========================
// LADO DERECHO Y DIENTES BLANCOS
// ===========================
bool isWhiteTeeth(int col, int row) {
  // Solo aparecen en la cara feliz, animFrame == 2
  if (soundValue <= 50 && animFrame == 2) {
    return ((row == 21 || row == 22) && (col >= 8 && col <= 22));
  }
  return false;
}
bool isRightRandomPattern(int col, int row) {
  if (soundValue <= 50) {
    switch (choiceBien) {
      case 0: return (
      //biensigue asi  
                         (col == 1 && (row >= 0 && row <= 9 || row >= 12 && row <= 15 || row >= 19 && row <= 20 || row >= 24 && row <= 31)) ||
                         (col == 2 && (row >= 0 && row <= 9 || row >= 11 && row <= 16 || row >= 19 && row <= 20 || row >= 23 && row <= 31)) ||
                         (col == 3 && (row >= 0 && row <= 1 || row >= 4 && row <= 5 || row >= 8 && row <= 12 && row != 10 || row >= 15 && row <= 16 || row >= 19 && row <= 23 && row != 21 || row >= 27 && row <= 28)) ||
                         (col == 4 && (row >= 0 && row <= 1 || row >= 4 && row <= 5 || row >= 8 && row <= 12 && row != 10 || row >= 15 && row <= 16 || row >= 19 && row <= 23 && row != 21 || row >= 27 && row <= 28)) ||
                         (col == 5 && (row >= 0 && row <= 12 && row != 10 || row >= 15 && row <= 20 || row >= 23 && row <= 31)) ||
                         (col == 6 && (row >= 1 && row <= 3 || row >= 6 && row <= 8 || row >= 11 && row <= 12 || row >= 16 && row <= 19 || row >= 24 && row <= 31)) ||

                         (col == 9 && (row >= 0 && row <= 1 || row >= 8 && row <= 12 && row != 10 || row >= 19 && row <= 20 || row >= 23 && row <= 26 || row >= 30 && row <= 31)) ||
                         (col == 10 && (row >= 0 && row <= 1 || row >= 8 && row <= 12 && row != 10 || row >= 19 && row <= 20 || row >= 22 && row <= 26 || row >= 30 && row <= 31)) ||
                         (col == 11 && (row >= 0 && row <= 23 && row != 10 && row != 21 || row >= 26 && row <= 27 || row >= 30 && row <= 31)) ||
                         (col == 12 && (row >= 0 && row <= 23 && row != 10 && row != 21 || row >= 26 && row <= 27 || row >= 30 && row <= 31)) ||
                         (col == 13 && (row >= 0 && row <= 1 || row >= 8 && row <= 12 && row != 10 || row >= 19 && row <= 23 && row != 21 || row >= 27 && row <= 31)) ||
                         (col == 14 && (row >= 0 && row <= 1 || row >= 8 && row <= 12 && row != 10 || row >= 19 && row <= 23 && row != 21 || row >= 27 && row <= 30)) ||

                         (col == 17 && (row >= 0 && row <= 9 || row >= 12 && row <= 19 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 18 && (row >= 0 && row <= 9 || row >= 11 && row <= 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 19 && (row >= 0 && row <= 1 || row >= 4 && row <= 5 || row >= 8 && row <= 12 && row != 10 || row >= 19 && row <= 31 && row != 21)) ||
                         (col == 20 && (row >= 0 && row <= 1 || row >= 4 && row <= 5 || row >= 8 && row <= 12 && row != 10 || row == 16 || row >= 19 && row <= 31 && row != 21)) ||
                         (col == 21 && (row >= 0 && row <= 1 || row >= 8 && row <= 12 && row != 10 || row >= 15 && row <= 16 || row >= 19 && row <= 23 && row != 21 || row >= 30 && row <= 31)) ||
                         (col == 22 && (row >= 0 && row <= 1 || row >= 8 && row <= 12 && row != 10 || row >= 15 && row <= 23 && row != 21 || row >= 30 && row <= 31)) ||
                         (col == 23 && (row == 12 || row >= 16 && row <= 19)) ||

                         (col == 24 && row >= 25 && row <= 26) ||

                         (col == 25 && (row >= 0 && row <= 9 || row >= 13 && row <= 20 || row >= 24 && row <= 27)) ||
                         (col == 26 && (row >= 1 && row <= 9 || row >= 12 && row <= 20 || row >= 24 && row <= 28)) ||
                         (col == 27 && (row >= 3 && row <= 4 || row >= 11 && row <= 12 || row >= 16 && row <= 17 || row >= 25 && row <= 29)) ||
                         (col == 28 && (row >= 5 && row <= 6 || row >= 11 && row <= 12 || row >= 16 && row <= 17 || row >= 24 && row <= 28)) ||
                         (col == 29 && (row >= 0 && row <= 8 || row >= 12 && row <= 20 || row >= 24 && row <= 27)) ||
                         (col == 30 && (row >= 0 && row <= 9 || row >= 13 && row <= 20 || row >= 25 && row <= 26))
                       );

      case 1: return (
      //joyasigueasi
                         (col == 1 && (row >= 0 && row <= 1 || row >= 7 && row <= 8 || row >= 12 && row <= 15 || row >= 19 && row <= 20 || row >= 24 && row <= 31)) ||
                         (col == 2 && (row >= 0 && row <= 1 || row >= 8 && row <= 16 && row != 10 || row >= 19 && row <= 20 || row >= 23 && row <= 31)) ||
                         (col == 3 && (row >= 0 && row <= 1 || row >= 8 && row <= 12 && row != 10 || row >= 15 && row <= 16|| row >= 19 && row <= 23 && row != 21 || row >= 27 && row <= 28)) ||
                         (col == 4 && (row >= 0 && row <= 12 && row != 10 || row >= 15 && row <= 23 && row != 21 || row >= 27 && row <= 28)) ||
                         (col == 5 && (row >= 0 && row <= 8 || row >= 11 && row <= 12 || row >= 16 && row <= 19 || row >= 23 && row <= 31)) ||
                         (col == 6 && (row >= 0 && row <= 1 || row >= 24 && row <= 31)) ||
                         (col == 7 && (row >= 11 && row <= 12 || row >= 19 && row <= 20)) ||
                         ((col == 8 && row >= 11 && row <= 20)) ||

                         (col == 9 && (row >= 1 && row <= 8 || row >= 11 && row <= 20 || row >= 23 && row <= 26 || row >= 30 && row <= 31)) ||
                         (col == 10 && (row >= 0 && row <= 12 && row != 10 || row >= 19 && row <= 26 && row != 21 || row >= 30 && row <= 31)) ||
                         (col == 11 && (row >= 0 && row <= 1 || row >= 8 && row <= 9 || row >= 22 && row <= 23 || row >= 26 && row <= 27 || row >= 30 && row <= 31)) ||
                         (col == 12 && (row >= 0 && row <= 1 || row >= 8 && row <= 9 || row >= 12 && row <= 19 || row >= 22 && row <= 23 || row >= 26 && row <= 27 || row >= 30 && row <= 31)) ||
                         (col == 13 && (row >= 0 && row <= 23 && row != 10 && row != 21 || row >= 27 && row <= 31)) ||
                         (col == 14 && (row >= 1 && row <= 8 || row >= 11 && row <= 12 || row >= 19 && row <= 20 || row >= 22 && row <= 23 || row >= 27 && row <= 30)) ||
                         (col == 15 && (row >= 11 && row <= 12 || row >= 16 && row <= 20 && row != 18)) ||
                         (col == 16 && (row >= 11 && row <= 12 || row >= 15 && row <= 16 || row >= 19 && row <= 20)) ||
                         (col == 17 && (row >= 0 && row <= 3 || row == 12 || row >= 16 && row <= 19 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 18 && (row >= 0 && row <= 4 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 19 && (row >= 3 && row <= 19 && row != 10 || row >= 22 && row <= 31)) ||
                         (col == 20 && (row >= 3 && row <= 31 && row != 10 && row != 21)) ||
                         (col == 21 && (row >= 0 && row <= 4 || row >= 19 && row <= 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 22 && (row >= 0 && row <= 3 || row >= 19 && row <= 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 23 && row >= 11 && row <= 20) ||
                         (col == 24 && (row >= 11 && row <= 19 || row >= 25 && row <= 26)) ||
                         (col == 25 && (row >= 2 && row <= 9 || row >= 24 && row <= 27)) ||
                         (col == 26 && (row >= 1 && row <= 9 || row >= 11 && row <= 20 || row >= 24 && row <= 28)) ||
                         (col == 27 && (row >= 0 && row <= 1 || row >= 5 && row <= 6 || row >= 11 && row <= 20 || row >= 25 && row <= 29)) ||
                         (col == 28 && (row >= 0 && row <= 1 || row >= 5 && row <= 6 || row >= 11 && row <= 12 || row >= 15 && row <= 16 || row >= 19 && row <= 20 || row >= 24 && row <= 28)) ||
                         (col == 29 && (row >= 1 && row <= 12 && row != 10 || row >= 15 && row <= 16 || row >= 19 && row <= 20 || row >= 24 && row <= 27)) ||
                         (col == 30 && (row >= 2 && row <= 12 && row != 10 || row >= 19 && row <= 20 || row >= 25 && row <= 26))
                       );
      case 2: return (
                         ((col >= 10 && col <= 20) && (row >= 10 && row <= 20)) 
        );
    }
  } else if (soundValue <= 200) {
    switch (choiceCuidado) {
      case 0: return (
                         (row == 1 && (col == 5 || col == 15 || col == 25)) ||
                         (row == 2 && (col >= 4 && col <= 6 || col >= 14 && col <= 16 || col >= 24 && col <= 26)) ||
                         (row == 3 && (col >= 4 && col <= 6 || col >= 14 && col <= 16 || col >= 24 && col <= 26)) ||
                         (row == 4 && (col >= 3 && col <= 7 && col != 5 || col >= 13 && col <= 17 && col != 15 || col >= 23 && col <= 27 && col != 25)) ||
                         (row == 5 && (col >= 3 && col <= 7 && col != 5 || col >= 13 && col <= 17 && col != 15 || col >= 23 && col <= 27 && col != 25)) ||
                         (row == 6 && (col >= 2 && col <= 8 && col != 5 || col >= 12 && col <= 18 && col != 15 || col >= 22 && col <= 28 && col != 25)) ||
                         (row == 7 && (col >= 2 && col <= 8 || col >= 12 && col <= 18 || col >= 22 && col <= 28)) ||
                         (row == 8 && (col >= 1 && col <= 9 && col != 5 || col >= 11 && col <= 19 && col != 15 || col >= 21 && col <= 29 && col != 25)) ||
                         (row == 9 && (col >= 1 && col <= 9 || col >= 11 && col <= 19 || col >= 21 && col <= 29)) ||

                         //vertical de las H
                         (col >= 9 && col <= 10 && row >= 11 && row <= 20) ||
                         (col >= 13 && col <= 14 && row >= 11 && row <= 20) ||
                         (col >= 17 && col <= 18 && row >= 11 && row <= 20) ||
                         (col >= 21 && col <= 22 && row >= 11 && row <= 20) ||
                         (col >= 25 && col <= 26 && row >= 11 && row <= 20) ||
                         (col >= 29 && col <= 30 && row >= 11 && row <= 20) ||
                         //resto de las H y la S
                         (row == 11 && col >= 2 && col <= 6) ||
                         (row == 12 && col >= 1 && col <= 6) ||
                         (row >= 13 && row <= 14 && (col >= 1 && col <= 2)) ||
                         (row == 15 && (col >= 1 && col <= 5 || col >= 9 && col <= 14 || col >= 17 && col <= 22 || col >= 25 && col <= 30)) ||
                         (row == 16 && (col >= 2 && col <= 6 || col >= 9 && col <= 14 || col >= 17 && col <= 22 || col >= 25 && col <= 30)) ||
                         (row >= 17 && row <= 18 && (col >= 5 && col <= 6)) ||
                         (row == 19 && col >= 1 && col <= 6) ||
                         (row == 20 && col >= 1 && col <= 5) ||

                         (row == 22 && (col == 5 || col == 15 || col == 25)) ||
                         (row == 23 && (col >= 4 && col <= 6 || col >= 14 && col <= 16 || col >= 24 && col <= 26)) ||
                         (row == 24 && (col >= 4 && col <= 6 || col >= 14 && col <= 16 || col >= 24 && col <= 26)) ||
                         (row == 25 && (col >= 3 && col <= 7 && col != 5 || col >= 13 && col <= 17 && col != 15 || col >= 23 && col <= 27 && col != 25)) ||
                         (row == 26 && (col >= 3 && col <= 7 && col != 5 || col >= 13 && col <= 17 && col != 15 || col >= 23 && col <= 27 && col != 25)) ||
                         (row == 27 && (col >= 2 && col <= 8 && col != 5 || col >= 12 && col <= 18 && col != 15 || col >= 22 && col <= 28 && col != 25)) ||
                         (row == 28 && (col >= 2 && col <= 8 || col >= 12 && col <= 18 || col >= 22 && col <= 28)) ||
                         (row == 29 && (col >= 1 && col <= 9 && col != 5 || col >= 11 && col <= 19 && col != 15 || col >= 21 && col <= 29 && col != 25)) ||
                         (row == 30 && (col >= 1 && col <= 9 || col >= 11 && col <= 19 || col >= 21 && col <= 29))

                       );

      case 1: return (
                         (col == 0 && (row >= 1 && row <= 8 || row >= 23 && row <= 31)) ||
                         (col == 1 && (row == 0 || row == 9 || row >= 22 && row <= 31)) ||
                         (col == 2 && (row == 0 || row == 9 || row >= 22 && row <= 26 && row != 24)) ||
                         (col == 3 && (row == 1 || row == 8 || row >= 12 && row <= 19 || row >= 22 && row <= 26 && row != 24)) ||
                         (col == 4 && (row == 11 || row == 20 || row >= 22 && row <= 31 && row != 26)) ||
                         (col == 5 && (row >= 0 && row <= 8 || row == 11 || row == 20 || row >= 23 && row <= 31 && row != 26)) ||
                         (col == 6 && (row == 9 || row == 12 || row == 19)) ||
                         (col == 7 && (row == 9 || row >= 22 && row <= 30)) ||
                         (col == 8 && (row >= 0 && row <= 8 || row >= 12 && row <= 19 || row >= 22 && row <= 31)) ||
                         (col == 9 && (row == 11 || row == 20 || row >= 30 && row <= 31)) ||
                         (col == 10 && (row == 0 || row == 9 || row == 11 || row == 20 || row >= 30 && row <= 31)) ||
                         (col == 11 && (row >= 0 && row <= 9 || row >= 12 && row <= 19 || row >= 22 && row <= 31)) ||
                         (col == 12 && (row == 0 || row == 9 || row >= 22 && row <= 30)) ||
                         (col == 13 && (row >= 11 && row <= 20)) ||
                         (col == 14 && (row >= 0 && row <= 9 || row >= 13 && row <= 15 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 15 && (row == 0 || row == 9 || row >= 16 && row <= 18 || row >= 22 && row <= 31)) ||
                         (col == 16 && (row == 0 || row == 9 || row >= 11 && row <= 20 || row >= 22 && row <= 31)) ||
                         (col == 17 && (row >= 1 && row <= 8 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 19 && (row >= 1 && row <= 9 || row >= 22 && row <= 31)) ||
                         (col == 20 && (row == 0 || row == 2 || row >= 11 && row <= 20 || row >= 22 && row <= 31)) ||
                         (col == 21 && (row >= 1 && row <= 9 || row == 11 || row == 14 || row == 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 22 && (row == 11 || row == 14 || row == 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 23 && (row >= 0 && row <= 9 || row == 11 || row == 20 || row >= 23 && row <= 30)) ||
                         (col == 24 && (row == 0 || row == 9 || row >= 24 && row <= 29)) ||
                         (col == 25 && (row == 0 || row == 9 || row >= 11 && row <= 20)) ||
                         (col == 26 && (row >= 1 && row <= 8 || row == 20 || row >= 23 && row <= 30)) ||
                         (col == 27 && (row == 20 || row >= 22 && row <= 31)) ||
                         (col == 28 && (row >= 1 && row <= 8 || row == 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 29 && (row == 0 || row == 9 || row >= 22 && row <= 31)) ||
                         (col == 30 && (row == 0 || row == 9 || row >= 23 && row <= 30)) ||
                         (col == 31 && (row >= 1 && row <= 8)) 
                                       );

        case 2: return (
                         (col == 0 && (row >= 1 && row <= 8 || row >= 23 && row <= 31)) ||
                         (col == 1 && (row == 0 || row == 9 || row >= 22 && row <= 31)) 
                        );
        }
  } else {
    switch (choiceMuchoRuido) {
      case 0: return (
      //Funadito
                         (col == 1 && (row >= 0 && row <= 9 || row >= 11 && row <= 20 || row >= 22 && row <= 23 || row >= 29 && row <= 30)) ||
                         (col == 2 && (row >= 0 && row <= 20 && row != 10 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 3 && (row >= 0 && row <= 1 || row >= 4 && row <= 5 || row >= 11 && row <= 12 || row >= 19 && row <= 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 4 && (row >= 0 && row <= 1 || row >= 4 && row <= 5 || row >= 11 && row <= 12 || row >= 19 && row <= 20 || row >= 22 && row <= 31)) ||
                         (col == 5 && (row >= 0 && row <= 1 || row >= 12 && row <= 19 || row >= 22 && row <= 30)) ||
                         (col == 6 && (row >= 0 && row <= 1 || row >= 14 && row <= 17 || row >= 22 && row <= 23)) ||
                         (col == 9 && (row >= 0 && row <= 8 || row >= 11 && row <= 12 || row >= 19 && row <= 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 10 && (row >= 0 && row <= 9 || row >= 11 && row <= 12 || row >= 19 && row <= 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 11 && (row >= 8 && row <= 9 || row >= 11 && row <= 20 || row >= 22 && row <= 31)) ||
                         (col == 12 && (row >= 8 && row <= 9 || row >= 11 && row <= 20 || row >= 22 && row <= 31)) ||
                         (col == 13 && (row >= 0 && row <= 9 || row >= 11 && row <= 12 || row >= 19 && row <= 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 14 && (row >= 0 && row <= 8 || row >= 11 && row <= 12 || row >= 19 && row <= 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 17 && (row >= 0 && row <= 9 || row >= 11 && row <= 12 || row >= 22 && row <= 23 || row >= 29 && row <= 30)) ||
                         (col == 18 && (row >= 1 && row <= 9 || row >= 11 && row <= 12 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 19 && (row >= 3 && row <= 4 || row >= 11 && row <= 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 20 && (row >= 5 && row <= 6 || row >= 11 && row <= 20 || row >= 22 && row <= 31)) ||
                         (col == 21 && (row >= 0 && row <= 8 || row >= 11 && row <= 12 || row >= 22 && row <= 30)) ||
                         (col == 22 && (row >= 0 && row <= 9 || row >= 11 && row <= 12 || row >= 22 && row <= 23)) ||
                         (col == 25 && (row >= 1 && row <= 9 || row >= 12 && row <= 19 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 26 && (row >= 0 && row <= 8 || row >= 11 && row <= 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 27 && (row >= 0 && row <= 1 || row >= 5 && row <= 6 || row >= 11 && row <= 12 || row >= 19 && row <= 20 || row >= 22 && row <= 31)) ||
                         (col == 28 && (row >= 0 && row <= 1 || row >= 5 && row <= 6 || row >= 11 && row <= 12 || row >= 19 && row <= 20 || row >= 22 && row <= 31)) ||
                         (col == 29 && (row >= 0 && row <= 8 || row >= 11 && row <= 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 30 && (row >= 1 && row <= 9 || row >= 12 && row <= 19 || row >= 22 && row <= 23 || row >= 30 && row <= 31))
                       );
      case 1: return (
      //Bajale al ruido
                         (col >= 0 && col <= 1 && (row >= 0 && row <= 9 || row >= 22 && row <= 31)) ||
                         (col == 2 && (row >= 0 && row <= 4 && row != 2 || row >= 8 && row <= 9 || row >= 22 && row <= 23 || row == 26)) ||
                         (col == 3 && (row >= 0 && row <= 9 || row >= 22 && row <= 23 || row >= 26 && row <= 27)) ||
                         (col == 4 && (row >= 1 && row <= 8 && row != 4 || row >= 22 && row <= 31)) ||
                         (col == 5 && (row >= 23 && row <= 31 && row != 27)) ||
                         (col == 6 && (row >= 1 && row <= 9)) ||
                         (col == 7 && (row >= 0 && row <= 9 || row >= 22 && row <= 30)) ||
                         (col == 8 && (row >= 0 && row <= 1 || row >= 4 && row <= 5 || row >= 22 && row <= 31)) ||
                         (col == 9 && (row >= 0 && row <= 9 || row >= 30 && row <= 31)) ||
                         (col == 10 && (row >= 1 && row <= 9 || row >= 12 && row <= 20 || row >= 30 && row <= 31)) ||
                         (col == 11 && (row >= 11 && row <= 20 || row >= 22 && row <= 31)) ||
                         (col == 12 && (row >= 0 && row <= 1 || row >= 7 && row <= 8 || row >= 11 && row <= 12 || row >= 15 && row <= 16 || row >= 22 && row <= 30)) ||
                         (col == 13 && (row >= 0 && row <= 1 || row >= 8 && row <= 9 || row >= 11 && row <= 20)) ||
                         (col == 14 && (row >= 0 && row <= 9 || row >= 12 && row <= 20 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 15 && (row >= 0 && row <= 8 || row >= 22 && row <= 31)) ||
                         (col == 16 && (row >= 11 && row <= 31 && row != 21)) ||
                         (col == 17 && (row >= 1 && row <= 20 && row != 10 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 18 && (row >= 0 && row <= 9 || row >= 19 && row <= 20)) ||
                         (col == 19 && (row >= 0 && row <= 1 || row >= 4 && row <= 5 || row >= 19 && row <= 20 || row >= 22 && row <= 31)) ||
                         (col == 20 && (row >= 0 && row <= 9 || row >= 19 && row <= 31 && row != 21)) ||
                         (col == 21 && (row >= 1 && row <= 9 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col == 22 && (row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col >= 23 && col <= 24 && (row >= 0 && row <= 9)) ||
                         (col == 23 && (row >= 23 && row <= 30)) ||
                         (col == 24 && (row >= 24 && row <= 29)) ||
                         (col >= 25 && col <= 26 && (row >= 8 && row <= 9)) ||
                         (col == 26 && (row >= 23 && row <= 30)) ||
                         (col == 27 && (row >= 22 && row <= 31)) ||
                         (col >= 28 && col <= 29 && (row >= 0 && row <= 9 || row >= 22 && row <= 23 || row >= 30 && row <= 31)) ||
                         (col >= 30 && col <= 31 && (row >= 0 && row <= 1 || row >= 8 && row <= 9 || row >= 23 && row <= 30)) ||
                         (col == 30 && (row >= 4 && row <= 5 || row == 22 || row == 31))
                       );
                       
      case 2: return (
      //Bajale a tu volumen  
                         (col >= 0 && col <= 1 && (row >= 0 && row <= 9)) ||
                         (col == 0 && (row >= 22 && row <= 30)) ||
                         (col >= 1 && col <= 2 && row == 31) ||
                         (col == 2 && (row >= 0 && row <= 4 && row != 2 || row >= 8 && row <= 9)) ||
                         (col == 3 && (row >= 0 && row <= 9 || row >= 22 && row <= 30)) ||
                         (col == 4 && (row >= 1 && row <= 8 && row != 4)) ||
                         (col == 5 && (row >= 23 && row <= 30)) ||
                         (col >= 6 && col <= 7 && (row >= 1 && row <= 9 || row == 22 || row == 31)) || 
                         (col == 7 && row == 0) ||
                         (col == 8 && (row >= 0 && row <= 1 || row >= 4 && row <= 5 || row >= 12 && row <= 20 || row >= 23 && row <= 30)) ||
                         (col >= 9 && col <= 10 && (row >= 1 && row <= 9 || row == 11 || row == 15)) || 
                         (col == 9 && row == 0) || (col == 10 && (row >= 22 && row <= 31)) ||
                         (col == 11 && (row >= 12 && row <= 20 || row == 31)) ||
                         (col >= 12 && col <= 15 && (row >= 0 && row <= 1)) || 
                         (col == 12 && (row >= 7 && row <= 8 || row == 31)) ||
                         (col == 13 && (row >= 8 && row <= 9)) ||
                         (col == 14 && (row == 9 || row >= 22 && row <= 31)) ||
                         (col >= 14 && col <= 15 && (row >= 2 && row <= 8)) || 
                         (col == 15 && (row == 11 || row == 31)) ||
                         (col == 16 && (row == 11 || row >= 22 && row <= 31)) ||
                         (col == 17 && (row >= 1 && row <= 9 || row >= 11 && row <= 20)) ||
                         (col == 18 && (row >= 0 && row <= 9 || row == 11 || row >= 22 && row <= 31)) ||
                         (col == 19 && (row >= 0 && row <= 1 || row >= 4 && row <= 5 || row == 11 || row >= 22 && row <= 23)) ||
                         (col == 20 && (row >= 0 && row <= 9 || row >= 23 && row <= 24)) ||
                         (col == 21 && (row >= 1 && row <= 9 || row >= 11 && row <= 20 || row >= 22 && row <= 23)) ||
                         (col == 22 && (row == 20 || row >= 22 && row <= 31)) ||
                         (col == 23 && (row >= 0 && row <= 9 || row == 20)) ||
                         (col == 24 && (row >= 0 && row <= 9 || row >= 11 && row <= 20 || row >= 22 && row <= 31)) ||
                         (col >= 25 && col <= 26 && (row >= 8 && row <= 9 || row == 22 || row == 31)) || 
                         (col == 25 && row == 25) ||
                         (col == 28 && (row >= 0 && row <= 9 || row >= 22 && row <= 31)) ||
                         (col == 29 && (row >= 0 && row <= 9 || row >= 24 && row <= 26)) ||
                         (col == 30 && (row >= 0 && row <= 1 || row >= 4 && row <= 5 || row >= 8 && row <= 9 || row >= 27 && row <= 29)) ||
                         (col == 31 && (row >= 0 && row <= 1 || row >= 8 && row <= 9 || row >= 22 && row <= 31))
                       );
    }
  }
  return false;
}