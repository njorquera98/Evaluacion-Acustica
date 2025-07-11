#include <Arduino.h>

// ===========================
// PINES DE MATRIZ 32x64
// ===========================
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
#define OE   13

// ===========================
// CONTROL DE TIEMPOS
// ===========================
const unsigned long duracionAnimacion = 500;
const unsigned long duracionFrase     = 2000;

// ===========================
// VARIABLES DE ESTADO
// ===========================
int value              = 20;
int animFrame          = 0;
unsigned long lastFrameChange  = 0;
unsigned long lastChoiceTime   = 0;
int choiceBien        = 0;
int choiceCuidado     = 0;
int choiceMuchoRuido  = 0;

// ===========================
// SETUP
// ===========================
void setup() {
  DDRD |= 0b11111100;
  DDRB |= 0b00111111;
  DDRC |= 0b00000001;
  PORTB |= (1 << PB5);
  randomSeed(analogRead(0));
}

// ===========================
// LOOP PRINCIPAL
// ===========================
void loop() {
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

  for (int row = 0; row < 16; row++) {
    selectRow(row);

    for (int repeat = 0; repeat < 3; repeat++) { // enviar tres veces la misma imagen
      for (int col = 0; col < 64; col++) {
        int colorTop = 0;
        int colorBottom = 0;

        if (col < 32) {
          if (isWhiteTeeth(col, row)) {
            colorTop = 7;
          } else if (isAnimatedFace(col, row)) {
            colorTop = determineColor(value);
          }

          if (isWhiteTeeth(col, row + 16)) {
            colorBottom = 7;
          } else if (isAnimatedFace(col, row + 16)) {
            colorBottom = determineColor(value);
          }

        } else {
          int rightCol = col - 32;
          if (isRightRandomPattern(rightCol, row))
            colorTop = determineColor(value);
          if (isRightRandomPattern(rightCol, row + 16))
            colorBottom = determineColor(value);
        }

        setPixels(colorTop, colorBottom);
        digitalWrite(CLK, HIGH);
        delayMicroseconds (1);
        digitalWrite(CLK, LOW);
      }
    }

    latchData();
  }
}

// ===========================
// FUNCIONES AUXILIARES
// ===========================
int determineColor(int val) {
  if (val <= 100) return 2;
  if (val <= 200) return 3;
  return 1;
}

void setPixels(int colorTop, int colorBottom) {
  byte topVal = 0, botVal = 0;
  if (colorTop & 1)    topVal |= (1 << 2);
  if (colorTop & 2)    topVal |= (1 << 3);
  if (colorTop & 4)    topVal |= (1 << 4);
  if (colorBottom & 1) botVal |= (1 << 5);
  if (colorBottom & 2) botVal |= (1 << 6);
  if (colorBottom & 4) botVal |= (1 << 7);
  PORTD = (PORTD & 0x03) | topVal | botVal;
}

void selectRow(int row) {
  PORTB = (PORTB & 0b11111000) | (row & 0x07);
  if (row & 0x08) PORTC |= (1 << PC0);
  else            PORTC &= ~(1 << PC0);
}

void latchData() {
  digitalWrite(LAT, HIGH);
  digitalWrite(LAT, LOW);
  digitalWrite(OE, LOW);
  delayMicroseconds(300);
  digitalWrite(OE, HIGH);
}

// Las funciones isAnimatedFace, isWhiteTeeth e isRightRandomPattern siguen igual.
// ===========================
// ANIMACIÓN LADO IZQUIERDO
// ===========================
bool isAnimatedFace(int col, int row) {
  if (value <= 100) {
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
  } else if (value <= 200) {
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
// DIENTES BLANCOS EN FRAME 2
// ===========================
bool isWhiteTeeth(int col, int row) {
  // Solo aparecen en la cara feliz, animFrame == 2
  if (value <= 100 && animFrame == 2) {
    return ((row == 21 || row == 22) && (col >= 8 && col <= 22));
  }
  return false;
}

// ===========================
// RANDOM A LA DERECHA
// ===========================
bool isRightRandomPattern(int col, int row) {
  if (value <= 100) {
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
  } else if (value <= 200) {
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
                         ((col >= 10 && col <= 20) && (row >= 10 && row <= 20)) 
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