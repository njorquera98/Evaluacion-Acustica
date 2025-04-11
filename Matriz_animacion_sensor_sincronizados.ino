// =====================================================================
// CONFIGURACIÓN DE PINES
// =====================================================================
#define R1 2   // PD2
#define G1 3   // PD3
#define B1 4   // PD4
#define R2 5   // PD5
#define G2 6   // PD6
#define B2 7   // PD7

#define A 8    // PB0
#define B 9    // PB1
#define C 10   // PB2
#define D 14   // A0 (PC0)

#define CLK 11 // PB3
#define LAT 12 // PB4
#define OE  13 // PB5

// =====================================================================
// VARIABLES GLOBALES
// =====================================================================
int value = 0;         // Variable para almacenar el valor del sensor.
bool valueSet = false; // Bandera para saber si ya se ha recibido un valor válido del sensor.
int animFrame = 0;     // Fotograma actual (0..2)
unsigned long lastFrameTime = 1000; // Tiempo del último cambio de fotograma
const unsigned long frameDuration = 1000; // Duración de cada fotograma en milisegundos

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(9600); // Inicia la comunicación serial para recibir datos y depurar

  DDRD |= 0b11111100;  // PD2-PD7 como salidas
  DDRB |= 0b00111111;  // PB0-PB5 como salidas
  DDRC |= 0b00000001;  // PC0 como salida
  PORTB |= (1 << PB5); // Deshabilitar salida (OE)
  Serial.println("Sistema iniciado. Esperando datos del sensor...");
}

// =====================================================================
// LOOP
// =====================================================================
void loop() {
  // Revisar si se han recibido datos nuevos desde Python
  if (Serial.available() > 0) {
    int newValue = Serial.parseInt(); // Lee el número enviado
    if (newValue != 0) {  
      value = newValue;
      valueSet = true;
      Serial.print("Nuevo valor recibido: ");
      Serial.println(value);
    }
  }

  if (!valueSet) {
    Serial.println("Esperando valor del sensor...");
    return; // Si no se ha recibido un valor válido, no actualizar la pantalla
  }

  // Cambiar de fotograma cada frameDuration milisegundos
  if (millis() - lastFrameTime >= frameDuration) {
    animFrame = (animFrame + 1) % 3; // Ciclar entre 0, 1 y 2
    lastFrameTime = millis(); // Reiniciar el temporizador
    Serial.print("Cambiando al fotograma: ");
    Serial.println(animFrame);
  }

  // Mostrar la animación correspondiente según el valor de decibelios
  for (int row = 0; row < 16; row++) {
    selectRow(row);
    for (int col = 0; col < 64; col++) {
      int colorTop = 0;
      if (isPartOfFace(col, row)) {
        colorTop = determineColor(value, col, row, animFrame);
      }

      int colorBottom = 0;
      if (isPartOfFace(col, row + 16)) {
        colorBottom = determineColor(value, col, row + 16, animFrame);
      }

      setPixels(colorTop, colorBottom);
      PORTB |= (1 << PB3);  // Pulso de CLK
      PORTB &= ~(1 << PB3);
    }
    latchData();
  }
}

// =====================================================================
// determineColor() - Devuelve el color según posición y frame
// =====================================================================
int determineColor(int val, int col, int row, int frame) {
  if (val <= 50) { // Cara verde (feliz)
    // Blanco en columnas 49-50, filas 8-23 (frame 2)
    if (frame == 2 && (col == 49 || col == 50) && (row >= 8 && row <= 23)) {
      return 4; // Blanco
    }
    return 2; // Verde para otras zonas
  } else if (val <= 90) {
    return 3; // Amarillo (indiferente)
  }
  return 1; // Rojo (enojado)
}

// =====================================================================
// setPixels() - Aplica colores con soporte para blanco
// =====================================================================
void setPixels(int colorTop, int colorBottom) {
  uint8_t topBits = 0;
  uint8_t botBits = 0;

  switch (colorTop) {
    case 1: topBits = 0b001; break; // Rojo
    case 2: topBits = 0b010; break; // Verde
    case 3: topBits = 0b011; break; // Amarillo
    case 4: topBits = 0b111; break; // Blanco (R+G+B)
    default: topBits = 0b000;
  }

  switch (colorBottom) {
    case 1: botBits = 0b001; break; // Rojo
    case 2: botBits = 0b010; break; // Verde
    case 3: botBits = 0b011; break; // Amarillo
    case 4: botBits = 0b111; break; // Blanco (R+G+B)
    default: botBits = 0b000;
  }

  // Escribir en PORTD (PD2-PD7)
  PORTD = (PORTD & 0x03) | 
          ((topBits & 0x01) << 2) | // R1 (PD2)
          ((topBits & 0x02) << 2) | // G1 (PD3)
          ((topBits & 0x04) << 2) | // B1 (PD4)
          ((botBits & 0x01) << 5) | // R2 (PD5)
          ((botBits & 0x02) << 5) | // G2 (PD6)
          ((botBits & 0x04) << 5);  // B2 (PD7)
}

// =====================================================================
// Selecciona la fila (0..15) con A, B, C, D
// =====================================================================
void selectRow(int row) {
  PORTB = (PORTB & 0b11111000) | (row & 0x07); // A, B, C (PB0-PB2)
  PORTC = (PORTC & 0b11111110) | ((row & 0x08) ? 0x01 : 0x00); // D (PC0)
}

// =====================================================================
// latchData() - Actualiza la salida
// =====================================================================
void latchData() {
  PORTB |= (1 << PB4);  // LAT HIGH
  PORTB &= ~(1 << PB4); // LAT LOW
  PORTB &= ~(1 << PB5); // OE LOW (encender LEDs)
  delayMicroseconds(100);
  PORTB |= (1 << PB5);  // OE HIGH (apagar LEDs)
}

// =====================================================================
// isPartOfFace(col, row):
// Devuelve true si (col, row) pertenece al fotograma actual (animFrame)
// de la cara correspondiente (verde, amarilla o roja).
// =====================================================================
bool isPartOfFace(int col, int row) {
  // 1) CARA VERDE (value <= 100)
  if (value <= 50) {
    if (animFrame == 0) {
      // Frame 0 de cara verde (sin cambios)
      if ((col == 31 && row >= 12 && row <= 19)   ||
        (col == 32 && row >= 9 && row <= 22)    ||
        (col == 33 && row >= 7 && row <= 24)    ||
        (col == 34 && row >= 6 && row <= 25)    ||
        (col == 35 && row >= 4 && row <= 27)    ||
        (col == 36 && row >= 4 && row <= 27)    ||
        (col == 37 && row >= 3 && row <= 28)    ||
        (col == 38 && row >= 2 && row <= 29)   ||
        (col == 39 && row >= 1 && row <= 30)   ||
        (col == 40 && row >= 1 && row <= 30)   ||
        (col == 41 && row >= 1 && row <= 30)   ||
        (col == 42 && row >= 0 && row <= 31)   ||
        //empiezan ojos
        (col == 43 && row >= 0 && row <= 7)   ||
            (col == 43 && row >= 13 && row <= 18)   ||
            (col == 43 && row >= 24 && row <= 31)   ||
        (col == 44 && row >= 0 && row <= 7)   ||
            (col == 44 && row >= 13 && row <= 18)   ||
            (col == 44 && row >= 24 && row <= 31)   ||
        (col == 45 && row >= 0 && row <= 7)   ||
            (col == 45 && row >= 13 && row <= 18)   ||
            (col == 45 && row >= 24 && row <= 31)   ||
        (col == 46 && row >= 0 && row <= 7)   ||
            (col == 46 && row >= 13 && row <= 18)   ||
            (col == 46 && row >= 24 && row <= 31)   ||
        (col == 47 && row >= 0 && row <= 7)   ||
            (col == 47 && row >= 13 && row <= 18)   ||
            (col == 47 && row >= 24 && row <= 31)   ||
        //terminan ojos
        (col == 48 && row >= 0 && row <= 31)   ||
        (col == 49 && row >= 1 && row <= 30)   ||
        (col == 50 && row >= 1 && row <= 30)   ||
        //empieza boca
        (col == 51 && row >= 1 && row <= 8)   ||
        (col == 51 && row >= 23 && row <= 30)   ||
        (col == 52 && row >= 2 && row <= 8)   ||
        (col == 52 && row >= 23 && row <= 29)   ||
        //termina boca
        (col == 53 && row >= 3 && row <= 28)   ||
        (col == 54 && row >= 4 && row <= 27)   ||
        (col == 55 && row >= 4 && row <= 27)   ||
        (col == 56 && row >= 6 && row <= 25)   ||
        (col == 57 && row >= 7 && row <= 24)   ||
        (col == 58 && row >= 9 && row <= 22)   ||
        (col == 59 && row >= 12 && row <= 19)
       ) {
        return true;
      }
    }
    else if (animFrame == 1) {
      // Frame 1 de cara verde (sin cambios)
      if ((col == 31 && row >= 12 && row <= 19)   ||
        (col == 32 && row >= 9 && row <= 22)    ||
        (col == 33 && row >= 7 && row <= 24)    ||
        (col == 34 && row >= 6 && row <= 25)    ||
        (col == 35 && row >= 4 && row <= 27)    ||
        (col == 36 && row >= 4 && row <= 27)    ||
        (col == 37 && row >= 3 && row <= 28)    ||
        (col == 38 && row >= 2 && row <= 29)   ||
        (col == 39 && row >= 1 && row <= 30)   ||
        (col == 40 && row >= 1 && row <= 30)   ||
        (col == 41 && row >= 1 && row <= 30)   ||
        (col == 42 && row >= 0 && row <= 31)   ||                       
        //empiezan ojos
        (col == 43 && row >= 0 && row <= 9)   ||
            (col == 43 && row >= 12 && row <= 19)   ||
            (col == 43 && row >= 22 && row <= 31)   ||
        (col == 44 && row >= 0 && row <= 8)   ||
            (col == 44 && row >= 12 && row <= 19)   ||
            (col == 44 && row >= 23 && row <= 31)   ||
        (col == 45 && row >= 0 && row <= 7)   ||
            (col == 45 && row >= 13 && row <= 18)   ||
            (col == 45 && row >= 24 && row <= 31)   ||
        (col == 46 && row >= 0 && row <= 7)   ||
            (col == 46 && row >= 13 && row <= 18)   ||
            (col == 46 && row >= 24 && row <= 31)   ||
        (col == 47 && row >= 0 && row <= 7)   ||
            (col == 47 && row >= 13 && row <= 18)   ||
            (col == 47 && row >= 24 && row <= 31)   ||
        //terminan ojos
        (col == 48 && row >= 0 && row <= 31)   ||
        (col == 49 && row >= 1 && row <= 30)   ||
        //empieza boca
        (col == 50 && row >= 1 && row <= 7)   ||
                (col == 50 && row >= 11 && row <= 20)   ||
                (col == 50 && row >= 24 && row <= 30)   ||
        (col == 51 && row >= 1 && row <= 7)   ||
                (col == 51 && row >= 24 && row <= 30)   ||
        (col == 52 && row >= 2 && row <= 7)   ||
                (col == 52 && row >= 24 && row <= 29)   ||
        (col == 53 && row >= 3 && row <= 10)   ||
                (col == 53 && row >= 21 && row <= 28)   ||
        //termina boca
        (col == 54 && row >= 4 && row <= 27)   ||
        (col == 55 && row >= 4 && row <= 27)   ||
        (col == 56 && row >= 6 && row <= 25)   ||
        (col == 57 && row >= 7 && row <= 24)   ||
        (col == 58 && row >= 9 && row <= 22)   ||
        (col == 59 && row >= 12 && row <= 19)
       ) {
        return true;
      }
    }
    else { // animFrame == 2
      // Frame 2 de cara verde (con cambios en las columnas 49 y 50)
      if ((col == 31 && row >= 12 && row <= 19)   ||
        (col == 32 && row >= 9  && row <= 22)   ||
        (col == 33 && row >= 7  && row <= 24)   ||
        (col == 34 && row >= 6  && row <= 25)   ||
        (col == 35 && row >= 4  && row <= 27)   ||
        (col == 36 && row >= 4  && row <= 27)   ||
        (col == 37 && row >= 3  && row <= 28)   ||
        (col == 38 && row >= 2  && row <= 29)   ||
        // ojos (cambio)
        (col == 39 && row >= 1  && row <= 7)    ||
        (col == 39 && row >= 10 && row <= 21)   ||
        (col == 39 && row >= 24 && row <= 30)   ||
        (col == 40 && row >= 1  && row <= 7)    ||
        (col == 40 && row >= 10 && row <= 21)   ||
        (col == 40 && row >= 24 && row <= 30)   ||
        (col == 41 && row >= 1  && row <= 6)    ||
        (col == 41 && row >= 11 && row <= 20)   ||
        (col == 41 && row >= 25 && row <= 30)   ||
        (col == 42 && row >= 0  && row <= 6)    ||
        (col == 42 && row >= 11 && row <= 20)   ||
        (col == 42 && row >= 25 && row <= 31)   ||
        (col == 43 && row >= 0  && row <= 6)    ||
        (col == 43 && row >= 11 && row <= 20)   ||
        (col == 43 && row >= 25 && row <= 31)   ||
        (col == 44 && row >= 0  && row <= 6)    ||
        (col == 44 && row >= 11 && row <= 20)   ||
        (col == 44 && row >= 25 && row <= 31)   ||
        (col == 45 && row >= 0  && row <= 6)    ||
        (col == 45 && row >= 8  && row <= 9)    ||
        (col == 45 && row >= 11 && row <= 20)   ||
        (col == 45 && row >= 22 && row <= 23)   ||
        (col == 45 && row >= 25 && row <= 31)   ||
        // terminan ojos
        (col == 46 && row >= 0  && row <= 31)   ||
        (col == 47 && row >= 0  && row <= 31)   ||
        // empieza boca
        (col == 48 && row >= 0  && row <= 5)    ||
        (col == 48 && row >= 26 && row <= 31)   ||
        // Cambios específicos en las columnas 49 y 50
    (col == 49 && (
      (row >= 1 && row <= 5) || 
      (row >= 8 && row <= 23) || 
      (row >= 26 && row <= 30)
    )) || 
    (col == 50 && (
      (row >= 1 && row <= 5) || 
      (row >= 8 && row <= 23) || 
      (row >= 26 && row <= 30)
    )) || 
    
        // termina boca
        (col == 51 && row >= 1  && row <= 5)    ||
        (col == 51 && row >= 26 && row <= 30)   ||
        (col == 52 && row >= 2  && row <= 7)    ||
        (col == 52 && row >= 24 && row <= 29)   ||
        (col == 53 && row >= 3  && row <= 8)    ||
        (col == 53 && row >= 23 && row <= 28)   ||
        (col == 54 && row >= 4  && row <= 9)    ||
        (col == 54 && row >= 21 && row <= 27)   ||
        // termina boca
        (col == 55 && row >= 4  && row <= 27)   ||
        (col == 56 && row >= 6  && row <= 25)   ||
        (col == 57 && row >= 7  && row <= 24)   ||
        (col == 58 && row >= 9  && row <= 22)   ||
        (col == 59 && row >= 12 && row <= 19)
       ) {
        return true;
      }
    }
  }

   // 2) CARA AMARILLA (101..200)
  else if (value <= 90) {
    if (animFrame == 0) {
      // Frame 0 de cara amarilla
      if ((col == 17 && row >= 12 && row <= 19)   ||
        (col == 18 && row >= 9 && row <= 22)    ||
        (col == 19 && row >= 7 && row <= 24)    ||
        (col == 20 && row >= 6 && row <= 25)    ||
        (col == 21 && row >= 4 && row <= 27)    ||
        (col == 22 && row >= 4 && row <= 27)    ||
        (col == 23 && row >= 3 && row <= 28)    ||
        (col == 24 && row >= 2 && row <= 29)   ||
        (col == 25 && row >= 1 && row <= 30)   ||
        (col == 26 && row >= 1 && row <= 30)   ||
        (col == 27 && row >= 1 && row <= 30)   ||
        (col == 28 && row >= 0 && row <= 31)   ||
        //empiezan ojos
        (col == 29 && row >= 0 && row <= 7)   ||
            (col == 29 && row >= 13 && row <= 18)   ||
            (col == 29 && row >= 24 && row <= 31)   ||
        (col == 30 && row >= 0 && row <= 7)   ||
            (col == 30 && row >= 13 && row <= 18)   ||
            (col == 30 && row >= 24 && row <= 31)   ||
        (col == 31 && row >= 0 && row <= 7)   ||
            (col == 31 && row >= 13 && row <= 18)   ||
            (col == 31 && row >= 24 && row <= 31)   ||
        (col == 32 && row >= 0 && row <= 7)   ||
            (col == 32 && row >= 13 && row <= 18)   ||
            (col == 32 && row >= 24 && row <= 31)   ||
        (col == 33 && row >= 0 && row <= 7)   ||
            (col == 33 && row >= 13 && row <= 18)   ||
            (col == 33 && row >= 24 && row <= 31)   ||
        //terminan ojos
        (col == 34 && row >= 0 && row <= 31)   ||
        (col == 35 && row >= 1 && row <= 30)   ||
        (col == 36 && row >= 1 && row <= 30)   ||
        //empieza boca
        (col == 37 && row >= 1 && row <= 8)   ||
        (col == 37 && row >= 23 && row <= 30)   ||
        (col == 38 && row >= 2 && row <= 8)   ||
        (col == 38 && row >= 23 && row <= 29)   ||
        //termina boca
        (col == 39 && row >= 3 && row <= 28)   ||
        (col == 40 && row >= 4 && row <= 27)   ||
        (col == 41 && row >= 4 && row <= 27)   ||
        (col == 42 && row >= 6 && row <= 25)   ||
        (col == 43 && row >= 7 && row <= 24)   ||
        (col == 44 && row >= 9 && row <= 22)   ||
        (col == 45 && row >= 12 && row <= 19)
       ) {
        return true;
      }
    }
    else if (animFrame == 1) {
      // Frame 1 de cara amarilla
      if ((col == 17 && row >= 12 && row <= 19)   ||
          (col == 18 && row >= 9  && row <= 22)    ||
          (col == 19 && row >= 7  && row <= 24)    ||
          (col == 20 && row >= 6  && row <= 25)    ||
          (col == 21 && row >= 4  && row <= 27)    ||
          (col == 22 && row >= 4  && row <= 27)    ||
          (col == 23 && row >= 3  && row <= 28)    ||
          (col == 24 && row >= 2  && row <= 29)    ||
          (col == 25 && row >= 1  && row <= 30)    ||
          (col == 26 && row >= 1  && row <= 30)    ||
//cejas y ojos
    (col == 27 && row >= 1  && row <= 7)    ||
        (col == 27 && row >= 11  && row <= 20)    ||
        (col == 27 && row >= 24  && row <= 30)    ||
    (col == 28 && row >= 0  && row <= 4)    ||
        (col == 28 && row >= 9  && row <= 22)    || 
        (col == 28 && row >= 27  && row <= 31)    ||         
    (col == 29 && row >= 0  && row <= 4)     ||
        (col == 29 && row >= 6 && row <= 25)    ||
        (col == 29 && row >= 27 && row <= 31)    ||
    (col == 30 && row >= 0  && row <= 3)     ||
        (col == 30 && row >= 6 && row <= 6)    ||
        (col == 30 && row >= 12 && row <= 18)    ||
        (col == 30 && row >= 24 && row <= 25)    ||
        (col == 30 && row >= 28 && row <= 31)    ||
    (col == 31 && row >= 0  && row <= 6)     ||
          (col == 31 && row >= 12 && row <= 18)    ||
          (col == 31 && row >= 24 && row <= 31)    ||
    (col == 32 && row >= 0  && row <= 6)     ||
        (col == 32 && row >= 11 && row <= 11)    ||
        (col == 32 && row >= 14 && row <= 18)    ||
        (col == 32 && row >= 23 && row <= 23)    ||
        (col == 32 && row >= 26 && row <= 31)    ||
    (col == 33 && row >= 0  && row <= 6)     ||
          (col == 33 && row >= 11  && row <= 18)     ||
          (col == 33 && row >= 23 && row <= 31)    ||
  // terminan ojos
    (col == 34 && row >= 0  && row <= 31)     ||
    (col == 35 && row >= 1  && row <= 30)    ||
    (col == 36 && row >= 1  && row <= 30)    ||
    (col == 37 && row >= 1  && row <= 30)     ||

  //empieza boca
    (col == 38 && row >= 2 && row <= 9)    ||
          (col == 38 && row >= 22  && row <= 29)     ||
    (col == 39 && row >= 3 && row <= 8)    ||         
          (col == 39 && row >= 23 && row <= 28)    ||   
    (col == 40 && row >= 4 && row <= 8)    || 
          (col == 40 && row >= 11 && row <= 20)    ||
          (col == 40 && row >= 23 && row <= 27)    ||
   // termina boca
         
          (col == 41 && row >= 4  && row <= 27)    ||
          (col == 42 && row >= 6  && row <= 25)    ||
          (col == 43 && row >= 7  && row <= 24)    ||
          (col == 44 && row >= 9  && row <= 22)    ||
          (col == 45 && row >= 12 && row <= 19)
       ) { 
        return true;
      }
    }
    else { // animFrame == 2
      // Frame 2 de cara amarilla
      if ((col == 17 && row >= 12 && row <= 19)   ||
        (col == 18 && row >= 9  && row <= 22)   ||
        (col == 19 && row >= 7  && row <= 24)   ||
        (col == 20 && row >= 6  && row <= 25)   ||
        (col == 21 && row >= 4  && row <= 27)   ||
        (col == 22 && row >= 4  && row <= 27)   ||
        (col == 23 && row >= 3  && row <= 28)   ||
        (col == 24 && row >= 2  && row <= 29)   ||
        (col == 25 && row >= 1  && row <= 30)   ||
        (col == 26 && row >= 1  && row <= 30)   ||
        // cejas
        (col == 27 && row >= 1  && row <= 5)    ||
        (col == 27 && row >= 12 && row <= 19)   ||
        (col == 27 && row >= 26 && row <= 30)   ||
        (col == 28 && row >= 0  && row <= 4)    ||
        (col == 28 && row >= 12 && row <= 20)   ||
        (col == 28 && row >= 27 && row <= 31)   ||
        (col == 29 && row >= 0  && row <= 1)    ||
        (col == 29 && row >= 6  && row <= 25)   ||
        (col == 29 && row >= 30 && row <= 31)   ||
        (col == 30 && row >= 0  && row <= 1)    ||
        (col == 30 && row >= 6  && row <= 25)   ||
        (col == 30 && row >= 30 && row <= 31)   ||
        // ojos
        (col == 31 && row >= 0  && row <= 6)    ||
        (col == 31 && row >= 12 && row <= 19)   ||
        (col == 31 && row >= 25 && row <= 31)   ||
        (col == 32 && row >= 0  && row <= 6)    ||
        (col == 32 && row >= 12 && row <= 19)   ||
        (col == 32 && row >= 25 && row <= 31)   ||
        (col == 33 && row >= 0  && row <= 6)    ||
        (col == 33 && row >= 10 && row <= 11)   ||
        (col == 33 && row >= 14 && row <= 19)   ||
        (col == 33 && row >= 23 && row <= 24)   ||
        (col == 33 && row >= 27 && row <= 31)   ||
        (col == 34 && row >= 0  && row <= 6)    ||
        (col == 34 && row >= 10 && row <= 11)   ||
        (col == 34 && row >= 14 && row <= 19)   ||
        (col == 34 && row >= 23 && row <= 24)   ||
        (col == 34 && row >= 27 && row <= 31)   ||
        (col == 35 && row >= 1  && row <= 30)   ||
        (col == 36 && row >= 1  && row <= 30)   ||
        (col == 37 && row >= 1  && row <= 30)   ||
        // boca
        (col == 38 && row >= 2  && row <= 11)   ||
        (col == 38 && row >= 20 && row <= 29)   ||
        (col == 39 && row >= 3  && row <= 9)    ||
        (col == 39 && row >= 22 && row <= 28)   ||
        (col == 40 && row >= 4  && row <= 9)    ||
        (col == 40 && row >= 13 && row <= 18)   ||
        (col == 40 && row >= 22 && row <= 27)   ||
        (col == 41 && row >= 4  && row <= 27)   ||
        (col == 42 && row >= 6  && row <= 25)   ||
        (col == 43 && row >= 7  && row <= 24)   ||
        (col == 44 && row >= 9  && row <= 22)   ||
        (col == 45 && row >= 12 && row <= 19)
       ) {
        return true;
      }
    }
  }

  // 3) CARA ROJA (>200)
  else {
    if (animFrame == 0) {
      // Frame 0 de cara roja
      if ((col == 3 && row >= 12 && row <= 19)   ||
        (col == 4 && row >= 9 && row <= 22)    ||
        (col == 5 && row >= 7 && row <= 24)    ||
        (col == 6 && row >= 6 && row <= 25)    ||
        (col == 7 && row >= 4 && row <= 27)    ||
        (col == 8 && row >= 4 && row <= 27)    ||
        (col == 9 && row >= 3 && row <= 28)    ||
        (col == 10 && row >= 2 && row <= 29)   ||
        (col == 11 && row >= 1 && row <= 30)   ||
        (col == 12 && row >= 1 && row <= 30)   ||
        (col == 13 && row >= 1 && row <= 30)   ||
        (col == 14 && row >= 0 && row <= 31)   ||
        //empiezan ojos
        (col == 15 && row >= 0 && row <= 7)   ||
            (col == 15 && row >= 13 && row <= 18)   ||
            (col == 15 && row >= 24 && row <= 31)   ||
        (col == 16 && row >= 0 && row <= 7)   ||
            (col == 16 && row >= 13 && row <= 18)   ||
            (col == 16 && row >= 24 && row <= 31)   ||
        (col == 17 && row >= 0 && row <= 7)   ||
            (col == 17 && row >= 13 && row <= 18)   ||
            (col == 17 && row >= 24 && row <= 31)   ||
        (col == 18 && row >= 0 && row <= 7)   ||
            (col == 18 && row >= 13 && row <= 18)   ||
            (col == 18 && row >= 24 && row <= 31)   ||
        (col == 19 && row >= 0 && row <= 7)   ||
            (col == 19 && row >= 13 && row <= 18)   ||
            (col == 19 && row >= 24 && row <= 31)   ||
        //terminan ojos
        (col == 20 && row >= 0 && row <= 31)   ||
        (col == 21 && row >= 1 && row <= 30)   ||
        (col == 22 && row >= 1 && row <= 30)   ||
        //empieza boca
        (col == 23 && row >= 1 && row <= 8)   ||
        (col == 23 && row >= 23 && row <= 30)   ||
        (col == 24 && row >= 2 && row <= 8)   ||
        (col == 24 && row >= 23 && row <= 29)   ||
        //termina boca
        (col == 25 && row >= 3 && row <= 28)   ||
        (col == 26 && row >= 4 && row <= 27)   ||
        (col == 27 && row >= 4 && row <= 27)   ||
        (col == 28 && row >= 6 && row <= 25)   ||
        (col == 29 && row >= 7 && row <= 24)   ||
        (col == 30 && row >= 9 && row <= 22)   ||
        (col == 31 && row >= 12 && row <= 19)
       ) {
        return true;
      }
    }
    else if (animFrame == 1) {
      // Frame 1 de cara roja
      if ((col == 3 && row >= 12 && row <= 19)   ||
        (col == 4 && row >= 9 && row <= 22)    ||
        (col == 5 && row >= 7 && row <= 24)    ||
        (col == 6 && row >= 6 && row <= 25)    ||
        (col == 7 && row >= 4 && row <= 27)    ||
        (col == 8 && row >= 4 && row <= 27)    ||
        (col == 9 && row >= 3 && row <= 28)    ||
        (col == 10 && row >= 2 && row <= 29)   ||      
        (col == 11 && row >= 1 && row <= 30)   ||
        (col == 12 && row >= 1 && row <= 30)   ||
    //empiezan los ojos y cejas
        (col == 13 && row >= 1 && row <= 4)   ||
            (col == 13 && row >= 11 && row <= 20)   ||
            (col == 13 && row >= 27 && row <= 30)   ||
        (col == 14 && row >= 0 && row <= 4)   ||
            (col == 14 && row >= 11 && row <= 20)   ||
            (col == 14 && row >= 27 && row <= 31)   ||
        (col == 15 && row >= 0 && row <= 7)   ||
            (col == 15 && row >= 13 && row <= 18)   ||
            (col == 15 && row >= 24 && row <= 31)   ||
        (col == 16 && row >= 0 && row <= 7)   ||
            (col == 16 && row >= 13 && row <= 18)   ||
            (col == 16 && row >= 24 && row <= 31)   ||
        (col == 17 && row >= 0 && row <= 7)   ||
            (col == 17 && row >= 13 && row <= 18)   ||
            (col == 17 && row >= 24 && row <= 31)   ||
        (col == 18 && row >= 0 && row <= 7)   ||
            (col == 18 && row >= 13 && row <= 18)   ||
            (col == 18 && row >= 24 && row <= 31)   ||
        (col == 19 && row >= 0 && row <= 7)   ||
            (col == 19 && row >= 13 && row <= 18)   ||
            (col == 19 && row >= 24 && row <= 31)   ||
        //terminan ojos
        (col == 20 && row >= 0 && row <= 31)   ||
        (col == 21 && row >= 1 && row <= 30)   ||
        (col == 22 && row >= 1 && row <= 30)   ||
        //empieza boca
        (col == 23 && row >= 1 && row <= 9)   ||
           (col == 23 && row >= 22 && row <= 30)   ||
        (col == 24 && row >= 2 && row <= 9)   ||
           (col == 24 && row >= 22 && row <= 29)   ||
        (col == 25 && row >= 3 && row <= 9)   ||
           (col == 25 && row >= 12 && row <= 19)   ||
           (col == 25 && row >= 22 && row <= 28)   ||
        //termina boca
        (col == 26 && row >= 4 && row <= 27)   ||
        (col == 27 && row >= 4 && row <= 27)   ||
        (col == 28 && row >= 6 && row <= 25)   ||
        (col == 29 && row >= 7 && row <= 24)   ||
        (col == 30 && row >= 9 && row <= 22)   ||
        (col == 31 && row >= 12 && row <= 19)
       ) {
        return true;
      }
    }
    else { // animFrame == 2
      // Frame 2 de cara roja
      if ((col == 3 && row >= 12 && row <= 19)   ||
        (col == 4 && row >= 9  && row <= 22)   ||
        (col == 5 && row >= 7  && row <= 24)   ||
        (col == 6 && row >= 6  && row <= 25)   ||
        (col == 7 && row >= 4  && row <= 27)   ||
        (col == 8 && row >= 4  && row <= 27)   ||
        (col == 9 && row >= 3  && row <= 28)   ||
        // aquí empieza cambios
        (col == 10 && row >= 2 && row <= 6)    ||
        (col == 10 && row >= 9 && row <= 22)   ||
        (col == 10 && row >= 25 && row <= 29)  ||
        (col == 11 && row >= 1 && row <= 6)    ||
        (col == 11 && row >= 10 && row <= 21)  ||
        (col == 11 && row >= 25 && row <= 30)  ||
        (col == 12 && row >= 1 && row <= 7)    ||
        (col == 12 && row >= 11 && row <= 20)  ||
        (col == 12 && row >= 24 && row <= 30)  ||
        (col == 13 && row >= 1 && row <= 8)    ||
        (col == 13 && row >= 12 && row <= 19)  ||
        (col == 13 && row >= 23 && row <= 30)  ||
        (col == 14 && row >= 0 && row <= 9)    ||
        (col == 14 && row >= 13 && row <= 18)  ||
        (col == 14 && row >= 22 && row <= 31)  ||
        // ojos
        (col == 15 && row >= 0 && row <= 7)    ||
        (col == 15 && row >= 13 && row <= 18)  ||
        (col == 15 && row >= 24 && row <= 31)  ||
        (col == 16 && row >= 0 && row <= 7)    ||
        (col == 16 && row >= 13 && row <= 18)  ||
        (col == 16 && row >= 24 && row <= 31)  ||
        (col == 17 && row >= 0 && row <= 7)    ||
        (col == 17 && row >= 13 && row <= 18)  ||
        (col == 17 && row >= 24 && row <= 31)  ||
        (col == 18 && row >= 0 && row <= 7)    ||
        (col == 18 && row >= 13 && row <= 18)  ||
        (col == 18 && row >= 24 && row <= 31)  ||
        (col == 19 && row >= 0 && row <= 7)    ||
        (col == 19 && row >= 13 && row <= 18)  ||
        (col == 19 && row >= 24 && row <= 31)  ||
        // terminan ojos
        (col == 20 && row >= 0 && row <= 31)   ||
        (col == 21 && row >= 1 && row <= 30)   ||
        // empieza boca
        (col == 22 && row >= 1 && row <= 12)   ||
        (col == 22 && row >= 9 && row <= 30)   ||
        (col == 23 && row >= 1 && row <= 10)   ||
        (col == 23 && row >= 21 && row <= 30)  ||
        (col == 24 && row >= 2 && row <= 9)    ||
        (col == 24 && row >= 13 && row <= 18)  ||
        (col == 24 && row >= 22 && row <= 29)  ||
        (col == 25 && row >= 3 && row <= 9)    ||
        (col == 25 && row >= 12 && row <= 19)  ||
        (col == 25 && row >= 22 && row <= 28)  ||
        // termina boca
        (col == 26 && row >= 4 && row <= 27)   ||
        (col == 27 && row >= 4 && row <= 27)   ||
        (col == 28 && row >= 6 && row <= 25)   ||
        (col == 29 && row >= 7 && row <= 24)   ||
        (col == 30 && row >= 9 && row <= 22)   ||
        (col == 31 && row >= 12 && row <= 19)
       ) {
        return true;
      }
    }
  }

  // Si no coincidió nada, return false
  return false;
}