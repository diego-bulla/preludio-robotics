int pinLeds[] = {1, 2, 3, 4, 5, 6, 7, 8}; // Array con los pines de los LEDs
int tiempo = 500;

void setup() {
  // Configurar todos los pines de los LEDs como salidas
  for (int i = 0; i < 8; i++) {
        pinMode(pinLeds[i], OUTPUT);
      }
    }

    void loop() {
      // Encender y apagar cada LED en secuencia con un retraso de 
      for (int i = 0; i < 8; i++) {
        digitalWrite(pinLeds[i], HIGH);
        delay(tiempo);
        digitalWrite(pinLeds[i], LOW);
        delay(tiempo);
         }
       } 