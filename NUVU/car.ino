// Pines de los motores
const int int1 = 3;
const int int2 = 4;
const int ena_A = 5;
const int int3 = 6;
const int int4 = 7;
const int ena_B = 8;

// Pines de los LEDs
const int ledAvanzar = 9;   // avanzar
const int ledRetroceder = 10;  // retroceder
const int ledIzquierda = 11;   // giro a la izquierda
const int ledDerecha = 12;     // giro a la derecha
const int ledParar = 13;       // parar

// Pines del sensor de ultrasonido
const int trigPin = A0;
const int echoPin = A1;

// Pines de los sensores infrarrojos
const int irLeft = A2;
const int irRight = A3;

// Umbral de distancia para detección de obstáculos
const int distanceThreshold = 20;  // en centímetros

// Función para medir la distancia usando el sensor de ultrasonido
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2; // Calcular distancia en cm
  return distance;
}

// Funciones de movimiento del vehículo
void forward() {
  resetLeds();
  digitalWrite(int1, HIGH);
  digitalWrite(int2, LOW);
  digitalWrite(int3, HIGH);
  digitalWrite(int4, LOW);
  digitalWrite(ledAvanzar, HIGH);
  Serial.println("Avanzando");
}

void backward() {
  resetLeds();
  digitalWrite(int1, LOW);
  digitalWrite(int2, HIGH);
  digitalWrite(int3, LOW);
  digitalWrite(int4, HIGH);
  digitalWrite(ledRetroceder, HIGH);
  Serial.println("Retrocediendo");
}

void left() {
  resetLeds();
  digitalWrite(int1, LOW);
  digitalWrite(int2, HIGH);
  digitalWrite(int3, HIGH);
  digitalWrite(int4, LOW);
  digitalWrite(ledIzquierda, HIGH);
  Serial.println("Girando a la izquierda");
}

void right() {
  resetLeds();
  digitalWrite(int1, HIGH);
  digitalWrite(int2, LOW);
  digitalWrite(int3, LOW);
  digitalWrite(int4, HIGH);
  digitalWrite(ledDerecha, HIGH);
  Serial.println("Girando a la derecha");
}

void stop() {
  digitalWrite(ena_A, LOW);
  digitalWrite(ena_B, LOW);
  resetLeds();
  digitalWrite(ledParar, HIGH);
  Serial.println("Parado");
}

// Función para resetear los LEDs
void resetLeds() {
  digitalWrite(ledAvanzar, LOW);
  digitalWrite(ledRetroceder, LOW);
  digitalWrite(ledIzquierda, LOW);
  digitalWrite(ledDerecha, LOW);
  digitalWrite(ledParar, LOW);
}

// Función para seguir la línea negra
void followLine() {
  int leftSensor = digitalRead(irLeft);
  int rightSensor = digitalRead(irRight);

  if (leftSensor == LOW && rightSensor == LOW) {
    // Ambos sensores detectan la línea, avanzar recto
    forward();
  } else if (leftSensor == LOW && rightSensor == HIGH) {
    // Solo el sensor izquierdo detecta la línea, girar a la izquierda
    left();
  } else if (leftSensor == HIGH && rightSensor == LOW) {
    // Solo el sensor derecho detecta la línea, girar a la derecha
    right();
  } else {
    // Ningún sensor detecta la línea, detenerse
    stop();
  }
}

void setup() {
  // Inicializar pines de motores
  pinMode(int1, OUTPUT);
  pinMode(int2, OUTPUT);
  pinMode(ena_A, OUTPUT);
  pinMode(int3, OUTPUT);
  pinMode(int4, OUTPUT);
  pinMode(ena_B, OUTPUT);

  // Inicializar pines de los LEDs
  pinMode(ledAvanzar, OUTPUT);
  pinMode(ledRetroceder, OUTPUT);
  pinMode(ledIzquierda, OUTPUT);
  pinMode(ledDerecha, OUTPUT);
  pinMode(ledParar, OUTPUT);

  // Inicializar pines del sensor de ultrasonido
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Inicializar pines de los sensores infrarrojos
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);

  // Iniciar comunicación serie
  Serial.begin(9600);
}

void loop() {
  // Verificar la distancia del sensor de ultrasonido
  long distance = getDistance();
  if (distance < distanceThreshold) {
    // Si el obstáculo está a menos de 20 cm
    Serial.print("Obstáculo detectado a ");
    Serial.print(distance);
    Serial.println(" cm");
    stop();
  } else {
    // Seguir la línea mientras no haya obstáculos
    followLine();
  }

  delay(100);  // Esperar 100 ms entre iteraciones
}
