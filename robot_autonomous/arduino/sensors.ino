/*
 * sensors.ino - Control de 4 Sensores HC-SR04 para Robot Autónomo
 * 
 * Conexiones:
 * Sensor Frontal:  Trig=2, Echo=3
 * Sensor Trasero:  Trig=4, Echo=5
 * Sensor Izquierdo: Trig=6, Echo=7
 * Sensor Derecho:  Trig=8, Echo=9
 * 
 * Comunicación Serial: 115200 baud
 * Protocolo: START|F:XXX|B:XXX|L:XXX|R:XXX|CHK:XX\n
 */

// === DEFINICIONES DE PINES ===
#define TRIG_FRONT    2
#define ECHO_FRONT    3
#define TRIG_BACK     4
#define ECHO_BACK     5
#define TRIG_LEFT     6
#define ECHO_LEFT     7
#define TRIG_RIGHT    8
#define ECHO_RIGHT    9

// === CONFIGURACIONES ===
#define BAUD_RATE     115200
#define MAX_DISTANCE  400        // Distancia máxima en cm
#define MIN_DISTANCE  2          // Distancia mínima en cm
#define TIMEOUT_US    23000      // Timeout para echo (400cm * 58)
#define TRIGGER_PULSE 10         // Duración del pulso trigger en microsegundos
#define SENSOR_DELAY  70         // Delay entre sensores para evitar interferencia (ms)
#define READ_INTERVAL 100        // Intervalo entre lecturas completas (ms)

// === VARIABLES GLOBALES ===
struct SensorData {
  int front;
  int back;
  int left;
  int right;
  unsigned long timestamp;
};

SensorData currentData;
unsigned long lastReadTime = 0;
bool sensorEnabled = true;
int readFrequency = 10; // Hz

// === SETUP ===
void setup() {
  // Inicializar comunicación serial
  Serial.begin(BAUD_RATE);
  
  // Configurar pines de sensores
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_BACK, OUTPUT);
  pinMode(ECHO_BACK, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Asegurar que todos los triggers estén en LOW
  digitalWrite(TRIG_FRONT, LOW);
  digitalWrite(TRIG_BACK, LOW);
  digitalWrite(TRIG_LEFT, LOW);
  digitalWrite(TRIG_RIGHT, LOW);
  
  // Inicializar datos
  currentData.front = 0;
  currentData.back = 0;
  currentData.left = 0;
  currentData.right = 0;
  currentData.timestamp = 0;
  
  // Delay inicial para estabilización
  delay(100);
  
  // Mensaje de inicio
  Serial.println("START|SYSTEM:READY|CHK:OK");
  
  Serial.flush();
}

// === FUNCIÓN PARA LEER UN SENSOR HC-SR04 ===
int readUltrasonicSensor(int trigPin, int echoPin) {
  // Enviar pulso trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(TRIGGER_PULSE);
  digitalWrite(trigPin, LOW);
  
  // Leer tiempo de echo con timeout
  unsigned long duration = pulseIn(echoPin, HIGH, TIMEOUT_US);
  
  // Si no hay respuesta (timeout)
  if (duration == 0) {
    return -1; // Error de timeout
  }
  
  // Convertir tiempo a distancia en cm
  // Fórmula: distancia = (tiempo_microsegundos) / 58
  int distance = duration / 58;
  
  // Validar rango
  if (distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
    return -2; // Error de rango
  }
  
  return distance;
}

// === FUNCIÓN PARA LEER TODOS LOS SENSORES ===
void readAllSensors() {
  // Leer sensor frontal
  currentData.front = readUltrasonicSensor(TRIG_FRONT, ECHO_FRONT);
  delay(SENSOR_DELAY);
  
  // Leer sensor trasero
  currentData.back = readUltrasonicSensor(TRIG_BACK, ECHO_BACK);
  delay(SENSOR_DELAY);
  
  // Leer sensor izquierdo
  currentData.left = readUltrasonicSensor(TRIG_LEFT, ECHO_LEFT);
  delay(SENSOR_DELAY);
  
  // Leer sensor derecho
  currentData.right = readUltrasonicSensor(TRIG_RIGHT, ECHO_RIGHT);
  delay(SENSOR_DELAY);
  
  // Actualizar timestamp
  currentData.timestamp = millis();
}

// === FUNCIÓN PARA CALCULAR CHECKSUM SIMPLE ===
int calculateChecksum(int front, int back, int left, int right) {
  return (front + back + left + right) % 100;
}

// === FUNCIÓN PARA ENVIAR DATOS POR SERIAL ===
void sendSensorData() {
  int checksum = calculateChecksum(currentData.front, currentData.back, 
                                   currentData.left, currentData.right);
  
  // Formato: START|F:XXX|B:XXX|L:XXX|R:XXX|T:XXXXXX|CHK:XX
  Serial.print("START|F:");
  Serial.print(currentData.front);
  Serial.print("|B:");
  Serial.print(currentData.back);
  Serial.print("|L:");
  Serial.print(currentData.left);
  Serial.print("|R:");
  Serial.print(currentData.right);
  Serial.print("|T:");
  Serial.print(currentData.timestamp);
  Serial.print("|CHK:");
  Serial.println(checksum);
  
  Serial.flush();
}

// === FUNCIÓN PARA PROCESAR COMANDOS RECIBIDOS ===
void processCommand(String command) {
  command.trim();
  
  if (command.startsWith("CMD|")) {
    // Procesar comando de configuración
    if (command.indexOf("FREQ:") > 0) {
      int freqStart = command.indexOf("FREQ:") + 5;
      int freqEnd = command.indexOf("|", freqStart);
      if (freqEnd == -1) freqEnd = command.length();
      
      int newFreq = command.substring(freqStart, freqEnd).toInt();
      if (newFreq >= 1 && newFreq <= 20) {
        readFrequency = newFreq;
        Serial.println("ACK|FREQ:OK");
      } else {
        Serial.println("ERR|FREQ:INVALID");
      }
    }
    
    if (command.indexOf("ENABLE:") > 0) {
      int enableStart = command.indexOf("ENABLE:") + 7;
      char enableValue = command.charAt(enableStart);
      
      if (enableValue == '1') {
        sensorEnabled = true;
        Serial.println("ACK|ENABLE:ON");
      } else if (enableValue == '0') {
        sensorEnabled = false;
        Serial.println("ACK|ENABLE:OFF");
      } else {
        Serial.println("ERR|ENABLE:INVALID");
      }
    }
    
  } else if (command == "STATUS") {
    // Enviar estado del sistema
    Serial.print("STATUS|FREQ:");
    Serial.print(readFrequency);
    Serial.print("|ENABLE:");
    Serial.print(sensorEnabled ? "ON" : "OFF");
    Serial.print("|UPTIME:");
    Serial.println(millis());
    
  } else if (command == "TEST") {
    // Modo de prueba - una lectura inmediata
    readAllSensors();
    sendSensorData();
    
  } else {
    Serial.println("ERR|CMD:UNKNOWN");
  }
}

// === FUNCIÓN PARA MANEJAR ERRORES ===
void handleSensorError(int sensorId, int errorCode) {
  Serial.print("ERR|SENSOR:");
  Serial.print(sensorId);
  Serial.print("|CODE:");
  Serial.println(errorCode);
}

// === LOOP PRINCIPAL ===
void loop() {
  unsigned long currentTime = millis();
  
  // Verificar si hay comandos en el puerto serial
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
  
  // Leer sensores según la frecuencia configurada
  if (sensorEnabled && (currentTime - lastReadTime >= (1000 / readFrequency))) {
    readAllSensors();
    
    // Verificar errores en sensores individuales
    if (currentData.front < 0) handleSensorError(0, currentData.front);
    if (currentData.back < 0) handleSensorError(1, currentData.back);
    if (currentData.left < 0) handleSensorError(2, currentData.left);
    if (currentData.right < 0) handleSensorError(3, currentData.right);
    
    // Enviar datos
    sendSensorData();
    
    lastReadTime = currentTime;
  }
  
  // Pequeño delay para no saturar el loop
  delay(10);
}

// === FUNCIÓN DE DIAGNÓSTICO (OPCIONAL) ===
void diagnosticMode() {
  Serial.println("=== MODO DIAGNÓSTICO ===");
  
  // Probar cada sensor individualmente
  int sensors[4][2] = {{TRIG_FRONT, ECHO_FRONT}, {TRIG_BACK, ECHO_BACK},
                       {TRIG_LEFT, ECHO_LEFT}, {TRIG_RIGHT, ECHO_RIGHT}};
  String names[4] = {"FRONTAL", "TRASERO", "IZQUIERDO", "DERECHO"};
  
  for (int i = 0; i < 4; i++) {
    Serial.print("Probando sensor ");
    Serial.print(names[i]);
    Serial.print(": ");
    
    int distance = readUltrasonicSensor(sensors[i][0], sensors[i][1]);
    
    if (distance > 0) {
      Serial.print(distance);
      Serial.println(" cm - OK");
    } else {
      Serial.print("ERROR ");
      Serial.println(distance);
    }
    
    delay(500);
  }
  
  Serial.println("=== FIN DIAGNÓSTICO ===");
}