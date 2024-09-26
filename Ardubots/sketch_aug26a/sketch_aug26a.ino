#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define enA 10
#define enB 5

const int irSensor1 = A0;  // IR sensor 1 connected to analog pin A0
const int irSensor2 = A1;  // IR sensor 2 connected to analog pin A1

int motorSpeed = 150;
int LeftRotationSpeed = 250;  // Left Rotation Speed
int RightRotationSpeed = 250; // Right Rotation Speed

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(irSensor1, INPUT); // initialize Left sensor as an input
  pinMode(irSensor2, INPUT); // initialize Right sensor as an input

  Serial.begin(9600);  // Initialize serial communication at 9600 baud
}

void loop() {
  int sensor1Value = digitalRead(irSensor1);  // Read digital value from IR sensor 1
  int sensor2Value = digitalRead(irSensor2);  // Read digital value from IR sensor 2

  if (sensor1Value == 1 && sensor2Value == 1) {
    forward(); // FORWARD
  }

  else if (sensor1Value == 0 && sensor2Value == 1) {
    right(); // Move Right
  }

  else if (sensor1Value == 1 && sensor2Value == 0) {
    left(); // Move Left
  }

  Serial.print("IR Sensor 1: ");
  Serial.print(sensor1Value);
  Serial.print("  IR Sensor 2: ");
  Serial.println(sensor2Value);
  delay(50); // Wait 50ms before taking the next reading
}

void forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, motorSpeed);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, motorSpeed);

  Serial.println("Moving forward");
}

// Function to move backward
void backward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, motorSpeed);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, motorSpeed);

  Serial.println("Moving backward");
}

// Function to turn left
void left() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, motorSpeed);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, motorSpeed);

  Serial.println("Turning left");
}

// Function to turn right
void right() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, motorSpeed);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, motorSpeed);

  Serial.println("Turning right");
}

// Function to stop the robot
void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);

  Serial.println("Stopping motors");
}