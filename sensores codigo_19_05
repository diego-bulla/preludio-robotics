#define in1 7
#define in2 8
#define in3 9
#define in4 10
#define enA 6
#define enB 11

const int trigPin = 2;          // Define the trig pin for the front sensor
const int echoPin = 3;          // Define the echo pin for the front sensor
const int trigPinLeft = 4;      // Define the trig pin for the left sensor
const int echoPinLeft = 5;      // Define the echo pin for the left sensor
const int trigPinRight = 12;    // Define the trig pin for the right sensor
const int echoPinRight = 13;     // Define the echo pin for the right sensor

const int led1 = 14;            // Define LED pins
const int led2 = 15;
const int led3 = 16;
const int led4 = 17;

int duration = 150;
int motorSpeed = 210; // Assign motor speed value

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);  
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  Serial.begin(9600);
}

void forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, motorSpeed);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, motorSpeed);

  digitalWrite(led1, HIGH);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
  digitalWrite(led4, LOW);
  Serial.println("Moving forward");
}

void backward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, motorSpeed);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, motorSpeed);

  digitalWrite(led1, LOW);
  digitalWrite(led2, HIGH);
  digitalWrite(led3, LOW);
  digitalWrite(led4, LOW);
  Serial.println("Moving backward");
}

void left() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, motorSpeed);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, motorSpeed);
  
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, HIGH);
  digitalWrite(led4, LOW);
  Serial.println("Turning left");
}

void right() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, motorSpeed);
  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, motorSpeed);

  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
  digitalWrite(led4, HIGH);
  Serial.println("Turning right");
}

void loop() {
  int DURACION = 0;
  int distance = 0;
  int leftDistance = 0;
  int rightDistance = 0;

  // Calculate the distance using the front sensor
  digitalWrite(trigPin, HIGH); 
  delay(1); 
  digitalWrite(trigPin, LOW); 
  DURACION = pulseIn(echoPin, HIGH);
  distance = DURACION * 0.034 / 2;

  // Calculate the left distance using the left sensor
  digitalWrite(trigPinLeft, HIGH); 
  delay(1); 
  digitalWrite(trigPinLeft, LOW); 
  DURACION = pulseIn(echoPinLeft, HIGH);
  leftDistance = DURACION * 0.034 / 2;

  // Calculate the right distance using the right sensor
  digitalWrite(trigPinRight, HIGH); 
  delay(1); 
  digitalWrite(trigPinRight, LOW); 
  DURACION = pulseIn(echoPinRight, HIGH);
  rightDistance = DURACION * 0.034 / 2;

  if (distance > 20 && leftDistance > 15 && rightDistance > 15) {
    forward();  // Move forward if no obstacles are nearby
  } else if (distance <= 20) {
    if (leftDistance > rightDistance) {
      left();  // Turn left if there is more space
    } else {
      right();  // Turn right if there is more space
    }
  }

  int LEFT_SENSOR = digitalRead(A0);
  int RIGHT_SENSOR = digitalRead(A1);

  if (RIGHT_SENSOR == 0 && LEFT_SENSOR == 0) {
    forward(); // Move forward
  } else if (RIGHT_SENSOR == 0 && LEFT_SENSOR == 1) {
    right(); // Move right
  } else if (RIGHT_SENSOR == 1 && LEFT_SENSOR == 0) {
    left(); // Move left
  }
}
