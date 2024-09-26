// Define motor control pins
#define in1 9
#define in2 8
#define enA 10

#define in3 7
#define in4 6
#define enB 5

// Motor speeds (0-255)
int motorSpeed = 200; // Set motor speed (adjust as needed)

void setup() {
  // Set all motor control pins as output
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

  // Start serial communication for debugging (optional)
  Serial.begin(9600);
}

void loop() {
  // Example movements (you can replace with your control logic)
  
  forward();  // Move forward
  delay(2000); // Move for 2 seconds

  backward(); // Move backward
  delay(2000); // Move for 2 seconds

  left(); // Turn left
  delay(1000); // Turn for 1 second

  right(); // Turn right
  delay(1000); // Turn for 1 second

  stopMotors(); // Stop the robot
  delay(2000); // Stop for 2 seconds
}

// Function to move forward
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