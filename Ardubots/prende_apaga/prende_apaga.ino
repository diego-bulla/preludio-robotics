void setup (){
 pinMode (5, OUTPUT);          // pin 2 como entrada
 pinMode(2, INPUT);          // pin 3 como salida
} 
void loop(){
 if (digitalRead(2) == HIGH) {          // evaluo si la entrada esta a nivel alto
  digitalWrite(5, HIGH);                // enciendo led
 }
 else {
  digitalWrite (5, LOW);                  // apago led
 }
}