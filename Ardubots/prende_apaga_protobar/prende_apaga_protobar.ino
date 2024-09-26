int PULSADOR = 2;
int LED =3;
int ESTADO = LOW;

void setup (){
 pinMode (PULSADOR, INPUT);          // pin 2 como entrada
 pinMode(LED, OUTPUT);          // pin 3 como salida
} 
void loop(){
 while (digitalRead (PULSADOR) == LOW){         // ESPERO A QUE EL pulsador se presione

 }
 
 ESTADO = digitalRead(LED);                  // leo estado del LED
 digitalWrite(LED, !ESTADO);                   // escribo valor contrario
 while (digitalRead(PULSADOR) == HIGH){       // antirebote

 }
}
