int TRIG = 11;			// trigger en pin 10
int ECO = 12;			// echo en pin 9
int LED = 3;			// LED en pin 3
int DURACION;
int DISTANCIA;

void setup()
{
  pinMode(TRIG, OUTPUT); 	// trigger como salida
  pinMode(ECO, INPUT);		// echo como entrada
  pinMode(LED, OUTPUT);		// LED como salida
  Serial.begin(9600);  		// inicializacion de comunicacion serial a 9600 bps

}

void loop()
{
  
  digitalWrite(TRIG, HIGH); 		// generacion del pulso a enviar
  delay(1);				// al pin conectado al trigger
  digitalWrite(TRIG, LOW);		// del sensor
  
  DURACION = pulseIn(ECO, HIGH);	// con funcion pulseIn se espera un pulso
  					// alto en Echo
  DISTANCIA = DURACION / 58.2;		// distancia medida en centimetros
  Serial.println(DISTANCIA);		// envio de valor de distancia por monitor serial
  delay(200);				// demora entre datos

  if (DISTANCIA <= 20 && DISTANCIA >= 0){	// si distancia entre 0 y 20 cms.
    digitalWrite(LED, HIGH);			// enciende LED
    delay(DISTANCIA * 10);			// demora proporcional a la distancia
    digitalWrite(LED, LOW);			// apaga LED
    }
   
}