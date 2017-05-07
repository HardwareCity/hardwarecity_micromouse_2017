#define VELOCIDAD 17000


int steps1 = 9;
int direccion1 = 10;
int steps2 = 11;
int direccion2 = 12;
int reset = 8;
int pasos = 2000;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(steps1, OUTPUT); 
  pinMode(direccion1, OUTPUT); 
  pinMode(steps2, OUTPUT); 
  pinMode(direccion2, OUTPUT); 
  pinMode(reset, OUTPUT);

  digitalWrite(reset, LOW);    //Mientras reset este en LOW el motor permanecerá apagado y no sufrirá. El chip apagará todos los puertos y no leerá comandos.
  delay(100);
  digitalWrite(reset, HIGH);   //Cuando reset se encuentre en HIGH el motor arrancará y leerá los comandos enviados.
}

// the loop routine runs over and over again forever:
void loop() {
//  digitalWrite(reset, LOW);    //Mientras reset este en LOW el motor permanecerá apagado y no sufrirá. El chip apagará todos los puertos y no leerá comandos.
  delay(100);
//  digitalWrite(reset, HIGH);   //Cuando reset se encuentre en HIGH el motor arrancará y leerá los comandos enviados.
  digitalWrite(direccion1, HIGH);
  digitalWrite(direccion2, LOW);
    

  for (int i = 0; i<pasos; i++)       //Equivale al numero de vueltas (200 es 360º grados) o micropasos
  {
    digitalWrite(steps1, HIGH);  // This LOW to HIGH change is what creates the
    digitalWrite(steps2, HIGH);  // This LOW to HIGH change is what creates the
    delayMicroseconds(VELOCIDAD);
    digitalWrite(steps1, LOW); // al A4988 de avanzar una vez por cada pulso de energia.  
    digitalWrite(steps2, LOW); // al A4988 de avanzar una vez por cada pulso de energia.  
    delayMicroseconds(VELOCIDAD);     // Regula la velocidad, cuanto mas bajo mas velocidad.

  } 
  
//  digitalWrite(reset, LOW);   //Mientras reset este en LOW el motor permanecerá apagado y no sufrirá. El chip apagará todos los puertos y no leerá comandos.
  delay(100);
//  digitalWrite(reset, HIGH);   //Cuando reset se encuentre en HIGH el motor arrancará y leerá los comandos enviados.
  digitalWrite(direccion1, LOW);
  digitalWrite(direccion2, HIGH);

  for (int i = 0; i<pasos; i++)       //Equivale al numero de vueltas (200 es 360º grados) o micropasos
  {
      
    digitalWrite(steps1, HIGH);   // LOW to HIGH hace que el motor avance ya que da la orden
    digitalWrite(steps2, HIGH);   // LOW to HIGH hace que el motor avance ya que da la orden
    delayMicroseconds(VELOCIDAD);
    digitalWrite(steps1, LOW);    // al A4988 de avanzar una vez por cada pulso de energia.
    digitalWrite(steps2, LOW);    // al A4988 de avanzar una vez por cada pulso de energia.
    delayMicroseconds(VELOCIDAD);         // Regula la velocidad, cuanto mas bajo mas velocidad.
  }
}





