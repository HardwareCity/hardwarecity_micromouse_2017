/*
Send COMMA- or TAB- SEPARATED ASCII-encoded strings to serial port.
(Commas/tabs are also helpful to read numbers when you are debugging)
Values 0-1023 or even higher can be sent, followed by a newline, or newline and carriage return
Output can be viewed in Arduino Serial Monitor, *or* Processing with code below

Created 30 Apr 2012, Modified August 2015
by Eric Forman
(based on Virtual Color Mixer and Graph examples by Tom Igoe)
*/

// rename these for your sensors:
int Apin = A0;    // analog
int Bpin = A1;    // analog 
int Cpin = 2;     // digital pin
int A=0, B=0, C=0;

void setup()
{
  Serial.begin(230400);
  //pinMode(Cpin, INPUT);
  //Test Sharp Sensor: Fire frequency for IR Led
  tone(2, 37000);
}

void loop()
{
  A = analogRead(Apin)/4-85;       // 0-1023 (in theory)
  B = analogRead(Bpin)/4-85;       // 0-1023 (in theory)
  C = B-A;

  Serial.print(A);
  Serial.print(",");
  Serial.print(B);
  Serial.print(",");
  Serial.println(C);

  delay(100);                  // slight delay to slow down data flow
}
