#define PIN_BEACON 2

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_BEACON, OUTPUT);
}


//IR Beacon signal
// "on-off keying", a 600 Hz sobre uma portadora de 38 kHz. O "duty-cycle" do sinal modulante (600 Hz) é de 30%
// f=38KHz; tON=500us; tOFF= 1200us 
void loop() {
  // put your main code here, to run repeatedly:
  tone(PIN_BEACON, 38);
    delayMicroseconds(500);
  noTone(PIN_BEACON);
    delayMicroseconds(1200);
    /*digitalWrite(PIN_BEACON, HIGH);
    delayMicroseconds(500);
  digitalWrite(PIN_BEACON, LOW);
    delayMicroseconds(500);*/
}
