#define PIN_IN_BEACON A1
#define PIN_ON_BEACON 3
#define PIN_ON_BEACON2 4
byte beacon_detected = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_IN_BEACON, INPUT);
  pinMode(PIN_ON_BEACON, OUTPUT);   //Sensor ON/Off
  pinMode(PIN_ON_BEACON2, OUTPUT);   //Sensor ON/Off
  
  digitalWrite(PIN_ON_BEACON, LOW);
  digitalWrite(PIN_ON_BEACON2, LOW);

//  attachInterrupt(digitalPinToInterrupt(PIN_IN_BEACON), int_in_beacon, FALLING); // RISING - to trigger when the pin goes from low to high; FALLING - for when the pin goes from high to low;
    
  Serial.begin(230400);
  Serial.println("Start");
}

//840us(HIGH) 1400us(Total) 560us(LOW)
byte read_beacon(){
byte sum=0;
    digitalWrite(PIN_ON_BEACON, HIGH);    //Turn beacon on
    digitalWrite(PIN_ON_BEACON2, HIGH);    //Turn beacon on
    delay(20);
    for(int i=0; i<14; i++){
       sum += digitalRead(PIN_IN_BEACON);
       delayMicroseconds(100);
    }  
    digitalWrite(PIN_ON_BEACON, LOW);    //Turn beacon off
    digitalWrite(PIN_ON_BEACON2, LOW);    //Turn beacon on
    //delay(20);
    if(sum > 12)
      return true; //Signal is allways high
    else 
      return false;
}


//IR Beacon signal
// "on-off keying", a 600 Hz sobre uma portadora de 38 kHz. O "duty-cycle" do sinal modulante (600 Hz) é de 30%
// f=38KHz; tON=500us; tOFF= 1200us 
void loop() {
  // put your main code here, to run repeatedly:
  Serial.write(0x0C);
  Serial.print("Beacon:_");
  if(read_beacon())
    Serial.print("off");
  else
      Serial.print("on"); 
  delay(20);
}
