
#define PIN_START 2 // A1
#define PIN_STOP 3 // A0

#define OFF 0
#define ON 1

#define STATE_WAITTING_TO_START 0
#define STATE_ALL_DONE 10
#define STATE_ABORTED 11

unsigned int state = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  Serial.println("Setup...");
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_STOP, INPUT_PULLUP);
  // Interrupções
  attachInterrupt(digitalPinToInterrupt(PIN_STOP), int_stop_pressed, CHANGE); // RISING - to trigger when the pin goes from low to high; FALLING - for when the pin goes from high to low;

  state = STATE_WAITTING_TO_START;
}

// the loop function runs over and over again forever
void loop() {
  Serial.print("A espera de START... PIN_START=");
  Serial.print(digitalRead(PIN_START));
  Serial.print('\n');
  while(digitalRead(PIN_START) == ON){}
  Serial.print("PIN_START pressed\n");
  state = STATE_WAITTING_TO_START;

  // while(digitalRead(PIN_STOP) == OFF && state != STATE_ALL_DONE){
  while(state != STATE_ALL_DONE && state != STATE_ABORTED){
//    Serial.print(digitalRead(PIN_START));
//    Serial.print(' ');
//    Serial.print(digitalRead(PIN_STOP));
//    Serial.print('\n');
  }

  if (state == STATE_ALL_DONE){
    Serial.print("Terminei... A espera de START para recomeçar... PIN_START=");
    Serial.print(digitalRead(PIN_START));
    Serial.print("; PIN_STOP=");
    Serial.print(digitalRead(PIN_STOP));
    Serial.print(";");
    Serial.print('\n'); Serial.print('\n'); Serial.print('\n');
    state = STATE_WAITTING_TO_START;
  }

  if (state == STATE_ABORTED){
    Serial.print("ABORTED! PIN_START=");
    Serial.print(digitalRead(PIN_START));
    Serial.print("; PIN_STOP=");
    Serial.print(digitalRead(PIN_STOP));
    Serial.print(";");
    Serial.print('\n'); Serial.print('\n'); Serial.print('\n');
    state = STATE_WAITTING_TO_START;
  }
  
}

void int_stop_pressed(){
  Serial.print("INTERRUPT! STOP!\n");
  state = STATE_ABORTED;
}

