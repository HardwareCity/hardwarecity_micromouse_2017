
// States
byte state = 0;
#define STATE_WAITTING_TO_START 0
#define STATE_ALL_DONE 10
#define STATE_ABORTED 11

// Interruptores
#define PIN_START 2 // A1
#define PIN_STOP 3 // A0

// DRIVER: Motor
// wired connections
#define HG7881_A_IA 13 // D10 --> Motor B Input A --> MOTOR B +
#define HG7881_A_IB 12 // D11 --> Motor B Input B --> MOTOR B - 
#define HG7881_B_IA 11 // D10 --> Motor B Input A --> MOTOR B +
#define HG7881_B_IB 10 // D11 --> Motor B Input B --> MOTOR B - 
// functional connections
#define MOTOR_A_PWM HG7881_A_IA // Motor B PWM Speed
#define MOTOR_A_DIR HG7881_A_IB // Motor B Direction
#define MOTOR_B_PWM HG7881_B_IA // Motor B PWM Speed
#define MOTOR_B_DIR HG7881_B_IB // Motor B Direction
// Adjust for each motor
#define MOTOR_MIN 100 //[0, 255]
#define MOTOR_MAX 255 //[0, 255]
byte m_left_s, m_right_s;             //[0, 255] used by the driver
int m_left_speed = 0, m_right_speed = 0;  //[-100%, +100%] used by the decision maker

//DRIVER: Sharp IR Sensor GP1U5
// #define PIN_IR_LED1   2
#define PIN_IR_IN_1   A0
#define PIN_IR_IN_2   A1
#define IR_IN_MIN   85
#define IR_IN_MAX   255

// Program Variables 
const unsigned long control_interval = 100; //ms
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long currentMillis = 0;

// void motor_update()
// m_XXX_speed[-100, 100] (%)
#define ML_MAX 100  //100%  
#define MR_MAX 100
#define ML_MIN 0    //0%  
#define MR_MIN 0
#define M_HARD_STOP 101  //Hard Stop =101%; normal speed = [-100%, +100%]; 
void motor_update(){
  Serial.println("motor_update()");
  Serial.print("m_left_speed=");
  Serial.println(m_left_speed);
  Serial.print("m_right_speed=");
  Serial.println(m_right_speed);
  //Left Motor
  //convert speed values [-100, 100](%) to [0, 255]. MOTOR_MIN is > 0
  switch (m_left_speed)
  {
  case (0):               //STOP
    m_left_s = 0;
    break;
    
  case (M_HARD_STOP):     //HARD STOP
    digitalWrite( MOTOR_A_DIR, HIGH );
    digitalWrite( MOTOR_B_DIR, HIGH );
    digitalWrite( MOTOR_A_PWM, HIGH );
    digitalWrite( MOTOR_B_PWM, HIGH );
    break;

  default:                //Normal speed
    m_left_s  = map(abs(m_left_speed),  0, 100, MOTOR_MIN, MOTOR_MAX);
    //Left motor
    if(m_left_speed >0){       //Forward
      digitalWrite( MOTOR_A_DIR, HIGH );
      analogWrite( MOTOR_A_PWM, MOTOR_MAX-m_left_s );
    }else{                      //Backward
      digitalWrite( MOTOR_A_DIR, LOW );
      analogWrite( MOTOR_A_PWM, MOTOR_MAX-m_left_s );
    }
  }//SWITCH END
  
  //Right Motor
  //convert speed values [-100, 100](%) to [0, 255]. MOTOR_MIN is > 0
  switch (m_right_speed)
  {
  case (0):               //STOP
    m_right_s = 0;
    break;
    
  case (M_HARD_STOP):     //HARD STOP
    digitalWrite( MOTOR_A_DIR, HIGH );
    digitalWrite( MOTOR_B_DIR, HIGH );
    digitalWrite( MOTOR_A_PWM, HIGH );
    digitalWrite( MOTOR_B_PWM, HIGH );
    break;

  default:                //Normal speed
    m_right_s = map(abs(m_right_speed), 0, 100, MOTOR_MIN, MOTOR_MAX);
    //Left motor
    if(m_right_speed >0){       //Forward
      digitalWrite( MOTOR_B_DIR, HIGH );
      analogWrite(  MOTOR_B_PWM, MOTOR_MAX-m_right_s );
    }else{                      //Backward
      digitalWrite( MOTOR_B_DIR, LOW );
      analogWrite(  MOTOR_B_PWM, MOTOR_MAX-m_right_s );
    }
  }//SWITCH END
} //motor_update() END

// sensors_read()
int sensorIR_1, sensorIR_2;   //[0, 255]
void sensors_read(){
  int aux;
  aux = analogRead(PIN_IR_IN_1)/4-IR_IN_MIN;       // 0-1023 (in theory)
  if (aux < 0) aux = 0;
  sensorIR_1 = map(aux, 0, IR_IN_MAX, 0, 100);      //Normalized [0, 100%]
  
  aux = analogRead(PIN_IR_IN_2)/4-IR_IN_MIN;       // 0-1023 (in theory)
  if (aux < 0) aux = 0;
  sensorIR_2 = map(aux, 0, IR_IN_MAX, 0, 100);      //Normalized [0, 100%]
}

void decision(){
  
}

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  Serial.println("Setup...");
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_STOP, INPUT_PULLUP);
  // Interrupções
  attachInterrupt(digitalPinToInterrupt(PIN_STOP), int_stop_pressed, FALLING); // RISING - to trigger when the pin goes from low to high; FALLING - for when the pin goes from high to low;

  state = STATE_WAITTING_TO_START;

  //Sharp IR Sensor GP1U5: Fire frequency for IR Led (37KHz)
  ///tone(PIN_IR_LED1, 37000);
  
  //MOTOR Setup A (Left)
  pinMode( MOTOR_A_DIR, OUTPUT );
  pinMode( MOTOR_A_PWM, OUTPUT );
  digitalWrite( MOTOR_A_DIR, LOW );
  digitalWrite( MOTOR_A_PWM, LOW );
  //MOTOR Setup B (Right)
  pinMode( MOTOR_B_DIR, OUTPUT );
  pinMode( MOTOR_B_PWM, OUTPUT );
  digitalWrite( MOTOR_B_DIR, LOW );
  digitalWrite( MOTOR_B_PWM, LOW );
}

// ISR stop button
void int_stop_pressed(){
  Serial.print("INTERRUPT! STOP!\n");
  state = STATE_ABORTED;
}

void kill(){
  //Motor Hard STOP
  digitalWrite( MOTOR_A_DIR, HIGH );
  digitalWrite( MOTOR_A_PWM, HIGH );
  digitalWrite( MOTOR_B_DIR, HIGH );
  digitalWrite( MOTOR_B_PWM, HIGH );
  //Sensor stop
}

// the loop function runs over and over again forever
void loop() {
  Serial.print("A espera de START... PIN_START=");
  Serial.println(digitalRead(PIN_START));
  state = STATE_WAITTING_TO_START;
  
  while(digitalRead(PIN_START) == HIGH){}
  Serial.print("PIN_START pressed\n");
  

  // Main loop
  while(state != STATE_ALL_DONE && state != STATE_ABORTED){
    // control loop ms
    currentMillis = millis();
    if(currentMillis - previousMillis > control_interval) {
      previousMillis = currentMillis;
      
      Serial.println("Update");
      //1º Sensor Update
      sensors_read();
      //2º Decision 
      decision();
      //3º - 
      motor_update();
      //digitalWrite( MOTOR_B_DIR, HIGH ); // direction = forward
      //analogWrite( MOTOR_B_PWM, 100 ); // PWM speed = slow
      m_left_speed  = 50;
      m_right_speed = 50;
    }
  }//END WHILE

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
    kill();
    state = STATE_WAITTING_TO_START;
  }
  
}


