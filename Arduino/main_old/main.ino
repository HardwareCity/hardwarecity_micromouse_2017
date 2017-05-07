
// PINS
#define PIN_START 2 // INTERRUPTOR START
#define PIN_STOP 3 // INTERRUPTOR STOP
#define PIN_TRIG_LEFT 4  // Sensor de distancia LEFT
#define PIN_ECHO_LEFT 5  // Sensor de distancia LEFT
#define PIN_TRIG_RIGHT 6  // Sensor de distancia RIGHT
#define PIN_ECHO_RIGHT 7  // Sensor de distancia RIGHT
#define PIN_MOTOR_LEFT_DIR 8
#define PIN_MOTOR_LEFT_STEP 9
#define PIN_MOTOR_RIGHT_DIR 10
#define PIN_MOTOR_RIGHT_STEP 11

// States
#define STATE_WAITTING_TO_START 1
#define STATE_GOING_TO_CHEESE 2
#define STATE_ON_CHEESE 3
#define STATE_BACK_TO_HOME 4
#define STATE_ALL_DONE 5
#define STATE_ABORTED 6

#define MIN_DELAY_MOTORS 1000 // Microseconds

////DRIVER: Sharp IR Sensor GP1U5
//// #define PIN_IR_LED1   2
//#define PIN_IR_IN_1   A0
//#define PIN_IR_IN_2   A1
//#define IR_IN_MIN   85
//#define IR_IN_MAX   255

byte state = STATE_WAITTING_TO_START;
float duration_left=0, duration_right=0; //durações dos sensores de distância 1 e 2, repect.
float distance_left=0, distance_right=0; //distância estim. aos sensores de distância 1 e 2, repect.



//// sensors_read()
//int sensorIR_1, sensorIR_2;   //[0, 255]
//void sensors_read(){
//  unsigned int aux;
//  aux = analogRead(PIN_IR_IN_1)/4-IR_IN_MIN;       // 0-1023 (in theory)
//  if (aux < 0) aux = 0;
//  sensorIR_1 = map(aux, 0, IR_IN_MAX, 0, 100);      //Normalized [0, 100%]
//  
//  aux = analogRead(PIN_IR_IN_2)/4-IR_IN_MIN;       // 0-1023 (in theory)
//  if (aux < 0) aux = 0;
//  sensorIR_2 = map(aux, 0, IR_IN_MAX, 0, 100);      //Normalized [0, 100%]
//}

void decision(){
  
}


void actualizar_sensores_distancia(){
    digitalWrite(PIN_TRIG_LEFT, LOW);  // Added this line
    //delayMicroseconds(2); // Added this line
    digitalWrite(PIN_TRIG_LEFT, HIGH);
    //delayMicroseconds(10); // Added this line
    digitalWrite(PIN_TRIG_LEFT, LOW);
    duration_left = pulseIn(PIN_ECHO_LEFT, HIGH); // 875, Timout para 30 cm
    distance_left = (duration_left/2) / 29.154518; // TOF_1cm;
    //return duration_left;
  
//    digitalWrite(PIN_TRIG_RIGHT, LOW);  // Added this line
//    //delayMicroseconds(2); // Added this line
//    digitalWrite(PIN_TRIG_RIGHT, HIGH);
//    //delayMicroseconds(10); // Added this line
//    digitalWrite(PIN_TRIG_RIGHT, LOW);
//    duration_right = pulseIn(PIN_ECHO_RIGHT, HIGH); // 875, Timout para 30 cm
//    distance_right = (duration_right/2) / 29.154518; // TOF_1cm;
    //return distance_right;

    Serial.print("LEFT: ");
    Serial.print(distance_left);
    Serial.print("\t | RIGHT: ");
    Serial.print(distance_right);
    Serial.print('\n');
}



// ISR stop button
void int_stop_pressed(){
  Serial.print("INTERRUPT! STOP!\n");
  state = STATE_ABORTED;
}



void posServo(int pos){
  
}

void rodar_motores(int steps_left, int steps_right){
  while(steps_left!=0 || steps_right!=0){
    // Motor Esquerdo
      if (steps_left>0){
        steps_left-=1;
        posiciona_o_motor(MOTOR_LEFT, HIGH);
      }else{
        if (steps_left<0){
          steps_left+=1;
        posiciona_o_motor(MOTOR_LEFT, LOW);
        }
      }
      // Motor direito
      if (steps_right>0){
        steps_right-=1;
        posiciona_o_motor(MOTOR_RIGHT, HIGH);
      }else{
        if (steps_right<0){
          steps_right+=1;
        posiciona_o_motor(MOTOR_RIGHT, LOW);
        }
      }
      delayMicroseconds(MIN_DELAY_MOTORS);
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  Serial.println("Setup...");
  pinMode(PIN_TRIG_LEFT, OUTPUT);
  pinMode(PIN_ECHO_LEFT, INPUT);
  pinMode(PIN_TRIG_RIGHT, OUTPUT);
  pinMode(PIN_ECHO_RIGHT, INPUT);
  
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_STOP, INPUT_PULLUP);
  // Interrupções
  attachInterrupt(digitalPinToInterrupt(PIN_STOP), int_stop_pressed, FALLING); // RISING - to trigger when the pin goes from low to high; FALLING - for when the pin goes from high to low;

  state = STATE_WAITTING_TO_START;

  //Sharp IR Sensor GP1U5: Fire frequency for IR Led (37KHz)
  ///tone(PIN_IR_LED1, 37000);
  
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
      
      Serial.println("Update");
      //1º Sensor Update
//      sensors_read();
      //2º Decision 
      decision();
      
      
      //digitalWrite( MOTOR_B_DIR, HIGH ); // direction = forward
      //analogWrite( MOTOR_B_PWM, 100 ); // PWM speed = slow
      
      actualizar_sensores_distancia();
//      m_left_speed  = 15;
//      m_right_speed = 45;
//      if (distance_left < 5 && distance_left != 0){
//        m_left_speed = m_left_speed - 10;
//        m_right_speed = m_right_speed - 15;
//      }else if (distance_left < 10 && distance_left != 0){
//        m_left_speed = m_left_speed - 5;
//        m_right_speed = m_right_speed - 10;
//      }else if (distance_left < 15 && distance_left != 0){
//        m_left_speed = m_left_speed - 2;
//        m_right_speed = m_right_speed - 5;
//      }
      

      //3º - 
      // motor_update();

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
    state = STATE_WAITTING_TO_START;
  }
  
}


