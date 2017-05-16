/*
# Stepper ML    # Timer X
# Stepper MR    # Timer Y
# Stepper Farol # Timer Z
# Ultrasound USL    #
# Ultrasound USR    #
# Ultrasound USM    #
# Led RED
# Interrupt START
# Interrupt STOP
# Bluetooth Programação e Debug
# IR Floor
*/

#include <Servo.h>
//#include "AccelStepper/AccelStepper.h"
#include <AccelStepper.h>
//#include "Adafruit_TCS34725/Adafruit_TCS34725.h"
//TMP #include "Adafruit_TCS34725.h"
#include "DefinePins.h"
#include "STATE_MACHINES.h"
//FM
#include "TimerOne.h"   //https://playground.arduino.cc/Code/Timer1
#include "TimerThree.h"

//#define DEBUG // Uncoment for DEBUG mode     <<<-------------

#define BOUDRATE 230400 // 115200 ; 230400 ; 500000
#define DISTANCE_BETWEEN_WHEELS 150 // Distancia entre as 2 rodas em mm

#define LEFT 0
#define RIGHT 1

// IDs
#define SENSOR_LEFT 0
#define SENSOR_RIGHT 1
#define SENSOR_FRONTL 3
#define SENSOR_FRONTR 4

#define MOTOR_TOTAL_STEPS 200 // Microseconds
#define MOTOR_MICROSTEPS 8 // Microseconds
#define STEPS_MM  6.1 //Wheel diameter 84mm; 8uSteps

#define SQ1 1 // Square Line
#define SQ2 2 // Square Rotate

// Se RGB for superior ao THRESHOLD, então é branco, senão é preto
// TODO: Calibrar estes valores apra Preto em vez de ser para Branco
//#define THRESHOLD_RED 1000
//#define THRESHOLD_GREEN 1000
//#define THRESHOLD_BLUE 1000

unsigned int THRESHOLD_BLACK = 700;

// Constants:
const double pi = 3.141592;

// Global Variables
int myInts[5100][3]; // <-- Era apra salvar o mapa, mas não está em uso
unsigned long currentMillis=0, time_start, time_stop;
long aux1=0, aux2=0; // Auxiliar Variables
byte state_square = SQ1; // Estado do quadrado (para testes)
//TMP Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
uint16_t r, g, b, c, colorTemp, lux;
Servo motor_servo_farol;
AccelStepper stepper_left(AccelStepper::DRIVER, PIN_MOTOR_LEFT_STEP, PIN_MOTOR_LEFT_DIR);
AccelStepper stepper_right(AccelStepper::DRIVER, PIN_MOTOR_RIGHT_STEP, PIN_MOTOR_RIGHT_DIR);
byte mouse_state = STATE_MOUSE_WAITTING_TO_START;
byte journey_state = STATE_JOURNEY_GOING_TO_CHEESE;
int motor_servo_farol_pos=-1, motor_servo_farol_dir=LEFT;
float tmp_duration=0, duration_left=0, duration_right=0, duration_frontL=0, duration_frontR=0;
float tmp_distance=0;
unsigned int distance_left=0, distance_right=0, distance_frontL=0, distance_frontR=0;
int tmp_pin_trig=-1, tmp_pin_echo=-1;
unsigned long previousMillis = 0;
const long tick = 10; //t(ms)
// Position
//unsigned int d_right=0, d_left=0, d_center=0;
//float theta=0.0, theta_=0.0;
float _xPosition=0, _yPosition=0, _theta;
double leftDegrees=0, rightDegrees=0;
double dLeft=0, dRight=0, dCenter=0;
double phi=0;
double time_seconds=0;
int floor_value_left=-1;
int floor_value_right=-1;

// TEST ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void test(){
    Serial.println("Running test()...");

    turn_on_main_motors();
    stepper_left.setCurrentPosition(0);
    stepper_right.setCurrentPosition(0);
    stepper_left.moveTo(66*STEPS_MM);  // 264mm(perimeter wheel) Straight
    stepper_right.moveTo(66*STEPS_MM);  // 264mm(perimeter wheel) Straight
    turn_on_led_red();
    for (unsigned int i=100000; i>0; i--){
        stepper_left.run();
        stepper_right.run();
    }
    turn_off_led_red();
    stepper_left.moveTo(-66*STEPS_MM);  // 264mm(perimeter wheel) Straight
    stepper_right.moveTo(-66*STEPS_MM);  // 264mm(perimeter wheel) Straight
    turn_on_led_red();
    for (unsigned int i=100000; i>0; i--){
        stepper_left.run();
        stepper_right.run();
    }
    turn_off_led_red();
    turn_off_main_motors();

    turn_on_servo();
    Serial.println("0");
    set_servo_degree(0);
    Serial.println("0 OK");
    Serial.println("180");
    set_servo_degree(180);
    Serial.println("180 OK");
//    set_servo_degree(0);
    Serial.println("90");
    set_servo_degree(90);
    Serial.println("90 OK");
    turn_off_servo();

    Serial.println("Running test()... OK");
}

void square(){
    if(state_square == SQ1){
        stepper_left.moveTo(528*STEPS_MM);    //264mm(perimeter wheel) Straight
        stepper_right.moveTo(528*STEPS_MM);

        //VERIFY FINISH
        //distanceToGo
        aux1 = stepper_left.distanceToGo();
        aux2 = stepper_right.distanceToGo();
        if(aux2 != 0 && aux2 != 0){
            Serial.print(aux1); Serial.print(" : "); Serial.println(aux2);
        }else{
            Serial.println("SQ2");
            state_square = SQ2;
            stepper_left.setCurrentPosition(0); stepper_right.setCurrentPosition(0);
        }
    }else if(state_square == SQ2){

        stepper_left.moveTo(124*STEPS_MM);    //Rotate CW 90º
        stepper_right.moveTo(-124*STEPS_MM);

        //VERIFY FINISH
        //distanceToGo
        aux1 = stepper_left.distanceToGo();
        aux2 = stepper_right.distanceToGo();
        if(aux2 != 0 && aux2 != 0){
            Serial.print(aux1); Serial.print(" : "); Serial.println(aux2);
        }else{
            Serial.println("SQ1");
            delay(200);
            state_square = SQ1;
            stepper_left.setCurrentPosition(0); stepper_right.setCurrentPosition(0);
        }
    }
}
// TEST ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// MAIN MOTORS /////////////////////////////////////////////////////////////////////////////////////////////////////////
void turn_on_main_motors(){
    digitalWrite(PIN_MOTORS_EN, HIGH);
}

void turn_off_main_motors(){
    digitalWrite(PIN_MOTORS_EN, LOW);
}
// MAIN MOTORS /////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// SERVO MOTOR /////////////////////////////////////////////////////////////////////////////////////////////////////////
void turn_on_servo(){
    motor_servo_farol.attach(PIN_MOTOR_SERVO);
}

void turn_off_servo(){
    motor_servo_farol.detach();
}

void set_servo_degree(int degree){
    if (motor_servo_farol_pos > degree){
        for (int pos = motor_servo_farol_pos; pos >= degree; pos -= 1) {
            motor_servo_farol.write(pos);  // tell servo to go to position in variable 'pos'
            delay(15);  // waits 15ms for the servo to reach the position
        }
        motor_servo_farol_pos = degree;
    } else if (motor_servo_farol_pos < degree){
        for (int pos = motor_servo_farol_pos; pos <= degree; pos += 1) {
            motor_servo_farol.write(pos);  // tell servo to go to position in variable 'pos'
            delay(15);  // waits 15ms for the servo to reach the position
        }
        motor_servo_farol_pos = degree;
    }
}

void servo_rotate(){
    if(motor_servo_farol_dir==RIGHT){
        motor_servo_farol_pos++;
    } else{
        motor_servo_farol_pos--;
    }
    if(motor_servo_farol_pos>=180){
        motor_servo_farol_pos=180;
        motor_servo_farol_dir=LEFT;
    }
    if(motor_servo_farol_pos<=0){
        motor_servo_farol_pos=0;
        motor_servo_farol_dir=RIGHT;
    }
    motor_servo_farol.write(motor_servo_farol_pos);
}
// SERVO MOTOR /////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// DISTANCE SENSORS ////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int get_distance(int sensor_id){
//    Serial.print("A ler sensor: ");
//    Serial.print(sensor_id);
//    Serial.print("\n");

    tmp_pin_trig=-1;
    tmp_pin_echo=-1;
    tmp_distance=-1;

    switch (sensor_id){
        case SENSOR_LEFT:
            tmp_pin_trig = PIN_TRIG_LEFT;
            tmp_pin_echo = PIN_ECHO_LEFT;
            break;
        case SENSOR_RIGHT:
            tmp_pin_trig = PIN_TRIG_RIGHT;
            tmp_pin_echo = PIN_ECHO_RIGHT;
            break;
        case SENSOR_FRONTL:
            tmp_pin_trig = PIN_TRIG_FRONTL;
            tmp_pin_echo = PIN_ECHO_FRONTL;
            break;
        case SENSOR_FRONTR:
            tmp_pin_trig = PIN_TRIG_FRONTR;
            tmp_pin_echo = PIN_ECHO_FRONTR;
            break;
    }

    if (tmp_pin_trig>0 and tmp_pin_echo>0) {
        digitalWrite(tmp_pin_trig, LOW);  // Added this line //delayMicroseconds(2); // Added this line
        digitalWrite(tmp_pin_trig, HIGH); //delayMicroseconds(10); // Added this line
        digitalWrite(tmp_pin_trig, LOW);
        ///tmp_duration = pulseIn(tmp_pin_echo, HIGH, 1000); // 875, Timout(us) para 30 cm
        tmp_duration = pulseIn(tmp_pin_echo, HIGH, 2000);    //Timout(us) 2000us - 220mm
        ///tmp_distance = (tmp_duration / 2) / 29.154518; // TOF_1cm;
        tmp_distance = (tmp_duration / 2) / 29 * 10;           // TOF_1mm; 2,9us/mm
        //return duration_left;
    }

    // Serial.print("DISTANCE: "); Serial.print(tmp_distance); Serial.print('\n');
    return (unsigned int)tmp_distance;
}

void refresh_all_distance_sensors(){
    distance_left = get_distance(SENSOR_LEFT);
    distance_right = get_distance(SENSOR_RIGHT);
    distance_frontL = get_distance(SENSOR_FRONTL);
    distance_frontR = get_distance(SENSOR_FRONTR);

//    #ifdef DEBUG
//        Serial.print("L FL FR R :");
//        Serial.print(distance_left); Serial.print("\t");
//        //Serial.print(",\tF.L=");
//        Serial.print(distance_frontL); Serial.print("\t");
//        //Serial.print(",\tF.R=");
//        Serial.print(distance_frontR); Serial.print("\t");
//        //Serial.print(",\tRIGHT=");
//        Serial.print(distance_right); Serial.print("\t");
//
//        Serial.print("\n");
//    #endif
}
// DISTANCE SENSORS ////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// FLOOR SENSOR ////////////////////////////////////////////////////////////////////////////////////////////////////////
void floor_sensor_auto_calibration(){
    int reads[100];
    //Motor move forward
    for(int i=0; i>100; i+=2){
        reads[i] = analogRead(PIN_IR_FLOOR_LEFT);
        reads[i+1] = analogRead(PIN_IR_FLOOR_RIGHT);

    }
}

bool floor_sensor_on_cheese(){
//    #ifdef DEBUG
//        Serial.print(" FLOOR READ ");
//    #endif
    /*for(int i=0; i>10; i++){
      aux += analogRead(PIN_IR_FLOOR);  
    }
    aux = aux / 10;*/


    floor_value_left = analogRead(PIN_IR_FLOOR_LEFT);
    floor_value_right = analogRead(PIN_IR_FLOOR_RIGHT);
//    #ifdef DEBUG
//        Serial.println(floor_value);
//    #endif
    // TODO: Melhorar (exemplo com filtros e media, etc amostragens)
    if(floor_value_left >= THRESHOLD_BLACK or floor_value_right >= THRESHOLD_BLACK)
        return true;
    else
        return false;

}
// FLOOR SENSOR ////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// BEACON SENSOR ///////////////////////////////////////////////////////////////////////////////////////////////////////
void turn_on_beacon(){
    digitalWrite(PIN_BEACON_EN1, HIGH);
    digitalWrite(PIN_BEACON_EN2, HIGH);
}

void turn_off_beacon(){
    digitalWrite(PIN_BEACON_EN1, LOW);
    digitalWrite(PIN_BEACON_EN2, LOW);
}

//840us(HIGH) 1400us(Total) 560us(LOW)
bool read_beacon(){
    byte sum=0;
    turn_on_beacon(); // Liga-se e volta-se a desligar, porque se tiver sempre ligado, ele fica tolo.
    delay(20);  // Time to stabilization power suply // TODO: Colocar aqui um link para a foto do osciloscopio
    for(int i=0; i<14; i++){
        sum += digitalRead(PIN_BEACON);
        delayMicroseconds(100);
    }
    turn_off_beacon();
    //delay(20);
    if(sum > 12)
        return true; //Signal is allways high
    else
        return false;
}

// BEACON SENSOR ///////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// LED RED /////////////////////////////////////////////////////////////////////////////////////////////////////////////
void turn_on_led_red(){
    digitalWrite(PIN_LED_RED, HIGH);
}

void turn_off_led_red(){
    digitalWrite(PIN_LED_RED, LOW);
}
// LED RED /////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// LOCALIZATION ////////////////////////////////////////////////////////////////////////////////////////////////////////

void updatePosition()
{
    // get the angular distance traveled by each wheel since the last update
//    leftDegrees = stepper_left.currentPosition() * 360.0 / MOTOR_TOTAL_STEPS;
//    rightDegrees = stepper_right.currentPosition() * 360.0 / MOTOR_TOTAL_STEPS;

    // convert the angular distances to linear distances
//    dLeft = leftDegrees / _degreesPerMillimeter;
//    dRight = rightDegrees / _degreesPerMillimeter
    dLeft = stepper_left.currentPosition();
    dRight = stepper_right.currentPosition();

    // calculate the length of the arc traveled by Colin
    dCenter = (dLeft + dRight) / 2.0;

    // calculate Colin's change in angle
    phi = (dRight - dLeft) / (double)DISTANCE_BETWEEN_WHEELS;
    // add the change in angle to the previous angle
    _theta += phi;
    // constrain _theta to the range 0 to 2 pi
    if (_theta > 2.0 * pi) _theta -= 2.0 * pi;
    if (_theta < 0.0) _theta += 2.0 * pi;

    // update Colin's x and y coordinates
    _xPosition += dCenter * cos(_theta);
    _yPosition += dCenter * sin(_theta);
}

// LOCALIZATION ////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// INTERRUPTIONS ///////////////////////////////////////////////////////////////////////////////////////////////////////
void int_stop_pressed(){  // ISR stop button
    noInterrupts();
    if (mouse_state != STATE_MOUSE_ABORTED) {
        Serial.print("INTERRUPT! STOP!\n");
        mouse_state = STATE_MOUSE_ABORTED;
    }
    interrupts();
}

void ISR_Timer3(){
  ///sei();
  digitalWrite(PIN_DEBUG, HIGH); 
  stepper_left.run();
  stepper_right.run(); 
  Serial.print("T3");
  digitalWrite(PIN_DEBUG, LOW); 
  ///cli();
}

// INTERRUPTIONS ///////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// AUXILIAR FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////////////
void print_all_variables(){
    Serial.write(0x0C);
    Serial.print("STATUS:\t"); Serial.println(mouse_state);
    Serial.print("POSITION: X= "); Serial.print(digitalRead(_xPosition));
    Serial.print(";\tY= "); Serial.print(digitalRead(_yPosition));
    Serial.print(";\ttheta= "); Serial.println(digitalRead(_theta));
    Serial.print("L FL FR R : ");
    Serial.print(distance_left); Serial.print("\t");
    Serial.print(distance_frontL); Serial.print("\t");
    Serial.print(distance_frontR); Serial.print("\t");
    Serial.println(distance_right);
    Serial.print("FLOOR SENSOR:\tLEFT: "); Serial.print(floor_value_left); Serial.print("\tRIGHT: "); Serial.println(floor_value_right);
    Serial.print("SERVO DIR: "); Serial.println(motor_servo_farol_dir);
    Serial.print("BEACON: "); Serial.println(read_beacon());
    Serial.print("                                                    ");
    Serial.print("                                                    ");
    Serial.print("                                                    ");
    Serial.print("                                                    ");

//    Serial.print("distanceToGo:    "); Serial.println(stepper_left.distanceToGo());
//    Serial.print("move:            "); Serial.println(66*STEPS_MM);
//    Serial.print("currentposition: "); Serial.println(stepper_left.currentPosition());

}

void do_things_on_cheese(){
    noInterrupts();
    if (mouse_state!=STATE_MOUSE_ABORTED) {
        mouse_state = STATE_MOUSE_ON_CHEESE;
        journey_state = STATE_JOURNEY_RETURN_TO_HOME;
        turn_on_led_red();
        mouse_state = STATE_MOUSE_WALKING;
        stepper_left.setCurrentPosition(0);
        stepper_right.setCurrentPosition(0);
    }
    interrupts();
    stepper_left.moveTo(264*STEPS_MM);  // 264mm(perimeter wheel) Straight
    stepper_right.moveTo(-264*STEPS_MM);  // 264mm(perimeter wheel) Straight
    while (stepper_left.currentPosition() < (264*STEPS_MM) or stepper_left.currentPosition() > (-264*STEPS_MM)){
        stepper_left.run();
        stepper_right.run();
    }
}

void timeout(){
    stepper_left.stop();
    stepper_right.stop();
    turn_on_led_red();
    for (int i=0; i<10; i++){
        turn_on_led_red();
        delay(500);
        turn_off_led_red();
        delay(500);
    }
    turn_off_servo();
    turn_off_main_motors();
}

void dbg_motor_status(){


}

void set_pins_mode(){
    // PINS
    pinMode(PIN_STOP, INPUT_PULLUP);
    pinMode(PIN_START, INPUT_PULLUP);
    pinMode(PIN_MOTOR_SERVO, OUTPUT);
//    pinMode(PIN_LED_IR_BACK, INPUT);
    pinMode(PIN_MOTORS_EN, OUTPUT);
    pinMode(PIN_MOTOR_LEFT_DIR, OUTPUT);
    pinMode(PIN_MOTOR_LEFT_STEP, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_DIR, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_STEP, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_TRIG_LEFT, OUTPUT);     pinMode(PIN_ECHO_LEFT, INPUT);  //US SENSORS HC-SR04
    pinMode(PIN_TRIG_RIGHT, OUTPUT);    pinMode(PIN_ECHO_RIGHT, INPUT);  //US SENSORS HC-SR04
    pinMode(PIN_TRIG_FRONTL, OUTPUT);   pinMode(PIN_ECHO_FRONTL, INPUT);  //US SENSORS HC-SR04
    pinMode(PIN_TRIG_FRONTR, OUTPUT);   pinMode(PIN_ECHO_FRONTR, INPUT);  //US SENSORS HC-SR04
//    pinMode(PIN_COLOR_FLOOR, INPUT);  // IR floor
    pinMode(PIN_IR_FLOOR_LEFT, INPUT);  // IR floor
    pinMode(PIN_IR_FLOOR_RIGHT, INPUT);  // IR floor

    pinMode(PIN_BEACON, INPUT);
    pinMode(PIN_BEACON_EN1, OUTPUT);   //Sensor ON/Off
    pinMode(PIN_BEACON_EN2, OUTPUT);   //Sensor ON/Off
}
// AUXILIAR FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
    Serial.begin(BOUDRATE);
    set_pins_mode();
    turn_off_beacon();
    turn_off_main_motors();
    turn_off_led_red();
    /*FM
    turn_on_servo();
    set_servo_degree(90);
    turn_off_servo();
*/
    attachInterrupt(digitalPinToInterrupt(PIN_STOP), int_stop_pressed, FALLING); // RISING - to trigger when the pin goes from low to high; FALLING - for when the pin goes from high to low;

    stepper_left.setMaxSpeed(1000.0 * MOTOR_MICROSTEPS);      //v(uSteps/s)
    stepper_left.setAcceleration(1000.0 * MOTOR_MICROSTEPS);
    stepper_right.setMaxSpeed(1000.0 * MOTOR_MICROSTEPS);
    stepper_right.setAcceleration(1000.0 * MOTOR_MICROSTEPS);
    stepper_right.setPinsInverted  ( true, false, false );

    #ifdef DEBUG
        Serial.println("Setup... OK");
    #endif

    #ifdef DEBUG
  //TMP      test(); // Testar Motores e Led
//        test();
    #endif

    //FM Timer
  //  Timer3.initialize(800);
    //Timer3.attachInterrupt(ISR_Timer3);

    //FM
    pinMode(PIN_DEBUG, OUTPUT);
}



// the loop function runs over and over again forever
void loop() {
    //WAIT TO START
    mouse_state = STATE_MOUSE_WAITTING_TO_START;
    Serial.println(STATE_MOUSE_WAITTING_TO_START);
        Serial.print("Waitting for START... PIN_START="); Serial.println(digitalRead(PIN_START));
    while(digitalRead(PIN_START) == HIGH){ /* square(); */}
        Serial.println("PIN_START pressed\n");

    //INICIALIZE
    stepper_left.setCurrentPosition(0);
    stepper_right.setCurrentPosition(0);
    turn_on_main_motors();

    //TEST: MOVE FWD 
    stepper_left.move(264*STEPS_MM);  // 264mm(perimeter wheel) Straight
    stepper_right.move(264*STEPS_MM);  // 264mm(perimeter wheel) Straight
    //Start count
    time_start = millis();

    //FM TEST SERVO
    #define SERVO_CW 1
    #define SERVO_CCW 2
    int servo_pos=0;
    byte Servo_State = SERVO_CW;
    turn_on_servo();
    set_servo_degree(0);
    while( mouse_state != STATE_MOUSE_ABORTED){    
      currentMillis = millis();
      if (currentMillis - previousMillis >= 100) {
          previousMillis = currentMillis;
          
          if(Servo_State == SERVO_CCW){
            if(servo_pos >10){
              servo_pos -= 4;
            }else{
              Servo_State == SERVO_CW;
            }
          }//END IF
          if(Servo_State == SERVO_CW){
            if(servo_pos < 170){
              servo_pos += 4;
            }else{
              Servo_State == SERVO_CCW;
            }
          }//END IF
          Serial.print("servo pos "); Serial.println(servo_pos);
          set_servo_degree(servo_pos);
      }//END IF
    
    }//END WHILE
    turn_off_servo();
    
    //FM TEST MOTOR FWD
    /*
    while( mouse_state != STATE_MOUSE_ABORTED){    
        stepper_left.run();
        stepper_right.run();
        ///delay(1);
        #ifdef DEBUG
            print_all_variables();
        #endif

        ///Serial.println(stepper_left.currentPosition());
        if(stepper_left.currentPosition() == 1610){  //(264*STEPS_MM)=1610
          mouse_state = STATE_MOUSE_ABORTED;
          ///Serial.println("STOP");
          ///Serial.println(STATE_MOUSE_ABORTED);
        }
    }//END WHILE
    */
    
    //Stop count
    time_stop = millis();
    Serial.print("Time="); Serial.println(time_stop-time_start);
    Serial.println("");
    //delay(3000);
    
} // END loop()

