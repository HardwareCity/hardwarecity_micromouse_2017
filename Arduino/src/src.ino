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

//#define DEBUG_HC // Uncoment for DEBUG_HC mode     <<<-------------

const unsigned long TICK = 50; //t(ms)

#define SERVO_STEP_ANGLE 8
#define PERIMETER_ROBOT_PERIMETER 481 // Perimeter between wheels

#define TIMEOUT 180000 // 3 Minutos até timeout

#define BOUDRATE 2000000 // 115200 ; 230400 ; 500000
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
long aux1=0, aux2=0; // Auxiliar Variables
unsigned long colid = 0;
unsigned long count_beacon=0;
unsigned long time_start=0;
unsigned long currentMillis=0;
unsigned long tmp_currentMillis1=0, tmp_prev_currentMillis1=0;
unsigned long tmp_currentMillis2=0, tmp_prev_currentMillis2=0;
byte state_square = SQ1; // Estado do quadrado (para testes)
Servo motor_servo_farol;
int motor_servo_farol_pos=0;
byte motor_servo_farol_dir=LEFT;
bool servo_motor_is_on = false;
AccelStepper stepper_left(AccelStepper::DRIVER, PIN_MOTOR_LEFT_STEP, PIN_MOTOR_LEFT_DIR);
AccelStepper stepper_right(AccelStepper::DRIVER, PIN_MOTOR_RIGHT_STEP, PIN_MOTOR_RIGHT_DIR);
byte mouse_state = STATE_MOUSE_WAITTING_TO_START;
byte journey_state = STATE_JOURNEY_GOING_TO_CHEESE;
float tmp_duration=0, duration_left=0, duration_right=0, duration_frontL=0, duration_frontR=0;
float tmp_distance=0;
unsigned int distance_left=0, distance_right=0, distance_frontL=0, distance_frontR=0;
int tmp_pin_trig=-1, tmp_pin_echo=-1;
unsigned long previousMillis_t1=0, previousMillis_t2=0;
// Position
//unsigned int d_right=0, d_left=0, d_center=0;
//float theta=0.0, theta_=0.0;
float _xPosition=0, _yPosition=0, _theta;
double leftDegrees=0, rightDegrees=0;
double dLeft=0, dRight=0, dCenter=0;
double phi=0;
double time_seconds = 0;
int floor_value_left =- 1;
int floor_value_right =- 1;

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
    servo_motor_is_on = true;
}

void turn_off_servo(){
    motor_servo_farol.detach();
    servo_motor_is_on = false;
}

void set_servo_degree(int degree){
//void set_servo_degree(int degree, bool move_fast){
    motor_servo_farol.write(degree);
//    if (move_fast){
//        motor_servo_farol.write(degree);
//    }
//    else{
//        if (motor_servo_farol_pos > degree){
//            for (int pos = motor_servo_farol_pos; pos >= degree; pos -= 1) {
//                motor_servo_farol.write(pos);  // tell servo to go to position in variable "pos"
//                delay(15);  // waits 15ms for the servo to reach the position
//            }
//            motor_servo_farol_pos = degree;
//        } else if (motor_servo_farol_pos < degree){
//            for (int pos = motor_servo_farol_pos; pos <= degree; pos += 1) {
//                motor_servo_farol.write(pos);  // tell servo to go to position in variable "pos"
//                delay(15);  // waits 15ms for the servo to reach the position
//            }
//            motor_servo_farol_pos = degree;
//        }
//    };
}

void servo_rotate(){
    if (servo_motor_is_on) {
        if (motor_servo_farol_dir == RIGHT) {
            motor_servo_farol_pos+=SERVO_STEP_ANGLE;
        } else {
            motor_servo_farol_pos-=SERVO_STEP_ANGLE;
        }
        if (motor_servo_farol_pos >= 170) {
            motor_servo_farol_pos = 170;
            motor_servo_farol_dir = LEFT;
        }
        if (motor_servo_farol_pos <= 10) {
            motor_servo_farol_pos = 10;
            motor_servo_farol_dir = RIGHT;
        }
        set_servo_degree(motor_servo_farol_pos);
    }
}
// SERVO MOTOR /////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// DISTANCE SENSORS ////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int get_distance(int sensor_id){
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

    // Serial.print("DISTANCE: "); Serial.print(tmp_distance); Serial.print("\n");
    return (unsigned int)tmp_distance;
}

void refresh_all_distance_sensors(){
    distance_left = get_distance(SENSOR_LEFT);
    distance_right = get_distance(SENSOR_RIGHT);
    distance_frontL = get_distance(SENSOR_FRONTL);
    distance_frontR = get_distance(SENSOR_FRONTR);
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
//    #ifdef DEBUG_HC_HC
//        Serial.print(" FLOOR READ ");
//    #endif
    /*for(int i=0; i>10; i++){
      aux += analogRead(PIN_IR_FLOOR);
    }
    aux = aux / 10;*/


    floor_value_left = analogRead(PIN_IR_FLOOR_LEFT);
    floor_value_right = analogRead(PIN_IR_FLOOR_RIGHT);
//    #ifdef DEBUG_HC_HC
//        Serial.println(floor_value);
//    #endif
    // TODO: Melhorar (exemplo com filtros e media, etc amostragens)
    if(floor_value_left >= THRESHOLD_BLACK and floor_value_right >= THRESHOLD_BLACK)
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
    unsigned int sum=0;
//    turn_on_beacon(); // Liga-se e volta-se a desligar, porque se tiver sempre ligado, ele fica tolo.
    // delay(20);  // Time to stabilization power suply // TODO: Colocar aqui um link para a foto do osciloscopio
    digitalWrite(PIN_DEBUG, HIGH);

    for(int i=0; i<14*40; i++){
        sum += digitalRead(PIN_BEACON);
        delayMicroseconds(10); // 100
    }
    digitalWrite(PIN_DEBUG, LOW);
    turn_off_beacon();
    // delay(20);
//    Serial.write(0x0C);
//    Serial.println(sum);
//    if(sum >= 10*40) {
    if(sum >= 500) {
        count_beacon = 0;
//        Serial.write(0x0C);
//        Serial.println(count_beacon);
        return false; //Signal is allways high
    }
    else{
        count_beacon++;
//        Serial.write(0x0C);
//        Serial.println(count_beacon);
        return true;
    }
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

    // calculate Colin"s change in angle
    phi = (dRight - dLeft) / (double)DISTANCE_BETWEEN_WHEELS;
    // add the change in angle to the previous angle
    _theta += phi;
    // constrain _theta to the range 0 to 2 pi
    if (_theta > 2.0 * pi) _theta -= 2.0 * pi;
    if (_theta < 0.0) _theta += 2.0 * pi;

    // update Colin"s x and y coordinates
    _xPosition += dCenter * cos(_theta);
    _yPosition += dCenter * sin(_theta);
}

// LOCALIZATION ////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// INTERRUPTIONS ///////////////////////////////////////////////////////////////////////////////////////////////////////
void int_stop_pressed(){  // ISR stop button
//    noInterrupts();
    if (mouse_state != STATE_MOUSE_ABORTED) {
        Serial.print("INTERRUPT! STOP!\n");
        mouse_state = STATE_MOUSE_ABORTED;
    }
    turn_off_led_red();
    stepper_left.stop();
    stepper_right.stop();
    stepper_left.setCurrentPosition(0);
    stepper_right.setCurrentPosition(0);
    stepper_left.moveTo(0);
    stepper_right.moveTo(0);

//    interrupts();
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

//    Serial.print("distanceToGo:    "); Serial.println(stepper_left.distanceToGo());
//    Serial.print("move:            "); Serial.println(66*STEPS_MM);
//    Serial.print("currentposition: "); Serial.println(stepper_left.currentPosition());

}

void delay_with_run(unsigned long delay){
    // TODO: TEM UM BUG
    tmp_currentMillis1 = millis();
    tmp_prev_currentMillis1 = tmp_currentMillis1;
    while (tmp_currentMillis1 - tmp_prev_currentMillis1 < delay){
        tmp_currentMillis1 = millis();
        stepper_left.run();
        stepper_right.run();
    }
}

void do_things_on_cheese(){
    noInterrupts();
    if (mouse_state != STATE_MOUSE_ABORTED) {

        mouse_state = STATE_MOUSE_ON_CHEESE;
        journey_state = STATE_JOURNEY_RETURN_TO_HOME;
        turn_on_led_red();
//        delay_with_run(100);
        kill_motors();
        delay(1000);
        mouse_state = STATE_MOUSE_WALKING;
    }
    interrupts();
    stepper_left.moveTo(PERIMETER_ROBOT_PERIMETER/2*STEPS_MM);  // 264mm(perimeter wheel) Straight [ /2 => 90º ]
    stepper_right.moveTo(-PERIMETER_ROBOT_PERIMETER/2*STEPS_MM);  // 264mm(perimeter wheel) Straight [ /2 => 90º ]
    while (mouse_state != STATE_MOUSE_ABORTED and (stepper_left.currentPosition() < (PERIMETER_ROBOT_PERIMETER/2*STEPS_MM) or stepper_left.currentPosition() > (-PERIMETER_ROBOT_PERIMETER/2*STEPS_MM))){
        stepper_left.run();
        stepper_right.run();
    }
}

void timeout(){
    stepper_left.stop();
    stepper_right.stop();
    stepper_left.moveTo(0);
    stepper_right.moveTo(0);
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
    pinMode(PIN_DEBUG, OUTPUT);
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

void kill_motors(){
    stepper_left.setCurrentPosition(0);
    stepper_right.setCurrentPosition(0);
    stepper_left.stop();
    stepper_right.stop();
    stepper_left.moveTo(0);
    stepper_right.moveTo(0);
}
// AUXILIAR FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
    Serial.begin(BOUDRATE);
    set_pins_mode();
    turn_off_beacon();
    turn_off_main_motors();
    turn_off_led_red();
    turn_on_servo();
    set_servo_degree(90);
    turn_off_servo();

    attachInterrupt(digitalPinToInterrupt(PIN_STOP), int_stop_pressed, FALLING); // RISING - to trigger when the pin goes from low to high; FALLING - for when the pin goes from high to low;

    stepper_left.setMaxSpeed(500.0 * MOTOR_MICROSTEPS);
    stepper_left.setAcceleration(1000.0 * MOTOR_MICROSTEPS);
    stepper_right.setMaxSpeed(500.0 * MOTOR_MICROSTEPS);
    stepper_right.setAcceleration(1000.0 * MOTOR_MICROSTEPS);
    stepper_right.setPinsInverted  ( true, false, false );

    Serial.println("Setup... OK");

//  test();
}

// the loop function runs over and over again forever
void loop() {
<<<<<<< Updated upstream
    mouse_state = STATE_MOUSE_WAITTING_TO_START;
    #ifdef DEBUG_HC
        Serial.print("Waitting for START... PIN_START="); Serial.println(digitalRead(PIN_START));
    #endif
    // WAITTING START BUTTON
    while(digitalRead(PIN_START) == HIGH){ /* square(); */}
    time_start = millis();
    #ifdef DEBUG_HC
        Serial.print("PIN_START pressed\n");
    #endif
    stepper_left.stop();
    stepper_right.stop();
    stepper_left.setCurrentPosition(0);
    stepper_right.setCurrentPosition(0);
    stepper_left.moveTo(0);
    stepper_right.moveTo(0);
    turn_on_main_motors();
    turn_on_servo();
    set_servo_degree(90);
    turn_off_led_red();
    mouse_state = STATE_MOUSE_SEARCH_BEACON;

    // Main while inside loop()
    while(mouse_state != STATE_MOUSE_ALL_DONE && mouse_state != STATE_MOUSE_ABORTED){
        // TASK 1
        currentMillis = millis();
        if ((currentMillis - previousMillis_t1) >= TICK) {
            previousMillis_t1 = currentMillis;

            refresh_all_distance_sensors();  //SENSOR READING
//            mouse_state = STATE_MOUSE_AVOIDING_COLISION;
            Serial.println(mouse_state);
            if (journey_state == STATE_JOURNEY_GOING_TO_CHEESE and floor_sensor_on_cheese() == true){
                colid = 0;
                Serial.println("Cheguei ao queijo!");
                do_things_on_cheese();
                delay(1000);
            } else if (
                    journey_state == STATE_JOURNEY_GOING_TO_CHEESE and
                    (
                    (distance_left!=0 and distance_left<150) or
                    (distance_right!=0 and distance_right<150) or
                    (distance_frontL!=0 and distance_frontL<150) or
                    (distance_frontR!=0 and distance_frontR<150)
                    )
                    and mouse_state != STATE_MOUSE_AVOIDING_COLISION
                    ){
                Serial.println("Perigo!");
                mouse_state = STATE_MOUSE_AVOIDING_COLISION;
            } if (mouse_state == STATE_MOUSE_SEARCH_BEACON){
//                Serial.println("STATE_MOUSE_SEARCH_BEACON");
                Serial.println("S_S_B");
                colid = 0;

                kill_motors();

                stepper_left.moveTo(PERIMETER_ROBOT_PERIMETER*STEPS_MM);  // 264mm(perimeter wheel) Straight
                stepper_right.moveTo(-PERIMETER_ROBOT_PERIMETER*STEPS_MM);  // 264mm(perimeter wheel) Straight

                aux1 = 90; // Numero de Samples do Beacon por rotação
                int tmp_beacon = false;
                while (aux1 > 0 and !tmp_beacon) {
                    // 1st - TURN ON - DELAY; 2ns - READ; 3rd - TURN OFF; 4th - DELAY
                    turn_on_beacon();
                    delay_with_run(18);
                    tmp_beacon = read_beacon();
                    if (tmp_beacon){
//                        Serial.println("LED ON and STOP");
//                        turn_on_led_red();
                        kill_motors();
                    } else {
                        turn_off_led_red();
                    }
                    delay_with_run(18);

                    stepper_left.run();
                    stepper_right.run();
//                    print_all_variables();
                    aux1--;
                } // END WHILE
                // TODO:
                // O aux, neste contexto dá para no retorno, saber a direcção do beacon
                mouse_state = STATE_MOUSE_WALKING;
                Serial.println("STATE changed to WALKING");
//                delay(5000);

            } else if (mouse_state == STATE_MOUSE_AVOIDING_COLISION){
                colid++;
                if(colid>=200){
                    colid=0;
                    kill_motors();
                    stepper_left.moveTo(PERIMETER_ROBOT_PERIMETER/2*STEPS_MM);  // 264mm(perimeter wheel) Straight [ /2 => 90º ]
                    stepper_right.moveTo(-PERIMETER_ROBOT_PERIMETER/2*STEPS_MM);  // 264mm(perimeter wheel) Straight [ /2 => 90º ]
                    while (mouse_state != STATE_MOUSE_ABORTED and (stepper_left.currentPosition() < (PERIMETER_ROBOT_PERIMETER/2*STEPS_MM) or stepper_left.currentPosition() > (-PERIMETER_ROBOT_PERIMETER/2*STEPS_MM))){
                        stepper_left.run();
                        stepper_right.run();
                    }
                    mouse_state = STATE_MOUSE_WALKING;
                }
//                Serial.println("STATE_MOUSE_AVOIDING_COLISION");
//                Serial.println("S_A_C");
//                updatePosition();
//                Serial.print("L FL FR R : ");
                Serial.print(distance_left); Serial.print("\t");
                Serial.print(distance_frontL); Serial.print("\t");
                Serial.print(distance_frontR); Serial.print("\t");
                Serial.print(distance_right); Serial.print(" | ");
                Serial.print(stepper_left.currentPosition());
                Serial.print("\t");
                Serial.print(stepper_right.currentPosition());
                Serial.print("\t");
                Serial.print(stepper_left.targetPosition());
                Serial.print("\t");
                Serial.println(stepper_right.targetPosition());

                Serial.print("STATE  ");
                // (1 | 1 )& 0 & 0 : Dir Fast
                if ( (distance_right > 0 or distance_frontR > 0) and distance_left == 0 and distance_frontL == 0 ){
                    stepper_left.move(-10*STEPS_MM);  // 264mm(perimeter wheel) Straight
                    stepper_right.move(10*STEPS_MM);  // 264mm(perimeter wheel) Straight
                // 0 & 0 & (1 | 1 ) : Esq Fast
                } else if ( distance_right == 0 and distance_frontR == 0 and (distance_left > 0 or distance_frontL > 0 )){
                    stepper_left.move(10*STEPS_MM);  // 264mm(perimeter wheel) Straight
                    stepper_right.move(-10*STEPS_MM);  // 264mm(perimeter wheel) Straigh
                // 1 & 1 & 1 & 1 : Rotate Esq fast
                } else if ( distance_right > 0 and distance_frontR > 0 and distance_left > 0 and distance_frontL > 0 ) {
                    stepper_left.move(10*STEPS_MM);  // 264mm(perimeter wheel) Straight
                    stepper_right.move(-10*STEPS_MM);  // 264mm(perimeter wheel) Straight
                // 1 & 0 & 0 & 0 : Dir slow
                } else if ( distance_right > 0 and distance_frontR == 0 and distance_left == 0 and distance_frontL == 0 ) {
                    stepper_left.move(-5*STEPS_MM);  // 264mm(perimeter wheel) Straight
                    stepper_right.move(5*STEPS_MM);  // 264mm(perimeter wheel) Straight
                // 0 & 0 & 0 & 1 : Esq slow
                }else if ( distance_right == 0 and distance_frontR == 0 and distance_left == 0 and distance_frontL > 0 ) {
                    stepper_left.move(5*STEPS_MM);  // 264mm(perimeter wheel) Straight
                    stepper_right.move(-5*STEPS_MM);  // 264mm(perimeter wheel) Straight

                // 0 & 1 & 1 & 0 : Esq fast
                }else if ( distance_right == 0 and distance_frontR > 0 and distance_left > 0 and distance_frontL == 0 ) {
                    stepper_left.move(10*STEPS_MM);  // 264mm(perimeter wheel) Straight
                    stepper_right.move(-10*STEPS_MM);  // 264mm(perimeter wheel) Straight
                // 0 & 1 & 1 & 1 : Esq fast
                }else if ( distance_right == 0 and distance_frontR > 0 and distance_left > 0 and distance_frontL > 0 ) {
                    stepper_left.move(10*STEPS_MM);  // 264mm(perimeter wheel) Straight
                    stepper_right.move(-10*STEPS_MM);  // 264mm(perimeter wheel) Straight
                // 1 & 1 & 1 & 0 : Dir fast
                }else if ( distance_right > 0 and distance_frontR > 0 and distance_left > 0 and distance_frontL == 0 ) {
                    stepper_left.move(-10*STEPS_MM);  // 264mm(perimeter wheel) Straight
                    stepper_right.move(10*STEPS_MM);  // 264mm(perimeter wheel) Straight
                // 1 & 0 & 0 & 1 : Rot Esq fast
                }else if ( distance_right > 0 and distance_frontR == 0 and distance_left == 0 and distance_frontL > 0 ) {
                    stepper_left.move(10*STEPS_MM);  // 264mm(perimeter wheel) Straight
                    stepper_right.move(-10*STEPS_MM);  // 264mm(perimeter wheel) Straight
                }
                else{
                    mouse_state = STATE_MOUSE_WALKING;
                    Serial.println("WALKING");
                }
//                if (distance_frontL > 0 and distance_frontR > 0 and distance_left == 0 and distance_right == 0 ){
//                    stepper_left.move(-50*STEPS_MM);  // 264mm(perimeter wheel) Straight
//                    stepper_right.move(50*STEPS_MM);  // 264mm(perimeter wheel) Straight
////                    delay_with_run(100);
//
//                }
// else if ( (distance_left + distance_frontL) > (distance_right + distance_frontR) ){
////                    stepper_left.setCurrentPosition(0);
////                    stepper_right.setCurrentPosition(0);
//                    stepper_left.move(50*STEPS_MM);  // 264mm(perimeter wheel) Straight
//                    stepper_right.move(-50*STEPS_MM);  // 264mm(perimeter wheel) Straight
//                }else{
////                    stepper_left.setCurrentPosition(0);
////                    stepper_right.setCurrentPosition(0);
//                    stepper_left.move(-50*STEPS_MM);  // 264mm(perimeter wheel) Straight
//                    stepper_right.move(50*STEPS_MM);  // 264mm(perimeter wheel) Straight
//                }
//                if (!(
//                        (distance_left!=0 and distance_left<150) or
//                        (distance_right!=0 and distance_right<150) or
//                        (distance_frontL!=0 and distance_frontL<150) or
//                        (distance_frontR!=0 and distance_frontR<150)
//                )){
////                    noInterrupts();
//                    if (mouse_state!=STATE_MOUSE_ABORTED) {
//                        mouse_state = STATE_MOUSE_WALKING;
//                    }
////                    interrupts();
//                }
            } else if (mouse_state==STATE_MOUSE_TIMEOUT){
                // TODO: Por led a piscar?
                Serial.println("STATE_MOUSE_TIMEOUT");
                colid = 0;
                while(mouse_state != STATE_MOUSE_ABORTED){
                    turn_on_led_red();
                    delay(1000);
                    turn_off_led_red();
                    delay(1000);
                }
            }else{ // WALKING AROUND
//                Serial.println("WALKING");
                colid = 0;
                Serial.println("WLK");
                stepper_left.move(15*STEPS_MM);  // 264mm(perimeter wheel) Straight
                stepper_right.move(15*STEPS_MM);  // 264mm(perimeter wheel) Straight
            }
        }//IF MILLIS()



        // TASK 2
        if ((millis() - time_start) >= TIMEOUT) {
            mouse_state = STATE_MOUSE_TIMEOUT;
        }//IF MILLIS()


        stepper_left.run();
        stepper_right.run();
        #ifdef DEBUG_HC
            print_all_variables();
        #endif

    } //END Main while inside loop()

    if (mouse_state == STATE_MOUSE_ALL_DONE){
        #ifdef DEBUG_HC
            Serial.print("Terminei... A espera de START para rerecomeçar...");
            Serial.print(" PIN_START="); Serial.print(digitalRead(PIN_START));
            Serial.print("; PIN_STOP="); Serial.print(digitalRead(PIN_STOP)); Serial.print(";");
            Serial.print("\n"); Serial.print("\n"); Serial.print("\n");
        #endif
//        noInterrupts();
        if (mouse_state!=STATE_MOUSE_ABORTED) {
            mouse_state = STATE_MOUSE_WAITTING_TO_START;
        }
//        interrupts();
        turn_off_led_red();
        turn_off_main_motors();
    }

    if (mouse_state == STATE_MOUSE_ABORTED){
        #ifdef DEBUG_HC
            Serial.print("ABORTED! PIN_START="); Serial.print(digitalRead(PIN_START));
            Serial.print("; PIN_STOP="); Serial.print(digitalRead(PIN_STOP)); Serial.print(";");
            Serial.print("\n"); Serial.print("\n"); Serial.print("\n");
        #endif
//        noInterrupts();
        if (mouse_state!=STATE_MOUSE_ABORTED) {
            mouse_state = STATE_MOUSE_WAITTING_TO_START;
        }
        mouse_state = STATE_MOUSE_WAITTING_TO_START;
        journey_state = STATE_JOURNEY_GOING_TO_CHEESE;
//        interrupts();
        turn_off_led_red();
        turn_off_servo();
        stepper_left.stop();
        stepper_right.stop();
        stepper_left.setCurrentPosition(0);
        stepper_right.setCurrentPosition(0);
        stepper_left.moveTo(0);
        stepper_right.moveTo(0);
        turn_off_main_motors();
        turn_off_beacon();
    }
} // END loop()
=======
    } // END loop()
>>>>>>> Stashed changes

