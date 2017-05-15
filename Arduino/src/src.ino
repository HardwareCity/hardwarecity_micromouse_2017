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
#include "Adafruit_TCS34725.h"
#include "DefinePins.h"
#include "STATE_MACHINES.h"

#define DEBUG // Uncoment for DEBUG mode     <<<-------------

#define BOUDRATE 230400 // 115200 ; 230400 ; 500000

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
#define THRESHOLD_RED 1000
#define THRESHOLD_GREEN 1000
#define THRESHOLD_BLUE 1000

// Global Variables
// int myInts[5000][3]; // <-- Era apra salvar o mapa, mas não está em uso
unsigned long currentMillis=0;
long aux1=0, aux2=0; // Auxiliar Variables
byte state_square = SQ1; // Estado do quadrado (para testes)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
uint16_t r, g, b, c, colorTemp, lux;
Servo motor_servo_farol;
AccelStepper stepper_left(AccelStepper::DRIVER, PIN_MOTOR_LEFT_STEP, PIN_MOTOR_LEFT_DIR);
AccelStepper stepper_right(AccelStepper::DRIVER, PIN_MOTOR_RIGHT_STEP, PIN_MOTOR_RIGHT_DIR);
byte mouse_state = STATE_MOUSE_WAITTING_TO_START;
byte journey_state = STATE_JOURNEY_GOING_TO_CHEESE;
int motor_servo_farol_pos = 0;
float tmp_duration=0, duration_left=0, duration_right=0, duration_frontL=0, duration_frontR=0;
float tmp_distance=0;
unsigned int distance_left=0, distance_right=0, distance_frontL=0, distance_frontR=0;
int tmp_pin_trig=-1, tmp_pin_echo=-1;
unsigned long previousMillis = 0;
const long tick = 100; //t(ms)

// TEST ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void test(){
    Serial.println("Running test()...");

    turn_on_main_motors();
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
    set_servo_degree(0);
    set_servo_degree(180);
    set_servo_degree(0);
    set_servo_degree(90);
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

    #ifdef DEBUG
        //Form Feed char(0x0C): Page break on terminal. Select "Handle Form Feed Character" at CoolTerm preferences
        Serial.write(0x0C);

        Serial.print("L FL FR R :");
        Serial.print(distance_left); Serial.print("\t");
        //Serial.print(",\tF.L=");
        Serial.print(distance_frontL); Serial.print("\t");
        //Serial.print(",\tF.R=");
        Serial.print(distance_frontR); Serial.print("\t");
        //Serial.print(",\tRIGHT=");
        Serial.print(distance_right); Serial.print("\t");

        Serial.print("\n");
    #endif
}
// DISTANCE SENSORS ////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// FLOOR SENSOR ////////////////////////////////////////////////////////////////////////////////////////////////////////
bool floor_sensor_on_cheese(){
    return false;
    // TODO: Mudar este valor!

    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    lux = tcs.calculateLux(r, g, b);

    #ifdef DEBUG
        Serial.print("R: "); Serial.print(r); Serial.print(" ");
        Serial.print("G: "); Serial.print(g); Serial.print(" ");
        Serial.print("B: "); Serial.print(b); Serial.print(" ");
        Serial.print("C: "); Serial.print(c); Serial.print("|");
        Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
        Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" ");
        Serial.println(" ");
    #endif

    if (r>THRESHOLD_RED && g>THRESHOLD_GREEN && b>THRESHOLD_BLUE){
        #ifdef DEBUG
            Serial.println("WHITE");
        #endif
        return false;
    }else{
        #ifdef DEBUG
            Serial.println("BLACK");
        #endif
        return true;
    }

}
// FLOOR SENSOR ////////////////////////////////////////////////////////////////////////////////////////////////////////
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


// INTERRUPTIONS ///////////////////////////////////////////////////////////////////////////////////////////////////////
void int_stop_pressed(){  // ISR stop button
    if (mouse_state != STATE_MOUSE_ABORTED) {
        Serial.print("INTERRUPT! STOP!\n");
        mouse_state = STATE_MOUSE_ABORTED;
    }
}
// INTERRUPTIONS ///////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// AUXILIAR FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////////////
void do_things_on_cheese(){
    mouse_state = STATE_MOUSE_ON_CHEESE;
    journey_state = STATE_JOURNEY_RETURN_TO_HOME;
    turn_on_led_red();
    mouse_state = STATE_MOUSE_WALKING;
}

void set_pins_mode(){
    // PINS
    pinMode(PIN_STOP, INPUT_PULLUP);
    pinMode(PIN_START, INPUT_PULLUP);
    pinMode(PIN_MOTOR_SERVO, OUTPUT);
//    pinMode(PIN_LED_IR_FRONT, INPUT);
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
}
// AUXILIAR FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
    set_pins_mode();
    turn_off_main_motors();
    turn_off_led_red();
    Serial.begin(BOUDRATE);

    attachInterrupt(digitalPinToInterrupt(PIN_STOP), int_stop_pressed, FALLING); // RISING - to trigger when the pin goes from low to high; FALLING - for when the pin goes from high to low;

    stepper_left.setMaxSpeed(1000.0 * MOTOR_MICROSTEPS);
    stepper_left.setAcceleration(1000.0 * MOTOR_MICROSTEPS);
    stepper_right.setMaxSpeed(1000.0 * MOTOR_MICROSTEPS);
    stepper_right.setAcceleration(1000.0 * MOTOR_MICROSTEPS);
    stepper_right.setPinsInverted  ( true, false, false );


    // Francisco: Comentar isto enquanto Não tivermos sensor: -----
    if (tcs.begin()) {
    #ifdef DEBUG
        Serial.println("Found RGB sensor");
    #endif
    } else {
        for (int i = 10; i>0; i--){
            Serial.println("No TCS34725 found ... check your connections");
            turn_on_led_red();
            delay(50);
            turn_off_led_red();
            delay(50);
        }
        Serial.println("No TCS34725 found ... check your connections");
//        while (1){
//            Serial.println("No TCS34725 found ... check your connections");
//        }; // halt!
    }
    // Francisco: Comentar isto enquanto Não tivermos sensor: -----


    #ifdef DEBUG
        Serial.println("Setup... OK");
    #endif

    #ifdef DEBUG
        test(); // Testar Motores e Led
    #endif
}

// the loop function runs over and over again forever
void loop() {
    mouse_state = STATE_MOUSE_WAITTING_TO_START;
    #ifdef DEBUG
        Serial.print("Waitting for START... PIN_START="); Serial.println(digitalRead(PIN_START));
    #endif
    while(digitalRead(PIN_START) == HIGH){ /* square(); */}
    #ifdef DEBUG
        Serial.print("PIN_START pressed\n");
    #endif
    turn_on_main_motors();
    turn_on_servo();
    set_servo_degree(0);
    mouse_state = STATE_MOUSE_WALKING;

    // Main while inside loop()
    while(mouse_state != STATE_MOUSE_ALL_DONE && mouse_state != STATE_MOUSE_ABORTED){
        currentMillis = millis();
        if (currentMillis - previousMillis >= tick) {
            previousMillis = currentMillis;

            refresh_all_distance_sensors();  //SENSOR READING
            if (journey_state == STATE_JOURNEY_GOING_TO_CHEESE and floor_sensor_on_cheese() == true){
                do_things_on_cheese();
            } else if (
                    journey_state == STATE_JOURNEY_GOING_TO_CHEESE and
                    (
                            (distance_left!=0 and distance_left<100) or
                            (distance_right!=0 and distance_right<100) or
                            (distance_frontL!=0 and distance_frontL<100) or
                            (distance_frontR!=0 and distance_frontR<100)
                    )
                    and mouse_state != STATE_MOUSE_AVOIDING_COLISION
                    ){
//                for (int i = 2; i>0; i--){
//                    turn_on_led_red();
//                    delay(100);
//                    turn_off_led_red();
//                    delay(100);
////                    stepper_left.move(-660*STEPS_MM);  // 264mm(perimeter wheel) Straight
////                    stepper_right.move(-66*STEPS_MM);  // 264mm(perimeter wheel) Straight
//                }
                mouse_state=STATE_MOUSE_AVOIDING_COLISION;
            } if (mouse_state==STATE_MOUSE_AVOIDING_COLISION){
                if ( (distance_left + distance_frontL) > (distance_right + distance_frontR) ){
                    stepper_left.move(5*STEPS_MM);  // 264mm(perimeter wheel) Straight
                    stepper_right.move(-5*STEPS_MM);  // 264mm(perimeter wheel) Straight
                }else{
                    stepper_left.move(-5*STEPS_MM);  // 264mm(perimeter wheel) Straight
                    stepper_right.move(5*STEPS_MM);  // 264mm(perimeter wheel) Straight
                }
                if (!(
                        (distance_left!=0 and distance_left<100) or
                        (distance_right!=0 and distance_right<100) or
                        (distance_frontL!=0 and distance_frontL<100) or
                        (distance_frontR!=0 and distance_frontR<100)
                )){
                    mouse_state=STATE_MOUSE_WALKING;
                }
            }else{ // WALKING AROUND
                stepper_left.move(66*STEPS_MM);  // 264mm(perimeter wheel) Straight
                stepper_right.move(66*STEPS_MM);  // 264mm(perimeter wheel) Straight

            }

        }//IF MILLIS()

        stepper_left.run();
        stepper_right.run();

    } //END Main while inside loop()

    if (mouse_state == STATE_MOUSE_ALL_DONE){
        #ifdef DEBUG
            Serial.print("Terminei... A espera de START para rerecomeçar...");
            Serial.print(" PIN_START="); Serial.print(digitalRead(PIN_START));
            Serial.print("; PIN_STOP="); Serial.print(digitalRead(PIN_STOP)); Serial.print(";");
            Serial.print('\n'); Serial.print('\n'); Serial.print('\n');
        #endif
        mouse_state = STATE_MOUSE_WAITTING_TO_START;
        turn_off_led_red();
        turn_off_main_motors();
    }

    if (mouse_state == STATE_MOUSE_ABORTED){
        #ifdef DEBUG
            Serial.print("ABORTED! PIN_START="); Serial.print(digitalRead(PIN_START));
            Serial.print("; PIN_STOP="); Serial.print(digitalRead(PIN_STOP)); Serial.print(";");
            Serial.print('\n'); Serial.print('\n'); Serial.print('\n');
        #endif
        mouse_state = STATE_MOUSE_WAITTING_TO_START;
        turn_off_led_red();
        turn_off_servo();
        stepper_left.stop();
        stepper_right.stop();
        turn_off_main_motors();
    }

}

