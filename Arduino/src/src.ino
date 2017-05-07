#include <Servo.h>

// PINS
// PIN 2 Livre, reservado para algo que precise de interrupções, como sensor de batimento
#define PIN_STOP 3 // INTERRUPTOR STOP [USA INTERRUPÇÕES]
#define PIN_START 4 // INTERRUPTOR START
#define PIN_MOTOR_SERVO 5  // Motor para o farol [USA PWM]
#define PIN_LED_IR_FRONT 6
#define PIN_LED_IR_BACK 7
#define PIN_MOTORS_EN 8  // Activar os dois motores
#define PIN_MOTOR_LEFT_DIR 10  // Motor Esquerdo Direção
#define PIN_MOTOR_LEFT_STEP 9    // Motor Esquerdo passo
#define PIN_MOTOR_RIGHT_DIR 12  // Motor Direito Direção
#define PIN_MOTOR_RIGHT_STEP 11  // Motor Direito passo
#define PIN_LED_RED 13

#define PIN_TRIG_LEFT A0  // Sensor de distancia LEFT
#define PIN_ECHO_LEFT A1  // Sensor de distancia LEFT
#define PIN_TRIG_RIGHT A2  // Sensor de distancia RIGHT
#define PIN_ECHO_RIGHT A3  // Sensor de distancia RIGHT
#define PIN_TRIG_FRONT A4  // Sensor de distancia FRONT
#define PIN_ECHO_FRONT A5  // Sensor de distancia FRONT
//#define PIN_COLOR_FLOOR A6  // Sensor do chão
// #define PIN_LIVRE A7 // LIVRE


// States
#define STATE_WAITTING_TO_START 1
#define STATE_GOING_TO_CHEESE 2
#define STATE_ON_CHEESE 3
#define STATE_BACK_TO_HOME 4
#define STATE_ALL_DONE 5
#define STATE_ABORTED 6

// IDs
#define SENSOR_LEFT 0
#define SENSOR_RIGHT 1
#define SENSOR_FRONT 2

#define MIN_DELAY_MOTORS 17000 // Microseconds

// Variaveis globais
Servo motor_servo_farol;
int motor_servo_farol_pos = 0;
byte state = STATE_WAITTING_TO_START;
float tmp_duration=0, duration_left=0, duration_right=0, duration_front=0;
float tmp_distance=0, distance_left=0, distance_right=0, distance_front=0;

int tmp_pin_trig=-1, tmp_pin_echo=-1;

// TEST ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void test(){
    Serial.println("Running test()...");

    turn_on_main_motors();
    rodar_motores(-200, 200);
    delay(100);
    rodar_motores(200, -200);
    turn_off_main_motors();

    turn_on_servo();
    set_servo_degree(0);
    set_servo_degree(180);
    set_servo_degree(0);
    set_servo_degree(90);
    turn_off_servo();

    turn_on_led_red();
    delay(2000);
    turn_off_led_red();


    turn_on_main_motors();
    rodar_motores(100, 100);
    delay(100);
    rodar_motores(100, -100);
    turn_off_main_motors();

    Serial.println("Running test()... OK");
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

void rodar_motores(int steps_left, int steps_right){
    while(steps_left!=0 || steps_right!=0){
//        Serial.print("A rodar. steps_left=");
//        Serial.print(steps_left);
//        Serial.print("\t | steps_right=");
//        Serial.print(steps_right);
//        Serial.print("\n");
        // Motor Esquerdo
        if (steps_left>0){
            steps_left-=1;
            digitalWrite(PIN_MOTOR_LEFT_DIR, HIGH);
            digitalWrite(PIN_MOTOR_LEFT_STEP, HIGH);
            delayMicroseconds(MIN_DELAY_MOTORS);
            digitalWrite(PIN_MOTOR_LEFT_STEP, LOW);
            delayMicroseconds(MIN_DELAY_MOTORS);
        }else{
            if (steps_left<0){
                steps_left+=1;
                digitalWrite(PIN_MOTOR_LEFT_DIR, LOW);
                digitalWrite(PIN_MOTOR_LEFT_STEP, HIGH);
                delayMicroseconds(MIN_DELAY_MOTORS);
                digitalWrite(PIN_MOTOR_LEFT_STEP, LOW);
                delayMicroseconds(MIN_DELAY_MOTORS);
            }
        }
        // Motor direito
        if (steps_right>0){
            steps_right-=1;
            digitalWrite(PIN_MOTOR_RIGHT_DIR, LOW);
            digitalWrite(PIN_MOTOR_RIGHT_STEP, HIGH);
            delayMicroseconds(MIN_DELAY_MOTORS);
            digitalWrite(PIN_MOTOR_RIGHT_STEP, LOW);
            delayMicroseconds(MIN_DELAY_MOTORS);
        }else{
            if (steps_right<0){
                steps_right+=1;
                digitalWrite(PIN_MOTOR_RIGHT_DIR, HIGH);
                digitalWrite(PIN_MOTOR_RIGHT_STEP, HIGH);
                delayMicroseconds(MIN_DELAY_MOTORS);
                digitalWrite(PIN_MOTOR_RIGHT_STEP, LOW);
                delayMicroseconds(MIN_DELAY_MOTORS);
            }
        }
//        delayMicroseconds(MIN_DELAY_MOTORS);
    }
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
float get_distance(int sensor_id){
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
        case SENSOR_FRONT:
            tmp_pin_trig = PIN_TRIG_FRONT;
            tmp_pin_echo = PIN_ECHO_FRONT;
            break;
    }

    if (tmp_pin_trig>0 and tmp_pin_echo>0) {
        digitalWrite(tmp_pin_trig, LOW);  // Added this line //delayMicroseconds(2); // Added this line
        digitalWrite(tmp_pin_trig, HIGH); //delayMicroseconds(10); // Added this line
        digitalWrite(tmp_pin_trig, LOW);
        tmp_duration = pulseIn(tmp_pin_echo, HIGH); // 875, Timout para 30 cm
        tmp_distance = (tmp_duration / 2) / 29.154518; // TOF_1cm;
        //return duration_left;
    }


//    Serial.print("DISTANCE: ");
//    Serial.print(tmp_distance);
//    Serial.print('\n');
    return tmp_distance;
}

void refresh_all_distance_sensors(){
    distance_left = get_distance(SENSOR_LEFT);
    distance_right = get_distance(SENSOR_RIGHT);
    distance_front = get_distance(SENSOR_FRONT);
}
// DISTANCE SENSORS ////////////////////////////////////////////////////////////////////////////////////////////////////
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
    if (state != STATE_ABORTED) {
        Serial.print("INTERRUPT! STOP!\n");
        state = STATE_ABORTED;
    }
}
// INTERRUPTIONS ///////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// the setup function runs once when you press reset or power the board
void setup() {
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
    pinMode(PIN_TRIG_LEFT, OUTPUT);
    pinMode(PIN_ECHO_LEFT, INPUT);
    pinMode(PIN_TRIG_RIGHT, OUTPUT);
    pinMode(PIN_ECHO_RIGHT, INPUT);
    pinMode(PIN_TRIG_FRONT, OUTPUT);
    pinMode(PIN_ECHO_FRONT, INPUT);

    state = STATE_WAITTING_TO_START;

    turn_off_main_motors();
    turn_off_servo();
    turn_off_led_red();

    // Interrupções
    attachInterrupt(digitalPinToInterrupt(PIN_STOP), int_stop_pressed, FALLING); // RISING - to trigger when the pin goes from low to high; FALLING - for when the pin goes from high to low;

    Serial.begin(115200);
    Serial.println("Setup... OK");

    // test
    test();

}

// the loop function runs over and over again forever
void loop() {
    Serial.print("A espera de START... PIN_START=");
    Serial.println(digitalRead(PIN_START));
    state = STATE_WAITTING_TO_START;

    while(digitalRead(PIN_START) == HIGH){}
    Serial.print("PIN_START pressed\n");
    turn_on_main_motors();

    // Main while inside loop()
    while(state != STATE_ALL_DONE && state != STATE_ABORTED){
        refresh_all_distance_sensors();
        Serial.print("LEFT=");
        Serial.print(distance_left);
        Serial.print(",\tRIGHT=");
        Serial.print(distance_right);
        Serial.print(",\tFRONT=");
        Serial.print(distance_front);
        Serial.print("\n");


    } //END Main while inside loop()

    if (state == STATE_ALL_DONE){
        Serial.print("Terminei... A espera de START para rerecomeçar...");
        Serial.print(" PIN_START=");
        Serial.print(digitalRead(PIN_START));
        Serial.print("; PIN_STOP=");
        Serial.print(digitalRead(PIN_STOP));
        Serial.print(";");
        Serial.print('\n'); Serial.print('\n'); Serial.print('\n');
        state = STATE_WAITTING_TO_START;
        turn_off_servo();
        turn_off_led_red();
        turn_off_main_motors();
    }

    if (state == STATE_ABORTED){
        Serial.print("ABORTED! PIN_START=");
        Serial.print(digitalRead(PIN_START));
        Serial.print("; PIN_STOP=");
        Serial.print(digitalRead(PIN_STOP));
        Serial.print(";");
        Serial.print('\n'); Serial.print('\n'); Serial.print('\n');
        state = STATE_WAITTING_TO_START;
        turn_off_servo();
        turn_off_led_red();
        turn_off_main_motors();
    }

}

