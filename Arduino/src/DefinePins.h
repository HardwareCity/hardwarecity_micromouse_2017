
#ifndef SRC_DEFINEPINS_H
#define SRC_DEFINEPINS_H
    // PINS
    // PIN 2 Livre, reservado para algo que precise de interrupções, como sensor de batimento

    #define PIN_STOP 3 // INTERRUPTOR STOP [USA INTERRUPÇÕES]
    #define PIN_START 4 // INTERRUPTOR START
    #define PIN_MOTOR_SERVO 5  // Motor para o farol [USA PWM]
    //#define PIN_LED_IR_FRONT 6 // Por usar
    //#define PIN_LED_IR_BACK 7 // Por usar
    #define PIN_MOTORS_EN 8  // Activar os dois motores
    #define PIN_MOTOR_LEFT_DIR 9  // Motor Esquerdo Direção
    #define PIN_MOTOR_LEFT_STEP 10    // Motor Esquerdo passo
    #define PIN_MOTOR_RIGHT_DIR 11  // Motor Direito Direção
    #define PIN_MOTOR_RIGHT_STEP 12  // Motor Direito passo
    #define PIN_LED_RED 13

    #define PIN_TRIG_LEFT   A0  // Sensor de distancia LEFT
    #define PIN_ECHO_LEFT   A1  // Sensor de distancia LEFT
    #define PIN_TRIG_FRONTL A2  // Sensor de distancia RIGHT
    #define PIN_ECHO_FRONTL A3  // Sensor de distancia RIGHT
    #define PIN_TRIG_FRONTR A4  // Sensor de distancia FRONT
    #define PIN_ECHO_FRONTR A5  // Sensor de distancia FRONT
    #define PIN_TRIG_RIGHT  A6  // Sensor de distancia FRONT
    #define PIN_ECHO_RIGHT  A7  // Sensor de distancia FRONT
    #define PIN_IR_FLOOR  A8  // Sensor do chão
    #define PIN_IR_FLOOR2  A9  // Sensor do chão
//    #define PIN_COLOR_FLOOR  A8  // Sensor do chão
    // #define PIN_COLOR_FLOOR A?  // Sensor do chão
    // #define PIN_LIVRE A? // LIVRE

#endif //SRC_DEFINEPINS_H
