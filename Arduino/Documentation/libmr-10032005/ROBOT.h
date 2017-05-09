//*******************************************************************************************
// ROBOT.H
//
// Header file para incluir em programas que utilizem as funcoes definidas
//  no ficheiro "robot.c"
//
// DETUA, Mar/2005
//*******************************************************************************************


// Definicoes espcificas para a placa de extensao DetPIC16F876

#define EN_IV		RB7	// Bit de Enable/Disable\ dos sensores do robot
#define LED_FAROL	RC2	// Saida para o LED de indicacao de chegada ao farol
#define START		RA5	// Botao de START
#define STOP		RA4	// Botao de STOP
#define SENS_CHAO	RC0	// Sensor de chao
#define SERVO_CNTL	RC1	// Bit de controlo do servo de deteccao do farol
#define MOTOR_0		RB1	// Saida de PWM para o motor 0
#define MOTOR_1		RB4	// Saida de PWM para o motor 1
#define DIR_MOT_0	RB5	// Controlo do sentido de rotacao do Motor_0
#define DIR_MOT_1	RB6	// Controlo do sentido de rotacao do Motor_1

// Inicializacoes relativas ao microcontrolador
void initPic(void);

// Funcao para definicao da velocidade e sentido de rotacao dos motores de traccao
void motor(unsigned char n_motor, signed char speed);

// Funcao para definicao da posicao angular do servo
void servo(signed char offset);

// Funcao para aquisicao do sinal presente na entrada "chn" do multiplexer analogico
unsigned char analog(unsigned char chn);

// Funcao para envio da string apontada por "str" para a porta serie
void printStr(const char *str);

// Funcao para envio de um valor para a porta serie, em base 2, 10 ou 16
void printVal(unsigned char base, unsigned int value);

// Funcao de envio p/ a porta serie da string "str" e do valor "val" (por esta ordem)
void printStrVal10(const char *str, unsigned int val);

// Funcao para gerar um atraso multiplo de 0,1 ms
void wait(unsigned int delay);

// Funcao para leitura de um caracter da porta serie
unsigned char readChar(void);

// Funcao para converter 1 digito hexadecimal (4 bits) no correspondente caracter ASCII
unsigned char bin2asc(unsigned char num);
