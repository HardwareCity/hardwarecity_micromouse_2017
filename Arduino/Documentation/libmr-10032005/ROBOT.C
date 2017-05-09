//*******************************************************************************************
// ROBOT.C
//
// Dep. Electronica da Universidade de Aveiro
// Jose Luis Azevedo
//
// Mar/2005
//********************************************************************************************
#include <pic.h>
#include "robot.h"

#define	FORW		1
#define	BACK		0


// O Timer 1 é utilizado para gerar o sinal de PWM para os motores e para o servo
#define F_CPU		5e3	// Frequência de funcionamento do CPU em KHz

#define T1_PRES 1	// Prescaler seleccionado na programacao do timer 1
#define T1_OUT	10	// Periodo do sinal de saida do Timer 1 (em mS)
#define KT1		(unsigned int)(65535-(F_CPU/T1_PRES)*T1_OUT) // Constante de divisão p/ o Timer 1
#define KT1_H	(unsigned char)(KT1 >> 8)
#define KT1_L	(unsigned char)(KT1 & 0xFF)

#define KNORM	(65535 - KT1) / 100 // Constante de normalizacao para o calculo
									// da constante do timer em funcao da velocidade

// Variaveis globais
static bank3 unsigned char kccp1_l, kccp1_h, kccp2_l, kccp2_h;
static bank3 unsigned char ktmr_0;

unsigned char timeTick20, timeTick40, timeTick80, timeTick640;


//********************************************************************************************
// Inicializacoes relativas ao microcontrolador
//
void initPic()
{
	TRISA=0b00110001;	// RA0,RA4,RA5 input / RA1...RA3 output
	TRISB=0b00000001;	// RB0 input / RB1...RB7 outputs
	TRISC=0b10010001;	// RC0,RC4,RC7 input / RC1...RC3,RC5,RC6 output
	
// Programacao do CCP1 e CCP2
// CCP1: Compare mode, generate interrupt on match
// CCP2: Compare mode, generate interrupt on match
	
	CCP1M0 = 0;
	CCP1M1 = 1;
	CCP1M2 = 0;
	CCP1M3 = 1;

	CCP2M0 = 0;
	CCP2M1 = 1;
	CCP2M2 = 0;
	CCP2M3 = 1;

// Valor inicia do CCP1 (PWM = 0)
	CCPR1H = KT1_H;
	CCPR1L = KT1_L;
	
// Valor inicia do CCP2 (PWM = 0)
	CCPR2H = KT1_H;
	CCPR2L = KT1_L;

	CCP1IE = 1;		// CCP1 interrupt enable
	CCP2IE = 1;		// CCP2 interrupt enable

// Programacao do Timer 1

	TMR1ON=1;		// Timer 1 ON
	TMR1IE=1;		// Timer1 interrupt enable

	TMR1H = KT1_H;	//
	TMR1L = KT1_L;	// T = 10ms (f = 100Hz)  

	T1CKPS1 = 0;
	T1CKPS0 = 0;	// prescaler 1:1

// Programacao do Timer 0

	PS2 = 1;	//#
	PS1 = 1;	//# Set prescaler to 256
	PS0 = 1;	//#
	PSA = 0;	// Prescaler assigned to the Timer0 module
	T0SE = 0;	// Increment on low to high transition
	T0CS = 0;	// Source clock: internal clock
	T0IE = 1;	// Enable interrupts from Timer0

// Activacao global das interrupcoes

	GIE=1;		//	SET Global Int enable
	PEIE=1; 	//	SET Peripheral Int enable
// Programacao da Usart

	SPBRG = 64;	// BR=19200
	BRGH = 1;	// BR high speed
	SPEN = 1;	// Serial Port Enable
	TXEN = 1;	// Transmit Enable 
	CREN = 1;	// Continuous Reception Enable

// Programacao da ADC

	ADCON0=0b10000001; // Fosc/32, channel 0(RA0), enable ADC converter
	ADCON1=0x0E;	   // Left justified, RA0 analog, RA1..RA5 digital
}


//*******************************************************************************************
// Funcao p/ definicao da velocidade e do sentido de rotacao dos motores de traccao do robot
// Parametros de entrada:
//
//	* mn	-> numero do motor: 0 ou 1
//	* speed-> velocidade: 0 a  100  motor a rodar para a frente
//						  0 a -100 motor a rodar para tras
//
void motor(unsigned char n_motor, signed char speed)
{
	unsigned int compVal;	// Compare value
	char dir;				// Motor direction

	n_motor &= 0x01;				//  "nm" so pode tomar os valores 0/1
	speed = speed >  100 ?	100 : speed;
	speed = speed < -100 ? -100 : speed;

	dir = FORW;
	if(speed < 0) {
		speed = ~speed + 1; // speed = abs(speed)
		dir = BACK;
	}

	compVal = KT1+(speed * KNORM);

	if (n_motor == 0) {		// Motor 0
		kccp1_h = compVal>>8;
		kccp1_l = compVal;
		DIR_MOT_0 = dir;
	} else {				// Motor 1
		kccp2_h = compVal>>8;
		kccp2_l = compVal;	
		DIR_MOT_1 = dir;
	}
}

//*******************************************************************************************
// Funcao para definicao da posicao angular do servo (utilizado no detector de farol )
// Parametro de entrada:
//
//	* offset -> valor na gama [-17, +17] (00 servo na posicao central)
//
void servo(signed char offset)
{
	offset = offset >  18 ?  18 : offset;
	offset = offset < -16 ? -16 : offset;

	ktmr_0 = 256 - (offset + 28);	// PWM tem que estar na gama
									// [0.6ms, 2.3ms] => 11, 45 clocks do timer0
}

//*******************************************************************************************
// Rotina de servico 'a interrupcao
//
void interrupt ISR(void)
{
	static unsigned char cntTicks = 0;

	if (CCP1IF == 1) {		// CCP1 Interrupt
		RB1 = 0;			// PWM_Motor_0 = 0
		CCP1IF = 0;			// Reset CCP1 interrupt flag
	}

	if (CCP2IF == 1) {		// CCP2 Interrupt
		RB4 = 0;			// PWM_Motor_1 = 0
		CCP2IF = 0;			// Reset CCP2 interrupt flag
	}

	if (TMR1IF == 1) {		// Timer1 Interrupt
		TMR1H = KT1_H;
		TMR1L = KT1_L;		// Reload Timer 1 count constant

		TMR0 = ktmr_0;		// Timer 0 count constant
		SERVO_CNTL = 1;		// PWM_Servo = 1

		CCPR1H = kccp1_h;
		CCPR1L = kccp1_l;	// CCP1 compare constant
		
		CCPR2H = kccp2_h;
		CCPR2L = kccp2_l;	// CCP2 compare constant
		
		MOTOR_0 = 1;		// PWM_Motor_0 = 1
		MOTOR_1 = 1;		// PWM_Motor_1 = 1

		if( (++cntTicks % 2) == 0 ) timeTick20 = 1;	// Set every 20 ms
		if( (cntTicks % 4) == 0 ) timeTick40 = 1;	// Set every 40 ms
		if( (cntTicks % 8) == 0 ) timeTick80 = 1;	// Set every 80 ms
		if( (cntTicks % 64) == 0 ) timeTick640 = 1;	// Set every 640 ms
	
		T0IE = 1;			// Enable Timer 0 interrupts
		TMR1IF=0;			// Reset Timer1 interrupt flag
	}	

	if( T0IF == 1 ) {		// Timer0 Interrupt
		T0IF = 0;			// Reset Timer0 interrupt flag
		T0IE = 0;			// Disable Timer 0 interrupts
		SERVO_CNTL = 0;		// PWM_Servo = 0
	}
}

//*******************************************************************************************
// Funcao para envio da string apontada por "str" para a porta serie
//
void printStr(const char *str)
{
	while( *str != 0 ) {
		while( TXIF == 0 );
		TXREG = *str++;
	}
}

//*******************************************************************************************
// Funcao para leitura de um caracter da porta serie
//
unsigned char readChar(void)
{
	while( RCIF == 0 );
	return RCREG;
}

//*******************************************************************************************
// Funcao para converter 1 digito hexadecimal (4 bits) no correspondente caracter ASCII
//
unsigned char bin2asc(unsigned char num)
{
	num += '0';
	if(num > '9')
		num += 'A'-'9'-1;
	return num;
}

//*******************************************************************************************
// Funcao para envio de um valor para a porta serie, em base 2, 10 ou 16
//
void printVal(unsigned char base, unsigned int val)
{
// Base representa a base em que o valor deve ser visualizado (ex: 2, 10, 16)

	unsigned char str[17];	// Preve a impressao em base 2 (16 bits)
	unsigned char i, digito;

	i = 0;
	do {
		digito = val % base;
		val /= base;
		str[i++] = bin2asc(digito);
	} while( val > 0 );

	while( i > 0 ) {
		while( TXIF == 0 );
		TXREG = str[--i];
	}
}

//*******************************************************************************************
// Funcao de envio p/ a porta serie da string "str" e do valor "val" (por esta ordem)
//
void printStrVal10(const char *s, unsigned int val)
{
	printStr(s);
	printStr("\t");
	printVal(10, val);
}


//*******************************************************************************************
// Funcao para gerar um atraso (multiplos de 0,1 ms)
//
void wait(unsigned int value)
{
	unsigned int i,k;

	for(k=0;k<value;k++)		// Wait value*100us
		for(i=0;i<70;i++)		// Ciclo de espera de aprox. 100 uS
			asm("nop");

}

//*******************************************************************************************
// Funcao para aquisicao do sinal presente na entrada "chn" do multiplexer analogico
//
unsigned char analog(unsigned char chn)
{
	unsigned char i;

    chn &= 0x07;
	PORTA = (PORTA & 0xF1) | (chn << 1);

//  RA3 = ((chn & 0x04) >> 2);	// Seleccao do canal activo do multiplexer analogico
//	RA2 = ((chn & 0x02) >> 1);	//
//	RA1 = (chn & 0x01);	//

	for(i=0;i<100;i++) asm("nop");// Ciclo de espera de aprox. 140 uS (acquisition time)

	ADGO=1; 					// Start conversion (GO Bit = 1)
	while(ADGO);				// Espera ate' a conversao terminar
	//wait(2);					// Delay de 200 uS
	return(ADRESH);				// Retorna o valor da conversao
}
