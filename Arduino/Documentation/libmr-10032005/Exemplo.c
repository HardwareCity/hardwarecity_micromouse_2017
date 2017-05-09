//*******************************************************************
// EXEMPLO.C
// Programa exemplo p/ os robots Micro-Rato (placa PIC16F876)
// (utiliza funções implementadas no ficheiro "robot.c")
//
// Dep.Electronica da Univ.de Aveiro
// Jose Luis Azevedo
//
// Marco/2005
//*******************************************************************

#include <pic.h>
#include "robot.h"

/*******************************************************************
* Constantes
*/
#define VELOC		25	// velocidade de rotacao dos motores
#define MOTOR_DIR	0	// motor direito
#define MOTOR_ESQ	1	// motor esquerdo

#define SENS_FRENTE	4	// O sensor da frente esta ligado ao canal 4
						// do MUX analogico (ver esquema)
/*******************************************************************
* Variaveis globais
*/
extern unsigned char timeTick20, timeTick40, timeTick80;


//*******************************************************************
// Main
//
void main(void)
{
	initPic();

	printStr("\r\nMICRO-RATO 2005\r\n\r\n");

	servo(0);
	motor(MOTOR_DIR, 0);
	motor(MOTOR_ESQ, 0);
	wait(10000);	// Espera 1,0 s
	servo(-10);
	wait(10000);	// Espera 1,0 s
	servo(+10);
	wait(10000);	// Espera 1,0 s
	servo(0);

//	motor(MOTOR_DIR, VELOC);
//	motor(MOTOR_ESQ, VELOC);

	
	while(1) {
		while(START);
		printStr("\r\nStart -> sensores ON\r\n");
		EN_IV = 1;	// Activa os sensores de obstáculos e de chegada
		do {
			while(!timeTick40);	// Variavel booleana activada cada 40 ms
			timeTick40=0;		// Reset da variavel
			// NB: A saida dos sensores so e actualizada de 30 em 30 ms
			printStrVal10("\rSensor frente: ", analog(SENS_FRENTE) );
			printStr("  ");
		} while(STOP);
		printStr("\r\nStop -> sensores OFF\r\n");
	}
}
