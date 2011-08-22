/*
 * RC5.c
 *
 *  Created on: 14/08/2011
 *      Author: ARCESCH
 */

#include "RC5.h"

unsigned char reseteo_general;
unsigned char habilitacion=TRUE;

PINSEL_CFG_Type PinCfg;
TIM_TIMERCFG_Type TIM_ConfigStruct;
TIM_MATCHCFG_Type TIM_MatchConfigStruct;
TIM_CAPTURECFG_Type TIM_CaptureConfigStruct;


//RC5 <--
/********************************************************************
; LPC1769 - RC5 decoder
;
; This package uses T0-CAP0 input (capture and interrupt on both edges)
; CAP0.0 (P1.26) is connected to P0.24 (to check high / low level by software)
; RC5 format:
;
; | S | F | C | 5 system bits | 6 command bits |
; | 1 | 1 | 0 | 0 | 0 | 0 | 0 | 1 | 1 | 0 | 1 | 1 | 1 | 1 |
;
; -----+ +-+ +-+ +-+ +-+ +-+ +---+ +-+ +---+ +-+ +-+ +-+ +----
;      | | | | | | | | | | | | | | | | | | | | | | | |
;      +-+ +---+ +-+ +-+ +-+ +-+ +-+ +---+ +-+ +-+ +-+ +-+
;
**************************************************************************/


void RC5_Shift_Bit(char val)
{
    if (sys & 0x80)
    {
        if (cmd & 0x80)                    // command full ?
        {
            sys = 0;                       // yes, ERROR
            cmd = 0;
        }
        else
        {
            cmd = (cmd << 1) | val;        // shift command

            if (cmd & 0x80) // command full ?
            {
            	RC5_Command = cmd & 0x7F; // OK! Save command byte
            	RC5_System = sys & 0x1F; // save system byte
            	RC5_flag = 1; // set event to application
            	sys = 0;
            	cmd = 0;
            	if (RETARDO!=0)
            	habilitacion=FALSE;
            }
        }
    }
    else
        sys = (sys << 1) | val;            // shift system

}

/************************************************************************
; RC5_Decode (we only take action at a rising edge)
;
; Half(prev) Bit   Low Time        High Time      Action     New Half Bit
; -------------------------------------------------------------------
;     0               0               0          Shift 0         0
;     0               0               1          Shift 1         1
;     0               1               0          -ERROR-         *
;     0               1               1          Shift 1,0       0
;     1               0               0          Shift 1         1
;     1               0               1          -ERROR-         *
;     1               1               0          Shift 1,0       0
;     1               1               1          -ERROR-         *
;
***********************************************************************/
void RC5_Decode(void)
{
  unsigned char action;

    action = half_bit << 2;

// Check High Time
    if ((high_time > MIN_FULL_BIT) && (high_time < MAX_FULL_BIT))
        action = action | 1;               // high_time = long
    else if (!((high_time > MIN_HALF_BIT) && (high_time < MAX_HALF_BIT)))
    {
        sys = 0;                           // RC5 ERROR
        cmd = 0;
        return;
    }

// Check Low Time

    if ((low_time > MIN_FULL_BIT) && (low_time < MAX_FULL_BIT))
        action = action | 2;               // low_time = long
    else if (!((low_time > MIN_HALF_BIT) && (low_time < MAX_HALF_BIT)))
    {
        sys = 0;                           // RC5 ERROR
        cmd = 0;
        return;
    }



    switch (action)
    {
      case 0:  RC5_Shift_Bit(0);           // short low, short high, shift 0
               break;
      case 1:  RC5_Shift_Bit(1);           // short low, long high, shift 1
               half_bit = 1;               // new half bit is true
               break;
      case 2:  sys = 0;                    // long low, short high, ERROR
               cmd = 0;
      case 3:  RC5_Shift_Bit(1);           // long low, long high, shift 1,0
               RC5_Shift_Bit(0);
               break;
      case 4:  RC5_Shift_Bit(1);           // short low, short high, shift 1
               break;
      case 5:  sys = 0;                    // short low, long high, ERROR
               cmd = 0;
               break;
      case 6:  RC5_Shift_Bit(1);           // long low, short high, shift 1,0
               RC5_Shift_Bit(0);
               half_bit = 0;               // new half bit is false
               break;
      case 7:  sys = 0;                    // long low, long high, ERROR
               cmd = 0;
      default: break;                      // invalid
    }
}

/************************************************************************
; TIMER0_IRQHandler - Interrupcion temporal del TIMER 0
;
; Ingresará en cada flanco ascendente o descendente en el pin
; del CAP0.0
;
***********************************************************************/

void TIMER0_IRQHandler(void)
{
static signed int comodin = 10;       //Se usa comodin para dar el valor inicial
									  //ya que se debe asignar un valor fijo al momento de
									  //compilación.-


static signed int valor_relativo = 0; //Almacenará el valor del timer capturado cuando interrumpe
static signed int valor_absoluto = 0; //Almacenará el valor en useg acumulados entre flancos
static signed int cero = 0;           //Es el cero que tomará el valor final del nivel anterior




if (habilitacion==TRUE){              //Estoy habilitado para operar?

	if (comodin==10){ cero = TIM_GetCaptureValue(LPC_TIM0,0); //Es la primera vez?
					  comodin=20;                             //Cambio el valor para no volver
					}

	valor_relativo=TIM_GetCaptureValue(LPC_TIM0,0);	//Copio el valor del timer capturado
	valor_absoluto=valor_relativo-cero; //Me quedo con el valor absolito de tiempo en useg

	if (valor_absoluto>MAX_FULL_BIT) //Me fijo si ese valor está fuera del valor máximo según norma
		{							 //Si era mayor es un error. Acá puede caer, si por ejemplo
		sys = 0;					 //habilito la medición y decodificación y justo cae en medio
		cmd = 0;                     //de un código.
		}

	if (GPIO_ReadValue(0) & (1<<24)) // Chequeo el GPIO P0.24 para ver que tipo de flanco es
		{
		if (sys == 0) // Es el primer pulso?
			{
			low_time = HALF_BIT_TIME; // Asumo tiempo corto en nivel bajo
			high_time = HALF_BIT_TIME; // Asumo tiempo corto en nivel alto
			half_bit = 1; // Asumo que es la mitad de un bit
			cmd = 0x02; // = 00000010, Preparo el byte del comando
			}
		else
		{
			low_time = valor_absoluto; // Es flanco positivo, copio valor bajo
		}
		RC5_Decode();
		}
	else
		high_time = valor_absoluto; //Sino en flanco negativo, copio valor alto

	cero=valor_relativo; //El valor del final del nivel (alto o bajo) será mi nuevo cero
	TIM_ClearIntCapturePending(LPC_TIM0,0); //Reseteo la interrupción
}

if (habilitacion==FALSE){ //Si estoy acá, estoy en el tiempo donde no habilito la ,
						  //edición y decodificación. Lo hago para que no detecte muchos pulsos
					      //en un pulsado.

	valor_relativo=TIM_GetCaptureValue(LPC_TIM0,0); //Analizo cuanto tiempo pasó desde el último
	valor_absoluto=valor_relativo-cero;				//flanco.
	if (valor_absoluto>RETARDO) //Si es mayor a RETARDO useg
	{
		habilitacion=TRUE;  //Ya pasó el tiempo de resguardo
		cero=valor_relativo; //Actualizo el cero
		TIM_ClearIntCapturePending(LPC_TIM0,0); //Reseteo interrupción
	}
	else TIM_ClearIntCapturePending(LPC_TIM0,0); //Si no pasó el tiempo sólo reseteo interrupción

}

}

/************************************************************************
; RC5_Init - Inicializo los pines
;
; Defino P1.26 como CAP0.0 para capturar el valor del timer entre flancos
;
; Defino P0.24 como I/O para detectar que tipo de flanco fue
;
***********************************************************************/

 void RC5_Init(void)
 {


		 reseteo_general=0;  // Esto estaba en el main() antes.


	     //Config P1.26 as CAP0.0
	     PinCfg.Funcnum = PINSEL_FUNC_3;
	     PinCfg.OpenDrain = 0;
	     PinCfg.Pinmode = 0;
	     PinCfg.Portnum = PINSEL_PORT_1;
	     PinCfg.Pinnum = PINSEL_PIN_26;
	     PINSEL_ConfigPin(&PinCfg);

	     //Config P0.24 como GPIO
	     PinCfg.Funcnum=PINSEL_FUNC_0;
	     PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	     PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	     PinCfg.Pinnum = PINSEL_PIN_24;
	     PinCfg.Portnum = PINSEL_PORT_0;
	     PINSEL_ConfigPin(&PinCfg);


	     // Initialize timer 0, prescale count time of 1000000uS = 1S lo cambio yo
	     TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	     TIM_ConfigStruct.PrescaleValue   = 1;

	     // use channel 0, CAPn.0
	     TIM_CaptureConfigStruct.CaptureChannel = 0;
	     // Enable capture on CAPn.0 rising edge
	     TIM_CaptureConfigStruct.RisingEdge = ENABLE;
	     // Enable capture on CAPn.0 falling edge
	     TIM_CaptureConfigStruct.FallingEdge = ENABLE;
	     // Generate capture interrupt
	     TIM_CaptureConfigStruct.IntOnCaption = ENABLE;


	     // Set configuration for Tim_config and Tim_MatchConfig
	     TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);
	     TIM_ConfigCapture(LPC_TIM0, &TIM_CaptureConfigStruct);
	     TIM_ResetCounter(LPC_TIM0);


	     /* preemption = 1, sub-priority = 1 */
	     NVIC_SetPriority(TIMER0_IRQn, ((0x01<<3)|0x01));
	     /* Enable interrupt for timer 0 */
	     NVIC_EnableIRQ(TIMER0_IRQn);
	     // To start timer 0
	     TIM_Cmd(LPC_TIM0,ENABLE);
} // --> RC5
