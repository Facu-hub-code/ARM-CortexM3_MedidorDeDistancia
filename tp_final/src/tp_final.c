#include "LPC17xx.h"

#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_systick.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"
#include "lcd.h"    //User defined LCD library which contains the lcd routines
#include "delay.h"  //User defined library which contains the delay routines
#include "lcd.c"
#include "delay.c"
#include "gpio.c"

void conf_pines_sensor(void);
void conf_pines_leds(void);
void conf_pin_buzzer(void);
void config_int_gpio(void);
void conf_adc(void);
void conf_int_pulsador(void);
void config_timer0(void); //debouncing
void config_timer3(void); //buzzer
void config_timer1(void); //disparo de medicion
void config_timer2(void); //conteo de tiempo de pulso
void disparo_medicion(void);
void calculo_distancia(void);
void comparacion(void);
void act_timer(void);
void off_buzzer(void);
void retardo(void);
void conf_pin_uart(void);
void conf_uart(void);
void enviar_info_uart(void);

uint16_t ADC0Value = 0;
uint8_t dist_limite = 15;

uint8_t modo_seteo = 0;

uint32_t tiempo_echo = 0;
uint32_t dist_medida = 0;

uint32_t nivel_anterior=0;
uint32_t nivel_actual=5;
uint32_t cont_nivel=0;
uint32_t cont_nuevo_nivel=0;

uint8_t estado_boton_anterior=0;
uint32_t debounce_cont=0;

uint8_t seteo=0;
uint8_t print_midiendo=0;

int main(void) {


	conf_pines_sensor(); //p0.5 como GPIO y de salida pin0.4 como entrada
	conf_pines_leds(); //P2.0 GPIO   P2.1 GPIO P2.2 GPIO
	conf_int_pulsador(); //P2.10 como EINT0
	config_timer0();	//para debouncing
	conf_adc();
	config_timer3();	//para buzzer
	conf_uart();
	conf_pin_uart(); //P0.2  TXD0 — Transmitter output for UART0.

	LCD_SetUp(P2_6,P2_7,P2_8,P_NC,P_NC,P_NC,P_NC,P0_22,P0_27,P0_28,P2_13); //setesa todos estos pines como salida los P_NC no los usa
	//LCD_Init(2,16); //especifica el tipo de lcd (usamos las  dos  lineas y los 16 caracteres

	while(1) {

		if(modo_seteo){

			if(!seteo){ //flag que se utiliza para que vuelva a resetear todo luego de un modo de medicion
				seteo=1;
				print_midiendo=0;	//variable utilizada al entrar a modo de medicion
				cont_nivel=0;
				cont_nuevo_nivel=0;
				nivel_anterior=5;	//variable para manejar el buzzer
				off_buzzer();		//cambia el pin conectado al buzzer como gpio y lo apaga
				LPC_GPIO2->FIOCLR |= (1<<0) | (1<<1) |  (1<<2);	//apaga los 3 leds

			}

			LPC_ADC->ADCR |= (1<<24); // COMENZAR CONVERSION

			while((LPC_ADC->ADDR0 & (1<<31)) == 0); //esperar a que termine de convertir

			ADC0Value = ((LPC_ADC->ADDR0)>>4) & 0xFFF;

			uint32_t relacion = ADC0Value / 500;		//500 es lo equivalente a cada escalon de distancia seteada

			dist_limite = 15+(relacion*5);		//a partir de la relacion, se setea la distancia limite en cm

			//LCD_Clear();
			//LCD_Printf("SETEANDO DIST.");					//se muestra en el lcd el modo en el que esta
			//LCD_Printf("\nDist.Limite:%2dcm",dist_limite);	//y la distancia limite seteada

			DELAY_ms(100);		//espera para volver a a tomar una medidicion

		}

		else{

			if(!print_midiendo){	//solo la primera vez que entra a medir, muestra en el LCD
				seteo=0; //
				print_midiendo=1;
				conf_pin_buzzer();		//vuelve a poner el pin de buzzer en modo MAT3.0
				//LCD_Clear();
				//LCD_Printf("DIST. SETEADA");
				//LCD_Printf("\nDist.Limite:%2dcm",dist_limite);

				uint8_t dist_ascii_decena=48+(dist_limite/10);
				uint8_t dist_ascii_unidad=48+(dist_limite-(dist_limite/10)*10);

				uint8_t info[] = "Nueva distancia limite seteada: \r\n";
				UART_Send(LPC_UART0, info, sizeof(info), BLOCKING);
				uint8_t info_dist[]={ dist_ascii_decena,dist_ascii_unidad,0x20,0x63,0x6D,(uint8_t)'\r',(uint8_t)'\n'};
				UART_Send(LPC_UART0, info_dist, sizeof(info_dist), BLOCKING);
			}

			disparo_medicion();		//pulso de 10us
			while((LPC_GPIO0->FIOPIN & (1<<4))); //Mientras el echo NO este en HIGH, espera
			config_timer2();		//para medir el tiempo en alto del ECHO, mientras el ECHO esta en alto mide la onda que rebota
			while(!LPC_GPIO0->FIOPIN & (1<<4));//Mientras el echo este en HIGH, espera
			tiempo_echo=LPC_TIM2->TC;	//obtengo el tiempo en alto del ECHO
			TIM_Cmd(LPC_TIM2, DISABLE);
			dist_medida=tiempo_echo/58;	//obtengo la medicion
			comparacion();				//comparo para activar leds y buzzer

			DELAY_ms(10);		//espera para volver a a tomar una medidicion
		}
	}

    return 0 ;
}

void conf_pines_sensor(void){

	//p0.5 como GPIO y de salida
	PINSEL_CFG_Type pin_conf;
	pin_conf.Portnum = PINSEL_PORT_0;
	pin_conf.Pinnum = PINSEL_PIN_5;
	pin_conf.Funcnum = PINSEL_FUNC_0;
	pin_conf.Pinmode = PINSEL_PINMODE_PULLDOWN;
	pin_conf.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin( &pin_conf);
	GPIO_SetDir(0,(1<<5),1); //como salida

	//pin0.4
	pin_conf.Portnum = PINSEL_PORT_0;
	pin_conf.Pinnum = PINSEL_PIN_4;
	pin_conf.Funcnum = PINSEL_FUNC_0;
	pin_conf.Pinmode = PINSEL_PINMODE_PULLDOWN;
	pin_conf.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin( &pin_conf);
	GPIO_SetDir(0,(1<<4),0); //como entrada



	return;
}

void conf_pines_leds(void){

	PINSEL_CFG_Type pin_conf;
	pin_conf.Portnum = PINSEL_PORT_2;
	pin_conf.Pinnum = PINSEL_PIN_0;
	pin_conf.Funcnum = PINSEL_FUNC_0;//P2.0 GPIO
	pin_conf.Pinmode = PINSEL_PINMODE_PULLDOWN;
	pin_conf.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin( &pin_conf);
	GPIO_SetDir(2,(1<<0),1); //como salida

	pin_conf.Portnum = PINSEL_PORT_2;
	pin_conf.Pinnum = PINSEL_PIN_1;
	pin_conf.Funcnum = PINSEL_FUNC_0;  //P2.1 GPIO
	pin_conf.Pinmode = PINSEL_PINMODE_PULLDOWN;
	pin_conf.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin( &pin_conf);
	GPIO_SetDir(2,(1<<1),1); //como salida

	pin_conf.Portnum = PINSEL_PORT_2;
	pin_conf.Pinnum = PINSEL_PIN_2;
	pin_conf.Funcnum = PINSEL_FUNC_0; //P2.2 GPIO
	pin_conf.Pinmode = PINSEL_PINMODE_PULLDOWN;
	pin_conf.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin( &pin_conf);
	GPIO_SetDir(2,(1<<2),1); //como salida

}

void conf_adc(void){

	PINSEL_CFG_Type pin_conf;
	pin_conf.Portnum = PINSEL_PORT_0;
	pin_conf.Pinnum = PINSEL_PIN_23;
	pin_conf.Funcnum = PINSEL_FUNC_1;  //AD0.0
	pin_conf.Pinmode = PINSEL_PINMODE_TRISTATE;
	pin_conf.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin( &pin_conf);

	LPC_SC->PCONP 			|= 	(1<<12);//converter (ADC) power/clock control bit.
	LPC_ADC->ADCR 			|= 	(1<<21);//habilita el ADC
	LPC_SC->PCLKSEL0 		|= 	(3<<24); // CCLK/8
	LPC_ADC->ADCR 			&= 	~(255<<8);  //Divisor interno no divide, osea que el ADC con el CCLK default de 100MHz es de 12.5Mhz frec de trabajo
	LPC_ADC->ADCR 			&= ~(1<<16); //Modo burst apagado, uso software para controlarlo
	LPC_ADC->ADCR 			&= ~(0b111<<24);//Start conversion when the edge selected by bit 27 occurs on MAT1.1.

	return;
}

void config_timer3(void){


	TIM_TIMERCFG_Type timer_config;
	TIM_MATCHCFG_Type match_config;

	timer_config.PrescaleOption		=	TIM_PRESCALE_USVAL;
	timer_config.PrescaleValue		=	1;

	match_config.MatchChannel		=	0;
	match_config.IntOnMatch			=	DISABLE;
	match_config.ResetOnMatch		=	ENABLE;
	match_config.StopOnMatch		=	DISABLE;
	match_config.ExtMatchOutputType	=	TIM_EXTMATCH_TOGGLE;
	match_config.MatchValue			=	0; //se va a cargar segun la necesidad

	TIM_Init(LPC_TIM3, TIM_TIMER_MODE, &timer_config);
	TIM_ConfigMatch(LPC_TIM3, &match_config);

	TIM_ResetCounter(LPC_TIM3);
	TIM_Cmd(LPC_TIM3, DISABLE);

}

void config_timer0(void){

	TIM_TIMERCFG_Type timer_config;
	TIM_MATCHCFG_Type match_config;

	timer_config.PrescaleOption		=	TIM_PRESCALE_USVAL;
	timer_config.PrescaleValue		=	1;

	match_config.MatchChannel		=	0;
	match_config.IntOnMatch			=	ENABLE;
	match_config.ResetOnMatch		=	ENABLE;
	match_config.StopOnMatch		=	DISABLE;
	match_config.ExtMatchOutputType	=	TIM_EXTMATCH_TOGGLE;
	match_config.MatchValue			=	9999;  //10ms

	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timer_config);
	TIM_ConfigMatch(LPC_TIM0, &match_config);
	TIM_ResetCounter(LPC_TIM0);
	TIM_Cmd(LPC_TIM0, DISABLE);

	NVIC_EnableIRQ(TIMER0_IRQn);

}

void disparo_medicion(void){

	LPC_GPIO0->FIOSET |= (1<<5); //se envia el alto al trigger
	config_timer1();
	return;

}

void config_timer1(void){

	TIM_TIMERCFG_Type timer_config;
	TIM_MATCHCFG_Type match_config;

	timer_config.PrescaleOption		=	TIM_PRESCALE_USVAL;
	timer_config.PrescaleValue		=	1;

	match_config.MatchChannel		=	0;
	match_config.IntOnMatch			=	ENABLE;
	match_config.ResetOnMatch		=	ENABLE;
	match_config.StopOnMatch		=	ENABLE;
	match_config.ExtMatchOutputType	=	TIM_EXTMATCH_NOTHING;
	match_config.MatchValue			=	9; //para 10us del trigger del sensor(onda que viaja a chocar con el objeto a medir)

	TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &timer_config);
	TIM_ConfigMatch(LPC_TIM1, &match_config);

	TIM_ResetCounter(LPC_TIM1);
	TIM_Cmd(LPC_TIM1, ENABLE);

	NVIC_EnableIRQ(TIMER1_IRQn);
	return;
}

void TIMER1_IRQHandler(void){

	LPC_GPIO0->FIOCLR |= (1<<5); //se detiene el alto al trigger
	LPC_TIM1->IR|=1;

	return;
}

void config_timer2(void){

	TIM_TIMERCFG_Type timer_config;
	TIM_MATCHCFG_Type match_config;

	timer_config.PrescaleOption		=	TIM_PRESCALE_USVAL;
	timer_config.PrescaleValue		=	1;

	match_config.MatchChannel		=	0;
	match_config.IntOnMatch			=	DISABLE;
	match_config.ResetOnMatch		=	DISABLE;
	match_config.StopOnMatch		=	DISABLE;
	match_config.ExtMatchOutputType	=	TIM_EXTMATCH_NOTHING;
	match_config.MatchValue			=	4000000000;

	TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timer_config);
	TIM_ConfigMatch(LPC_TIM2, &match_config);

	TIM_ResetCounter(LPC_TIM2);

	TIM_Cmd(LPC_TIM2, ENABLE);

	return;

}

void comparacion(void){

	if(nivel_anterior != nivel_actual){   //si se detecta cambio de nivel
		cont_nuevo_nivel++;		//cont de que el cambio sea estable
	}else{
		if(cont_nuevo_nivel){	//mientras este cont no sea 0, lo aumentamos en cada medicion estable
			cont_nuevo_nivel++;
			if(cont_nuevo_nivel>=10){ //cuando llega a 10, el cambio de nivel fue estable
				cont_nivel=0;		//conteo de nivel para envio de info y act de buzzer
				cont_nuevo_nivel=0;
			}
		}else{
			cont_nivel++;
		}
	}

	if(cont_nivel==10){		//envio de informacion por uart y act de buzzers y leds
		if(dist_medida > dist_limite+30){
			nivel_actual=4;
			LPC_GPIO2->FIOSET |= (1<<2);
			LPC_GPIO2->FIOCLR |= (1<<0);
			LPC_GPIO2->FIOCLR |= (1<<1);

		}else if(dist_medida > dist_limite+15){
			nivel_actual=2;
			LPC_GPIO2->FIOSET |= (1<<1);
			LPC_GPIO2->FIOCLR |= (1<<0);
			LPC_GPIO2->FIOCLR |= (1<<2);

		}else if (dist_medida > dist_limite){
			nivel_actual=1;
			LPC_GPIO2->FIOSET |= (1<<0);
			LPC_GPIO2->FIOCLR |= (1<<1);
			LPC_GPIO2->FIOCLR |= (1<<2);

		}else if(dist_medida <= dist_limite){
			nivel_actual=0;
			LPC_GPIO2->FIOSET |= (1<<0);
			LPC_GPIO2->FIOSET |= (1<<1);
			LPC_GPIO2->FIOSET |= (1<<2);
		}
		act_timer();
		enviar_info_uart();
		cont_nuevo_nivel=0;
	}

	nivel_anterior=nivel_actual;

	return;
}

void conf_int_pulsador(void){

	PINSEL_CFG_Type pin_conf;
	pin_conf.Portnum = PINSEL_PORT_2;
	pin_conf.Pinnum = PINSEL_PIN_10;
	pin_conf.Funcnum = PINSEL_FUNC_1; //P2.10 como EINT0
	pin_conf.Pinmode = PINSEL_PINMODE_PULLUP;
	pin_conf.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin( &pin_conf);

	LPC_SC->EXTMODE |= 1;			//Selecciona interrupcion por flanco, 1 en bit menos significativo (int por flanco)
	LPC_SC->EXTPOLAR |= 0;			//Interrumpe cuando el flanco es de subida
	LPC_SC->EXTINT |= 1;			//limpia flag de interrupcion

	NVIC_EnableIRQ(EINT0_IRQn);

	return;

}

void EINT0_IRQHandler(void){

	NVIC_DisableIRQ(EINT0_IRQn);

	TIM_ResetCounter(LPC_TIM0);
	TIM_Cmd(LPC_TIM0, ENABLE);

	LPC_SC->EXTINT |= 1;			//limpia flag de interrupcion
	return;
}

void TIMER0_IRQHandler(void){

	if((LPC_GPIO2->FIOPIN & (1<<10)) == 0){  //P2.10 interrumpio?

		if(debounce_cont>=5){

			modo_seteo ^= 1; //Togglea de modo medicion/estado
			NVIC_EnableIRQ(EINT0_IRQn);
			TIM_Cmd(LPC_TIM0, DISABLE);

		}else{
			debounce_cont++;
		}

	}else{
		debounce_cont=0;
		NVIC_EnableIRQ(EINT0_IRQn);
		TIM_Cmd(LPC_TIM0, DISABLE);

	}

	LPC_TIM0->IR|=1;

	return;
}

void act_timer(){
	TIM_Cmd(LPC_TIM3, DISABLE); //habilito el timer
	TIM_ResetCounter(LPC_TIM3);
	if(nivel_actual==0){
		TIM_UpdateMatchValue(LPC_TIM3,0, (100000));
	}else{
		TIM_UpdateMatchValue(LPC_TIM3,0, (nivel_actual*250000));
	}

	TIM_Cmd(LPC_TIM3, ENABLE); //habilito el timer
}

void conf_pin_buzzer(void){

	PINSEL_CFG_Type pin_conf;
	pin_conf.Portnum = PINSEL_PORT_0;
	pin_conf.Pinnum = PINSEL_PIN_10;
	pin_conf.Funcnum = PINSEL_FUNC_3;
	pin_conf.Pinmode = PINSEL_PINMODE_PULLDOWN;
	pin_conf.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin( &pin_conf);

}

void off_buzzer(void){
	TIM_Cmd(LPC_TIM3, DISABLE); //apago el timer
	PINSEL_CFG_Type pin_conf;
	pin_conf.Portnum = PINSEL_PORT_0;
	pin_conf.Pinnum = PINSEL_PIN_10;
	pin_conf.Funcnum = PINSEL_FUNC_0;
	pin_conf.Pinmode = PINSEL_PINMODE_PULLDOWN;
	pin_conf.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin( &pin_conf);

	GPIO_SetDir(0,(1<<10),1); //como salida

	LPC_GPIO0->FIOCLR |= (1<<10);

}

void conf_pin_uart(void){
	PINSEL_CFG_Type PinCfg;
	//pines Tx y Rx
	PinCfg.Funcnum = 1; //P0.2  TXD0 — Transmitter output for UART0.
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 2;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 3; //P0.3
	PINSEL_ConfigPin(&PinCfg);
	return;
}
void conf_uart(void){
	UART_CFG_Type UARTConfigStruct;
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;

	UART_ConfigStructInit(&UARTConfigStruct); //config por defecto
	UART_Init(LPC_UART0, &UARTConfigStruct); //inicializa periferico
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct); //inicializa fifo
	UART_FIFOConfig(LPC_UART0, &UARTFIFOConfigStruct);
	UART_TxCmd(LPC_UART0, ENABLE); //habilita transmision
	return;
}
void enviar_info_uart(void){

	switch(nivel_actual){
		case 0:
		{
			uint8_t info4[] = "ALERTA - COLISION \n\r";
			UART_Send(LPC_UART0, info4, sizeof(info4), BLOCKING);
			break;
		}
		case 1:
		{
			uint8_t info1[] = "PELIGRO - OBSTACULO A MENOS DE 15cm del limite seteado\n\r";
			UART_Send(LPC_UART0, info1, sizeof(info1), BLOCKING);
			break;
		}
		case 2:
		{
			uint8_t info2[] = "ADVERTENCIA - OBSTACULO A MENOS DE 30cm del limite seteado \n\r";
			UART_Send(LPC_UART0, info2, sizeof(info2), BLOCKING);
			break;
		}
		case 4:
		{
			uint8_t info3[] = "NO HAY PELIGRO - OBSTACULO A MAS DE 30cm del limite seteado \n\r";
			UART_Send(LPC_UART0, info3, sizeof(info3), BLOCKING);
			break;
		}

	}

}



