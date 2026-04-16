/*
 * HCSR04.c
 *
 * Created: 1/4/2026 11:00:58
 *  Author: Usuario Principal
 */ 
#include "HCSR04.h"

void HCSR04_Init(HCSR04_Setting_t *setting, hcsr_write_ptr trigger, hcsr_read_ptr echo, hcsr_get_time_ptr timer) {
	
	// Guardo punteros a funciòn	
	setting->trigger_write = trigger;
	setting->echo_read = echo;
	setting->get_us = timer;
	
	// Inicializo variables internas
	
	setting->state = HCSR_IDLE;
	setting->t_ref = 0;
	setting->timeout = HCSR04_TIMEOUT_US;
	setting->distance = 0;
	
	//Uso la funcion
	setting->trigger_write(0);
}

void HCSR04_Process(HCSR04_Setting_t *setting) {
	// FSM
	
	switch (setting->state){
		
		case HCSR_IDLE:
		break;
		
		case HCSR_TRIGGER:
		
			setting->trigger_write(1);
			setting->t_ref = setting->get_us();
			setting->state = HCSR_TRIG_WAIT;
		
		break;
		
		case HCSR_TRIG_WAIT:
			
			if ((setting->get_us() - setting->t_ref) >= 10){		
				setting->trigger_write(0);
				setting->t_ref = setting->get_us();
				setting->state = HCSR_ECHO_WH;
			}
			
		break;
		
		case HCSR_ECHO_WH:
			if(setting->echo_read() == 1){
				setting->t_ref = setting->get_us();
				setting->state = HCSR_ECHO_WL;
				}else if((setting->get_us() - setting->t_ref) >= setting->timeout){
				setting->state = HCSR_ERROR;
			}
		break;
		
		case HCSR_ECHO_WL:
	
			if(setting->echo_read() == 0){
				
				uint32_t time_echo = setting->get_us() - setting->t_ref;
				setting->distance = (uint16_t)(time_echo/58);
				setting->state = HCSR_READY;
			}
		break;
		
		case HCSR_READY:
		break;

		case HCSR_ERROR:
		break;
		
		default:
			setting->state = HCSR_IDLE;
		break;
	}
	
		
		
	
}