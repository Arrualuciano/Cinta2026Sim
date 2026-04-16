/*
 * simucinta.c
 *
 * Created: 4/4/2026 17:59:18
 *  Author: Luciano
 */ 
#include <stddef.h>
#include "SimuCinta.h"

// Comandos MICRO ? SIMU
#define CMD_ALIVE           0xF0
#define CMD_START           0x50
#define CMD_STOP            0x51
#define CMD_SET_ARM         0x52
#define CMD_RESET           0x53
#define CMD_SET_VELOCITY    0x54

// Eventos SIMU ? MICRO
#define CMD_IR_SENSOR       0x5E
#define CMD_NEW_BOX         0x5F

void SimuCinta_Init(SimuCinta_t *simu, Comm_Protocol *comm, simucinta_newbox_cb onNewBox, simucinta_ir_cb onIRSensor) {
	
	simu->comm				= comm; 	// guardar puntero a comm
	simu->onNewBox			= onNewBox; // guardar callbacks
	simu->onIRSensor		= onIRSensor;
	simu->connected			= 0; // inicializar connected = 0
	simu->running			= 0; // inicializar running = 0
	simu->velocity			= 0; // inicializar velocity = 0
	simu->currentBoxType	= 0; // inicializar currentBoxType = 0
	simu->aliveTimer		= 1;
	
}

void SimuCinta_SendAlive(SimuCinta_t *simu){
	
	Comm_Send(simu->comm, CMD_ALIVE, NULL, 0);

}

void SimuCinta_Start(SimuCinta_t *simu){
	
	Comm_Send(simu->comm, CMD_START, NULL, 0);
	
}

void SimuCinta_Stop(SimuCinta_t *simu){
	
	Comm_Send(simu->comm, CMD_STOP, NULL, 0);
	
}

void SimuCinta_Reset(SimuCinta_t *simu){
	
	Comm_Send(simu->comm, CMD_RESET, NULL, 0);

}

void SimuCinta_SetVelocity(SimuCinta_t *simu, uint8_t velocity){
	
	Comm_Send(simu->comm, CMD_SET_VELOCITY, &velocity, 1);
	
}

void SimuCinta_SetArm(SimuCinta_t *simu, uint8_t boxType, uint8_t setPosition){
	
	uint8_t data[2] = {boxType, setPosition};
	
	Comm_Send(simu->comm, CMD_SET_ARM , data, 2);
}

void SimuCinta_OnCmd(SimuCinta_t *simu, uint8_t cmdId, uint8_t *payload, uint8_t nBytes){
	
	switch (cmdId) {
		case CMD_ALIVE:      // 0xF0 ACK ? connected = 1
		
			simu->connected = 1;
		
		break;
		case CMD_START:      // 0x50 ACK ? guardar velocity y boxTypes
			simu->velocity   = payload[0];    // v*10 que mand� el simulador
			simu->boxType[0] = payload[1];    // tipo de caja en salida 0
			simu->boxType[1] = payload[2];    // tipo de caja en salida 1
			simu->boxType[2] = payload[3];    // tipo de caja en salida 2
			simu->running    = 1;             // la cinta arranc�
		break;
		case CMD_STOP:       // 0x51 ACK ? running = 0
		
			simu->running	= 0;
		
		break;
		case CMD_NEW_BOX:    // 0x5F ? llamar onNewBox
		
			simu->currentBoxType = payload[0];
			simu->onNewBox(payload[0]);
		
		break;
		case CMD_IR_SENSOR:  // 0x5E ? llamar onIRSensor
			// El payload llega como pares (outNum, irState). Usamos i+1 < nBytes
			// como guardia defensiva por si el simulador enviara un nBytes impar
			// (corrupcion / bug); evitamos leer payload[i+1] fuera del rango.
			for (uint8_t i = 0; i + 1 < nBytes; i += 2){

				simu->onIRSensor(payload[i], payload[i+1]);

			}

		break;
	}
}

void SimuCinta_Process(SimuCinta_t *simu) {
	simu->aliveTimer--;
	if (simu->aliveTimer == 0) {
		simu->aliveTimer = 250;  // 250 � 2ms = 500ms
		SimuCinta_SendAlive(simu);
	}
}