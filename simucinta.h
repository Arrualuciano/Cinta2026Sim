/**
 * @file    simucinta.h
 * @brief   Capa de abstraccion para el simulador de cinta transportadora.
 *
 * @details Encapsula el protocolo de comunicacion especifico del simulador
 *          SimuCinta sobre la capa Comm (protocolo UNER).
 *
 *          Comandos MCU -> Simulador:
 *          | CMD        | ID   | Descripcion                     |
 *          |------------|------|---------------------------------|
 *          | CMD_ALIVE  | 0xF0 | Heartbeat (cada 500 ms)         |
 *          | CMD_START  | 0x50 | Solicitar inicio de la cinta    |
 *          | CMD_STOP   | 0x51 | Solicitar pausa de la cinta     |
 *          | CMD_RESET  | 0x53 | Solicitar reset del simulador   |
 *          | CMD_SET_VELOCITY | 0x54 | Cambiar velocidad          |
 *          | CMD_SET_ARM      | 0x52 | Mover un brazo              |
 *
 *          Eventos Simulador -> MCU:
 *          | EVT            | ID   | Payload                               |
 *          |----------------|------|---------------------------------------|
 *          | CMD_ALIVE ACK  | 0xF0 | (vacio)                               |
 *          | CMD_START ACK  | 0x50 | [vel, tipo0, tipo1, tipo2]            |
 *          | CMD_STOP  ACK  | 0x51 | (vacio)                               |
 *          | CMD_NEW_BOX    | 0x5F | [tipo]                                |
 *          | CMD_IR_SENSOR  | 0x5E | [sensor0, estado0, sensor1, estado1, ...]|
 *
 * @author  Luciano
 * @date    Abril 2026
 */

#ifndef SIMUCINTA_H_
#define SIMUCINTA_H_

#include <stdint.h>
#include "Comm.h"

/**
 * @brief Callback invocado cuando el simulador notifica una nueva caja medida.
 *
 * @param boxType Tipo de caja (ej: 6, 8 o 10 cm).
 */
typedef void (*simucinta_newbox_cb)(uint8_t boxType);

/**
 * @brief Callback invocado cuando cambia el estado de un sensor IR.
 *
 * @param outNum  Numero de salida/sensor (0, 1 o 2).
 * @param IRState Estado del sensor: 1 = caja entrando, 0 = caja saliendo.
 */
typedef void (*simucinta_ir_cb)(uint8_t outNum, uint8_t IRState);

/**
 * @brief Contexto del modulo SimuCinta.
 */
typedef struct {
    Comm_Protocol       *comm;           /**< Puntero a la instancia de comunicacion.       */
    uint8_t              connected;      /**< 1 si se recibio ACK de ALIVE, 0 si no.        */
    uint8_t              running;        /**< 1 si la cinta esta en marcha, 0 si pausada.   */
    simucinta_newbox_cb  onNewBox;       /**< Callback: nueva caja medida.                  */
    simucinta_ir_cb      onIRSensor;     /**< Callback: evento de sensor IR.                */
    uint8_t              aliveTimer;     /**< Contador regresivo para envio de ALIVE.       */
    uint8_t              velocity;       /**< Velocidad actual de la cinta (unidad x10).    */
    uint8_t              boxType[3];     /**< Tipo de caja asignado a cada salida.          */
    uint8_t              currentBoxType; /**< Tipo de la ultima caja notificada.            */
} SimuCinta_t;

/**
 * @brief Inicializa el modulo SimuCinta.
 *
 * @param simu        Puntero a la instancia a inicializar.
 * @param comm        Puntero a la instancia Comm_Protocol ya inicializada.
 * @param onNewBox    Callback para eventos CMD_NEW_BOX.
 * @param onIRSensor  Callback para eventos CMD_IR_SENSOR.
 */
void SimuCinta_Init(SimuCinta_t *simu, Comm_Protocol *comm,
                    simucinta_newbox_cb onNewBox, simucinta_ir_cb onIRSensor);

/**
 * @brief Tarea periodica: envia el heartbeat CMD_ALIVE cada 500 ms.
 *
 * @details Debe llamarse cada 2 ms desde el tick del Timer0.
 *
 * @param simu Puntero a la instancia SimuCinta_t.
 */
void SimuCinta_Process(SimuCinta_t *simu);

/**
 * @brief Envia un CMD_ALIVE (0xF0) al simulador.
 * @param simu Puntero a la instancia SimuCinta_t.
 */
void SimuCinta_SendAlive(SimuCinta_t *simu);

/**
 * @brief Envia CMD_START (0x50) para solicitar inicio de la cinta.
 * @param simu Puntero a la instancia SimuCinta_t.
 */
void SimuCinta_Start(SimuCinta_t *simu);

/**
 * @brief Envia CMD_STOP (0x51) para pausar la cinta.
 * @param simu Puntero a la instancia SimuCinta_t.
 */
void SimuCinta_Stop(SimuCinta_t *simu);

/**
 * @brief Envia CMD_RESET (0x53) para reiniciar el simulador.
 * @param simu Puntero a la instancia SimuCinta_t.
 */
void SimuCinta_Reset(SimuCinta_t *simu);

/**
 * @brief Envia CMD_SET_VELOCITY (0x54) para cambiar la velocidad de la cinta.
 *
 * @param simu     Puntero a la instancia SimuCinta_t.
 * @param velocity Velocidad deseada (el simulador la interpreta como valor x10).
 */
void SimuCinta_SetVelocity(SimuCinta_t *simu, uint8_t velocity);

/**
 * @brief Envia CMD_SET_ARM (0x52) para mover uno o varios brazos.
 *
 * @param simu        Puntero a la instancia SimuCinta_t.
 * @param mask        Bitmask de los brazos a modificar (bit 0 = brazo 0, etc.).
 * @param setPosition Bitmask de posiciones deseadas (1 = extender, 0 = retraer).
 */
void SimuCinta_SetArm(SimuCinta_t *simu, uint8_t mask, uint8_t setPosition);

/**
 * @brief Despacha un comando recibido del simulador.
 *
 * @details Debe llamarse desde el callback registrado en Comm_Init().
 *          Procesa los comandos entrantes del simulador, actualiza el estado
 *          interno e invoca los callbacks de aplicacion segun corresponda.
 *
 * @param simu    Puntero a la instancia SimuCinta_t.
 * @param cmdId   Identificador del comando recibido.
 * @param payload Puntero al arreglo de datos del comando.
 * @param nBytes  Cantidad de bytes de datos en @p payload.
 */
void SimuCinta_OnCmd(SimuCinta_t *simu, uint8_t cmdId,
                     uint8_t *payload, uint8_t nBytes);

#endif /* SIMUCINTA_H_ */
