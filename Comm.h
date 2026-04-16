/**
 * @file    Comm.h
 * @brief   Modulo de comunicacion serie con protocolo UNER.
 *
 * @details Implementa el protocolo de trama UNER sobre UART0 del ATmega.
 *          Formato de trama:
 *          @code
 *          [ 'U' | 'N' | 'E' | 'R' | LEN | ':' | CMDID | DATA... | CKS ]
 *          @endcode
 *          donde LEN = 1 (CMDID) + N (bytes de datos) + 1 (CKS), y
 *          CKS = XOR de todos los bytes desde 'U' hasta el ultimo dato.
 *
 *          El modulo utiliza dos buffers circulares (RingBuf) de 128 bytes
 *          para desacoplar la ISR de recepcion del procesamiento en el
 *          main loop.
 *
 * @author  Luciano
 * @date    Abril 2026
 */

#ifndef COMM_H_
#define COMM_H_

/** @brief Tamano maximo del payload (CMDID + datos) que se puede recibir. */
#define COMM_MAX_PAYLOAD  32

#include <stdint.h>
#include "RingBuf.h"

/**
 * @brief Tipo de callback invocado al recibir un comando valido.
 *
 * @param idComm  Identificador del comando recibido.
 * @param payload Puntero al arreglo de bytes de datos (sin CMDID ni CKS).
 * @param nBytes  Cantidad de bytes de datos en @p payload.
 */
typedef void (*comm_cmd_cb)(uint8_t idComm, uint8_t *payload, uint8_t nBytes);

/**
 * @brief Contexto completo de una instancia del protocolo UNER.
 *
 * @note  Debe inicializarse con Comm_Init() antes de usar cualquier otra
 *        funcion del modulo.
 */
typedef struct {
    RingBuf_t    rxBuf;                   /**< Buffer circular de recepcion.          */
    RingBuf_t    txBuf;                   /**< Buffer circular de transmision.        */
    comm_cmd_cb  onCmd;                   /**< Callback para comandos validos.        */
    uint8_t      cks;                     /**< Checksum acumulado durante recepcion.  */
    uint8_t      timeOut;                 /**< Temporizador de timeout (reservado).   */
    uint8_t      hdrState;                /**< Estado actual de la FSM de decodificacion. */
    uint8_t      nBytes;                  /**< Bytes restantes del payload en curso.  */
    uint8_t      payload[COMM_MAX_PAYLOAD]; /**< Buffer de payload en construccion.   */
    uint8_t      payloadIdx;              /**< Indice de escritura dentro de payload. */
} Comm_Protocol;

/**
 * @brief Inicializa el modulo de comunicacion y el hardware UART.
 *
 * @param comm     Puntero a la instancia Comm_Protocol a inicializar.
 * @param callback Funcion que se invocara cada vez que llegue un comando valido.
 */
void Comm_Init(Comm_Protocol *comm, comm_cmd_cb callback);

/**
 * @brief Configura el periferico UART0 del ATmega.
 *
 * @details 115200 baud, 8N1, con bit U2X activado.
 *          Habilita interrupcion de recepcion (RXCIE) para alimentar el
 *          buffer RX desde la ISR.
 */
void Comm_InitUART(void);

/**
 * @brief Agrega un byte recibido por la ISR al buffer circular de RX.
 *
 * @details Debe llamarse exclusivamente desde la ISR USART_RX_vect.
 *
 * @param comm    Puntero a la instancia Comm_Protocol.
 * @param byteISR Byte leido del registro UDR0 dentro de la ISR.
 */
void Comm_PutRxByte(Comm_Protocol *comm, uint8_t byteISR);

/**
 * @brief Procesa el modulo de comunicacion: envio y recepcion.
 *
 * @details Debe llamarse periodicamente desde el main loop (no bloqueante).
 *          - TX: transmite un byte pendiente si el registro UDR0 esta libre.
 *          - RX: corre la FSM UNER sobre todos los bytes disponibles en el
 *            buffer de recepcion e invoca el callback @p onCmd al completar
 *            una trama valida con checksum correcto.
 *
 * @param comm Puntero a la instancia Comm_Protocol.
 */
void Comm_Process(Comm_Protocol *comm);

/**
 * @brief Encola una trama UNER en el buffer de transmision.
 *
 * @details Construye y encola la trama completa: encabezado, LEN, ':', CMDID,
 *          datos y checksum. La transmision efectiva ocurre en Comm_Process().
 *
 * @param comm     Puntero a la instancia Comm_Protocol.
 * @param cmdId    Identificador del comando a enviar.
 * @param data     Puntero a los bytes de datos (puede ser NULL si nroBytes == 0).
 * @param nroBytes Cantidad de bytes de datos en @p data.
 */
void Comm_Send(Comm_Protocol *comm, uint8_t cmdId, uint8_t *data, uint8_t nroBytes);

#endif /* COMM_H_ */
