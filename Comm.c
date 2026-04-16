/**
 * @file    Comm.c
 * @brief   Implementacion del modulo de comunicacion serie con protocolo UNER.
 *
 * @details Los buffers circulares de RX y TX son arreglos estaticos de 128 bytes
 *          (potencia de 2) declarados en este archivo; sus punteros se asignan
 *          a la estructura Comm_Protocol durante Comm_Init().
 *
 *          Configuracion de UART0 (Comm_InitUART):
 *          - F_CPU = 16 MHz, U2X = 1, UBRR = 16 --> 115200 baud
 *          - Formato: 8N1
 *          - Interrupciones: RX Complete habilitada
 *
 * @author  Luciano
 * @date    Abril 2026
 */

#include "Comm.h"
#include <avr/io.h>

/** @brief Tamano del buffer circular de recepcion (debe ser potencia de 2). */
#define BUFRXSIZE   128

/** @brief Tamano del buffer circular de transmision (debe ser potencia de 2). */
#define BUFTXSIZE   128

/** @brief Almacenamiento estatico del buffer de recepcion. */
static uint8_t bufRx[BUFRXSIZE];

/** @brief Almacenamiento estatico del buffer de transmision. */
static uint8_t bufTx[BUFTXSIZE];

/**
 * @brief Escribe un byte en el buffer circular de transmision.
 *
 * @note  No bloqueante. Si el buffer esta lleno, sobreescribe el byte mas
 *        antiguo (nunca ocurre en condiciones normales de uso).
 *
 * @param comm Puntero a la instancia Comm_Protocol.
 * @param byte Byte a encolar en TX.
 */
static void putTxByte(Comm_Protocol *comm, uint8_t byte) {
    comm->txBuf.buf[comm->txBuf.iw++] = byte;
    comm->txBuf.iw &= (comm->txBuf.size - 1);
}

/**
 * @brief Estados internos de la FSM de decodificacion de tramas UNER.
 *
 * @details La FSM sincroniza con el encabezado "UNER", luego captura el campo
 *          de longitud, el separador ':' y finalmente el payload byte a byte
 *          hasta verificar el checksum.
 */
typedef enum {
    WAIT_U,       /**< Esperando el caracter 'U' del encabezado.         */
    WAIT_N,       /**< Esperando el caracter 'N'.                        */
    WAIT_E,       /**< Esperando el caracter 'E'.                        */
    WAIT_R,       /**< Esperando el caracter 'R'.                        */
    WAIT_NBYTES,  /**< Esperando el byte de longitud (LEN).              */
    WAIT_COLON,   /**< Esperando el separador ':'.                       */
    RECV_PAYLOAD  /**< Recibiendo bytes del payload y checksum final.    */
} CommFsmState_t;

/* -------------------------------------------------------------------------
 * Implementacion de las funciones publicas
 * ---------------------------------------------------------------------- */

void Comm_InitUART(void) {
    UCSR0A = (1 << U2X0);               /* Modo doble velocidad               */
    UBRR0  = 16;                         /* 115200 baud con U2X y F_CPU=16MHz  */
    UCSR0C = (3 << UCSZ00);             /* 8 bits de datos, sin paridad, 1 stop */
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
}

void Comm_Init(Comm_Protocol *comm, comm_cmd_cb callback) {
    /* Configurar buffer de recepcion */
    comm->rxBuf.buf  = bufRx;
    comm->rxBuf.size = BUFRXSIZE;
    comm->rxBuf.iw   = 0;
    comm->rxBuf.ir   = 0;

    /* Configurar buffer de transmision */
    comm->txBuf.buf  = bufTx;
    comm->txBuf.size = BUFTXSIZE;
    comm->txBuf.iw   = 0;
    comm->txBuf.ir   = 0;

    /* Estado inicial de la FSM y variables de decodificacion */
    comm->onCmd      = callback;
    comm->hdrState   = WAIT_U;
    comm->nBytes     = 0;
    comm->cks        = 0;
    comm->timeOut    = 0;
    comm->payloadIdx = 0;

    Comm_InitUART();
}

void Comm_PutRxByte(Comm_Protocol *comm, uint8_t byteISR) {
    comm->rxBuf.buf[comm->rxBuf.iw++] = byteISR;
    comm->rxBuf.iw &= (comm->rxBuf.size - 1);
}

void Comm_Send(Comm_Protocol *comm, uint8_t cmdId, uint8_t *data, uint8_t nBytes) {
    /* LEN = CMDID (1) + datos (nBytes) + CKS (1) */
    uint8_t totalBytes = nBytes + 2;

    /* Calcular checksum: XOR de todos los campos desde 'U' hasta el ultimo dato */
    uint8_t cks = 'U' ^ 'N' ^ 'E' ^ 'R' ^ totalBytes ^ ':' ^ cmdId;
    for (uint8_t i = 0; i < nBytes; i++) {
        cks ^= data[i];
    }

    /* Encolar encabezado */
    putTxByte(comm, 'U');
    putTxByte(comm, 'N');
    putTxByte(comm, 'E');
    putTxByte(comm, 'R');

    /* Encolar LEN y separador */
    putTxByte(comm, totalBytes);
    putTxByte(comm, ':');

    /* Encolar CMDID y datos */
    putTxByte(comm, cmdId);
    for (uint8_t j = 0; j < nBytes; j++) {
        putTxByte(comm, data[j]);
    }

    /* Encolar checksum */
    putTxByte(comm, cks);
}

void Comm_Process(Comm_Protocol *comm) {
    /* --- TX: transmitir un byte si hay datos pendientes y UART libre --- */
    if (comm->txBuf.iw != comm->txBuf.ir) {
        if (UCSR0A & _BV(UDRE0)) {
            UDR0 = comm->txBuf.buf[comm->txBuf.ir++];
            comm->txBuf.ir &= (comm->txBuf.size - 1);
        }
    }

    /* --- RX: procesar todos los bytes disponibles con la FSM UNER --- */
    while (comm->rxBuf.ir != comm->rxBuf.iw) {

        uint8_t byte = comm->rxBuf.buf[comm->rxBuf.ir++];
        comm->rxBuf.ir &= (comm->rxBuf.size - 1);

        switch ((CommFsmState_t)comm->hdrState) {

            case WAIT_U:
                if (byte == 'U') comm->hdrState = WAIT_N;
                /* Si no es 'U', permanecer en WAIT_U (resincronizacion automatica) */
                break;

            case WAIT_N:
                if (byte == 'N') comm->hdrState = WAIT_E;
                else             comm->hdrState = WAIT_U;
                break;

            case WAIT_E:
                if (byte == 'E') comm->hdrState = WAIT_R;
                else             comm->hdrState = WAIT_U;
                break;

            case WAIT_R:
                if (byte == 'R') comm->hdrState = WAIT_NBYTES;
                else             comm->hdrState = WAIT_U;
                break;

            case WAIT_NBYTES:
                /* Guardar LEN e inicializar el checksum acumulado */
                comm->nBytes     = byte;
                comm->payloadIdx = 0;
                comm->cks        = 'U' ^ 'N' ^ 'E' ^ 'R' ^ byte ^ ':';
                comm->hdrState   = WAIT_COLON;
                break;

            case WAIT_COLON:
                if (byte == ':') comm->hdrState = RECV_PAYLOAD;
                else             comm->hdrState = WAIT_U;
                break;

            case RECV_PAYLOAD:
                /* nBytes: bytes restantes incluyendo CKS final
                 * Cuando nBytes == 1, el byte actual ES el checksum. */
                if (comm->nBytes > 1) {

                    /* Proteccion de desbordamiento de buffer de payload */
                    if (comm->payloadIdx >= COMM_MAX_PAYLOAD) {
                        comm->hdrState = WAIT_U;
                        break;
                    }

                    comm->payload[comm->payloadIdx++] = byte;
                    comm->cks ^= byte;
                    comm->nBytes--;

                } else {
                    /* Byte de checksum: validar e invocar callback */
                    if (comm->cks == byte) {
                        /* payload[0] = CMDID, &payload[1] = datos, payloadIdx-1 = nDatos */
                        comm->onCmd(comm->payload[0],
                                    &comm->payload[1],
                                    comm->payloadIdx - 1);
                    }
                    /* Reiniciar FSM para la proxima trama, valida o no */
                    comm->hdrState = WAIT_U;
                }
                break;
        }
    }
}
