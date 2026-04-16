/**
 * @file    RingBuf.h
 * @brief   Buffer circular (ring buffer) de proposito general.
 *
 * @details Estructura de datos para buffers circulares de tamano fijo.
 *          El tamano DEBE ser potencia de 2 para que el mascareo con
 *          (size - 1) funcione correctamente como reemplazo del modulo.
 *
 *          Escritura (ISR o productor):
 *          @code
 *          buf[iw++] = dato;
 *          iw &= (size - 1);
 *          @endcode
 *
 *          Lectura (main loop o consumidor):
 *          @code
 *          while (ir != iw) {
 *              uint8_t b = buf[ir++];
 *              ir &= (size - 1);
 *          }
 *          @endcode
 *
 * @note    iw e ir son uint8_t, por lo que el tamano maximo util es 128
 *          (potencia de 2, menor que 256).
 *
 * @author  Luciano
 * @date    Abril 2026
 */

#ifndef RINGBUF_H_
#define RINGBUF_H_

#include <stdint.h>

/**
 * @brief Descriptor de un buffer circular.
 *
 * @note  El campo @p buf debe apuntar a un arreglo estatico o global de
 *        exactamente @p size bytes. @p size debe ser potencia de 2.
 */
typedef struct {
    uint8_t *buf;   /**< Puntero al arreglo de almacenamiento.          */
    uint8_t  iw;    /**< Indice de escritura (write index).              */
    uint8_t  ir;    /**< Indice de lectura  (read  index).              */
    uint8_t  size;  /**< Capacidad del buffer en bytes (potencia de 2). */
} RingBuf_t;

#endif /* RINGBUF_H_ */
