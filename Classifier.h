/**
 * @file    Classifier.h
 * @brief   Clasificador de cajas para cinta transportadora.
 *
 * @details Mantiene una lista ordenada de cajas en transito en la cinta.
 *          Cada caja lleva un campo @p stage que indica el proximo sensor
 *          que se espera la detecte, lo que permite razonar sobre cada caja
 *          de forma independiente y evitar desincronizaciones tipicas de
 *          esquemas con multiples colas FIFO paralelas.
 *
 *          Flujo de uso:
 *          @code
 *          // Al iniciar (o tras RESET):
 *          Classifier_Init(&clas, boxTypes, onTrigger);
 *
 *          // Cuando el simulador mide una nueva caja:
 *          Classifier_NewBox(&clas, tipo);
 *
 *          // Cuando un sensor IR detecta la entrada de una caja:
 *          Classifier_OnSensor(&clas, numSalida);
 *          @endcode
 *
 *          La logica de despacho en Classifier_OnSensor():
 *          1. Busca la caja mas antigua cuyo @p stage coincida con @p outNum.
 *          2. Si el tipo coincide con la salida y el actuador esta libre: dispara y elimina la caja.
 *          3. Si no coincide o el actuador esta ocupado: avanza el @p stage de la caja.
 *          4. Si @p stage supera el ultimo sensor: la caja cae al final de la cinta y se descarta.
 *
 * @author  Luciano
 * @date    Abril 2026
 */

#ifndef CLASSIFIER_H_
#define CLASSIFIER_H_

#include <stdint.h>

/** @brief Cantidad maxima de cajas en transito que puede rastrear el clasificador. */
#define CLASSIFIER_MAX_BOXES    32

/** @brief Numero de sensores IR / salidas en la cinta. */
#define CLASSIFIER_NUM_SENSORS   3

/**
 * @brief Callback que intenta disparar el actuador de la salida @p outNum.
 *
 * @param outNum Numero de salida (0, 1 o 2).
 * @return       1 si el disparo fue exitoso, 0 si el brazo estaba ocupado.
 */
typedef uint8_t (*classifier_trigger_cb)(uint8_t outNum);

/**
 * @brief Representa una caja en transito en la cinta.
 */
typedef struct {
    uint8_t type;   /**< Tipo de caja (altura en cm: 6, 8 o 10).               */
    uint8_t stage;  /**< Proximo sensor esperado (0, 1 o 2).                   */
} classifier_box_t;

/**
 * @brief Contexto del clasificador.
 */
typedef struct {
    classifier_box_t      list[CLASSIFIER_MAX_BOXES]; /**< Lista de cajas en transito (orden de llegada). */
    uint8_t               count;                      /**< Cantidad de cajas actualmente en la lista.     */
    uint8_t               boxType[CLASSIFIER_NUM_SENSORS]; /**< Tipo de caja asignado a cada salida.      */
    classifier_trigger_cb onTrigger;                  /**< Callback para disparar actuadores.             */
} classifier_t;

/**
 * @brief Inicializa el clasificador con las asignaciones de salida.
 *
 * @details Resetea la lista de cajas en transito y guarda los tipos
 *          asignados a cada salida y el callback de disparo.
 *
 * @param clas      Puntero al clasificador a inicializar.
 * @param boxType   Arreglo de CLASSIFIER_NUM_SENSORS elementos con el tipo
 *                  de caja correspondiente a cada salida (ej: {6, 8, 10}).
 * @param onTrigger Callback que sera invocado cuando se deba disparar un actuador.
 */
void Classifier_Init(classifier_t *clas, uint8_t *boxType, classifier_trigger_cb onTrigger);

/**
 * @brief Vacia la lista de cajas en transito sin modificar la configuracion.
 *
 * @details Utlizado al recibir CMD_RESET del simulador para descartar
 *          cualquier caja que estuviera en vuelo.
 *
 * @param clas Puntero al clasificador.
 */
void Classifier_Reset(classifier_t *clas);

/**
 * @brief Registra una nueva caja medida en la estacion de inicio.
 *
 * @details Agrega la caja al final de la lista con @p stage = 0
 *          (esperando el sensor de la salida 0). Si la lista esta llena,
 *          la caja se descarta silenciosamente.
 *
 * @param clas Puntero al clasificador.
 * @param type Tipo de caja recibido via CMD_NEW_BOX (ej: 6, 8 o 10).
 */
void Classifier_NewBox(classifier_t *clas, uint8_t type);

/**
 * @brief Notifica al clasificador que el sensor de la salida @p outNum detecto una caja.
 *
 * @details Busca la caja mas antigua cuyo @p stage coincida con @p outNum y la procesa:
 *          - Si el tipo coincide con @p boxType[outNum] y el actuador acepta el disparo:
 *            la caja es expulsada y eliminada de la lista.
 *          - En cualquier otro caso (tipo incorrecto o brazo ocupado): se avanza el
 *            @p stage para que la caja siga siendo rastreada hacia la proxima salida.
 *          - Si el @p stage supera el ultimo sensor, la caja se descarta (cayo al final).
 *
 * @param clas   Puntero al clasificador.
 * @param outNum Numero de salida cuyo sensor disparo (0, 1 o 2).
 */
void Classifier_OnSensor(classifier_t *clas, uint8_t outNum);

#endif /* CLASSIFIER_H_ */
