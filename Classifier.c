/**
 * @file    Classifier.c
 * @brief   Implementacion del clasificador de cajas para cinta transportadora.
 *
 * @details Logica de clasificacion basada en una unica lista de cajas en transito.
 *          Cada caja lleva su propio campo @p stage (proximo sensor esperado), lo que
 *          permite razonar sobre cada caja en forma independiente y evita las
 *          desincronizaciones tipicas de los esquemas con varias colas FIFO paralelas.
 *
 *          Invariantes de la lista:
 *          - Las cajas estan ordenadas por orden de llegada (la mas antigua en list[0]).
 *          - El campo @p stage de cada caja solo avanza (nunca retrocede).
 *          - Una caja se elimina unicamente cuando es expulsada por un actuador
 *            o cuando supera el ultimo sensor sin ser clasificada.
 *
 * @author  Luciano
 * @date    Abril 2026
 */

#include <stdint.h>
#include "Classifier.h"

/**
 * @brief Elimina la caja en la posicion @p idx desplazando las siguientes.
 *
 * @details Operacion O(n). Mantiene el orden de llegada de las cajas restantes.
 *
 * @param clas Puntero al clasificador.
 * @param idx  Indice (0-based) de la caja a eliminar.
 */
static void removeAt(classifier_t *clas, uint8_t idx) {
    if (idx >= clas->count) return;
    for (uint8_t i = idx; i + 1 < clas->count; i++) {
        clas->list[i] = clas->list[i + 1];
    }
    clas->count--;
}

/* -------------------------------------------------------------------------
 * Implementacion de las funciones publicas
 * ---------------------------------------------------------------------- */

void Classifier_Init(classifier_t *clas, uint8_t *boxType,
                     classifier_trigger_cb onTrigger) {
    for (uint8_t i = 0; i < CLASSIFIER_NUM_SENSORS; i++) {
        clas->boxType[i] = boxType[i];
    }
    clas->count     = 0;
    clas->onTrigger = onTrigger;
}

void Classifier_Reset(classifier_t *clas) {
    clas->count = 0;
}

void Classifier_NewBox(classifier_t *clas, uint8_t type) {
    if (clas->count >= CLASSIFIER_MAX_BOXES) return;   /* Lista llena: descartar */

    clas->list[clas->count].type  = type;
    clas->list[clas->count].stage = 0;    /* Esperando el sensor de la salida 0 */
    clas->count++;
}

void Classifier_OnSensor(classifier_t *clas, uint8_t outNum) {
    if (outNum >= CLASSIFIER_NUM_SENSORS) return;

    /*
     * Buscar la caja mas antigua cuyo proximo sensor esperado sea outNum.
     * La lista esta ordenada por orden de llegada, por lo que la primera
     * coincidencia es fisicamente la que llego al sensor.
     */
    for (uint8_t i = 0; i < clas->count; i++) {

        if (clas->list[i].stage != outNum) continue;

        uint8_t type = clas->list[i].type;

        if (type == clas->boxType[outNum]) {
            /* El tipo coincide con esta salida: intentar disparar el actuador */
            if (clas->onTrigger(outNum)) {
                /* Expulsion exitosa: eliminar la caja de la lista */
                removeAt(clas, i);
                return;
            }
            /*
             * Brazo ocupado: la caja sigue fisicamente por la cinta sin ser
             * expulsada. Caemos al avance de stage para seguirla hacia la
             * proxima salida.
             */
        }

        /* Tipo incorrecto o brazo ocupado: avanzar al proximo sensor */
        clas->list[i].stage++;
        if (clas->list[i].stage >= CLASSIFIER_NUM_SENSORS) {
            /* La caja paso el ultimo sensor sin ser clasificada: descartar */
            removeAt(clas, i);
        }
        return;
    }

    /*
     * Ninguna caja de la lista esperaba este sensor. Posibles causas:
     * - Evento espurio del sensor.
     * - CMD_NEW_BOX perdido o aun no procesado.
     * Se ignora silenciosamente para no corromper el estado del clasificador.
     */
}
