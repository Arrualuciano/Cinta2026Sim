/**
 * @file    Actuator.h
 * @brief   Modulo de control de actuadores (pistones/brazos) de la cinta.
 *
 * @details Implementa una maquina de estados (FSM) no bloqueante para manejar
 *          el ciclo completo de un actuador mecanico:
 *
 *          @code
 *          ACT_IDLE --> [Trigger()] --> ACT_WAITING --> ACT_EXTENDING --> ACT_RETRACTING --> ACT_IDLE
 *          @endcode
 *
 *          - @b ACT_WAITING:    Espera un retardo configurable antes de extender
 *                               (permite sincronizar el disparo con la posicion
 *                               fisica de la caja frente al brazo).
 *          - @b ACT_EXTENDING:  Brazo extendido durante @p delayTicks ticks de 2 ms.
 *          - @b ACT_RETRACTING: Brazo retrayendose durante @p delayTicks ticks de 2 ms.
 *
 *          Actuator_Process() debe llamarse cada 2 ms (tick del Timer0).
 *
 * @author  Luciano
 * @date    Abril 2026
 */

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include <stdint.h>

/**
 * @brief Callback de hardware para extender o retraer un brazo.
 *
 * @param armNum Numero de brazo (0, 1 o 2).
 * @param state  1 = extender, 0 = retraer.
 */
typedef void (*actuator_hw_set_ptr)(uint8_t armNum, uint8_t state);

/**
 * @brief Estados internos de la FSM de cada actuador.
 */
typedef enum {
    ACT_IDLE,        /**< Brazo en reposo, listo para disparar.              */
    ACT_WAITING,     /**< Esperando retardo de sincronizacion pre-extension. */
    ACT_EXTENDING,   /**< Brazo en proceso de extension.                     */
    ACT_RETRACTING   /**< Brazo en proceso de retraccion.                    */
} ActuatorState;

/**
 * @brief Contexto de un actuador individual.
 */
typedef struct {
    ActuatorState        state;        /**< Estado actual de la FSM.                      */
    uint8_t              armNum;       /**< Identificador del brazo (0, 1 o 2).           */
    uint16_t             timer;        /**< Contador regresivo en ticks de 2 ms.          */
    uint16_t             delayTicks;   /**< Duracion de cada fase (waiting/extending/retracting) en ticks. */
    actuator_hw_set_ptr  SetActuator;  /**< Funcion de hardware para mover el brazo.      */
} Actuator_t;

/**
 * @brief Inicializa un actuador.
 *
 * @param act          Puntero al actuador a inicializar.
 * @param armNum       Numero de brazo asignado (0, 1 o 2).
 * @param delayMs      Duracion de cada fase (waiting, extending, retracting) en milisegundos.
 * @param SetActuator  Funcion de hardware que controla la salida fisica del brazo.
 */
void Actuator_Init(Actuator_t *act, uint8_t armNum, uint16_t delayMs, actuator_hw_set_ptr SetActuator);

/**
 * @brief Solicita el disparo del actuador.
 *
 * @details Si el actuador esta en @p ACT_IDLE, inicia el ciclo de extension.
 *          Si esta ocupado en cualquier otra fase, el disparo se ignora.
 *
 * @param act Puntero al actuador.
 * @return    1 si el disparo fue aceptado, 0 si el brazo estaba ocupado.
 */
uint8_t Actuator_Trigger(Actuator_t *act);

/**
 * @brief Ejecuta un paso de la FSM del actuador.
 *
 * @details Debe llamarse cada 2 ms desde el tick del Timer0.
 *          Avanza los estados de la FSM y controla el hardware en los
 *          momentos de transicion.
 *
 * @param act Puntero al actuador.
 */
void Actuator_Process(Actuator_t *act);

#endif /* ACTUATOR_H_ */
