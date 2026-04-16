/**
 * @file    Actuator.c
 * @brief   Implementacion del modulo de control de actuadores (FSM).
 *
 * @details Ciclo completo de un disparo:
 *          1. Actuator_Trigger() acepta el disparo --> ACT_WAITING (si delayTicks > 0).
 *          2. Tras el retardo de espera            --> ACT_EXTENDING + activa hardware.
 *          3. Tras delayTicks ticks en extension   --> ACT_RETRACTING + desactiva hardware.
 *          4. Tras delayTicks ticks en retraccion  --> ACT_IDLE.
 *
 *          Cada tick equivale a 2 ms (Timer0 del sistema).
 *          El retardo hardcodeado de extension/retraccion es de 75 ticks = 150 ms.
 *
 * @author  Luciano
 * @date    Abril 2026
 */

#include <stddef.h>
#include "Actuator.h"

/** @brief Duracion fija de las fases de extension y retraccion en ticks de 2 ms. */
#define ACT_PHASE_TICKS   75u    /* 75 x 2 ms = 150 ms */

void Actuator_Init(Actuator_t *act, uint8_t armNum, uint16_t delayMs,
                   actuator_hw_set_ptr SetActuator) {
    act->armNum      = armNum;
    act->SetActuator = SetActuator;
    act->state       = ACT_IDLE;
    act->timer       = 0;
    act->delayTicks  = delayMs / 2u;   /* Convertir ms a ticks de 2 ms */
}

uint8_t Actuator_Trigger(Actuator_t *act) {
    if (act->state != ACT_IDLE) return 0;   /* Brazo ocupado, rechazar disparo */

    if (act->delayTicks > 0) {
        /* Esperar antes de extender (sincronizacion con posicion de la caja) */
        act->state = ACT_WAITING;
        act->timer = act->delayTicks;
    } else {
        /* Sin retardo: extender inmediatamente */
        act->state = ACT_EXTENDING;
        act->timer = ACT_PHASE_TICKS;
        act->SetActuator(act->armNum, 1);
    }
    return 1;   /* Disparo aceptado */
}

void Actuator_Process(Actuator_t *act) {
    switch (act->state) {

        case ACT_IDLE:
            /* Nada que hacer */
            break;

        case ACT_WAITING:
            if (--act->timer == 0) {
                /* Retardo cumplido: activar brazo y comenzar extension */
                act->state = ACT_EXTENDING;
                act->timer = ACT_PHASE_TICKS;
                act->SetActuator(act->armNum, 1);
            }
            break;

        case ACT_EXTENDING:
            if (--act->timer == 0) {
                /* Extension completa: iniciar retraccion */
                act->SetActuator(act->armNum, 0);
                act->state = ACT_RETRACTING;
                act->timer = ACT_PHASE_TICKS;
            }
            break;

        case ACT_RETRACTING:
            if (--act->timer == 0) {
                /* Retraccion completa: volver a reposo */
                act->state = ACT_IDLE;
            }
            break;
    }
}
