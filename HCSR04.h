/**
 * @file    HCSR04.h
 * @brief   Driver para sensor ultrasonico HC-SR04.
 *
 * @details Implementa la secuencia de medicion mediante una FSM no bloqueante:
 *
 *          @code
 *          HCSR_IDLE
 *            --> [usuario setea state = HCSR_TRIGGER]
 *          HCSR_TRIGGER   : emite pulso trigger de 10 us
 *          HCSR_TRIG_WAIT : espera fin del pulso trigger
 *          HCSR_ECHO_WH   : espera flanco rising del pin ECHO
 *          HCSR_ECHO_WL   : mide duracion del pulso ECHO (falling edge)
 *          HCSR_READY     : distance contiene el resultado en cm
 *          HCSR_ERROR     : timeout (objeto muy lejos o sin respuesta)
 *          @endcode
 *
 *          El driver es independiente del hardware: las funciones de acceso
 *          al pin y al temporizador se inyectan como punteros a funcion en
 *          HCSR04_Init().
 *
 *          Conversion: distancia_cm = tiempo_echo_us / 58
 *
 * @author  Usuario Principal
 * @date    Abril 2026
 */

#ifndef HCSR04_H_
#define HCSR04_H_

/** @brief Timeout en microsegundos para el pin ECHO (equivale a ~6,5 m). */
#define HCSR04_TIMEOUT_US  38000

#include <stdint.h>

/**
 * @brief Estados de la FSM del sensor HC-SR04.
 */
typedef enum {
    HCSR_IDLE,       /**< Sensor inactivo, esperando solicitud de medicion.     */
    HCSR_TRIGGER,    /**< Iniciando pulso de trigger (>= 10 us).                */
    HCSR_TRIG_WAIT,  /**< Esperando que el pulso de trigger finalice.           */
    HCSR_ECHO_WH,    /**< Esperando flanco rising del pin ECHO.                 */
    HCSR_ECHO_WL,    /**< Midiendo duracion del pulso ECHO (flanco falling).    */
    HCSR_READY,      /**< Medicion completa; @p distance contiene el resultado. */
    HCSR_ERROR       /**< Timeout: el sensor no respondio dentro del plazo.     */
} hcsr_state;

/**
 * @brief Funcion de escritura del pin TRIGGER.
 * @param state 1 = pin alto, 0 = pin bajo.
 */
typedef void (*hcsr_write_ptr)(uint8_t state);

/**
 * @brief Funcion de lectura del pin ECHO.
 * @return 1 si el pin esta en alto, 0 si esta en bajo.
 */
typedef uint8_t (*hcsr_read_ptr)(void);

/**
 * @brief Funcion que retorna el tiempo actual en microsegundos.
 * @return Tiempo actual en us (puede ser libre-corriente, se usa diferencia).
 */
typedef uint32_t (*hcsr_get_time_ptr)(void);

/**
 * @brief Contexto del driver HC-SR04.
 */
typedef struct {
    hcsr_write_ptr    trigger_write; /**< Funcion para controlar el pin TRIGGER.     */
    hcsr_read_ptr     echo_read;     /**< Funcion para leer el pin ECHO.             */
    hcsr_get_time_ptr get_us;        /**< Funcion que retorna el tiempo en us.       */
    hcsr_state        state;         /**< Estado actual de la FSM.                   */
    uint32_t          t_ref;         /**< Marca de tiempo de referencia.             */
    uint32_t          timeout;       /**< Timeout maximo de espera en us.            */
    uint16_t          distance;      /**< Distancia medida en cm (valida en HCSR_READY). */
} HCSR04_Setting_t;

/**
 * @brief Inicializa el driver HC-SR04.
 *
 * @param setting       Puntero a la instancia HCSR04_Setting_t.
 * @param trigger       Funcion de escritura del pin TRIGGER.
 * @param echo          Funcion de lectura del pin ECHO.
 * @param timer         Funcion que retorna el tiempo en microsegundos.
 */
void HCSR04_Init(HCSR04_Setting_t *setting,
                 hcsr_write_ptr trigger,
                 hcsr_read_ptr  echo,
                 hcsr_get_time_ptr timer);

/**
 * @brief Ejecuta un paso de la FSM del sensor (no bloqueante).
 *
 * @details Debe llamarse frecuentemente desde el main loop.
 *          Para iniciar una medicion, setear @p setting->state = HCSR_TRIGGER
 *          antes de llamar a esta funcion.
 *          Al terminar, @p setting->state sera HCSR_READY o HCSR_ERROR.
 *
 * @param setting Puntero a la instancia HCSR04_Setting_t.
 */
void HCSR04_Process(HCSR04_Setting_t *setting);

#endif /* HCSR04_H_ */
