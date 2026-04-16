/**
 * @file    main.c
 * @brief   Aplicacion principal del clasificador de cajas en cinta transportadora.
 *
 * @details Integra los modulos Comm, SimuCinta, Actuator, Classifier y HCSR04
 *          sobre un ATmega328P a 16 MHz.
 *
 *          Arquitectura de tiempo:
 *          - Timer0 (CS02, prescaler 256): genera un tick cada 2 ms via ISR.
 *            Dentro del tick se ejecutan: On2ms, SimuCinta_Process y Actuator_Process.
 *          - Timer1 (CS11, prescaler 8, free-running): base de tiempo en us
 *            para el sensor HC-SR04 (resolucion: TCNT1/2 us @ 16 MHz / 8).
 *
 *          Pinout relevante:
 *          - PB5: LED de heartbeat (toggle cada 100 ms).
 *          - PD2: TRIGGER del sensor HC-SR04 (salida).
 *          - PD3: ECHO del sensor HC-SR04 (entrada).
 *          - USART0: comunicacion serie con el simulador (115200 baud, 8N1).
 *
 *          Protocolo con el simulador (via UART0):
 *          - Se usa el modulo Comm con protocolo UNER.
 *          - SimuCinta actua como capa de traduccion entre UNER y los callbacks
 *            de la aplicacion (OnNewBox, OnIRSensor).
 *
 * @author  Luciano
 * @date    Abril 2026
 */

/* -------------------------------------------------------------------------
 * Definicion de F_CPU antes de cualquier include de avr/
 * ---------------------------------------------------------------------- */
#define F_CPU  16000000u

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/common.h>
#include "HCSR04.h"
#include "Comm.h"
#include "simucinta.h"
#include "Actuator.h"
#include "Classifier.h"

/* -------------------------------------------------------------------------
 * Prototipos de funciones locales
 * ---------------------------------------------------------------------- */
static void     InitTMR0(void);
static void     InitTMR1(void);
static void     On2ms(void);

/* Puentes de hardware para HCSR04 */
static void     hw_trigger_write(uint8_t state);
static uint8_t  hw_echo_read(void);
static uint32_t hw_get_us(void);

/* Puente de hardware para Actuator */
static void hw_Act_Set(uint8_t armNum, uint8_t state);

/* Callbacks de la aplicacion */
static void    OnCommCmd(uint8_t cmdId, uint8_t *payload, uint8_t nBytes);
static void    OnNewBox(uint8_t boxType);
static void    OnIRSensor(uint8_t outNum, uint8_t irState);
static uint8_t OnActuatorTrigger(uint8_t outNum);

/* -------------------------------------------------------------------------
 * Variables globales
 * ---------------------------------------------------------------------- */

/** @brief Contador regresivo para generar el heartbeat del LED cada 100 ms (50 ticks x 2 ms). */
static uint8_t time100ms;

/** @brief Instancia del sensor ultrasonico HC-SR04. */
static HCSR04_Setting_t sensor;

/** @brief Instancia del protocolo de comunicacion UNER. */
static Comm_Protocol comm;

/** @brief Instancia del modulo de simulacion de cinta. */
static SimuCinta_t simu;

/** @brief Actuadores fisicos de las tres salidas. */
static Actuator_t act0, act1, act2;

/** @brief Instancia del clasificador de cajas. */
static classifier_t classifier;

/**
 * @brief Arreglo de punteros a los actuadores, indexado por numero de salida.
 *
 * @details Permite acceder al actuador de la salida N en tiempo O(1) desde
 *          OnActuatorTrigger() y OnIRSensor().
 */
static Actuator_t *actuators[3];

/**
 * @brief Indica si el clasificador fue inicializado con los tipos reales del simulador.
 *
 * @details Se pone en 0 al recibir CMD_RESET (el simulador garantiza cinta vacia).
 *          Se pone en 1 cuando llega el primer CMD_NEW_BOX tras un reset, momento en
 *          que se llama a Classifier_Init() con los tipos reales ya conocidos.
 */
static uint8_t cintaLista = 0;

/* =========================================================================
 * Rutinas de servicio de interrupcion (ISR)
 * ====================================================================== */

/**
 * @brief ISR del comparador A del Timer0: tick de 2 ms.
 *
 * @details Avanza OCR0A en 125 cuentas (125 x 16 us = 2 ms con prescaler 256)
 *          y setea el flag de tick en GPIOR0 para el main loop.
 */
ISR(TIMER0_COMPA_vect) {
    OCR0A += 125;
    GPIOR0 |= _BV(GPIOR00);
}

/**
 * @brief ISR de recepcion UART: transfiere el byte recibido al buffer circular.
 *
 * @details Llama a Comm_PutRxByte() que es la unica operacion segura desde ISR
 *          sobre la estructura Comm_Protocol (escribe en el ring buffer de RX).
 */
ISR(USART_RX_vect) {
    Comm_PutRxByte(&comm, UDR0);
}

/* =========================================================================
 * Inicializacion de perifericos
 * ====================================================================== */

/**
 * @brief Configura Timer0 para generar un tick periodico cada 2 ms.
 *
 * @details Modo CTC libre con OCR0A como valor de comparacion.
 *          Prescaler 256: 16 MHz / 256 = 62500 Hz --> tick cada 125 cuentas = 2 ms.
 *          Se usa la tecnica de "sumar al OCR0A" en la ISR para evitar deriva.
 */
static void InitTMR0(void) {
    TCCR0A = 0;
    TIFR0  = TIFR0;          /* Limpiar flags pendientes */
    OCR0A  = 124;            /* Primera comparacion en 125 cuentas */
    TIMSK0 = (1 << OCIE0A);  /* Habilitar interrupcion por comparacion A */
    TCCR0B = (1 << CS02);    /* Prescaler 256, iniciar contador */
}

/**
 * @brief Configura Timer1 en modo free-running con prescaler 8.
 *
 * @details Provee la base de tiempo para el driver HC-SR04.
 *          Resolucion: 1 cuenta = 0.5 us (16 MHz / 8 = 2 MHz).
 *          hw_get_us() retorna TCNT1 / 2 para obtener microsegundos.
 */
static void InitTMR1(void) {
    TCCR1A = 0;
    TCCR1B = (1 << CS11);    /* Prescaler 8, free running */
}

/* =========================================================================
 * Puentes de hardware (hardware abstraction layer)
 * ====================================================================== */

/**
 * @brief Controla el pin TRIGGER del HC-SR04 (PD2).
 * @param state 1 = pin alto, 0 = pin bajo.
 */
static void hw_trigger_write(uint8_t state) {
    if (state) PORTD |=  (1 << PD2);
    else       PORTD &= ~(1 << PD2);
}

/**
 * @brief Lee el pin ECHO del HC-SR04 (PD3).
 * @return 1 si el pin esta en alto, 0 si esta en bajo.
 */
static uint8_t hw_echo_read(void) {
    return (PIND & (1 << PD3)) ? 1 : 0;
}

/**
 * @brief Retorna el tiempo actual en microsegundos usando Timer1.
 *
 * @details TCNT1 corre a 2 MHz (prescaler 8), por lo tanto 1 cuenta = 0.5 us.
 *          Dividir por 2 convierte a microsegundos.
 *
 * @return Tiempo en us (libre-corriente, usar diferencias para medicion).
 */
static uint32_t hw_get_us(void) {
    return TCNT1 / 2;
}

/**
 * @brief Controla un brazo actuador via el simulador (puente Actuator -> SimuCinta).
 *
 * @details Construye el bitmask de mascara y posicion para CMD_SET_ARM y lo
 *          envia al simulador a traves de SimuCinta_SetArm().
 *
 * @param armNum Numero de brazo (0, 1 o 2).
 * @param state  1 = extender, 0 = retraer.
 */
static void hw_Act_Set(uint8_t armNum, uint8_t state) {
    uint8_t mask = (1 << armNum);      /* Bitmask: que brazo cambiar       */
    uint8_t pos  = (state ? mask : 0); /* Bitmask: posicion deseada        */
    SimuCinta_SetArm(&simu, mask, pos);
}

/* =========================================================================
 * Callbacks de la aplicacion
 * ====================================================================== */

/**
 * @brief Callback central: despacha todos los comandos recibidos del simulador.
 *
 * @details Se invoca desde Comm_Process() cada vez que se recibe una trama
 *          UNER valida. Primero delega en SimuCinta_OnCmd() para actualizar
 *          el estado del modulo SimuCinta, luego aplica la logica de la
 *          aplicacion segun el comando:
 *
 *          - CMD_START (0x50): si el clasificador ya estaba activo, actualiza
 *            los tipos de salida por si el usuario los modifico antes de
 *            reanudar. Si es el primer arranque post-RESET (cintaLista = 0),
 *            OnNewBox() se encargara de inicializar el clasificador en la
 *            primera caja.
 *
 *          - CMD_STOP (0x51): PAUSA. Las cajas siguen fisicamente en la cinta;
 *            NO se resetea el clasificador para preservar el rastreo.
 *
 *          - CMD_RESET (0x53): reset total. El simulador garantiza cinta vacia,
 *            por lo que se borra la lista de cajas y se fuerza re-inicializacion.
 *
 * @param cmdId   Identificador del comando recibido.
 * @param payload Puntero al arreglo de datos del comando.
 * @param nBytes  Cantidad de bytes de datos en @p payload.
 */
static void OnCommCmd(uint8_t cmdId, uint8_t *payload, uint8_t nBytes) {
    SimuCinta_OnCmd(&simu, cmdId, payload, nBytes);

    switch (cmdId) {

        case 0x50:  /* CMD_START: inicio o reanudacion de la cinta */
            /*
             * Si el clasificador ya esta activo (cinta estaba corriendo),
             * actualizar boxType por si el usuario cambio las asignaciones
             * antes de reanudar. Si cintaLista == 0 (post-RESET), el primer
             * OnNewBox() llamara a Classifier_Init() con los tipos correctos.
             */
            if (cintaLista) {
                for (uint8_t i = 0; i < 3; i++) {
                    classifier.boxType[i] = simu.boxType[i];
                }
            }
            break;

        case 0x51:  /* CMD_STOP: pausa (cajas fisicamente siguen en la cinta) */
            /* No resetear: preservar el rastreo para reanudar correctamente */
            break;

        case 0x53:  /* CMD_RESET: cinta vacia, borrar estado del clasificador */
            cintaLista = 0;
            Classifier_Reset(&classifier);
            break;
    }
}

/**
 * @brief Callback: el simulador midio una nueva caja en la estacion de entrada.
 *
 * @details En el primer llamado post-RESET, inicializa el clasificador con los
 *          tipos de salida reales (ya disponibles en simu.boxType tras CMD_START).
 *          Luego registra la caja en el clasificador.
 *
 * @param boxType Tipo de caja medido (ej: 6, 8 o 10 cm).
 */
static void OnNewBox(uint8_t boxType) {
    if (!cintaLista) {
        Classifier_Init(&classifier, simu.boxType, OnActuatorTrigger);
        cintaLista = 1;
    }
    Classifier_NewBox(&classifier, boxType);
}

/**
 * @brief Callback: el sensor IR de la salida @p outNum detecto un cambio de estado.
 *
 * @details Solo reacciona al flanco de entrada (irState == 1) para notificar
 *          al clasificador que una caja llego al sensor de la salida @p outNum.
 *
 * @param outNum  Numero de salida/sensor (0, 1 o 2).
 * @param irState 1 = caja entrando al sensor, 0 = caja saliendo del sensor.
 */
static void OnIRSensor(uint8_t outNum, uint8_t irState) {
    if (irState == 1 && outNum < 3) {
        Classifier_OnSensor(&classifier, outNum);
    }
}

/**
 * @brief Callback del clasificador: solicita el disparo del actuador @p outNum.
 *
 * @param outNum Numero de salida (0, 1 o 2).
 * @return       1 si el actuador acepto el disparo, 0 si estaba ocupado.
 */
static uint8_t OnActuatorTrigger(uint8_t outNum) {
    if (outNum < 3) return Actuator_Trigger(actuators[outNum]);
    return 0;
}

/* =========================================================================
 * Tarea periodica de 2 ms
 * ====================================================================== */

/**
 * @brief Ejecuta las tareas periodicas del sistema cada 2 ms.
 *
 * @details Limpia el flag de tick (GPIOR0) e implementa un contador para
 *          el heartbeat del LED (toggle cada 100 ms = 50 ticks x 2 ms).
 */
static void On2ms(void) {
    GPIOR0 &= ~_BV(GPIOR00);
    if (--time100ms == 0) {
        time100ms = 50;
        PINB = (1 << PINB5);   /* Toggle LED PB5 */
    }
}

/* =========================================================================
 * Punto de entrada
 * ====================================================================== */

/**
 * @brief Inicializa el sistema y ejecuta el super-loop.
 *
 * @details Secuencia de inicializacion:
 *          1. Deshabilitar interrupciones globales.
 *          2. Configurar DDR de pines de I/O.
 *          3. Inicializar Timer0 (tick 2 ms) y Timer1 (timebase us).
 *          4. Inicializar modulos: Comm -> HCSR04 -> SimuCinta -> Actuators -> Classifier.
 *          5. Habilitar interrupciones globales.
 *          6. Super-loop: procesar comunicacion, HCSR04 y tareas de 2 ms.
 *
 * @return  No retorna (loop infinito).
 */
int main(void) {
    cli();

    time100ms = 50;

    /* Configurar pines */
    DDRB =  (1 << PORTB5);      /* PB5: LED heartbeat (salida)       */
    DDRD |=  (1 << PD2);        /* PD2: TRIGGER HC-SR04 (salida)     */
    DDRD &= ~(1 << PD3);        /* PD3: ECHO    HC-SR04 (entrada)    */

    /* Tipos de salida por defecto hasta recibir CMD_START */
    uint8_t defaultBoxType[3] = {0, 0, 0};

    /* Tabla de actuadores indexada por numero de salida */
    actuators[0] = &act0;
    actuators[1] = &act1;
    actuators[2] = &act2;

    /* Inicializar perifericos y modulos */
    InitTMR0();
    InitTMR1();
    Comm_Init(&comm, OnCommCmd);
    HCSR04_Init(&sensor, hw_trigger_write, hw_echo_read, hw_get_us);
    SimuCinta_Init(&simu, &comm, OnNewBox, OnIRSensor);
    Actuator_Init(&act0, 0, 50, hw_Act_Set);
    Actuator_Init(&act1, 1, 50, hw_Act_Set);
    Actuator_Init(&act2, 2, 50, hw_Act_Set);
    Classifier_Init(&classifier, defaultBoxType, OnActuatorTrigger);

    sei();

    /* ---- Super-loop ---- */
    while (1) {

        /* Tareas de 2 ms: ejecutar solo cuando el tick del Timer0 esta activo */
        if (GPIOR0 & _BV(GPIOR00)) {
            On2ms();
            SimuCinta_Process(&simu);
            Actuator_Process(&act0);
            Actuator_Process(&act1);
            Actuator_Process(&act2);
        }

        /* Tareas libres (no periodicidad estricta) */
        Comm_Process(&comm);
        HCSR04_Process(&sensor);

        /* Procesar resultado del sensor ultrasonico */
        if (sensor.state == HCSR_READY) {
            /* sensor.distance contiene la distancia medida en cm */
            sensor.state = HCSR_IDLE;
        }

        if (sensor.state == HCSR_ERROR) {
            /* El sensor no respondio dentro del timeout */
            sensor.state = HCSR_IDLE;
        }
    }
}
