/*
 * TP3 - Procesamiento Digital de Señales
 * FFT de 512 / 1024 / 2048 muestras sobre FRDM-MCXN947
 *
 * app_config.h: todos los parámetros "tocables" del laboratorio en un solo lugar.
 */
#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Puerto serie (SCI / LPUART). Se usa la VCOM del MCU-Link (USB de debug, J17),
 * que en la FRDM-MCXN947 es LP_FLEXCOMM4 (P1_8 = RX, P1_9 = TX).
 * OJO: si cambian el baudrate acá, cámbienlo también en la GUI de MATLAB.
 * Con 115200 una trama de 2048 muestras (bypass) tarda ~0,7 s; con 921600
 * baja a ~90 ms.
 * ------------------------------------------------------------------------- */
#define APP_UART            LPUART4
#define APP_UART_BAUD       115200U
#define APP_UART_CLK_FREQ   12000000U   /* FRO 12 MHz -> FLEXCOMM4 */

/* ---------------------------------------------------------------------------
 * ADC: ADC0, canal A2 = pin P4_23 (J8 pin 28 en la FRDM-MCXN947).
 * Rango de entrada 0 .. 3,3 V -> montar la señal sobre un offset de ~1,65 V.
 * ------------------------------------------------------------------------- */
#define APP_ADC             ADC0
#define APP_ADC_IRQn        ADC0_IRQn
#define APP_ADC_IRQHandler  ADC0_IRQHandler
#define APP_ADC_CHANNEL     2U
#define APP_ADC_CMDID       1U
#define APP_ADC_VREF_V      3.3f        /* VDD_ANA usado como referencia */
#define APP_ADC_FULL_SCALE  65536.0f    /* modo alta resolución (16 bits) */

/* ---------------------------------------------------------------------------
 * Base de tiempo de muestreo: CTIMER0 con match 0 + reset del contador.
 * Cada match dispara una conversión del ADC -> fs = f_ctimer / (MR0 + 1).
 * ------------------------------------------------------------------------- */
#define APP_CTIMER          CTIMER0
#define APP_CTIMER_IRQn     CTIMER0_IRQn
#define APP_CTIMER_ID       0U

/* Frecuencias de muestreo que se recorren con SW3 (pulsación larga).
 * Desde la GUI se puede pedir cualquier fs entre APP_FS_MIN y APP_FS_MAX. */
#define APP_FS_LIST         {8000U, 16000U, 22050U, 44100U, 48000U}
#define APP_FS_DEFAULT_IDX  1U          /* 16 kHz */
#define APP_FS_MIN          1000U
#define APP_FS_MAX          100000U

/* Tamaños de ventana (cantidad de muestras del buffer = puntos de la FFT) */
#define APP_N_LIST          {512U, 1024U, 2048U}
#define APP_N_DEFAULT_IDX   0U          /* 512, el pedido por la consigna */
#define APP_N_MAX           2048U

/* Teclas de la placa (activas en bajo) */
#define APP_SW2_GPIO        GPIO0       /* SW2 -> FFT on / bypass */
#define APP_SW2_PIN         23U
#define APP_SW3_GPIO        GPIO0       /* SW3 corto -> N, largo -> fs */
#define APP_SW3_PIN         6U
#define APP_LONG_PRESS_MS   700U
#define APP_DEBOUNCE_MS     30U

/* LED RGB (activos en bajo) */
#define APP_LED_RED_GPIO    GPIO0
#define APP_LED_RED_PIN     10U
#define APP_LED_GREEN_GPIO  GPIO0
#define APP_LED_GREEN_PIN   27U
#define APP_LED_BLUE_GPIO   GPIO1
#define APP_LED_BLUE_PIN    2U

#endif /* APP_CONFIG_H_ */
