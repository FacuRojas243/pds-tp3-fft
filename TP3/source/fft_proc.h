/*
 * fft_proc.h - Procesamiento: ventana + FFT compleja (CMSIS-DSP) + módulo.
 *
 * Este módulo NO depende de ningún periférico: recibe un buffer de entrada
 * en volts y escribe el espectro de amplitud en un buffer de salida distinto.
 * Por eso se puede compilar y probar también en la PC (ver tools/host_test).
 */
#ifndef FFT_PROC_H_
#define FFT_PROC_H_

#include <stdbool.h>
#include <stdint.h>

#define FFT_PROC_N_MAX 2048U

typedef enum
{
    FFT_WIN_RECT = 0, /* sin ventana (rectangular) */
    FFT_WIN_HANN = 1, /* Hann periódica */
} fft_window_t;

typedef struct
{
    uint16_t peak_bin;   /* índice k del máximo (sin contar DC) */
    float peak_amp;      /* amplitud de pico en ese bin [V] */
    float dc;            /* valor medio del bloque [V] */
    uint32_t cyc_fft;    /* ciclos de CPU de arm_cfft_f32 solamente */
    uint32_t cyc_total;  /* ciclos de todo el procesamiento */
} fft_result_t;

/* Configura el tamaño N (512, 1024 o 2048) y recalcula la ventana.
 * Devuelve false si N no está soportado. */
bool fft_proc_set_size(uint16_t n);

/* Elige la ventana (recalcula la tabla). */
void fft_proc_set_window(fft_window_t w);

/* true -> se resta el valor medio antes de la FFT (saca el pico de DC
 * que mete el offset de 1,65 V de la entrada). */
void fft_proc_set_remove_dc(bool remove);

uint16_t fft_proc_get_size(void);
fft_window_t fft_proc_get_window(void);
bool fft_proc_get_remove_dc(void);

/* Contador de ciclos (en el micro: DWT->CYCCNT). NULL -> no mide. */
void fft_proc_set_cycle_counter(uint32_t (*get_cycles)(void));

/*
 * Calcula el espectro de amplitud de un lado.
 *   x       : N muestras de entrada [V] (no se modifica)
 *   mag_out : N/2 bins de salida [V de pico]; bin k <-> f = k*fs/N
 *   res     : información extra (pico, DC, ciclos). Puede ser NULL.
 * Devuelve la cantidad de bins escritos (N/2).
 */
uint16_t fft_proc_spectrum(const float *x, float *mag_out, fft_result_t *res);

#endif /* FFT_PROC_H_ */
