/*
 * frame_tx.h - Trama binaria que se manda por el puerto serie a la PC.
 *
 * Todo en little-endian. Formato (44 bytes de cabecera + datos + checksum):
 *
 *  off  tipo      campo
 *  ---  --------  -----------------------------------------------------------
 *   0   u8[4]     sync = A5 5A C3 3C
 *   4   u8        tipo: 0 = bypass (muestras en el tiempo), 1 = espectro FFT
 *   5   u8        ventana: 0 = rectangular, 1 = Hann
 *   6   u8        flags: bit0 = se restó la media (DC) antes de la FFT
 *   7   u8        reservado (0)
 *   8   u16       N: tamaño del bloque adquirido / puntos de la FFT
 *  10   u16       count: cantidad de floats que siguen (N/2 o N)
 *  12   f32       fs real [Hz] (la que sale del divisor del CTIMER)
 *  16   u16       bin del pico (sólo FFT)
 *  18   u16       reservado (0)
 *  20   f32       amplitud del pico [V]
 *  24   f32       valor medio del bloque [V]
 *  28   u32       ciclos de CPU de arm_cfft_f32
 *  32   u32       ciclos de CPU de todo el procesamiento
 *  36   u32       frecuencia del núcleo [Hz] (para pasar ciclos a segundos)
 *  40   u32       número de secuencia de trama
 *  44   f32[count] datos: espectro [V pico] o muestras [V]
 *  ..   u16       checksum = suma de todos los bytes desde el offset 4 hasta
 *                 el último byte de datos, módulo 65536
 */
#ifndef FRAME_TX_H_
#define FRAME_TX_H_

#include <stddef.h>
#include <stdint.h>

#define FRAME_HEADER_SIZE 44U
#define FRAME_TYPE_TIME   0U
#define FRAME_TYPE_FFT    1U

typedef struct
{
    uint8_t type;
    uint8_t window;
    uint8_t flags;
    uint16_t n;
    uint16_t count;
    float fs;
    uint16_t peak_bin;
    float peak_amp;
    float dc;
    uint32_t cyc_fft;
    uint32_t cyc_total;
    uint32_t core_clk;
    uint32_t seq;
} frame_info_t;

/* Función que manda bytes por el canal (en el micro: LPUART_WriteBlocking). */
typedef void (*frame_write_fn_t)(const uint8_t *data, size_t len);

/* Arma la cabecera en 'hdr' (FRAME_HEADER_SIZE bytes). */
void frame_build_header(const frame_info_t *info, uint8_t hdr[FRAME_HEADER_SIZE]);

/* Arma y manda la trama completa: cabecera + count floats + checksum. */
void frame_send(const frame_info_t *info, const float *data, frame_write_fn_t write);

#endif /* FRAME_TX_H_ */
