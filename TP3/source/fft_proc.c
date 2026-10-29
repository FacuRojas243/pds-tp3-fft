/*
 * fft_proc.c - Ventana + FFT compleja radix-2/4 de CMSIS-DSP + módulo.
 *
 * Pasos (los mismos de la clase "DFT y FFT", diapositivas CORTEX M4):
 *   1) x[n] -> (opcional) se le resta la media -> se multiplica por w[n]
 *   2) se arma el buffer complejo intercalado {Re, Im, Re, Im, ...} con Im = 0
 *   3) arm_cfft_f32()   : FFT in situ, con reordenamiento "bit inverso"
 *   4) arm_cmplx_mag_f32: |X(k)| para k = 0 .. N/2-1
 *   5) escalado a amplitud de pico:  A(k) = 2|X(k)| / (N * CG)   (k > 0)
 *                                    A(0) =  |X(0)| / (N * CG)
 *      CG = ganancia coherente de la ventana (1 rectangular, 0,5 Hann)
 *   6) arm_max_f32 para encontrar el bin de mayor amplitud
 */
#include "fft_proc.h"

#include <math.h>
#include <stddef.h>

#include "arm_math.h"
#include "arm_const_structs.h"

/* Buffer de trabajo complejo (2*N floats). Es interno: la entrada y la
 * salida que ve la aplicación son buffers distintos. */
static float32_t s_cbuf[2U * FFT_PROC_N_MAX];
static float32_t s_win[FFT_PROC_N_MAX];

static uint16_t s_n = 512U;
static fft_window_t s_win_type = FFT_WIN_RECT;
static bool s_remove_dc = true;
static float32_t s_cg = 1.0f; /* ganancia coherente = mean(w) */
static const arm_cfft_instance_f32 *s_inst = &arm_cfft_sR_f32_len512;
static uint32_t (*s_get_cycles)(void) = NULL;

static inline uint32_t now_cycles(void)
{
    return (s_get_cycles != NULL) ? s_get_cycles() : 0U;
}

static void build_window(void)
{
    float32_t sum = 0.0f;
    for (uint32_t i = 0U; i < s_n; i++)
    {
        float32_t w = 1.0f;
        if (s_win_type == FFT_WIN_HANN)
        {
            /* Hann periódica: w[n] = 0,5 - 0,5 cos(2*pi*n/N) */
            w = 0.5f - 0.5f * cosf(2.0f * PI * (float32_t)i / (float32_t)s_n);
        }
        s_win[i] = w;
        sum += w;
    }
    s_cg = sum / (float32_t)s_n;
}

bool fft_proc_set_size(uint16_t n)
{
    switch (n)
    {
        case 512U:
            s_inst = &arm_cfft_sR_f32_len512;
            break;
        case 1024U:
            s_inst = &arm_cfft_sR_f32_len1024;
            break;
        case 2048U:
            s_inst = &arm_cfft_sR_f32_len2048;
            break;
        default:
            return false;
    }
    s_n = n;
    build_window();
    return true;
}

void fft_proc_set_window(fft_window_t w)
{
    s_win_type = w;
    build_window();
}

void fft_proc_set_remove_dc(bool remove)
{
    s_remove_dc = remove;
}

uint16_t fft_proc_get_size(void)
{
    return s_n;
}

fft_window_t fft_proc_get_window(void)
{
    return s_win_type;
}

bool fft_proc_get_remove_dc(void)
{
    return s_remove_dc;
}

void fft_proc_set_cycle_counter(uint32_t (*get_cycles)(void))
{
    s_get_cycles = get_cycles;
}

uint16_t fft_proc_spectrum(const float *x, float *mag_out, fft_result_t *res)
{
    const uint32_t n     = s_n;
    const uint32_t nbins = n / 2U;
    float32_t mean;
    float32_t peak;
    uint32_t peak_idx;

    const uint32_t t0 = now_cycles();

    /* 1) y 2): media, resta de DC, ventana y armado del buffer complejo */
    arm_mean_f32(x, n, &mean);
    const float32_t offset = s_remove_dc ? mean : 0.0f;
    for (uint32_t i = 0U; i < n; i++)
    {
        s_cbuf[2U * i]      = (x[i] - offset) * s_win[i];
        s_cbuf[2U * i + 1U] = 0.0f;
    }

    /* 3) FFT compleja in situ, directa (ifftFlag = 0), con bit reversal (1) */
    const uint32_t t1 = now_cycles();
    arm_cfft_f32(s_inst, s_cbuf, 0U, 1U);
    const uint32_t t2 = now_cycles();

    /* 4) módulo de los primeros N/2 bins (la otra mitad es simétrica) */
    arm_cmplx_mag_f32(s_cbuf, mag_out, nbins);

    /* 5) escalado a volts de pico */
    const float32_t k_ac = 2.0f / ((float32_t)n * s_cg);
    mag_out[0] *= 0.5f * k_ac;
    arm_scale_f32(&mag_out[1], k_ac, &mag_out[1], nbins - 1U);

    /* 6) pico (se excluye el bin 0 = DC) */
    arm_max_f32(&mag_out[1], nbins - 1U, &peak, &peak_idx);

    const uint32_t t3 = now_cycles();

    if (res != NULL)
    {
        res->peak_bin  = (uint16_t)(peak_idx + 1U);
        res->peak_amp  = peak;
        res->dc        = mean;
        res->cyc_fft   = t2 - t1;
        res->cyc_total = t3 - t0;
    }
    return (uint16_t)nbins;
}
