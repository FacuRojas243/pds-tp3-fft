/*
 * host_test.c - Corre fft_proc.c y frame_tx.c en la PC (sin placa).
 *
 * Genera una senoidal + offset como la que vería el ADC (cuantizada a 16 bits
 * en 0..3,3 V), la procesa igual que el firmware y escribe en un archivo las
 * tramas binarias que mandaría la placa. Sirve para:
 *   - comparar el espectro de CMSIS-DSP contra numpy / MATLAB
 *   - probar el parser de la GUI sin tener la placa enchufada
 *
 * Uso: host_test <salida.bin> <N> <fs> <f0> <amp> <ventana 0|1> <modo 0|1>
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fft_proc.h"
#include "frame_tx.h"

static FILE *s_out;

static void file_write(const uint8_t *data, size_t len)
{
    fwrite(data, 1, len, s_out);
}

int main(int argc, char **argv)
{
    if (argc < 8)
    {
        fprintf(stderr, "uso: %s out.bin N fs f0 amp win mode\n", argv[0]);
        return 1;
    }
    const char *path  = argv[1];
    const int n       = atoi(argv[2]);
    const double fs   = atof(argv[3]);
    const double f0   = atof(argv[4]);
    const double amp  = atof(argv[5]);
    const int win     = atoi(argv[6]);
    const int mode    = atoi(argv[7]);

    static float x[FFT_PROC_N_MAX];
    static float y[FFT_PROC_N_MAX];

    if (!fft_proc_set_size((uint16_t)n))
    {
        fprintf(stderr, "N no soportado\n");
        return 1;
    }
    fft_proc_set_window(win ? FFT_WIN_HANN : FFT_WIN_RECT);
    fft_proc_set_remove_dc(true);

    for (int i = 0; i < n; i++)
    {
        double v   = 1.65 + amp * sin(2.0 * M_PI * f0 * (double)i / fs);
        long raw   = lround(v / 3.3 * 65536.0);
        raw        = raw < 0 ? 0 : (raw > 65535 ? 65535 : raw);
        x[i]       = (float)raw * (3.3f / 65536.0f);
    }

    frame_info_t info;
    memset(&info, 0, sizeof(info));
    info.n        = (uint16_t)n;
    info.fs       = (float)fs;
    info.window   = (uint8_t)win;
    info.flags    = 1U;
    info.core_clk = 150000000U;
    info.seq      = 7U;

    if (mode)
    {
        fft_result_t r;
        info.type      = FRAME_TYPE_FFT;
        info.count     = fft_proc_spectrum(x, y, &r);
        info.peak_bin  = r.peak_bin;
        info.peak_amp  = r.peak_amp;
        info.dc        = r.dc;
        printf("peak_bin=%u f=%.2f Hz amp=%.5f V dc=%.5f V\n", r.peak_bin, r.peak_bin * fs / n, r.peak_amp,
               r.dc);
    }
    else
    {
        info.type  = FRAME_TYPE_TIME;
        info.count = (uint16_t)n;
        memcpy(y, x, sizeof(float) * (size_t)n);
    }

    s_out = fopen(path, "wb");
    /* basura antes de la trama para probar la resincronización del parser */
    const char *junk = "TP3 PDS - texto de arranque\r\n\xA5\x5A";
    fwrite(junk, 1, strlen(junk), s_out);
    frame_send(&info, y, file_write);
    fclose(s_out);
    return 0;
}
