/*
 * frame_tx.c - Serialización de la trama (ver formato en frame_tx.h).
 */
#include "frame_tx.h"

#include <string.h>

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)(v >> 24);
}

static void put_f32(uint8_t *p, float f)
{
    uint32_t v;
    (void)memcpy(&v, &f, sizeof(v));
    put_u32(p, v);
}

static uint16_t sum_bytes(uint16_t acc, const uint8_t *p, size_t len)
{
    for (size_t i = 0U; i < len; i++)
    {
        acc = (uint16_t)(acc + p[i]);
    }
    return acc;
}

void frame_build_header(const frame_info_t *info, uint8_t hdr[FRAME_HEADER_SIZE])
{
    (void)memset(hdr, 0, FRAME_HEADER_SIZE);
    hdr[0] = 0xA5U;
    hdr[1] = 0x5AU;
    hdr[2] = 0xC3U;
    hdr[3] = 0x3CU;
    hdr[4] = info->type;
    hdr[5] = info->window;
    hdr[6] = info->flags;
    put_u16(&hdr[8], info->n);
    put_u16(&hdr[10], info->count);
    put_f32(&hdr[12], info->fs);
    put_u16(&hdr[16], info->peak_bin);
    put_f32(&hdr[20], info->peak_amp);
    put_f32(&hdr[24], info->dc);
    put_u32(&hdr[28], info->cyc_fft);
    put_u32(&hdr[32], info->cyc_total);
    put_u32(&hdr[36], info->core_clk);
    put_u32(&hdr[40], info->seq);
}

void frame_send(const frame_info_t *info, const float *data, frame_write_fn_t write)
{
    uint8_t hdr[FRAME_HEADER_SIZE];
    uint8_t chunk[64];
    uint16_t csum;

    frame_build_header(info, hdr);
    write(hdr, FRAME_HEADER_SIZE);
    csum = sum_bytes(0U, &hdr[4], FRAME_HEADER_SIZE - 4U);

    /* Los floats se mandan en tandas de 16 para no hacer una llamada por byte */
    size_t i = 0U;
    while (i < info->count)
    {
        size_t k = 0U;
        while ((k < sizeof(chunk)) && (i < info->count))
        {
            put_f32(&chunk[k], data[i]);
            k += 4U;
            i++;
        }
        write(chunk, k);
        csum = sum_bytes(csum, chunk, k);
    }

    uint8_t tail[2];
    put_u16(tail, csum);
    write(tail, sizeof(tail));
}
