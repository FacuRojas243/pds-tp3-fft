/*
 * TP3 / Laboratorio 3 - Procesamiento Digital de Señales
 * Implementación de la FFT de 512 (1024, 2048) muestras en la FRDM-MCXN947.
 *
 * Flujo:
 *   CTIMER0 (match a fs) --> dispara LPADC0 --> ISR del ADC guarda la muestra
 *   en el buffer de ENTRADA (s_adc_raw). Cuando se juntan N muestras se frena
 *   el timer y el lazo principal:
 *     - modo FFT   : ventana + arm_cfft_f32 + módulo  -> buffer de SALIDA
 *     - modo bypass: copia las muestras sin procesar   -> buffer de SALIDA
 *   y manda el buffer de salida por el puerto serie (LPUART4 / VCOM) en una
 *   trama binaria que grafica la GUI de MATLAB.
 *
 * Teclas:
 *   SW2           : habilita la FFT / bypass
 *   SW3 corto     : cambia N (512 -> 1024 -> 2048 -> 512 ...)
 *   SW3 largo     : cambia fs (8k -> 16k -> 22,05k -> 44,1k -> 48k -> ...)
 * LEDs: verde = FFT activa, rojo = bypass, azul = parpadea con cada trama.
 *
 * Comandos ASCII desde la PC (terminados en '\n'):
 *   N512 | N1024 | N2048   tamaño de ventana
 *   F<hz>                   frecuencia de muestreo (ej. F44100)
 *   M1 | M0                 FFT / bypass
 *   W0 | W1                 ventana rectangular / Hann
 *   D1 | D0                 restar / no restar el valor medio
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_ctimer.h"
#include "fsl_gpio.h"
#include "fsl_lpadc.h"
#include "fsl_lpuart.h"
#include "fsl_spc.h"
#include "fsl_vref.h"

#include "clock_config.h"

#include "arm_math.h"

#include "app_config.h"
#include "app_pins.h"
#include "fft_proc.h"
#include "frame_tx.h"

/*******************************************************************************
 * Tipos y variables
 ******************************************************************************/
typedef enum
{
    MODE_BYPASS = 0,
    MODE_FFT    = 1,
} app_mode_t;

typedef struct
{
    uint16_t n;
    uint32_t fs;
    app_mode_t mode;
    fft_window_t window;
    bool remove_dc;
} app_cfg_t;

static const uint32_t s_fs_list[] = APP_FS_LIST;
static const uint16_t s_n_list[]  = APP_N_LIST;
#define FS_LIST_LEN (sizeof(s_fs_list) / sizeof(s_fs_list[0]))
#define N_LIST_LEN  (sizeof(s_n_list) / sizeof(s_n_list[0]))

/* Buffer de ENTRADA: muestras crudas del ADC (16 bits) */
static volatile uint16_t s_adc_raw[APP_N_MAX];
static volatile uint32_t s_idx;
static volatile uint32_t s_acq_n;
static volatile bool s_block_ready;

/* Entrada en volts y buffer de SALIDA (espectro o copia en bypass) */
static float32_t s_x_in[APP_N_MAX];
static float32_t s_y_out[APP_N_MAX];

static app_cfg_t s_cfg;           /* configuración pedida (teclas / PC) */
static app_cfg_t s_active;        /* configuración del bloque en curso */
static float32_t s_fs_real;       /* fs que realmente sale del CTIMER */
static uint32_t s_seq;

/* SysTick y teclas */
static volatile uint32_t s_ms;
static volatile bool s_ev_sw2;
static volatile bool s_ev_sw3_short;
static volatile bool s_ev_sw3_long;

/* UART: recepción por interrupción con buffer circular del driver */
static lpuart_handle_t s_uart_handle;
static uint8_t s_uart_ring[64];
static char s_cmd[16];
static uint32_t s_cmd_len;

/*******************************************************************************
 * Utilidades
 ******************************************************************************/
static uint32_t cycles_now(void)
{
    return DWT->CYCCNT;
}

static void cycle_counter_init(void)
{
#if defined(DCB)
    DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
#else
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#endif
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void uart_write(const uint8_t *data, size_t len)
{
    (void)LPUART_WriteBlocking(APP_UART, data, len);
}

static void uart_puts(const char *s)
{
    uart_write((const uint8_t *)s, strlen(s));
}

static void leds_update(void)
{
    /* LEDs activos en bajo */
    GPIO_PinWrite(APP_LED_GREEN_GPIO, APP_LED_GREEN_PIN, (s_cfg.mode == MODE_FFT) ? 0U : 1U);
    GPIO_PinWrite(APP_LED_RED_GPIO, APP_LED_RED_PIN, (s_cfg.mode == MODE_BYPASS) ? 0U : 1U);
}

/*******************************************************************************
 * Teclas: antirrebote y pulsación larga en el SysTick (1 ms), así no se
 * pierden aunque el lazo principal esté ocupado transmitiendo.
 ******************************************************************************/
typedef struct
{
    GPIO_Type *gpio;
    uint32_t pin;
    uint8_t stable; /* 1 = suelta (pull-up), 0 = apretada */
    uint8_t cnt;
    uint32_t t_press;
} button_t;

static button_t s_sw2 = {APP_SW2_GPIO, APP_SW2_PIN, 1U, 0U, 0U};
static button_t s_sw3 = {APP_SW3_GPIO, APP_SW3_PIN, 1U, 0U, 0U};

/* Devuelve: 0 nada, 1 se soltó tras pulsación corta, 2 tras pulsación larga */
static uint8_t button_tick(button_t *b, uint32_t now)
{
    uint8_t raw = (uint8_t)GPIO_PinRead(b->gpio, b->pin);
    if (raw == b->stable)
    {
        b->cnt = 0U;
        return 0U;
    }
    if (++b->cnt < APP_DEBOUNCE_MS)
    {
        return 0U;
    }
    b->cnt    = 0U;
    b->stable = raw;
    if (raw == 0U)
    {
        b->t_press = now; /* flanco de bajada: se apretó */
        return 0U;
    }
    return ((now - b->t_press) >= APP_LONG_PRESS_MS) ? 2U : 1U;
}

void SysTick_Handler(void)
{
    uint32_t now = ++s_ms;
    if (button_tick(&s_sw2, now) != 0U)
    {
        s_ev_sw2 = true;
    }
    uint8_t r3 = button_tick(&s_sw3, now);
    if (r3 == 1U)
    {
        s_ev_sw3_short = true;
    }
    else if (r3 == 2U)
    {
        s_ev_sw3_long = true;
    }
}

/*******************************************************************************
 * Comandos desde la PC
 ******************************************************************************/
static void uart_cb(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    (void)base;
    (void)handle;
    (void)status;
    (void)userData;
}

static void process_command(const char *cmd)
{
    long v = strtol(&cmd[1], NULL, 10);
    switch (cmd[0])
    {
        case 'N':
        case 'n':
            for (uint32_t i = 0U; i < N_LIST_LEN; i++)
            {
                if ((long)s_n_list[i] == v)
                {
                    s_cfg.n = s_n_list[i];
                }
            }
            break;
        case 'F':
        case 'f':
            if ((v >= (long)APP_FS_MIN) && (v <= (long)APP_FS_MAX))
            {
                s_cfg.fs = (uint32_t)v;
            }
            break;
        case 'M':
        case 'm':
            s_cfg.mode = (v != 0) ? MODE_FFT : MODE_BYPASS;
            leds_update();
            break;
        case 'W':
        case 'w':
            s_cfg.window = (v != 0) ? FFT_WIN_HANN : FFT_WIN_RECT;
            break;
        case 'D':
        case 'd':
            s_cfg.remove_dc = (v != 0);
            break;
        default:
            break;
    }
}

static void uart_poll_commands(void)
{
    while (LPUART_TransferGetRxRingBufferLength(APP_UART, &s_uart_handle) > 0U)
    {
        uint8_t c;
        size_t got = 0U;
        lpuart_transfer_t xfer = {.data = &c, .dataSize = 1U};
        /* Como el buffer circular ya tiene datos, esto copia en el momento */
        (void)LPUART_TransferReceiveNonBlocking(APP_UART, &s_uart_handle, &xfer, &got);
        if (got == 0U)
        {
            break;
        }
        if ((c == '\n') || (c == '\r'))
        {
            if (s_cmd_len > 0U)
            {
                s_cmd[s_cmd_len] = '\0';
                process_command(s_cmd);
                s_cmd_len = 0U;
            }
        }
        else if (s_cmd_len < (sizeof(s_cmd) - 1U))
        {
            s_cmd[s_cmd_len++] = (char)c;
        }
        else
        {
            s_cmd_len = 0U; /* línea demasiado larga: se descarta */
        }
    }
}

static uint32_t next_fs(uint32_t fs)
{
    for (uint32_t i = 0U; i < FS_LIST_LEN; i++)
    {
        if (s_fs_list[i] > fs)
        {
            return s_fs_list[i];
        }
    }
    return s_fs_list[0];
}

static uint16_t next_n(uint16_t n)
{
    for (uint32_t i = 0U; i < N_LIST_LEN; i++)
    {
        if (s_n_list[i] > n)
        {
            return s_n_list[i];
        }
    }
    return s_n_list[0];
}

static void handle_events(void)
{
    if (s_ev_sw2)
    {
        s_ev_sw2   = false;
        s_cfg.mode = (s_cfg.mode == MODE_FFT) ? MODE_BYPASS : MODE_FFT;
        leds_update();
    }
    if (s_ev_sw3_short)
    {
        s_ev_sw3_short = false;
        s_cfg.n        = next_n(s_cfg.n);
    }
    if (s_ev_sw3_long)
    {
        s_ev_sw3_long = false;
        s_cfg.fs      = next_fs(s_cfg.fs);
    }
    uart_poll_commands();
}

/*******************************************************************************
 * Periféricos
 ******************************************************************************/
static void uart_init(void)
{
    lpuart_config_t cfg;

    CLOCK_SetClkDiv(kCLOCK_DivFlexcom4Clk, 1U);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);

    LPUART_GetDefaultConfig(&cfg);
    cfg.baudRate_Bps = APP_UART_BAUD;
    cfg.enableTx     = true;
    cfg.enableRx     = true;
    (void)LPUART_Init(APP_UART, &cfg, APP_UART_CLK_FREQ);

    LPUART_TransferCreateHandle(APP_UART, &s_uart_handle, uart_cb, NULL);
    LPUART_TransferStartRingBuffer(APP_UART, &s_uart_handle, s_uart_ring, sizeof(s_uart_ring));
}

static void adc_init(void)
{
    lpadc_config_t adc_cfg;
    lpadc_conv_command_config_t cmd_cfg;
    lpadc_conv_trigger_config_t trg_cfg;
    vref_config_t vref_cfg;

    /* Reloj del ADC: FRO_HF / 1 */
    CLOCK_SetClkDiv(kCLOCK_DivAdc0Clk, 1U);
    CLOCK_AttachClk(kFRO_HF_to_ADC0);

    /* El VREF tiene que estar prendido: le da la corriente de polarización al LPADC */
    SPC_EnableActiveModeAnalogModules(SPC0, kSPC_controlVref);
    VREF_GetDefaultConfig(&vref_cfg);
    vref_cfg.bufferMode = kVREF_ModeBandgapOnly;
    VREF_Init(VREF0, &vref_cfg);

    LPADC_GetDefaultConfig(&adc_cfg);
    adc_cfg.enableAnalogPreliminary = true;
    adc_cfg.referenceVoltageSource  = kLPADC_ReferenceVoltageAlt3; /* VDD_ANA = 3,3 V */
#if defined(FSL_FEATURE_LPADC_HAS_CFG_PWRSEL) && (FSL_FEATURE_LPADC_HAS_CFG_PWRSEL == 1U)
    adc_cfg.powerLevelMode = kLPADC_PowerLevelAlt4;
#endif
    LPADC_Init(APP_ADC, &adc_cfg);

    /* Calibración de offset + autocalibración (igual que el ejemplo del SDK) */
#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CALOFSMODE) && FSL_FEATURE_LPADC_HAS_CTRL_CALOFSMODE
    LPADC_SetOffsetCalibrationMode(APP_ADC, kLPADC_OffsetCalibration16bitMode);
#endif
#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CALOFS) && FSL_FEATURE_LPADC_HAS_CTRL_CALOFS
    LPADC_DoOffsetCalibration(APP_ADC);
#endif
#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CAL_REQ) && FSL_FEATURE_LPADC_HAS_CTRL_CAL_REQ
    SDK_DelayAtLeastUs(1U, SystemCoreClock);
    LPADC_DoAutoCalibration(APP_ADC);
#endif

    /* Comando 1: canal A2, single-ended, 16 bits */
    LPADC_GetDefaultConvCommandConfig(&cmd_cfg);
    cmd_cfg.channelNumber            = APP_ADC_CHANNEL;
    cmd_cfg.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
    LPADC_SetConvCommandConfig(APP_ADC, APP_ADC_CMDID, &cmd_cfg);

    /* Trigger 0 -> comando 1, por software (lo dispara la ISR del CTIMER) */
    LPADC_GetDefaultConvTriggerConfig(&trg_cfg);
    trg_cfg.targetCommandId       = APP_ADC_CMDID;
    trg_cfg.enableHardwareTrigger = false;
    LPADC_SetConvTriggerConfig(APP_ADC, 0U, &trg_cfg);

    /* Interrupción cuando hay un resultado en la FIFO0 */
    LPADC_EnableInterrupts(APP_ADC, kLPADC_FIFO0WatermarkInterruptEnable);
    NVIC_SetPriority(APP_ADC_IRQn, 1U);
    (void)EnableIRQ(APP_ADC_IRQn);
}

void APP_ADC_IRQHandler(void)
{
    lpadc_conv_result_t r;
    while (LPADC_GetConvResult(APP_ADC, &r, 0U))
    {
        uint32_t i = s_idx;
        if (i < s_acq_n)
        {
            s_adc_raw[i] = (uint16_t)r.convValue;
            s_idx        = ++i;
            if (i >= s_acq_n)
            {
                CTIMER_StopTimer(APP_CTIMER);
                s_block_ready = true;
            }
        }
    }
    SDK_ISR_EXIT_BARRIER;
}

static void ctimer_match_cb(uint32_t flags)
{
    (void)flags;
    LPADC_DoSoftwareTrigger(APP_ADC, 1U); /* máscara 1 = trigger 0 */
}

static ctimer_callback_t s_ctimer_cb[] = {ctimer_match_cb};

static void ctimer_init(void)
{
    ctimer_config_t cfg;

    CLOCK_SetClkDiv(kCLOCK_DivCtimer0Clk, 1U);
    CLOCK_AttachClk(kFRO_HF_to_CTIMER0);

    CTIMER_GetDefaultConfig(&cfg);
    CTIMER_Init(APP_CTIMER, &cfg);
    CTIMER_RegisterCallBack(APP_CTIMER, &s_ctimer_cb[0], kCTIMER_SingleCallback);
    NVIC_SetPriority(APP_CTIMER_IRQn, 0U);
}

/* Programa el match para la fs pedida y devuelve la fs real resultante */
static float32_t ctimer_set_fs(uint32_t fs)
{
    ctimer_match_config_t m;
    const uint32_t clk = CLOCK_GetCTimerClkFreq(APP_CTIMER_ID);
    uint32_t div       = (clk + fs / 2U) / fs; /* redondeo al entero más cercano */

    if (div < 2U)
    {
        div = 2U;
    }
    m.matchValue         = div - 1U;
    m.enableCounterReset = true;
    m.enableCounterStop  = false;
    m.outControl         = kCTIMER_Output_NoAction;
    m.outPinInitState    = false;
    m.enableInterrupt    = true;
    CTIMER_SetupMatch(APP_CTIMER, kCTIMER_Match_0, &m);
    (void)EnableIRQ(APP_CTIMER_IRQn);

    return (float32_t)clk / (float32_t)div;
}

static void start_acquisition(void)
{
    s_idx         = 0U;
    s_acq_n       = s_active.n;
    s_block_ready = false;
    CTIMER_Reset(APP_CTIMER);
    CTIMER_StartTimer(APP_CTIMER);
}

static void gpio_init(void)
{
    const gpio_pin_config_t in  = {kGPIO_DigitalInput, 0U};
    const gpio_pin_config_t out = {kGPIO_DigitalOutput, 1U}; /* LED apagado */

    GPIO_PinInit(APP_SW2_GPIO, APP_SW2_PIN, &in);
    GPIO_PinInit(APP_SW3_GPIO, APP_SW3_PIN, &in);
    GPIO_PinInit(APP_LED_RED_GPIO, APP_LED_RED_PIN, &out);
    GPIO_PinInit(APP_LED_GREEN_GPIO, APP_LED_GREEN_PIN, &out);
    GPIO_PinInit(APP_LED_BLUE_GPIO, APP_LED_BLUE_PIN, &out);
}

/*******************************************************************************
 * main
 ******************************************************************************/
int main(void)
{
    BOARD_InitBootClocks();
    APP_InitPins();

    gpio_init();
    uart_init();
    adc_init();
    ctimer_init();
    cycle_counter_init();
    (void)SysTick_Config(SystemCoreClock / 1000U);

    s_cfg.n         = s_n_list[APP_N_DEFAULT_IDX];
    s_cfg.fs        = s_fs_list[APP_FS_DEFAULT_IDX];
    s_cfg.mode      = MODE_FFT;
    s_cfg.window    = FFT_WIN_RECT;
    s_cfg.remove_dc = true;
    leds_update();

    fft_proc_set_cycle_counter(cycles_now);

    /* La GUI ignora todo lo que no sea una trama, así que esto no molesta */
    uart_puts("\r\nTP3 PDS - FFT 512/1024/2048 en FRDM-MCXN947\r\n");

    s_active.n = 0U; /* fuerza aplicar la configuración la primera vez */

    for (;;)
    {
        handle_events();

        /* Los cambios se aplican entre bloques, nunca a mitad de una adquisición */
        if ((s_active.n != s_cfg.n) || (s_active.fs != s_cfg.fs))
        {
            s_fs_real = ctimer_set_fs(s_cfg.fs);
        }
        if (s_active.n != s_cfg.n)
        {
            (void)fft_proc_set_size(s_cfg.n);
        }
        if ((s_active.window != s_cfg.window) || (fft_proc_get_window() != s_cfg.window))
        {
            fft_proc_set_window(s_cfg.window);
        }
        fft_proc_set_remove_dc(s_cfg.remove_dc);
        s_active = s_cfg;

        /* 1) Adquisición de N muestras en el buffer de entrada */
        start_acquisition();
        while (!s_block_ready)
        {
            handle_events();
        }

        /* 2) Cuentas del ADC -> volts */
        const uint32_t n = s_active.n;
        for (uint32_t i = 0U; i < n; i++)
        {
            s_x_in[i] = (float32_t)s_adc_raw[i] * (APP_ADC_VREF_V / APP_ADC_FULL_SCALE);
        }

        /* 3) FFT o bypass, siempre hacia el buffer de salida */
        frame_info_t info;
        (void)memset(&info, 0, sizeof(info));
        info.n        = (uint16_t)n;
        info.fs       = s_fs_real;
        info.window   = (uint8_t)s_active.window;
        info.flags    = s_active.remove_dc ? 1U : 0U;
        info.core_clk = SystemCoreClock;
        info.seq      = s_seq++;

        if (s_active.mode == MODE_FFT)
        {
            fft_result_t res;
            info.type     = FRAME_TYPE_FFT;
            info.count    = fft_proc_spectrum(s_x_in, s_y_out, &res);
            info.peak_bin = res.peak_bin;
            info.peak_amp = res.peak_amp;
            info.dc       = res.dc;
            info.cyc_fft  = res.cyc_fft;
            info.cyc_total = res.cyc_total;
        }
        else
        {
            float32_t mean;
            info.type  = FRAME_TYPE_TIME;
            info.count = (uint16_t)n;
            arm_copy_f32(s_x_in, s_y_out, n);
            arm_mean_f32(s_x_in, n, &mean);
            info.dc = mean;
        }

        /* 4) Transmisión del buffer de salida */
        frame_send(&info, s_y_out, uart_write);
        GPIO_PortToggle(APP_LED_BLUE_GPIO, 1UL << APP_LED_BLUE_PIN);
    }
}
