/*
 * app_pins.c - Ruteo de pines del TP3 (equivalente a lo que genera la
 * herramienta "Pins" de MCUXpresso, pero escrito a mano para que el proyecto
 * no dependa del pin_mux.c que tenga cada uno).
 *
 *   P1_8  -> FC4_P0  (LPUART4 RX, VCOM del MCU-Link)
 *   P1_9  -> FC4_P1  (LPUART4 TX, VCOM del MCU-Link)
 *   P4_23 -> ADC0_A2 (entrada analógica, J8 pin 28)
 *   P0_23 -> GPIO    (SW2)
 *   P0_6  -> GPIO    (SW3)
 *   P0_10 / P0_27 / P1_2 -> GPIO (LED rojo / verde / azul)
 */
#include "app_pins.h"

#include "fsl_common.h"
#include "fsl_port.h"

static void pin_cfg(PORT_Type *port, uint32_t pin, port_mux_t mux, bool digital_in, bool pull_up)
{
    const port_pin_config_t cfg = {
        .pullSelect            = pull_up ? kPORT_PullUp : kPORT_PullDisable,
        .pullValueSelect       = kPORT_LowPullResistor,
        .slewRate              = kPORT_FastSlewRate,
        .passiveFilterEnable   = kPORT_PassiveFilterDisable,
        .openDrainEnable       = kPORT_OpenDrainDisable,
        .driveStrength         = kPORT_LowDriveStrength,
        .mux                   = mux,
        .inputBuffer           = digital_in ? kPORT_InputBufferEnable : kPORT_InputBufferDisable,
        .invertInput           = kPORT_InputNormal,
        .lockRegister          = kPORT_UnlockRegister,
    };
    PORT_SetPinConfig(port, pin, &cfg);
}

void APP_InitPins(void)
{
    CLOCK_EnableClock(kCLOCK_Port0);
    CLOCK_EnableClock(kCLOCK_Port1);
    CLOCK_EnableClock(kCLOCK_Port4);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);

    /* UART de debug (MCU-Link VCOM) */
    pin_cfg(PORT1, 8U, kPORT_MuxAlt2, true, false);
    pin_cfg(PORT1, 9U, kPORT_MuxAlt2, true, false);

    /* Entrada analógica: sin buffer digital (obligatorio para funciones analógicas) */
    pin_cfg(PORT4, 23U, kPORT_MuxAlt0, false, false);

    /* Teclas con pull-up interno por las dudas (la placa ya trae pull-up) */
    pin_cfg(PORT0, 23U, kPORT_MuxAlt0, true, true);
    pin_cfg(PORT0, 6U, kPORT_MuxAlt0, true, true);

    /* LEDs */
    pin_cfg(PORT0, 10U, kPORT_MuxAlt0, true, false);
    pin_cfg(PORT0, 27U, kPORT_MuxAlt0, true, false);
    pin_cfg(PORT1, 2U, kPORT_MuxAlt0, true, false);
}
