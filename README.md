# TP3 – FFT de 512 / 1024 / 2048 muestras en la FRDM-MCXN947

Laboratorio 3 de **Procesamiento Digital de Señales**.

> **Consigna.** Implementar la FFT sobre un buffer de 512, 1024 o 2048 muestras adquirido con el ADC (Laboratorio #1), con distintas velocidades de muestreo. Una tecla de la placa habilita la FFT o hace *bypass* hacia un buffer de salida distinto del de entrada. El resultado se transmite por el puerto serie (SCI) y se grafica en la PC con una GUI de MATLAB. Al final se sacan conclusiones sobre el efecto de la frecuencia de muestreo y del tamaño de la ventana.

```
señal ──► ADC0_A2 ──► buffer de ENTRADA (N muestras, 16 bits)
          ▲                      │
   CTIMER0 a fs                  ├─ SW2 = FFT ──► ventana + arm_cfft_f32 + |X(k)| ──┐
   (dispara cada conversión)     └─ SW2 = bypass ──► copia sin procesar ─────────────┤
                                                                                     ▼
                                           buffer de SALIDA ──► LPUART4 (VCOM) ──► GUI MATLAB
```

## Contenido del repo

| Carpeta | Qué hay |
|---|---|
| `TP3/source/` | Firmware: `main.c`, `fft_proc.c/.h` (FFT con CMSIS-DSP), `frame_tx.c/.h` (trama serie), `app_pins.c/.h` (pines), `app_config.h` (parámetros) |
| `TP3/matlab/` | `tp3_fft_gui.m` (GUI), `tp3_parse_frames.m` (decodificador de tramas) y `tramas_ejemplo/` para probar la GUI sin la placa |
| `TP3/tools/arm_build/` | Makefile para compilar el firmware fuera del IDE (sólo para verificar) |
| `TP3/tools/host_test/` | Corre el procesamiento en la PC y lo compara contra numpy |

## Conexionado

| Señal | Pin MCU | Conector FRDM-MCXN947 |
|---|---|---|
| Entrada analógica (ADC0, canal A2) | P4_23 | **J8 pin 28** |
| GND | – | cualquier GND de la placa |
| Puerto serie a la PC | LPUART4 (P1_8 / P1_9) | USB del MCU-Link (**J17**) → aparece como COM / VCOM |

El ADC mide de **0 a 3,3 V**, así que la señal tiene que ir montada sobre un offset de ~1,65 V. Por ejemplo, en el generador: offset 1,65 V y amplitud de 1 V de pico como máximo. **No pasar de 3,3 V ni bajar de 0 V.**

## Armar el proyecto en MCUXpresso IDE

1. *File → New → Create a new C/C++ Project*, elegir **frdmmcxn947** y el core `cm33_core0`.
2. En **Drivers**, tildar `clock`, `common`, `ctimer`, `gpio`, `lpadc`, `lpflexcomm`, `lpflexcomm_lpuart`, `mcx_spc`, `port`, `reset` y `vref`.
3. Agregar **CMSIS-DSP**, igual que en el TP2. Si no aparece en el asistente: clic derecho al proyecto → *SDK Management → Manage SDK Components → CMSIS → DSP Library*.
4. Copiar todos los archivos de `TP3/source/` a la carpeta `source/` del proyecto. Borrar el `.c` con `main()` que generó el asistente: el `main` es el de `main.c`.
5. Compilar y depurar. Para medir los tiempos de la FFT conviene compilar con optimización (`-O2` o la configuración *Release*): con `-O0` los ciclos se disparan y las conclusiones quedan contaminadas.

> `main.c` usa `BOARD_InitBootClocks()` del `clock_config.c` que genera el IDE: el núcleo queda a 150 MHz y el FRO_HF a 48 MHz. Los pines los configura `APP_InitPins()`, así que **no** hace falta tocar el `pin_mux.c`.

## Uso

| Control | Acción |
|---|---|
| **SW2** | Habilita la FFT o pasa a bypass |
| **SW3** corto | Cambia N: 512 → 1024 → 2048 → 512 … |
| **SW3** largo (>0,7 s) | Cambia fs: 8 k → 16 k → 22,05 k → 44,1 k → 48 kHz → … |
| LED verde / rojo | FFT activa / bypass |
| LED azul | Cambia de estado con cada trama enviada |

### GUI de MATLAB

Requiere MATLAB R2020a o posterior. Abrir la carpeta `TP3/matlab` y ejecutar:

```matlab
tp3_fft_gui
```

Elegir el puerto COM del MCU-Link, dejar 115200 baud y tocar **Conectar**. Desde la GUI se pueden cambiar N, fs (hay valores predefinidos y también se puede escribir uno), el modo, la ventana (rectangular o Hann) y si se resta o no el valor medio. Muestra Δf, la duración de la ventana N/fs, el pico, los ciclos que tardó `arm_cfft_f32` y el tiempo total de procesamiento. **Guardar datos** exporta `.mat`/`.csv` y **Guardar figura** exporta `.png`/`.pdf` para el informe.

Para probar la GUI sin la placa: **Abrir .bin** → `tramas_ejemplo/demo_varias_tramas.bin`.

## Detalles de implementación

- **Muestreo.** CTIMER0 con *match* 0 y reset del contador; cada *match* dispara por software una conversión del LPADC0, que trabaja en 16 bits. La ISR del ADC guarda la muestra en el buffer de entrada y, al llegar a N, frena el timer. La fs real es `48 MHz / round(48 MHz / fs)` (por ejemplo, 44100 → 44117,6 Hz) y es la que viaja en la trama y usa la GUI para el eje de frecuencias.
- **FFT** (`fft_proc.c`). Pasa de cuentas a volts, resta la media (opcional), aplica la ventana, arma el buffer complejo `{Re, Im}` y llama a `arm_cfft_f32(&arm_cfft_sR_f32_lenN, buf, 0, 1)`, como en las diapositivas de clase. Después calcula el módulo con `arm_cmplx_mag_f32` sobre N/2 bins, escala a volts de pico con `A(k) = 2|X(k)| / (N·CG)` (CG = 1 rectangular, 0,5 Hann) y busca el pico con `arm_max_f32`. Los ciclos se miden con el contador `DWT->CYCCNT` del Cortex-M33.
- **Buffers.** Entrada `s_adc_raw[]` / `s_x_in[]`, salida `s_y_out[]` (espectro o copia en bypass) y un buffer interno de trabajo para la FFT in situ. Los cambios de N o fs se aplican entre bloques, nunca a mitad de una adquisición.
- **Funcionamiento por bloques.** Adquiere N muestras → procesa → transmite → vuelve a adquirir. La tasa de refresco la limita el puerto serie (ver tabla).

### Trama serie (little-endian)

`A5 5A C3 3C` + cabecera de 44 bytes (tipo, ventana, N, cantidad de datos, fs real, pico, DC, ciclos, reloj, número de secuencia) + `float32[count]` + checksum de 16 bits. El detalle campo por campo está en `TP3/source/frame_tx.h`.

### Comandos PC → placa (ASCII, terminados en `\n`)

`N512` · `N1024` · `N2048` · `F<hz>` (1000–100000) · `M1` FFT / `M0` bypass · `W0` rect / `W1` Hann · `D1` / `D0` restar DC

## Verificación hecha

- El firmware compila y linkea sin warnings propios con `arm-none-eabi-gcc` 14.2 contra el MCUXpresso SDK 26.09 (FRDM-MCXN947) y CMSIS-DSP 1.16.2 (`TP3/tools/arm_build`). Ocupa ~66 KB de flash y ~49 KB de RAM.
- `fft_proc.c` y `frame_tx.c` corridos en la PC con la misma CMSIS-DSP dan un espectro que difiere de `numpy.fft` en menos de 3 µV. El decodificador recupera las tramas aunque haya texto o un sync falso antes (`TP3/tools/host_test`).
- **Falta probarlo en la placa**: el ruteo de pines, el ADC y la VCOM siguen los ejemplos oficiales del SDK (`lpadc_interrupt`, `ctimer`, `gpio`), pero hay que validarlo con el generador.

## Guía para las conclusiones

**Resolución y duración de la ventana** (fs real):

| fs | N = 512 | N = 1024 | N = 2048 |
|---|---|---|---|
| 8 kHz | Δf 15,6 Hz · T 64 ms | 7,8 Hz · 128 ms | 3,9 Hz · 256 ms |
| 16 kHz | 31,3 Hz · 32 ms | 15,6 Hz · 64 ms | 7,8 Hz · 128 ms |
| 22,05 kHz | 43,1 Hz · 23,2 ms | 21,5 Hz · 46,4 ms | 10,8 Hz · 92,9 ms |
| 44,1 kHz | 86,2 Hz · 11,6 ms | 43,1 Hz · 23,2 ms | 21,5 Hz · 46,4 ms |
| 48 kHz | 93,8 Hz · 10,7 ms | 46,9 Hz · 21,3 ms | 23,4 Hz · 42,7 ms |

**Costo teórico** (multiplicaciones complejas, de la clase): FFT = N/2·log₂N → 2304 / 5120 / 11264 contra 262144 / 1048576 / 4194304 de la DFT. La FFT resulta 114×, 205× y 372× más rápida. Al duplicar N el costo crece ~2,2×, no 4×.

**Tiempo de transmisión por trama** a 115200 baud: FFT 93 / 182 / 360 ms y bypass 182 / 360 / 715 ms para N = 512 / 1024 / 2048. Con 921600 baud se divide por 8; hay que cambiarlo en `app_config.h` y en la GUI.

Experimentos sugeridos:

1. **Resolución vs. N y fs.** Con fs fija, al duplicar N, Δf se reduce a la mitad pero la ventana dura el doble, lo que da más latencia y menos capacidad de seguir señales que cambian. Con N fijo, al subir fs crece el rango (Nyquist = fs/2) pero empeora la resolución. Δf depende sólo de la duración de la ventana: Δf = 1/T = fs/N.
2. **Dos tonos cercanos** (por ejemplo, 1000 y 1020 Hz): ver con qué combinaciones de N y fs se separan y con cuáles se juntan en un solo pico.
3. **Leakage y ventana.** Con un tono justo en un bin (en la simulación: N = 1024, fs = 8 kHz, f0 = 1000 Hz) la ventana rectangular da la amplitud exacta (0,80 V de 0,80 V). Medio bin corrido (f0 = 1003,9 Hz) cae a 0,51 V (−3,9 dB) y aparecen lóbulos. Con Hann, en ese mismo caso, da 0,68 V (−1,4 dB) y los lóbulos laterales caen mucho más rápido, a cambio de un lóbulo principal más ancho.
4. **Aliasing.** Con fs = 8 kHz, un tono de 5 kHz aparece en 3 kHz (fs − f0). Comparar con 16 kHz.
5. **Tiempo de cómputo.** Anotar los ciclos de `arm_cfft_f32` para cada N (no dependen de fs) y compararlos con N/2·log₂N. Verificar que el procesamiento entra holgado en el tiempo de adquisición N/fs, que es el presupuesto para procesar en tiempo real.
6. **DC.** Con y sin `D1`: el offset de 1,65 V genera un pico enorme en k = 0 que tapa la escala si no se resta la media.
