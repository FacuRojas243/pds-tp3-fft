"""
check_frames.py - Parser de referencia de la trama del TP3 + comparación con numpy.

Uso: python3 check_frames.py frame.bin [frame2.bin ...]

Hace lo mismo que tp3_parse_frames.m (busca el sync, valida checksum,
decodifica) y, para tramas de FFT, recalcula el espectro con numpy a partir
de la misma senoidal para comparar contra CMSIS-DSP.
"""
import struct
import sys

import numpy as np

SYNC = bytes([0xA5, 0x5A, 0xC3, 0x3C])
HDR = 44


def parse(buf: bytes):
    frames, i = [], 0
    while True:
        j = buf.find(SYNC, i)
        if j < 0 or j + HDR > len(buf):
            return frames
        h = buf[j:j + HDR]
        typ, win, flags = h[4], h[5], h[6]
        n, count = struct.unpack_from("<HH", h, 8)
        fs, = struct.unpack_from("<f", h, 12)
        peak_bin, = struct.unpack_from("<H", h, 16)
        peak_amp, dc = struct.unpack_from("<ff", h, 20)
        cyc_fft, cyc_tot, clk, seq = struct.unpack_from("<IIII", h, 28)
        end = j + HDR + 4 * count + 2
        if end > len(buf):
            return frames
        payload = buf[j + HDR:end - 2]
        csum, = struct.unpack_from("<H", buf, end - 2)
        calc = (sum(buf[j + 4:end - 2])) & 0xFFFF
        if calc != csum:
            i = j + 1  # sync falso: seguir buscando
            continue
        data = np.frombuffer(payload, dtype="<f4")
        frames.append(dict(type=typ, win=win, flags=flags, n=n, count=count, fs=fs,
                           peak_bin=peak_bin, peak_amp=peak_amp, dc=dc,
                           cyc_fft=cyc_fft, cyc_tot=cyc_tot, clk=clk, seq=seq, data=data))
        i = end


def reference_spectrum(n, fs, f0, amp, win):
    k = np.arange(n)
    v = 1.65 + amp * np.sin(2 * np.pi * f0 * k / fs)
    raw = np.clip(np.round(v / 3.3 * 65536), 0, 65535)
    x = raw * (3.3 / 65536)
    x = x - x.mean()
    w = np.ones(n) if win == 0 else 0.5 - 0.5 * np.cos(2 * np.pi * k / n)
    X = np.fft.fft(x * w)[: n // 2]
    a = np.abs(X) * 2 / (n * w.mean())
    a[0] /= 2
    return a


if __name__ == "__main__":
    # argumentos opcionales por archivo: f0 amp (para comparar)
    args = sys.argv[1:]
    for path in args:
        fr = parse(open(path, "rb").read())
        assert len(fr) == 1, f"{path}: se esperaba 1 trama, hay {len(fr)}"
        f = fr[0]
        d = f["data"]
        kind = "FFT" if f["type"] == 1 else "TIEMPO"
        print(f"{path}: {kind} N={f['n']} count={f['count']} fs={f['fs']:.1f} "
              f"win={f['win']} seq={f['seq']} dc={f['dc']:.4f}")
        if f["type"] == 1:
            pk = f["peak_bin"]
            print(f"   pico bin {pk} -> {pk * f['fs'] / f['n']:.2f} Hz, {f['peak_amp']:.5f} V "
                  f"(argmax datos = {np.argmax(d[1:]) + 1})")
        else:
            print(f"   min={d.min():.4f} max={d.max():.4f}")
