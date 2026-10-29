function [frames, rest] = tp3_parse_frames(buf)
%TP3_PARSE_FRAMES Busca y decodifica las tramas que manda la FRDM-MCXN947.
%
%   [frames, rest] = tp3_parse_frames(buf)
%
%   buf    : vector uint8 con los bytes recibidos (puede tener basura,
%            texto de arranque o tramas cortadas al final).
%   frames : struct array con las tramas validas (checksum OK).
%   rest   : bytes que quedaron sin procesar (trama incompleta); hay que
%            concatenarlos adelante de los proximos bytes que lleguen.
%
%   Formato (little-endian), ver TP3/source/frame_tx.h:
%     sync A5 5A C3 3C | tipo u8 | ventana u8 | flags u8 | res u8 |
%     N u16 | count u16 | fs f32 | peak_bin u16 | res u16 | peak_amp f32 |
%     dc f32 | cyc_fft u32 | cyc_total u32 | core_clk u32 | seq u32 |
%     datos f32[count] | checksum u16 (suma de bytes desde 'tipo' hasta el
%     ultimo dato, modulo 65536)

    SYNC = char(uint8([165 90 195 60]));
    HDR  = 44;

    buf = uint8(buf(:)).';
    L   = numel(buf);

    frames = struct('type', {}, 'window', {}, 'flags', {}, 'n', {}, ...
        'count', {}, 'fs', {}, 'peak_bin', {}, 'peak_amp', {}, 'dc', {}, ...
        'cyc_fft', {}, 'cyc_total', {}, 'core_clk', {}, 'seq', {}, 'data', {});

    i = 1;
    while true
        if i > L
            rest = uint8([]);
            return;
        end
        k = strfind(char(buf(i:end)), SYNC);
        if isempty(k)
            % puede haber un sync cortado al final: se guardan 3 bytes
            rest = buf(max(i, L - 2):end);
            return;
        end
        j = i + k(1) - 1;

        if j + HDR - 1 > L
            rest = buf(j:end);          % cabecera incompleta
            return;
        end
        h     = buf(j:j + HDR - 1);
        n     = double(typecast(h(9:10), 'uint16'));
        count = double(typecast(h(11:12), 'uint16'));

        if ~any(n == [512 1024 2048]) || count > n
            i = j + 1;                  % sync falso, seguir buscando
            continue;
        end

        total = HDR + 4 * count + 2;
        if j + total - 1 > L
            rest = buf(j:end);          % faltan datos, esperar
            return;
        end

        csum = double(typecast(buf(j + total - 2:j + total - 1), 'uint16'));
        calc = mod(sum(double(buf(j + 4:j + total - 3))), 65536);
        if csum ~= calc
            i = j + 1;                  % trama corrupta, se descarta
            continue;
        end

        f.type      = double(h(5));
        f.window    = double(h(6));
        f.flags     = double(h(7));
        f.n         = n;
        f.count     = count;
        f.fs        = double(typecast(h(13:16), 'single'));
        f.peak_bin  = double(typecast(h(17:18), 'uint16'));
        f.peak_amp  = double(typecast(h(21:24), 'single'));
        f.dc        = double(typecast(h(25:28), 'single'));
        f.cyc_fft   = double(typecast(h(29:32), 'uint32'));
        f.cyc_total = double(typecast(h(33:36), 'uint32'));
        f.core_clk  = double(typecast(h(37:40), 'uint32'));
        f.seq       = double(typecast(h(41:44), 'uint32'));
        f.data      = double(typecast(buf(j + HDR:j + HDR + 4 * count - 1), 'single'));

        frames(end + 1) = f; %#ok<AGROW>
        i = j + total;
    end
end
