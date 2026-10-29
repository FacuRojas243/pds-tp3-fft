function tp3_fft_gui
%TP3_FFT_GUI  GUI del TP3 de PDS: grafica el espectro (FFT) o la senal en
%bypass que manda la FRDM-MCXN947 por el puerto serie (VCOM del MCU-Link).
%
%   Requiere MATLAB R2020a o posterior (serialport, uifigure, exportgraphics).
%   Uso: abrir la carpeta TP3/matlab en MATLAB y ejecutar  tp3_fft_gui
%
%   Desde la GUI se puede cambiar N, fs, modo FFT/bypass, ventana y resta de
%   DC: se le mandan comandos ASCII a la placa (N1024, F44100, M1, W1, D1).
%   Tambien se puede abrir un archivo .bin con tramas grabadas (ver
%   tramas_ejemplo/) para probar sin la placa.

    % ------------------------------------------------------------------
    % Estado
    % ------------------------------------------------------------------
    st.sp      = [];          % objeto serialport
    st.rx      = uint8([]);   % bytes recibidos sin procesar
    st.last    = [];          % ultima trama valida
    st.nframes = 0;

    FS_LIST = {'8000', '16000', '22050', '44100', '48000'};
    MU      = char(181);      % micro
    DELTA   = char(916);      % Delta

    % ------------------------------------------------------------------
    % Ventana
    % ------------------------------------------------------------------
    fig = uifigure('Name', 'TP3 PDS - FFT en FRDM-MCXN947', ...
        'Position', [80 80 1200 640], 'CloseRequestFcn', @on_close);
    gl = uigridlayout(fig, [1 2]);
    gl.ColumnWidth = {300, '1x'};

    pnl = uipanel(gl, 'Title', 'Control');
    g = uigridlayout(pnl, [19 2]);
    g.RowHeight   = [repmat({24}, 1, 12), {'1x'}, repmat({24}, 1, 6)];
    g.ColumnWidth = {'1x', '1x'};

    uilabel(g, 'Text', 'Puerto serie');
    ddPort = uidropdown(g, 'Items', {'(sin puertos)'});

    uibutton(g, 'Text', 'Refrescar puertos', 'ButtonPushedFcn', @(~, ~) refresh_ports());
    edBaud = uieditfield(g, 'numeric', 'Value', 115200, 'Limits', [1200 4000000], ...
        'ValueDisplayFormat', '%.0f baud');

    btnConnect = uibutton(g, 'Text', 'Conectar', 'ButtonPushedFcn', @(~, ~) toggle_connect());
    btnConnect.Layout.Column = [1 2];

    uilabel(g, 'Text', 'N (muestras)');
    ddN = uidropdown(g, 'Items', {'512', '1024', '2048'}, 'Value', '512', ...
        'ValueChangedFcn', @(src, ~) send_cmd(['N' src.Value]));

    uilabel(g, 'Text', 'fs [Hz]');
    ddFs = uidropdown(g, 'Items', FS_LIST, 'Value', '16000', 'Editable', 'on', ...
        'ValueChangedFcn', @(~, ~) on_fs());

    uilabel(g, 'Text', 'Modo (o tecla SW2)');
    ddMode = uidropdown(g, 'Items', {'FFT', 'Bypass'}, 'Value', 'FFT', ...
        'ValueChangedFcn', @(src, ~) send_cmd(['M' num2str(strcmp(src.Value, 'FFT'))]));

    uilabel(g, 'Text', 'Ventana');
    ddWin = uidropdown(g, 'Items', {'Rectangular', 'Hann'}, 'Value', 'Rectangular', ...
        'ValueChangedFcn', @(src, ~) send_cmd(['W' num2str(strcmp(src.Value, 'Hann'))]));

    chkDC = uicheckbox(g, 'Text', 'Restar valor medio (DC) antes de la FFT', 'Value', true, ...
        'ValueChangedFcn', @(src, ~) send_cmd(['D' num2str(src.Value)]));
    chkDC.Layout.Column = [1 2];

    uilabel(g, 'Text', 'Escala');
    ddScale = uidropdown(g, 'Items', {'Lineal [V]', 'dB [dBV]'}, 'Value', 'Lineal [V]', ...
        'ValueChangedFcn', @(~, ~) redraw());

    btnFreeze = uibutton(g, 'state', 'Text', 'Congelar');
    uibutton(g, 'Text', 'Abrir .bin', 'ButtonPushedFcn', @(~, ~) open_bin());

    uibutton(g, 'Text', 'Guardar datos', 'ButtonPushedFcn', @(~, ~) save_data());
    uibutton(g, 'Text', 'Guardar figura', 'ButtonPushedFcn', @(~, ~) save_fig());

    lblStatus = uilabel(g, 'Text', 'Desconectado', 'FontWeight', 'bold');
    lblStatus.Layout.Column = [1 2];

    txtInfo = uitextarea(g, 'Editable', 'off', 'FontName', 'Consolas');
    txtInfo.Layout.Row    = [13 19];
    txtInfo.Layout.Column = [1 2];

    ax = uiaxes(gl);
    grid(ax, 'on');
    hold(ax, 'on');
    hLine = plot(ax, NaN, NaN, '-', 'LineWidth', 1.1);
    hPeak = plot(ax, NaN, NaN, 'rv', 'MarkerFaceColor', 'r', 'Visible', 'off');
    hold(ax, 'off');
    title(ax, 'Esperando datos...');

    tmr = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.05, ...
        'BusyMode', 'drop', 'TimerFcn', @(~, ~) poll());

    refresh_ports();

    % ------------------------------------------------------------------
    % Puerto serie
    % ------------------------------------------------------------------
    function refresh_ports()
        p = cellstr(serialportlist("available"));
        if isempty(p)
            p = {'(sin puertos)'};
        end
        ddPort.Items = p;
    end

    function toggle_connect()
        if isempty(st.sp)
            connect();
        else
            disconnect();
        end
    end

    function connect()
        try
            st.sp = serialport(ddPort.Value, edBaud.Value, 'Timeout', 1);
            configureTerminator(st.sp, 'LF');
            flush(st.sp);
        catch ME
            st.sp = [];
            uialert(fig, ME.message, 'No se pudo abrir el puerto');
            return;
        end
        st.rx = uint8([]);
        btnConnect.Text = 'Desconectar';
        lblStatus.Text  = ['Conectado a ' ddPort.Value];
        % la placa queda con lo que diga la GUI
        send_cmd(['N' ddN.Value]);
        on_fs();
        send_cmd(['M' num2str(strcmp(ddMode.Value, 'FFT'))]);
        send_cmd(['W' num2str(strcmp(ddWin.Value, 'Hann'))]);
        send_cmd(['D' num2str(chkDC.Value)]);
        start(tmr);
    end

    function disconnect()
        if strcmp(tmr.Running, 'on')
            stop(tmr);
        end
        st.sp = [];                    % borrar el objeto cierra el puerto
        btnConnect.Text = 'Conectar';
        lblStatus.Text  = 'Desconectado';
    end

    function send_cmd(cmd)
        if isempty(st.sp)
            return;
        end
        try
            writeline(st.sp, cmd);
        catch ME
            uialert(fig, ME.message, 'Error al enviar comando');
        end
    end

    function on_fs()
        fs = str2double(ddFs.Value);
        if isnan(fs) || fs < 1000 || fs > 100000
            uialert(fig, 'fs tiene que estar entre 1000 y 100000 Hz', 'fs invalida');
            return;
        end
        send_cmd(sprintf('F%d', round(fs)));
    end

    function poll()
        if isempty(st.sp)
            return;
        end
        try
            nb = st.sp.NumBytesAvailable;
            if nb == 0
                return;
            end
            d = read(st.sp, nb, 'uint8');
            st.rx = [st.rx, uint8(d(:)).'];
            [fr, st.rx] = tp3_parse_frames(st.rx);
            if numel(st.rx) > 65536            % por las dudas, no crecer sin fin
                st.rx = st.rx(end - 32767:end);
            end
            if ~isempty(fr)
                st.nframes = st.nframes + numel(fr);
                st.last    = fr(end);
                if ~btnFreeze.Value
                    redraw();
                end
            end
        catch ME
            disconnect();
            uialert(fig, ME.message, 'Error en el puerto serie');
        end
    end

    % ------------------------------------------------------------------
    % Graficos
    % ------------------------------------------------------------------
    function [x, y, xl, yl] = frame_xy(f)
        if f.type == 1
            x  = (0:f.count - 1) * f.fs / f.n;
            y  = f.data;
            xl = 'Frecuencia [Hz]';
            if strcmp(ddScale.Value, 'dB [dBV]')
                y  = 20 * log10(max(y, 1e-6));
                yl = 'Amplitud [dBV]';
            else
                yl = 'Amplitud [V pico]';
            end
        else
            x  = (0:f.count - 1) / f.fs * 1e3;
            y  = f.data;
            xl = 'Tiempo [ms]';
            yl = 'Tension [V]';
        end
    end

    function redraw()
        f = st.last;
        if isempty(f)
            return;
        end
        [x, y, xl, yl] = frame_xy(f);
        set(hLine, 'XData', x, 'YData', y);
        xlabel(ax, xl);
        ylabel(ax, yl);
        df = f.fs / f.n;
        if f.type == 1
            k = f.peak_bin + 1;
            set(hPeak, 'XData', x(k), 'YData', y(k), 'Visible', 'on');
            xlim(ax, [0 f.fs / 2]);
            title(ax, sprintf('Espectro  |  N = %d, fs = %.1f Hz, \\Deltaf = %.3f Hz  |  pico %.1f Hz', ...
                f.n, f.fs, df, f.peak_bin * df));
        else
            set(hPeak, 'Visible', 'off');
            xlim(ax, [0 max(x(end), eps)]);
            title(ax, sprintf('Bypass (senal en el tiempo)  |  N = %d, fs = %.1f Hz', f.n, f.fs));
        end
        ylim(ax, 'auto');
        txtInfo.Value = info_lines(f);
    end

    function L = info_lines(f)
        wins = {'Rectangular', 'Hann'};
        df   = f.fs / f.n;
        L = {sprintf('Trama #%d   (recibidas: %d)', f.seq, st.nframes)
             sprintf('Modo       : %s', ternary(f.type == 1, 'FFT', 'Bypass'))
             sprintf('N          : %d muestras', f.n)
             sprintf('fs real    : %.2f Hz', f.fs)
             sprintf('%sf = fs/N  : %.3f Hz', DELTA, df)
             sprintf('T = N/fs   : %.2f ms', 1e3 * f.n / f.fs)
             sprintf('Nyquist    : %.1f Hz', f.fs / 2)
             sprintf('Valor medio: %.4f V', f.dc)};
        if f.type == 1
            clk = max(f.core_clk, 1);
            L = [L
                 {sprintf('Ventana    : %s, DC %s', wins{f.window + 1}, ternary(bitand(f.flags, 1), 'restada', 'sin restar'))
                  sprintf('Pico       : %.2f Hz  (bin %d), %.4f V', f.peak_bin * df, f.peak_bin, f.peak_amp)
                  sprintf('arm_cfft   : %d ciclos = %.1f %ss', f.cyc_fft, 1e6 * f.cyc_fft / clk, MU)
                  sprintf('Proc. total: %d ciclos = %.1f %ss', f.cyc_total, 1e6 * f.cyc_total / clk, MU)
                  sprintf('Nucleo     : %.0f MHz', clk / 1e6)}];
        end
    end

    function out = ternary(c, a, b)
        if c
            out = a;
        else
            out = b;
        end
    end

    % ------------------------------------------------------------------
    % Archivos
    % ------------------------------------------------------------------
    function open_bin()
        [fn, p] = uigetfile({'*.bin', 'Tramas grabadas (*.bin)'}, 'Abrir tramas');
        if isequal(fn, 0)
            return;
        end
        fid = fopen(fullfile(p, fn), 'r');
        b = fread(fid, Inf, 'uint8=>uint8');
        fclose(fid);
        fr = tp3_parse_frames(b);
        if isempty(fr)
            uialert(fig, 'No se encontro ninguna trama valida en el archivo.', 'Abrir .bin');
            return;
        end
        st.nframes = numel(fr);
        st.last    = fr(end);
        lblStatus.Text = sprintf('%s: %d trama(s)', fn, numel(fr));
        redraw();
    end

    function save_data()
        f = st.last;
        if isempty(f)
            uialert(fig, 'Todavia no hay datos.', 'Guardar');
            return;
        end
        [fn, p] = uiputfile({'*.mat', 'MATLAB (*.mat)'; '*.csv', 'CSV (*.csv)'}, 'Guardar datos', ...
            sprintf('tp3_%s_N%d_fs%d', ternary(f.type == 1, 'fft', 'tiempo'), f.n, round(f.fs)));
        if isequal(fn, 0)
            return;
        end
        file = fullfile(p, fn);
        [~, ~, ext] = fileparts(file);
        if f.type == 1
            x = (0:f.count - 1).' * f.fs / f.n;
            names = {'f_Hz', 'A_Vpico'};
        else
            x = (0:f.count - 1).' / f.fs;
            names = {'t_s', 'v_V'};
        end
        if strcmpi(ext, '.csv')
            writetable(table(x, f.data(:), 'VariableNames', names), file);
        else
            trama = f; %#ok<NASGU>
            save(file, 'trama');
        end
    end

    function save_fig()
        if isempty(st.last)
            uialert(fig, 'Todavia no hay datos.', 'Guardar figura');
            return;
        end
        f = st.last;
        [fn, p] = uiputfile({'*.png'; '*.pdf'}, 'Guardar figura', ...
            sprintf('tp3_%s_N%d_fs%d.png', ternary(f.type == 1, 'fft', 'tiempo'), f.n, round(f.fs)));
        if isequal(fn, 0)
            return;
        end
        exportgraphics(ax, fullfile(p, fn), 'Resolution', 200);
    end

    function on_close(~, ~)
        try
            if strcmp(tmr.Running, 'on')
                stop(tmr);
            end
            delete(tmr);
        catch
        end
        st.sp = [];
        delete(fig);
    end
end
