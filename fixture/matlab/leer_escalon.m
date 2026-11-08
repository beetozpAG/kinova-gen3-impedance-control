function [T, meta] = leer_escalon(archivo)
% LEER_ESCALON  Lee un escalón guardado por ensayo_PI.m y su _meta.txt.
% Recalcula t_ms con n_pre y ts_us del meta: las versiones del .ino
% anteriores al 2026-10-27 (tarde) mandaban t_ms mal para t < 0
% (resta sin signo -> ~4294965 ms). Así sirven los datos viejos y nuevos.
T = readtable(archivo);
meta = "";
fm = replace(string(archivo), ".csv", "_meta.txt");
if isfile(fm)
    meta = fileread(fm);
    npre = regexp(meta, 'n_pre=(\d+)', 'tokens', 'once');
    tsus = regexp(meta, 'ts_us=(\d+)', 'tokens', 'once');
    if ~isempty(npre) && ~isempty(tsus)
        T.t_ms = ((0:height(T)-1)' - str2double(npre{1})) * str2double(tsus{1}) * 1e-3;
    end
end
end
