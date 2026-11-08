%% ENSAYO_PI  Pruebas de escalón del lazo de corriente en el ESP32
%  (programa control_PI_corriente.ino, ROTOR BLOQUEADO)
%
%  plan = "SIMC"    : el controlador elegido (PI SIMC + LIN, sin FF) en muchos
%                     escalones, con repeticiones -> comparar_modelo.m
%  plan = "COMPARA" : las reglas clásicas + la estructura de 2 GDL
%  plan = "ZN_EXP"  : Ziegler-Nichols EXPERIMENTAL en lazo cerrado:
%                     control P (TI 0), KP creciente hasta oscilar.
%                     analisis_PI.m calcula Ku y Pu.
%  plan = "MANUAL"  : edita la lista "manual".
%
%  Cada escalón se guarda en datos/<plan>_<config>_<a>_<b>_r<n>_<fecha>.csv
%  OJO: abrir el puerto reinicia el ESP32 (normal).

clear; clc;

%% ===================== LO QUE CAMBIAS =====================
puerto = "COM14";
baud   = 921600;
plan   = "SIMC";               % "SIMC" | "COMPARA" | "ZN_EXP" | "MANUAL"
repeticiones = 3;            % cada escalón se repite (para ver la dispersión)

escalones = [100 300; 200 250; 300 150; -100 -300];   % [a b] en mA

PRE_ms = 2;  POST_ms = 12;  SETTLE_ms = 200;  IMAX_mA = 700;

% configuraciones: {nombre, LIN, FF, KP, TI_ms, TAUD_ms}
% (valores de sintonizar_PI.m con theta_e = 0.491 ms; KP con LIN en mA/mA)
compara = {
  "ZN_LC",      1, 0, 1.717, 1.400, 0.10
  "CHR20",      1, 0, 1.204, 0.985, 0.10
  "SIMC",       1, 0, 1.003, 0.985, 0.10
  "SIMC_FF",    1, 1, 1.003, 0.985, 0.10
  "SIMC_sinLIN",0, 0, 1.349, 0.985, 0.10    % PI clásico, KP en PWM/mA
};
% escalones del plan SIMC: pequeños, medianos, grandes, desde 0 y en IN2
escSIMC = [ 150  200;  200  150;  100  150;  150  100
            100  300;  300  100;  150  500
              0  150;    0  300
           -150 -200; -100 -300;    0 -150 ];
simc   = { "SIMC", 1, 0, 1.003, 0.985, 0.10 };
kpZN   = 2.0:0.25:5.5;   % barrido para ZN_EXP (LIN ON, solo P)
manual = { "PRUEBA", 1, 1, 1.0, 0.985, 0.10 };
%% ===========================================================

switch plan
    case "SIMC", cfgs = simc; escalones = escSIMC;
    case "COMPARA", cfgs = compara;
    case "ZN_EXP"
        cfgs = cell(numel(kpZN), 6);
        for j = 1:numel(kpZN)
            cfgs(j,:) = {sprintf("KP%.2f", kpZN(j)), 1, 0, kpZN(j), 0, 0.10};
        end
        escalones = [150 250];
        POST_ms = 15;
    case "MANUAL", cfgs = manual;
end

carpeta = fullfile(fileparts(mfilename('fullpath')), 'datos');
if ~exist(carpeta, 'dir'), mkdir(carpeta); end
sello = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));

s = serialport(puerto, baud, "Timeout", 30);
configureTerminator(s, "LF");
cleanup = onCleanup(@() cerrar(s));
flush(s);
esperar(s, "LISTO", 10);

enviar(s, sprintf("SET PRE %d", PRE_ms));
enviar(s, sprintf("SET POST %d", POST_ms));
enviar(s, sprintf("SET SETTLE %d", SETTLE_ms));
enviar(s, sprintf("IMAX %d", IMAX_mA));

for c = 1:size(cfgs, 1)
    [nombre, lin, ff, kp, ti, taud] = cfgs{c,:};
    fprintf("\n===== %s: LIN %d FF %d KP %.3f TI %.3f ms TAUD %.2f ms =====\n", nombre, lin, ff, kp, ti, taud);
    enviar(s, sprintf("LIN %s", onoff(lin)));
    enviar(s, sprintf("FF %s", onoff(ff)));
    enviar(s, sprintf("KP %.5f", kp));
    enviar(s, sprintf("TI %.5f", ti));
    enviar(s, sprintf("TAUD %.4f", taud));

    for e = 1:size(escalones, 1)
      for rep = 1:repeticiones
        a = escalones(e,1); b = escalones(e,2);
        writeline(s, sprintf("ESCALON %g %g", a, b));
        [lineas, ok, meta] = leerBloque(s, "BEGIN_PI", "END_PI", 30);
        if isempty(lineas), warning("Sin datos en %s %g->%g", nombre, a, b); continue; end
        T = parsear(lineas);
        archivo = fullfile(carpeta, sprintf("%s_%s_%+g_%+g_r%d_%s.csv", plan, nombre, a, b, rep, sello));
        writetable(T, archivo);
        fid = fopen(replace(archivo, ".csv", "_meta.txt"), 'w'); fprintf(fid, "%s\n", meta); fclose(fid);
        fprintf("  %g -> %g (r%d) guardado\n", a, b, rep);
        if ~ok || contains(meta, "trip=1")
            warning("Sobrecorriente: se detiene el ensayo."); writeline(s, "STOP"); return;
        end
        pause(0.3);                       % enfriar entre escalones
      end
    end
end
writeline(s, "STOP");
fprintf("\nListo. Corre comparar_modelo.m o analisis_PI.m con patron = ""%s_*%s*""\n", plan, sello);


%% ===================== FUNCIONES =====================
function t = onoff(v), if v, t = "ON"; else, t = "OFF"; end, end

function enviar(s, cmd)
    writeline(s, cmd); pause(0.05);
    while s.NumBytesAvailable > 0, readline(s); end
end

function esperar(s, palabra, tmax)
    t0 = tic;
    while toc(t0) < tmax
        if s.NumBytesAvailable > 0
            if contains(readline(s), palabra), return; end
        else, pause(0.01);
        end
    end
    error("No llegó '%s' del ESP32", palabra);
end

function [datos, ok, meta] = leerBloque(s, ini, fin, tmax)
    datos = strings(0,1); ok = false; meta = ""; dentro = false; t0 = tic;
    while toc(t0) < tmax
        if s.NumBytesAvailable == 0, pause(0.005); continue; end
        l = strtrim(readline(s));
        if startsWith(l, "ERROR"), fprintf("  ESP32: %s\n", l); return; end
        if l == ini, dentro = true; continue; end
        if l == fin, ok = true; return; end
        if dentro
            if startsWith(l, "#"), meta = l; else, datos(end+1,1) = l; end %#ok<AGROW>
        end
    end
    warning("Tiempo agotado esperando %s", fin);
end

function T = parsear(lineas)
    nombres = split(lineas(1), ",")';
    M = str2double(split(lineas(2:end), ","));
    T = array2table(M, 'VariableNames', cellstr(nombres));
end

function cerrar(s), try, writeline(s, "STOP"); catch, end, end
