%% GENERAR_MAPA  Crea control_PI_corriente/mapa_corriente.h con el mapa
%  estático PWM -> corriente (rotor bloqueado) para los dos sentidos.
%
%  Usa los archivos estatica_*.csv de 01_identificacion_corriente/matlab/datos.
%  Para medirlos: en adquirir_escalon.m pon plan = [] y
%     estatica = [0 750 25]     (IN1)  y luego
%     estatica = [0 -750 -25]   (IN2)
%  con IMAX_mA = 750. Si el barrido se aborta por corriente, lo medido se
%  guarda igual y aquí se extrapola con un ajuste cuadrático.

clear; clc;

%% ===================== LO QUE CAMBIAS =====================
carpetaDatos = fullfile(fileparts(mfilename('fullpath')), '..', '..', ...
                        '01_identificacion_corriente', 'matlab', 'datos');
patron   = "estatica_*.csv";
razonNeg = 0.88;     % solo si no hay datos de IN2: IN2 = IN1 con esta ganancia
%% ===========================================================

U = [0:50:1000 1023];
arch = dir(fullfile(carpetaDatos, patron));
if isempty(arch), error("No hay %s en %s", patron, carpetaDatos); end

pos = []; neg = [];
for f = 1:numel(arch)
    E = readtable(fullfile(carpetaDatos, arch(f).name));
    pos = [pos; E.pwm(E.pwm > 0), E.i_mA(E.pwm > 0)];            %#ok<AGROW>
    neg = [neg; -E.pwm(E.pwm < 0), E.i_mA(E.pwm < 0)];           %#ok<AGROW>
    cero = E.i_mA(E.pwm == 0);
    if ~isempty(cero), pos = [pos; zeros(size(cero)), cero]; neg = [neg; zeros(size(cero)), cero]; end %#ok<AGROW>
end
[Ipos, cPos] = curva(pos, U, "IN1 (+)");
if size(neg, 1) > 3
    [Ineg, cNeg] = curva(neg, U, "IN2 (-)");
else
    fprintf("Sin datos de IN2: se usa IN1 x %.2f (PROVISIONAL)\n", razonNeg);
    Ineg = Ipos(1) + razonNeg*(Ipos - Ipos(1)); cNeg = "IN1 x " + razonNeg + " (provisional)";
end

figure('Name', 'Mapa'); hold on; grid on;
plot(pos(:,1), pos(:,2), 'o'); plot(U, Ipos, '-');
if size(neg,1) > 0, plot(neg(:,1), neg(:,2), 's'); end
plot(U, Ineg, '--'); xlabel('|PWM|'); ylabel('i [mA]');
legend('IN1 medido', 'IN1 mapa', 'IN2 medido', 'IN2 mapa', 'Location', 'northwest');

% ---------- escribir el .h ----------
salida = fullfile(fileparts(mfilename('fullpath')), '..', 'control_PI_corriente', 'mapa_corriente.h');
fid = fopen(salida, 'w');
fprintf(fid, "// ============================================================\n");
fprintf(fid, "//  MAPA ESTÁTICO PWM -> CORRIENTE (rotor bloqueado)\n");
fprintf(fid, "//  Generado por generar_mapa.m el %s\n", string(datetime('now')));
fprintf(fid, "//  IN1: %s\n//  IN2: %s\n", cPos, cNeg);
fprintf(fid, "// ============================================================\n#pragma once\n\n");
fprintf(fid, "const int MAP_N = %d;\n\n", numel(U));
fprintf(fid, "const float MAP_U[MAP_N] = {\n  %s };\n\n", strjoin(compose("%g", U), ", "));
fprintf(fid, "const float MAP_I_POS[MAP_N] = {\n  %s };\n\n", strjoin(compose("%.2f", Ipos), ", "));
fprintf(fid, "const float MAP_I_NEG[MAP_N] = {\n  %s };\n", strjoin(compose("%.2f", Ineg), ", "));
fclose(fid);
fprintf("Escrito %s\nCopia también mapI (IN1) en sintonizar_PI.m si quieres re-simular.\n", salida);
fprintf("mapI = [%s];\n", strjoin(compose("%.2f", Ipos), " "));


function [I, desc] = curva(D, U, nombre)
    % promedio por PWM, interpolación donde hay datos, cuadrática después
    [uu, ~, g] = unique(D(:,1));
    ii = accumarray(g, D(:,2), [], @mean);
    m  = uu >= 150 & ii > ii(1) + 10;
    fq = @(p, u) p(1) + p(2)*max(u - p(3), 0).^2;
    p  = fminsearch(@(p) sum((fq(p, uu(m)) - ii(m)).^2), [30 1e-3 50]);
    I  = interp1(uu, ii, U, 'linear');
    fuera = U > max(uu);
    I(fuera) = fq(p, U(fuera)) - fq(p, max(uu)) + ii(end);   % continúa sin salto
    I = cummax(I);                                           % monótono para invertir
    for k = 2:numel(I), if I(k) <= I(k-1), I(k) = I(k-1) + 0.01; end, end
    desc = sprintf("medido hasta PWM %d; arriba extrapolado i = %.2f + %.3e (u - %.0f)^2", max(uu), p);
    fprintf("%s: %s\n", nombre, desc);
end
