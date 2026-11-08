%% ADQUIRIR_ESCALON  Pruebas de escalón PWM -> corriente con el ESP32
%  (programa identificacion_escalon.ino, ROTOR BLOQUEADO)
%
%  1) Edita el bloque "LO QUE CAMBIAS".
%  2) Sujeta el eje del motor (rotor bloqueado).
%  3) Corre el script. Cada escalón se guarda en datos/ como CSV.
%  4) Después corre identificar_escalon.m
%
%  OJO: abrir el puerto reinicia el ESP32 (normal).

clear; clc;

%% ===================== LO QUE CAMBIAS =====================
puerto = "COM14";          % puerto del ESP32
baud   = 921600;

% Barrido estático primero (para ver la zona muerta y la corriente máxima).
% [a b paso]; deja [] para omitirlo.
estatica = [0 600 50];

% Escalones: cada fila [a b N]  (a -> b, N repeticiones promediadas)
% PRIMERA CORRIDA: pon plan = [] para hacer solo la estática, mira la
% gráfica y luego ajusta a y b (zona muerta < a < b < PWM de ~650 mA).
% Usa a y b FUERA de la zona muerta y por debajo de ~700 mA.
% Positivos = sentido IN1, negativos = sentido IN2.
plan = [ 200  400  10
         300  500  10
         200  600  10
        -200 -400  10
        -300 -500  10
        -200 -600  10 ];

% Parámetros del ESP32 (se mandan con SET antes de empezar)
TS_us   = 250;    % 4 kHz, igual que el lazo PI
PRE_ms  = 10;
POST_ms = 40;
IMAX_mA = 700;

nota = "motor nuevo, rotor bloqueado";
%% ===========================================================

carpeta = fullfile(fileparts(mfilename('fullpath')), 'datos');
if ~exist(carpeta, 'dir'), mkdir(carpeta); end
sello = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));

s = serialport(puerto, baud, "Timeout", 30);
configureTerminator(s, "LF");
cleanup = onCleanup(@() cerrar(s));
flush(s);

% Esperar el "LISTO" del ESP32 (se reinicia al abrir el puerto)
esperar(s, "LISTO", 10);

enviar(s, sprintf("SET TS %d", TS_us));
enviar(s, sprintf("SET PRE %d", PRE_ms));
enviar(s, sprintf("SET POST %d", POST_ms));
enviar(s, sprintf("SET IMAX %d", IMAX_mA));

%% ---------- Barrido estático ----------
if ~isempty(estatica)
    fprintf("\nBarrido estatico %d..%d paso %d\n", estatica);
    writeline(s, sprintf("ESTATICA %d %d %d", estatica));
    [lineas, ok] = leerBloque(s, "BEGIN_ESTATICA", "END_ESTATICA", 120);
    T = parsear(lineas);
    if ~isempty(T)
        archivo = fullfile(carpeta, sprintf("estatica_%s.csv", sello));
        writetable(T, archivo);
        fprintf("  guardado %s\n", archivo);
        figure('Name', 'Estatica'); plot(T.pwm, T.i_mA, 'o-'); grid on;
        xlabel('PWM'); ylabel('i [mA]'); title('Corriente en estado estable (rotor bloqueado)');
    end
    if ~ok
        warning("El barrido se abortó por sobrecorriente. Revisa el plan antes de seguir.");
        return;
    end
end

%% ---------- Escalones ----------
for r = 1:size(plan, 1)
    a = plan(r,1); b = plan(r,2); N = plan(r,3);
    fprintf("\nEscalon %d/%d: %d -> %d (N = %d)\n", r, size(plan,1), a, b, N);
    writeline(s, sprintf("ESCALON %d %d %d", a, b, N));

    [lineas, ok, meta] = leerBloque(s, "BEGIN_ESCALON", "END_ESCALON", 60);
    if ~ok
        warning("Escalon %d -> %d abortado (sobrecorriente o error). Se detiene el plan.", a, b);
        break;
    end
    T = parsear(lineas);
    archivo = fullfile(carpeta, sprintf("escalon_%+d_%+d_%s.csv", a, b, sello));
    writetable(T, archivo);
    % metadatos en un .txt al lado
    fid = fopen(replace(archivo, ".csv", "_meta.txt"), 'w');
    fprintf(fid, "%s\nnota: %s\n", meta, nota); fclose(fid);
    fprintf("  %s\n  guardado %s\n", meta, archivo);
end

writeline(s, "STOP");
fprintf("\nListo. Ahora corre identificar_escalon.m\n");


%% ===================== FUNCIONES =====================
function enviar(s, cmd)
    writeline(s, cmd);
    pause(0.05);
    while s.NumBytesAvailable > 0
        fprintf("  ESP32: %s\n", strtrim(readline(s)));
    end
end

function esperar(s, palabra, tmax)
    t0 = tic;
    while toc(t0) < tmax
        if s.NumBytesAvailable > 0
            l = strtrim(readline(s));
            if contains(l, palabra), return; end
        else
            pause(0.01);
        end
    end
    error("No llegó '%s' del ESP32. ¿Puerto correcto? ¿Programa cargado?", palabra);
end

function [datos, ok, meta] = leerBloque(s, ini, fin, tmax)
    datos = strings(0,1); ok = false; meta = ""; dentro = false;
    t0 = tic;
    while toc(t0) < tmax
        if s.NumBytesAvailable == 0, pause(0.005); continue; end
        l = strtrim(readline(s));
        if startsWith(l, "*** ABORTADO") || startsWith(l, "ERROR")
            fprintf("  ESP32: %s\n", l); return;
        end
        if l == ini, dentro = true; continue; end
        if l == fin, ok = true; return; end
        if dentro
            if startsWith(l, "#"), meta = l; else, datos(end+1,1) = l; end %#ok<AGROW>
        elseif startsWith(l, "#")
            fprintf("  %s\n", l);           % avance de repeticiones
        end
    end
    warning("Tiempo agotado esperando %s", fin);
end

function T = parsear(lineas)
    T = table();
    if numel(lineas) < 2, return; end
    nombres = split(lineas(1), ",")';
    M = str2double(split(lineas(2:end), ","));
    if isvector(M), M = M(:)'; end
    T = array2table(M, 'VariableNames', cellstr(nombres));
end

function cerrar(s)
    try, writeline(s, "STOP"); catch, end
end
