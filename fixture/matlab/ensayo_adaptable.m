%% ENSAYO_ADAPTABLE  Corre un plan de prueba del PD adaptable y guarda TODO en CSV
%
% Firmware: 03_control_adaptable/control_adaptable/control_adaptable.ino (921600 baud).
%
% En "LO QUE CAMBIAS" eliges:
%   plan  -> qué secuencia se corre (IDENT_MEC, BASE, REPITE, COMPARA_ZM, PERT, CARGA, COMPARA,
%            GAMMA, K2, DOC, MANUAL)
%   modo  -> cómo se usa la adaptación (ADAPTZM, ADAPT, PD, PDG)
%   usarSintonia -> toma p, d, k1, k2, gamma, ZMIN y ZMOUT de datos/sintonia_recomendada.mat
%
% ORDEN PARA EL MOTOR NUEVO:
%   1) plan = "COMPARA_ZM" (ganancias finales escritas abajo) -> analisis_adaptable.m
%   2) plan = "PERT_ZM"    perturbación con ZM (ZMIN 0.25, ZMOUT 1.0) y sin ZM, phi1 inicial = phi1ini
%   ganancias y nota.
% Los planes usan REFERENCIAS NEGATIVAS (las del proyecto), igual que el PID homogéneo.
%
% Resultado en matlab/datos/:
%   <nombre>.csv          t_ms,qd_deg,q_deg,w_dps,u_mA,iref_mA,i_mA,phi1_mA,phi2_mA,d_mA,flags
%   <nombre>_eventos.csv  t_ms,evento   (comandos, marcas, fallas)
%   <nombre>_meta.txt     configuración, línea P del ESP32, nota, fecha
%   <nombre>_log.txt      todo lo que llegó por el puerto (sin las líneas D)
%
%   u    = tau del control (saturado) [mA]
%   iref = corriente pedida al lazo interno (con signo, incluye d e Idz)
%   i    = corriente medida por el INA226 (magnitud, lado de la fuente)
%   d    = perturbación artificial (el control NO la conoce)
%   Término adaptable: sin(q)*phi1 + phi2   (se calcula en analisis_adaptable.m)
%
% flags (bits): 1 control activo | 2 saturado | 4 adapta phi1 | 8 adapta phi2 |
%               16 ZM congelada | 32 rampa moviéndose | 64 phi en su límite
%
% Para analizar: analisis_adaptable.m (métricas por comando, brincos y figuras).
%
% ANTES DE CORRERLO
%   - Carga control_adaptable.ino y CIERRA el Monitor Serie.
%   - La carga debe estar COLGANDO Y QUIETA: abrir el puerto reinicia el
%     ESP32 y el encoder vuelve a 0 ahí (q = 0). phi1 y phi2 arrancan en 0.
%   - OJO: si tenías este archivo abierto en el editor cuando se actualizó,
%     ciérralo sin guardar y vuelve a abrirlo.

clear; clc;

%% ======================= LO QUE CAMBIAS =======================
puerto = "COM14";          % tu puerto

plan = "COMPARA_ZM";        % "PERT_ZM"   perturbaciones en -90 y -45: primero ADAPTZM, luego ADAPT (sin ZM)
                           % "IDENT_MEC" identificación mecánica: despegues (BREAK), rampas lentas, escalones
                           % "COMPARA_ZM" recorrido + repeticiones con ADAPT, ADAPTZM1 y ADAPTZM (modo 2)
                           % "BASE"    recorrido 0/-45/-90/-135/-90/-30/0 (seguimiento con rampa)
                           % "REPITE"  0 -> -90 -> 0 cinco veces (repetibilidad, brincos/stick-slip)
                           % "PERT"    perturbación desconocida DIST ±dpert y DSTEP en -90 y en -45
                           % "CARGA"   carga REAL: te pide poner/quitar el objeto en -90
                           % "COMPARA" mismo recorrido con ADAPT, ADAPTZM, PDG y PD en una corrida
                           % "GAMMA"   mismo recorrido con varios gamma (lista "gammas"), k2 fijo
                           % "K2"      varios k2 manteniendo k2*gamma constante (lista "k2s")
                           % "DOC"     corrida de documentación: seguimiento + perturbación (secciones con MARCA)
                           % "MANUAL"  escribes los comandos tú; FIN para terminar

modo = "ADAPTZM";          % "ADAPTZM" adaptación + zona muerta (ZMMODO zmmodo)  <- el que se prueba
                           % "ADAPT"   adaptación sin zona muerta (la cacería aparece aquí)
                           % "PD"      sin adaptación, phi1 = phi2 = 0 (referencia: ¿qué aporta la adaptación?)
                           % "PDG"     sin adaptación, phi1 = mg fijo (gravedad conocida, comparación)

p     = 1902;              % [mA/rad]
d     = 100;               % [mA*s/rad]
k1    = 476;               % [mA/rad]  adaptación de phi1 (gravedad, regresor sin q)
k2    = 476;               % [mA/rad]  adaptación de phi2 (par constante)
gamma = 32.61;             % [1/s]     integral equivalente = k2*gamma
phimax1 = 700;             % [mA] límite de |phi1|
phimax2 = 200;             % [mA] límite de |phi2|
zmin  = 0.25;              % [grados]   ZM: congela si |e| < zmin (justo arriba de 1 cuenta = 0.18)
zmout = 1.0;               % [grados]   ZM: vuelve a adaptar si |e| > zmout (con 2.0 no se rechazan
                           %            perturbaciones menores a ~100 mA: la carga se queda desviada)
usarZMdeSintonia = false;  % false: usa zmin/zmout de aquí aunque usarSintonia = true
phi1ini = 38;              % [mA] phi1 inicial en ADAPT/ADAPTZM
zmvel = 5;                 % [grados/s] ZM: "quieto"
zmmodo = 2;                % 1 = congela toda la adaptación | 2 = solo la integral (recomendado)
usarSintonia = false;      % true: p, d, k1, k2, gamma, zmin, zmout de datos/sintonia_recomendada.mat
mg    = 140;               % [mA] solo modo PDG (estimado de E3)
vf    = 8;                 % [Hz] filtro de velocidad
vel   = 90;                % [grados/s] velocidad de la rampa
idz   = 35;                % [mA]
logdt = 2;                 % [ms] periodo de registro (2 ms = 500 Hz)

dpert  = 60;               % [mA] perturbación chica (≈ gravedad m; fricción estática ~30-45 mA)
dpert2 = 120;              % [mA] perturbación grande (≈ 3 m)
gammas = [8 13 20];        % plan GAMMA
k2s    = [150 300 600];    % plan K2 (gamma = k2*gamma/k2 de cada uno)

nota  = "";                % texto libre (p. ej. "placa de 50 g")
%% ===============================================================

baud = 921600;

aqui = fileparts(mfilename("fullpath"));
fs = fullfile(aqui, "datos", "sintonia_recomendada.mat");
if usarSintonia && isfile(fs) && upper(plan) ~= "IDENT_MEC"
    S = load(fs);
    p = round(S.p); d = round(S.d); k1 = round(S.k1); k2 = round(S.k2); gamma = round(S.gamma, 2);
    if usarZMdeSintonia, zmin = round(S.E_IN, 2); zmout = round(S.E_OUT, 2); end
    fprintf("Ganancias de sintonia_recomendada.mat (w = %g rad/s)\n", S.w);
elseif usarSintonia && upper(plan) ~= "IDENT_MEC"
    warning("No existe datos/sintonia_recomendada.mat: uso las ganancias escritas arriba.");
end

%% ---------------- Configuración del modo ----------------
switch upper(modo)
    case "ADAPT",   adOn = 1; zm = 0; phi1 = phi1ini;
    case "ADAPTZM", adOn = 1; zm = 1; phi1 = phi1ini;
    case "PD",      adOn = 0; zm = 0; phi1 = 0;
    case "PDG",     adOn = 0; zm = 0; phi1 = mg;
    otherwise, error("Modo desconocido: %s", modo);
end

config = [
    "STOP"
    "MON OFF"
    "P " + p
    "D " + d
    "K1 " + k1
    "K2 " + k2
    "G " + gamma
    "PHIMAX1 " + phimax1
    "PHIMAX2 " + phimax2
    "ADAPT " + onoff(adOn)
    "ZM " + onoff(zm)
    "ZMIN " + zmin
    "ZMOUT " + zmout
    "ZMVEL " + zmvel
    "ZMMODO " + zmmodo
    "VF " + vf
    "RAMPA ON"
    "VEL " + vel
    "IDZ " + idz
    "LOGDT " + logdt
    "DIST 0"
    "PHIRESET"
    "PHI1 " + phi1
];

% Comandos que dejan cada modo listo (para los planes que comparan modos)
cfgModo = struct( ...
    "ADAPT",   ["ADAPT ON";  "ZM OFF"; "PHIRESET"; "PHI1 " + phi1ini], ...
    "ADAPTZM", ["ADAPT ON";  "ZM ON";  "ZMMODO 2"; "PHIRESET"; "PHI1 " + phi1ini], ...
    "ADAPTZM1",["ADAPT ON";  "ZM ON";  "ZMMODO 1"; "PHIRESET"; "PHI1 " + phi1ini], ...
    "PDG",     ["ADAPT OFF"; "ZM OFF"; "PHIRESET"; "PHI1 " + mg], ...
    "PD",      ["ADAPT OFF"; "ZM OFF"; "PHIRESET"]);

%% ---------------- Planes ----------------
% Cada fila: comando, segundos a esperar después.
% "PAUSA:<mensaje>" detiene el script hasta que presiones Enter.
recorrido = {"POS -45",4; "POS -90",4; "POS -135",5; "POS -90",4; "POS -30",4; "POS 0",5};
switch upper(plan)
    case "IDENT_MEC"
        % 1) despegues en q = 0 (carga colgando y quieta), 3 por sentido
        P = {"MARCA BREAK",0.1};
        for r = 1:3
            P = [P; {"BREAK 1 100",6; "BREAK -1 100",6}]; %#ok<AGROW>
        end
        % 2) rampas lentas con el PD sin adaptación (phi = 0)
        P = [P; {"ADAPT OFF",0.1; "PHIRESET",0.1; "ZM OFF",0.1; "VEL 10",0.1; "START",2; ...
                 "MARCA RAMPAS",0.1; "POS -150",17; "POS 0",17}];
        % 3) escalones sin rampa (aceleraciones grandes) en -45 y -90
        P = [P; {"VEL 90",0.1; "POS -45",4; "RAMPA OFF",0.1; "MARCA ESCALONES",0.1}];
        for r = 1:3
            P = [P; {"POS -35",1.5; "POS -45",1.5}]; %#ok<AGROW>
        end
        P = [P; {"RAMPA ON",0.1; "POS -90",3; "RAMPA OFF",0.1}];
        for r = 1:3
            P = [P; {"POS -80",1.5; "POS -90",1.5}]; %#ok<AGROW>
        end
        P = [P; {"MARCA FIN",0.1; "RAMPA ON",0.1; "POS 0",4}];
    case "PERT_ZM" % perturbación desconocida (DIST) con ZM y sin ZM; cada modo arranca con phi1 = phi1ini, phi2 = 0
        P = {"START",2};
        for mname = ["ADAPTZM", "ADAPT"]
            for c = cfgModo.(mname).'
                P = [P; {c, 0.2}]; %#ok<AGROW>
            end
            P = [P; {"POS -90",5; "MARCA PERT90_" + mname,0.1; ...
                     "DIST " + dpert,6; "DIST 0",5; "DIST " + (-dpert),6; "DIST 0",5; ...
                     "DIST " + dpert2,6; "DIST 0",5; "DIST " + (-dpert2),6; "DIST 0",5; ...
                     "DSTEP " + dpert2 + " 1500",5; ...
                     "POS -45",4; "MARCA PERT45_" + mname,0.1; ...
                     "DIST " + dpert,6; "DIST 0",5; "DIST " + dpert2,6; "DIST 0",5; ...
                     "POS 0",5}]; %#ok<AGROW>
        end
    case "COMPARA_ZM" % mismo recorrido + 3 idas y vueltas a -90, sin ZM y con ZM modo 1 y 2
        modos = ["ADAPT", "ADAPTZM1", "ADAPTZM"];
        P = {"START",2};
        for mname = modos
            for c = cfgModo.(mname).'
                P = [P; {c, 0.2}]; %#ok<AGROW>
            end
            P = [P; {"MARCA MODO_" + mname, 0.1}; recorrido; ...
                 {"POS -90",6; "POS 0",6; "POS -90",6; "POS 0",6; "POS -90",6; "POS 0",6}]; %#ok<AGROW>
        end
    case "BASE"
        P = [{"START",2; "MARCA SEGUIMIENTO",0.1}; recorrido];
    case "REPITE"
        P = {"START",2; "MARCA REPITE",0.1};
        for r = 1:5
            P = [P; {"POS -90",5; "POS 0",5}]; %#ok<AGROW>
        end
    case "PERT"
        P = {"START",2; "POS -90",5; "MARCA PERTURBACION_90",0.1; ...
             "DIST " + dpert,6; "DIST 0",5; "DIST " + (-dpert),6; "DIST 0",5; ...
             "DSTEP " + dpert + " 1500",5; ...
             "POS -45",4; "MARCA PERTURBACION_45",0.1; "DIST " + dpert,6; "DIST 0",5; ...
             "MARCA FIN",0.1; "POS 0",5};
    case "CARGA"   % mismo recorrido SIN y CON carga real; la carga se pone y se quita en -90
        rc = {"POS 0",5; "POS -45",4; "POS -90",4; "POS -135",5; "POS -90",5};
        P = [{"START",2; "MARCA SIN_CARGA",0.1}; rc; ...
             {"PAUSA:Pon la carga en el sujetador SIN mover el brazo y presiona Enter",0; ...
              "MARCA CARGA_PUESTA",6}; rc; ...
             {"PAUSA:Quita la carga SIN mover el brazo y presiona Enter",0; ...
              "MARCA CARGA_QUITADA",6; "POS 0",5}];
    case "COMPARA" % el mismo recorrido con 4 modos (cada uno empieza colgando en 0 con phi = 0)
        modos = ["ADAPT", "ADAPTZM", "PDG", "PD"];
        P = {"START",2};
        for mname = modos
            for c = cfgModo.(mname).'
                P = [P; {c, 0.2}]; %#ok<AGROW>
            end
            P = [P; {"MARCA MODO_" + mname, 0.1}; recorrido]; %#ok<AGROW>
        end
        for c = config(startsWith(config, ["ADAPT ", "ZM "])).'      % deja el modo elegido
            P = [P; {c, 0.2}]; %#ok<AGROW>
        end
    case "GAMMA"   % E4: misma k2, distinta integral equivalente k2*gamma
        P = {"START",2};
        for g = gammas
            P = [P; {"PHIRESET",0.2; "G " + g,0.2; "MARCA GAMMA_" + g,0.1}; recorrido]; %#ok<AGROW>
        end
        P = [P; {"G " + gamma, 0.2}];
    case "K2"      % misma integral k2*gamma, distinto "freno" k2
        ki = k2 * gamma;
        P = {"START",2};
        for kk = k2s
            gg = round(ki / kk, 3);
            P = [P; {"PHIRESET",0.2; "K2 " + kk,0.2; "G " + gg,0.2; ...
                     "MARCA K2_" + kk,0.1}; recorrido]; %#ok<AGROW>
        end
        P = [P; {"K2 " + k2, 0.2; "G " + gamma, 0.2}];
    case "DOC"     % documentación (~85 s). Las MARCA separan las secciones para las gráficas.
        P = [{"START",2; "MARCA SEGUIMIENTO",0.1}; recorrido(1:5,:); ...
             {"MARCA PERTURBACION_90",0.1; "POS -90",4; ...
              "DIST " + dpert,6; "DIST 0",5; "DIST " + (-dpert),6; "DIST 0",5; ...
              "DSTEP " + dpert + " 1500",5; ...
              "MARCA PERTURBACION_45",0.1; "POS -45",4; "DIST " + dpert,6; "DIST 0",5; ...
              "MARCA FIN",0.1; "POS 0",5}];
    case "MANUAL"
        P = {"START",1};
    otherwise
        error("Plan desconocido: %s", plan);
end

%% ---------------- Nombre ----------------
base = sprintf("%s_%s_P%g_D%g_K1-%g_K2-%g_G%g", upper(plan), upper(modo), p, d, k1, k2, gamma);
if zm, base = base + sprintf("_ZM%g-%g_M%d", zmin, zmout, zmmodo); end
if upper(modo) == "PDG", base = base + "_MG" + mg; end
if strlength(nota) > 0, base = base + "_" + regexprep(nota, "[^\w]", ""); end
base = strrep(string(base), ".", "_");

carpeta = fullfile(aqui, "datos");
if ~isfolder(carpeta), mkdir(carpeta); end
nombre = base;  r = 1;
while isfile(fullfile(carpeta, nombre + ".csv"))
    r = r + 1;  nombre = base + "_r" + r;
end
fprintf("Plan %s | modo %s -> '%s'\n", plan, modo, nombre);
fprintf("p=%g d=%g k1=%g k2=%g gamma=%g (k2*gamma=%g) | rampa %g grados/s | registro cada %g ms\n", ...
        p, d, k1, k2, gamma, k2*gamma, vel, logdt);

%% ---------------- Conexión ----------------
s = serialport(puerto, baud, "Timeout", 5);
configureTerminator(s, "LF");
limpiar = onCleanup(@() cerrar(s));
pause(2.5);                         % reinicio del ESP32 (q = 0 aquí)
flush(s);

B = struct("pend", "", "D", strings(0,1), "otras", strings(0,1));

for c = config.'
    writeline(s, c);  pause(0.12);
end
B = leerPor(s, 0.5, B);
writeline(s, "LOG ON");
B = leerPor(s, 0.5, B);

%% ---------------- Plan ----------------
t0 = tic;
try
    if upper(plan) == "MANUAL"
        writeline(s, "START");
        B = leerPor(s, 1, B);
        disp("Modo MANUAL: escribe comandos (POS 90, DIST 80, MARCA texto, ...). FIN para terminar.");
        while true
            cmd = string(input("> ", "s"));
            B = leerPor(s, 0, B);                   % lo acumulado mientras escribías
            if upper(strtrim(cmd)) == "FIN", break; end
            if strlength(strtrim(cmd)) == 0, continue; end
            writeline(s, cmd);
            B = leerPor(s, 0.3, B);
            mostrarRespuestas(B);
        end
    else
        for k = 1:size(P, 1)
            cmd = string(P{k, 1});  espera = P{k, 2};
            if startsWith(cmd, "PAUSA:")
                writeline(s, "MARCA PAUSA");
                input(char("  >> " + extractAfter(cmd, "PAUSA:") + " "), "s");
                B = leerPor(s, 0, B);
                continue;
            end
            fprintf("[%5.1f s] %s\n", toc(t0), cmd);
            writeline(s, cmd);
            B = leerPor(s, espera, B);
            if any(startsWith(B.otras, "***"))
                warning("El ESP32 reportó una FALLA; se detiene el plan.");
                break;
            end
        end
    end
catch ME
    warning("Interrumpido: %s", ME.message);
end

writeline(s, "LOG OFF");
confirmado = false;
for k = 1:15                                       % espera la confirmación (máx ~4.5 s)
    B = leerPor(s, 0.3, B);
    if any(contains(B.otras, "LOG OFF")), confirmado = true; break; end
end
if ~confirmado
    warning("El ESP32 no confirmó LOG OFF: la comunicación se cortó antes del final (revisa _log.txt).");
end
writeline(s, "STOP");
B = leerPor(s, 0.3, B);

%% ---------------- Guardar CSV ----------------
cols = ["t_ms","qd_deg","q_deg","w_dps","u_mA","iref_mA","i_mA","phi1_mA","phi2_mA","d_mA","flags"];
if isempty(B.D)
    fid = fopen(fullfile(carpeta, nombre + "_log.txt"), "w"); fprintf(fid, "%s\n", B.otras); fclose(fid);
    error("No llegó ninguna línea D (¿firmware correcto?, ¿921600 baud?). Revisa datos/%s_log.txt", nombre);
end
valida = count(B.D, ",") == 11 & count(B.D, "D,") == 1;    % líneas completas
nMalas = sum(~valida);
if nMalas > 0
    warning("%d líneas D incompletas o pegadas (se descartan solo esas).", nMalas);
    B.otras = [B.otras; "MALA: " + B.D(~valida)];
end
txt = char(strjoin(extractAfter(B.D(valida), 2), newline));
M = textscan(txt, repmat('%f', 1, 11), 'Delimiter', ',', 'CollectOutput', true);
M = M{1};
M = M(all(isfinite(M), 2), :);                      % descarta líneas cortadas
M(:, 2:3) = M(:, 2:3) / 1000;                       % mili-grados   -> grados
M(:, 4)   = M(:, 4)   / 100;                        % centi-grados/s -> grados/s
M(:, 5:10) = M(:, 5:10) / 10;                       % décimas de mA -> mA
T = array2table(M, "VariableNames", cols);
writetable(T, fullfile(carpeta, nombre + ".csv"));

ev  = B.otras(startsWith(B.otras, "E,"));
tok = regexp(ev, '^E,(\d+),(.*)$', 'tokens', 'once');
if ~iscell(tok), tok = {tok}; end
ok  = ~cellfun(@isempty, tok);
tok = tok(ok);
evT = table(cellfun(@(c) str2double(c{1}), tok(:)), ...
            string(cellfun(@(c) char(c{2}), tok(:), 'UniformOutput', false)), ...
            'VariableNames', ["t_ms", "evento"]);
writetable(evT, fullfile(carpeta, nombre + "_eventos.csv"));

Pline = B.otras(startsWith(B.otras, "P,"));
fid = fopen(fullfile(carpeta, nombre + "_meta.txt"), "w");
fprintf(fid, "nombre=%s\nplan=%s\nmodo=%s\nnota=%s\nfecha=%s\n", nombre, plan, modo, nota, string(datetime("now")));
fprintf(fid, "firmware=control_adaptable (Codigos_Nuevo_Motor/03)\n");
fprintf(fid, "config:\n");  fprintf(fid, "  %s\n", config);
if ~isempty(Pline), fprintf(fid, "%s\n", Pline(1)); end
fclose(fid);

fid = fopen(fullfile(carpeta, nombre + "_log.txt"), "w");
fprintf(fid, "%s\n", B.otras);
fclose(fid);

perd = B.otras(contains(B.otras, "perdidas="));
if ~isempty(evT) && height(T) > 0 && T.t_ms(end) < max(evT.t_ms) - 100
    warning("Los datos terminan en %.2f s pero hay eventos hasta %.2f s: faltan datos al final.", ...
            T.t_ms(end)/1000, max(evT.t_ms)/1000);
end
fprintf("Guardado: datos/%s.csv (%d muestras, %.1f s) + _eventos, _meta, _log\n", ...
        nombre, height(T), (T.t_ms(end) - T.t_ms(1)) / 1000);
if ~isempty(perd), fprintf("ESP32: %s\n", perd(end)); end
if height(T) > 1
    huecos = sum(diff(T.t_ms) > 1.5 * logdt);
    if huecos > 0, warning("%d huecos en el tiempo (muestras perdidas; revisa perdidas del ESP32).", huecos); end
end

%% ---------------- Gráfica rápida ----------------
if height(T) > 10
    t = T.t_ms / 1000;
    figure("Name", nombre, "Color", "w");
    ax(1) = subplot(4,1,1); plot(t, T.qd_deg, "k--", t, T.q_deg, "b"); ylabel("q [°]"); legend("qd","q"); grid on;
    ax(2) = subplot(4,1,2); plot(t, T.q_deg - T.qd_deg, "r"); ylabel("e [°]"); grid on; ylim([-5 5]);
    adap = sind(T.q_deg) .* T.phi1_mA + T.phi2_mA;
    ax(3) = subplot(4,1,3); plot(t, T.u_mA, "b", t, adap, "m", t, T.phi1_mA, "c", t, T.phi2_mA, "Color", [0.9 0.5 0]);
    hold on; plot(t, T.i_mA, "Color", [0.6 0.6 0.6]);
    ylabel("[mA]"); legend("u", "sin(q)\phi_1+\phi_2", "\phi_1", "\phi_2", "i medida"); grid on;
    ax(4) = subplot(4,1,4); plot(t, T.d_mA, "k", t, T.w_dps, "g"); ylabel("d [mA], w [°/s]"); legend("d","w"); grid on;
    xlabel("t [s]");
    for k = 1:height(evT)
        for a = ax, xline(a, evT.t_ms(k)/1000, ":", "Color", [0.5 0.5 0.5]); end
    end
    linkaxes(ax, "x");
end

%% ---------------- Funciones ----------------
function B = leerPor(s, T, B)
    % Lee todo lo que llega durante T segundos (T = 0: solo lo disponible)
    t = tic;
    while true
        n = s.NumBytesAvailable;
        if n > 0
            chunk = string(read(s, n, "char"));
            txt = B.pend + chunk;
            L = split(txt, newline);
            B.pend = L(end);
            L = strtrim(L(1:end-1));
            L = L(strlength(L) > 0);
            esD = startsWith(L, "D,");
            B.D = [B.D; L(esD)];
            B.otras = [B.otras; L(~esD)];
        end
        if toc(t) >= T, break; end
        pause(0.02);
    end
end

function mostrarRespuestas(B)
    k = numel(B.otras);
    i0 = max(1, k - 3);
    for j = i0:k
        if ~startsWith(B.otras(j), ["E,", "P,", "H,"]), fprintf("  ESP32: %s\n", B.otras(j)); end
    end
end

function t = onoff(x)
    if x, t = "ON"; else, t = "OFF"; end
end

function cerrar(s)
    try, writeline(s, "LOG OFF"); catch, end
    try, writeline(s, "STOP"); catch, end
    delete(s);
end
