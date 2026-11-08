%% ANALISIS_ADAPTABLE  Métricas, brincos y figuras de una corrida de ensayo_adaptable.m
%
% Lee datos/<archivo>.csv y datos/<archivo>_eventos.csv y genera en figuras/<archivo>/:
%   01_corrida_completa   posición, error, control/término adaptable/phi1/phi2/d, corriente
%   NN_<MARCA>            una figura por sección (cada MARCA abre una sección:
%                         MODO_*, GAMMA_*, K2_*, PERTURBACION_*, CARGA_PUESTA, ...)
%   metricas.csv          una fila por comando POS/DIST/DSTEP
%   resumen_secciones.csv promedios por sección (para comparar modos/ganancias)
%
% Métricas de cada POS (desde el FIN de la rampa, flag 32):
%   sobrepaso_deg      cuánto se pasa del objetivo
%   error_rampa_deg    error máximo mientras la rampa se mueve (retraso)
%   t_asent_s          tiempo hasta quedar dentro de ±tol para siempre
%   error_final_deg    promedio del error en los últimos 0.5 s de la ventana
%   iae_deg_s          integral de |e| en la ventana
%   brincos            veces que la carga se despega después de haberse detenido
%                      (stick-slip): quieta |w| < wQuieto durante >= tQuieto y
%                      luego |w| > wMov y se desplaza al menos aMin
%   brinco_max_deg     el desplazamiento más grande de esos despegues
%   phi1/phi2/adapt_final_mA, i_rms_final_mA
% Para DIST/DSTEP: desviación máxima y tiempo de recuperación a ±tol.

clear; clc;

%% ======================= LO QUE CAMBIAS =======================
archivo = "";              % "" = la corrida más reciente; o el nombre sin extensión
tol     = 1.5;             % [grados] banda de asentamiento (con zona muerta, ~E_IN o un poco más)
tolPert = 0.5;             % [grados] banda de recuperación después de una perturbación (DIST/DSTEP)
wQuieto = 2;               % [grados/s] por debajo: carga detenida
tQuieto = 0.10;            % [s] tiempo mínimo detenida antes de contar un despegue
wMov    = 10;              % [grados/s] por encima: se despegó
aMin    = 0.6;             % [grados] desplazamiento mínimo para contarlo (~2 cuentas)
formato = "png";           % "png" | "pdf" (vectorial, para LaTeX)
%% ===============================================================

aqui    = fileparts(mfilename("fullpath"));
carpeta = fullfile(aqui, "datos");
if strlength(archivo) == 0
    L = dir(fullfile(carpeta, "*.csv"));
    L = L(~endsWith({L.name}, "_eventos.csv"));
    if isempty(L), error("No hay corridas en %s", carpeta); end
    [~, k]  = max([L.datenum]);
    archivo = erase(string(L(k).name), ".csv");
end
fprintf("Corrida: %s\n", archivo);

T = readtable(fullfile(carpeta, archivo + ".csv"));
E = readtable(fullfile(carpeta, archivo + "_eventos.csv"), "TextType", "string", "Delimiter", ",");
salida = fullfile(aqui, "figuras", archivo);
if ~isfolder(salida), mkdir(salida); end

CUENTA = 360 / 1975;                       % [grados] resolución del encoder
t    = T.t_ms / 1000;
e    = T.q_deg - T.qd_deg;
fl   = T.flags;
adap = sind(T.q_deg) .* T.phi1_mA + T.phi2_mA;     % término adaptable [mA]
tEv  = E.t_ms / 1000;
tFin = t(end);
dtm  = median(diff(t));

esAcc = startsWith(E.evento, ["POS ", "DIST ", "DSTEP "]);
Acc   = E(esAcc, :);
Mar   = E(startsWith(E.evento, "MARCA "), :);

% Sección de cada instante: la última MARCA anterior
secDe = @(tt) seccionDe(tt, Mar);

%% ---------------- Métricas por comando ----------------
n = height(Acc);
cmd = strings(n,1); sec = strings(n,1); t0v = nan(n,1); ref = nan(n,1);
sobre = nan(n,1); eRampa = nan(n,1); tAs = nan(n,1); eFin = nan(n,1); iae = nan(n,1);
nBr = nan(n,1); brMax = nan(n,1); p1F = nan(n,1); p2F = nan(n,1); adF = nan(n,1); iRms = nan(n,1);
for k = 1:n
    a = Acc.t_ms(k) / 1000;
    b = tFin;
    if k < n, b = Acc.t_ms(k+1) / 1000; end
    sel = find(t >= a & t < b);
    if numel(sel) < 20, continue; end
    ts = t(sel); qs = T.q_deg(sel); es = e(sel); ws = T.w_dps(sel);
    cmd(k) = Acc.evento(k);  sec(k) = secDe(a);  t0v(k) = a;  ref(k) = T.qd_deg(sel(end));
    cola = ts > b - 0.5;
    eFin(k) = mean(es(cola));
    p1F(k)  = mean(T.phi1_mA(sel(cola)));
    p2F(k)  = mean(T.phi2_mA(sel(cola)));
    adF(k)  = mean(adap(sel(cola)));
    iRms(k) = sqrt(mean(T.i_mA(sel(cola)).^2));
    iae(k)  = sum(abs(es)) * dtm;
    if startsWith(cmd(k), "POS")
        rampa = bitand(fl(sel), 32) > 0;
        iFinR = find(~rampa, 1);                   % fin de la rampa
        if isempty(iFinR), continue; end
        if any(rampa), eRampa(k) = max(abs(es(rampa))); else, eRampa(k) = 0; end
        q0 = T.q_deg(find(t < a, 1, "last"));
        if isempty(q0), q0 = qs(1); end
        post = iFinR:numel(qs);
        if ref(k) >= q0, sobre(k) = max(0, max(qs(post)) - ref(k));
        else,            sobre(k) = max(0, ref(k) - min(qs(post))); end
        u = find(abs(es(post)) > tol, 1, "last");
        if isempty(u), tAs(k) = 0;
        elseif u == numel(post), tAs(k) = NaN;    % no se asentó en la ventana
        else, tAs(k) = ts(post(u)) - ts(iFinR); end
        [nBr(k), brMax(k)] = brincos(ws(post), qs(post), dtm, wQuieto, tQuieto, wMov, aMin);
    else
        eRampa(k) = max(abs(es));                  % desviación máxima por la perturbación
        u = find(abs(es) > tolPert, 1, "last");
        if isempty(u), tAs(k) = 0;
        elseif u == numel(es), tAs(k) = NaN;
        else, tAs(k) = ts(u) - a; end
        [nBr(k), brMax(k)] = brincos(ws, qs, dtm, wQuieto, tQuieto, wMov, aMin);
    end
end
M = table(sec, cmd, t0v, ref, sobre, eRampa, tAs, eFin, iae, nBr, brMax, p1F, p2F, adF, iRms, ...
    'VariableNames', ["seccion","comando","t_s","ref_deg","sobrepaso_deg","error_rampa_deg", ...
                      "t_asent_s","error_final_deg","iae_deg_s","brincos","brinco_max_deg", ...
                      "phi1_final_mA","phi2_final_mA","adapt_final_mA","i_rms_final_mA"]);
M = M(strlength(M.comando) > 0, :);
writetable(M, fullfile(salida, "metricas.csv"));
disp(M);
fprintf("POS : error_rampa = error máx. durante la rampa; t_asent (±%.2f°) desde el fin de la rampa (NaN = no se asentó)\n", tol);
fprintf("DIST: error_rampa = desviación máxima; t_asent = recuperación a ±%.2f° desde el comando (NaN = no se recuperó)\n", tolPert);

%% ---------------- Resumen por sección ----------------
secs = unique(M.seccion, "stable");
R = table();
for s = secs.'
    m = M(M.seccion == s & startsWith(M.comando, "POS"), :);
    if isempty(m), continue; end
    R = [R; table(s, height(m), mean(m.sobrepaso_deg, "omitnan"), max(m.sobrepaso_deg), ...
                  mean(m.t_asent_s, "omitnan"), sum(isnan(m.t_asent_s)), ...
                  mean(abs(m.error_final_deg)), max(abs(m.error_final_deg)), ...
                  sum(m.iae_deg_s), sum(m.brincos, "omitnan"), max(m.brinco_max_deg), ...
        'VariableNames', ["seccion","n_pos","sobrepaso_medio_deg","sobrepaso_max_deg", ...
                          "t_asent_medio_s","no_asentados","ess_medio_deg","ess_max_deg", ...
                          "iae_total_deg_s","brincos_total","brinco_max_deg"])]; %#ok<AGROW>
end
if ~isempty(R)
    writetable(R, fullfile(salida, "resumen_secciones.csv"));
    disp(R);
end

% ---- Resumen de perturbaciones por sección ----
RP = table();
for s = secs.'
    m = M(M.seccion == s & startsWith(M.comando, ["DIST", "DSTEP"]), :);
    if isempty(m), continue; end
    for k = 1:height(m)
        RP = [RP; table(s, m.comando(k), m.error_rampa_deg(k), m.t_asent_s(k), m.error_final_deg(k), m.brincos(k), ...
              m.phi1_final_mA(k), m.phi2_final_mA(k), m.i_rms_final_mA(k), ...
            'VariableNames', ["seccion","comando","desviacion_max_deg","t_recuperacion_s", ...
                              "error_final_deg","brincos","phi1_final_mA","phi2_final_mA","i_rms_final_mA"])]; %#ok<AGROW>
    end
end
if ~isempty(RP)
    writetable(RP, fullfile(salida, "resumen_perturbaciones.csv"));
    fprintf("\nPERTURBACIONES (recuperación a ±%.2f°):\n", tolPert);
    disp(RP);
end

%% ---------------- Figura 1: corrida completa ----------------
f1 = figure("Name", "Corrida completa", "Color", "w", "Position", [80 60 1000 950]);
ax = gobjects(4,1);
ax(1) = subplot(4,1,1);
plot(t, T.qd_deg, "k--", "LineWidth", 1.0); hold on;
plot(t, T.q_deg, "b", "LineWidth", 1.2);
ylabel("Posición [°]"); legend("q_d", "q", "Location", "best"); grid on;
title(strrep(archivo, "_", "\_"));

ax(2) = subplot(4,1,2);
plot(t, e, "r", "LineWidth", 1.0); hold on;
yline([-tol tol], ":", "Color", [0.4 0.4 0.4]);
ylabel("e = q - q_d [°]"); grid on;
ylim(max(1, ceil(prctile(abs(e), 99.5))) * [-1 1]);

ax(3) = subplot(4,1,3);
plot(t, T.u_mA, "b", t, adap, "m", t, T.phi1_mA, "c", t, T.phi2_mA, "LineWidth", 1.0); hold on;
plot(t, T.d_mA, "k", "LineWidth", 1.4);
ylabel("[mA]"); grid on;
legend("u (control)", "sin(q)\phi_1+\phi_2", "\phi_1", "\phi_2", "d (perturbación)", "Location", "best");

ax(4) = subplot(4,1,4);
plot(t, abs(T.iref_mA), "Color", [0.85 0.33 0.1]); hold on;
plot(t, T.i_mA, "Color", [0.3 0.3 0.3]);
ylabel("Corriente [mA]"); xlabel("t [s]");
legend("|i_{ref}|", "i medida", "Location", "best"); grid on;

for a = ax.'
    sombrear(a, t, T.d_mA);
    for k = 1:height(Mar), xline(a, Mar.t_ms(k)/1000, "-", "Color", [0.75 0.75 0.75]); end
end
linkaxes(ax, "x"); xlim(ax(1), [0 tFin]);
guardar(f1, salida, "01_corrida_completa", formato);

%% ---------------- Una figura por sección (MARCA) ----------------
for km = 1:height(Mar)
    nomSec = extractAfter(Mar.evento(km), "MARCA ");
    if nomSec == "FIN" || nomSec == "PAUSA", continue; end
    a = Mar.t_ms(km) / 1000;
    if km < height(Mar), b = Mar.t_ms(km+1) / 1000; else, b = tFin; end
    sel = t >= a - 0.5 & t <= b;
    if nnz(sel) < 20, continue; end
    f = figure("Name", nomSec, "Color", "w", "Position", [100 80 1000 800]);
    bx = gobjects(4,1);
    bx(1) = subplot(4,1,1);
    plot(t(sel), T.qd_deg(sel), "k--", t(sel), T.q_deg(sel), "b", "LineWidth", 1.2);
    ylabel("Posición [°]"); legend("q_d", "q", "Location", "best"); grid on;
    title(strrep(nomSec, "_", " "));
    bx(2) = subplot(4,1,2);
    plot(t(sel), e(sel), "r", "LineWidth", 1.0); hold on;
    yline([-tol tol], ":", "Color", [0.4 0.4 0.4]);
    ylabel("e [°]"); grid on;
    bx(3) = subplot(4,1,3);
    plot(t(sel), T.u_mA(sel), "b", t(sel), adap(sel), "m", t(sel), T.phi1_mA(sel), "c", ...
         t(sel), T.phi2_mA(sel), "LineWidth", 1.0); hold on;
    plot(t(sel), T.d_mA(sel), "k", "LineWidth", 1.4);
    ylabel("[mA]"); grid on;
    legend("u", "sin(q)\phi_1+\phi_2", "\phi_1", "\phi_2", "d", "Location", "best");
    bx(4) = subplot(4,1,4);
    plot(t(sel), T.w_dps(sel), "g"); hold on;
    zmOn = double(bitand(fl(sel), 16) > 0) * max(1, max(abs(T.w_dps(sel)))) * 0.5;
    if any(zmOn), area(t(sel), zmOn, "FaceColor", [0.6 0.8 1], "FaceAlpha", 0.4, "EdgeColor", "none"); end
    ylabel("w [°/s]"); xlabel("t [s]"); grid on;
    if any(zmOn), legend("velocidad", "ZM congelada", "Location", "best"); end
    for c = bx.'
        sombrear(c, t(sel), T.d_mA(sel));
        ka = find(tEv >= a & tEv <= b & esAcc);
        for j = ka.', xline(c, tEv(j), ":", E.evento(j), "FontSize", 7, ...
                            "LabelVerticalAlignment", "bottom", "Color", [0.5 0.5 0.5]); end
    end
    linkaxes(bx, "x"); xlim(bx(1), [a - 0.5, b]);
    guardar(f, salida, sprintf("%02d_%s", km + 1, nomSec), formato);
end

fprintf("Figuras, metricas.csv y resumen_secciones.csv en: %s\n", salida);

%% ---------------- Funciones ----------------
function s = seccionDe(tt, Mar)
    k = find(Mar.t_ms / 1000 <= tt, 1, "last");
    if isempty(k), s = "(inicio)"; else, s = extractAfter(Mar.evento(k), "MARCA "); end
end

function [nb, amax] = brincos(w, q, dt, wQ, tQ, wM, aMin)
    % Cuenta despegues: la carga estuvo quieta >= tQ, luego |w| > wM y se
    % desplazó al menos aMin antes de volver a detenerse.
    % amax = mayor desplazamiento entre el reposo previo y el siguiente reposo.
    nb = 0;  amax = 0;
    nQ = max(1, round(tQ / dt));
    quieta = abs(w) < wQ;
    cuenta = 0;  armado = false;  k = 1;  N = numel(w);
    while k <= N
        if quieta(k)
            cuenta = cuenta + 1;
            if cuenta >= nQ, armado = true; end
        else
            cuenta = 0;
        end
        if armado && abs(w(k)) > wM
            q0 = q(max(1, k - 1));
            j = k;
            while j <= N && ~quieta(j), j = j + 1; end   % hasta volver a detenerse
            dq = abs(q(min(j, N)) - q0);
            if dq >= aMin, nb = nb + 1;  amax = max(amax, dq); end
            armado = false;  cuenta = 0;  k = j;
            continue;
        end
        k = k + 1;
    end
end

function sombrear(ax, t, d)
    % Sombrea los intervalos con perturbación distinta de cero
    on = d(:) ~= 0;
    if ~any(on), return; end
    dd = diff([0; on; 0]);
    ini = find(dd == 1);  fin = find(dd == -1) - 1;
    yl = ylim(ax);
    hold(ax, "on");
    for k = 1:numel(ini)
        p = patch(ax, [t(ini(k)) t(fin(k)) t(fin(k)) t(ini(k))], [yl(1) yl(1) yl(2) yl(2)], ...
                  [1 0.85 0.4], "FaceAlpha", 0.18, "EdgeColor", "none", "HandleVisibility", "off");
        uistack(p, "bottom");
    end
    ylim(ax, yl);
end

function guardar(f, carpeta, nombre, formato)
    archivo = fullfile(carpeta, nombre + "." + formato);
    if formato == "pdf"
        exportgraphics(f, archivo, "ContentType", "vector");
    else
        exportgraphics(f, archivo, "Resolution", 200);
    end
end
