%% COMPARAR_MODELO  Escalones medidos del PI SIMC contra la simulación del modelo
%
%  Para cada escalón (a -> b) medido con ensayo_PI.m (plan "SIMC"):
%   - promedia las repeticiones,
%   - simula el MISMO controlador (ganancias leídas del _meta.txt) con el
%     modelo identificado (simular_PI.m: mapa estático + e^(-theta s)/(tau s+1)),
%   - calcula las mismas métricas en los dos: tr (10-90 %), t90, sobrepaso,
%     ts 2 %, ts 5 % (mínimo ±tolMin mA por el ruido), error final,
%   - y qué tanto se parecen las curvas: fit % y error RMS.
%
%  Salidas: tabla en pantalla, datos/comparacion_modelo.csv y dos figuras
%  (corriente y PWM) guardadas en ../analisis/.

clear; clc; close all;

%% ===================== LO QUE CAMBIAS =====================
patron = "SIMC_*.csv";       % p.ej. "SIMC_*20261027_1700*"
tolMin = 2;                  % mA
tau    = 0.985e-3;           % modelo identificado (01_identificacion_corriente)
theta  = 0.366e-3;
Ts     = 250e-6;
%% ===========================================================

aqui    = fileparts(mfilename('fullpath'));
carpeta = fullfile(aqui, 'datos');
A = dir(fullfile(carpeta, patron));
A = A(~contains({A.name}, '_meta') & ~startsWith({A.name}, 'metricas') & ~startsWith({A.name}, 'comparacion'));
if isempty(A), error("No hay archivos %s en %s", patron, carpeta); end

% ---------- mapa estático: el mismo que usa el ESP32 ----------
h = fileread(fullfile(aqui, '..', 'control_PI_corriente', 'mapa_corriente.h'));
mapU    = leerArreglo(h, 'MAP_U');
mapIpos = leerArreglo(h, 'MAP_I_POS');
mapIneg = leerArreglo(h, 'MAP_I_NEG');

% ---------- agrupar archivos por escalón ----------
info = struct('a', {}, 'b', {}, 'arch', {}, 'meta', {});
for f = 1:numel(A)
    [T, meta] = leer_escalon(fullfile(carpeta, A(f).name));
    sg = sign(median(T.u(T.u ~= 0))); if isnan(sg) || sg == 0, sg = 1; end
    a = sg*T.ref(find(T.t_ms < 0, 1, 'last')); b = sg*T.ref(end);
    if isempty(a), a = sg*T.ref(1); end
    k = find(arrayfun(@(s) s.a == a && s.b == b, info), 1);
    if isempty(k)
        info(end+1) = struct('a', a, 'b', b, 'arch', {{A(f).name}}, 'meta', meta); %#ok<SAGROW>
    else
        info(k).arch{end+1} = A(f).name;
    end
end
[~, o] = sort(arrayfun(@(s) s.a*1e4 + s.b, info)); info = info(o);
nE = numel(info);

R = table();
fI = figure('Name', 'Corriente: medido vs modelo', 'Position', [30 30 1400 800]);
fU = figure('Name', 'PWM: medido vs modelo', 'Position', [60 60 1400 800]);
nc = ceil(sqrt(nE*1.6)); nr = ceil(nE/nc);

for e = 1:nE
    a = info(e).a; b = info(e).b; meta = info(e).meta;
    % ---- promedio de las repeticiones ----
    Ts_ = []; I = []; U = [];
    for r = 1:numel(info(e).arch)
        T = leer_escalon(fullfile(carpeta, info(e).arch{r}));
        if isempty(Ts_), Ts_ = T.t_ms; end
        n = min(numel(Ts_), height(T));
        I(1:n, r) = T.i(1:n); U(1:n, r) = abs(T.u(1:n)); %#ok<SAGROW>
    end
    t  = Ts_(1:size(I,1)); im = mean(I, 2); um = mean(U, 2);

    % ---- simulación con el mismo controlador ----
    c = struct('Kp', num(meta, 'kp'), 'Ti', num(meta, 'ti_ms')*1e-3, 'lin', num(meta, 'lin') == 1, ...
               'ff', num(meta, 'ff') == 1, 'taud', num(meta, 'taud_ms')*1e-3);
    mapI = mapIpos; if a < 0 || b < 0, mapI = mapIneg; end
    P = struct('tau', tau, 'theta', theta, 'Ts', Ts, 'mapU', mapU, 'mapI', mapI, 'gain', 1, ...
               'Tend', max(t)*1e-3 + Ts);
    S = simular_PI(c, abs(a), abs(b), P);
    is = interp1(S.t*1e3, S.i, t, 'linear', 'extrap');
    us = interp1(S.t*1e3, S.u, t, 'previous', 'extrap');

    % ---- métricas (mismas funciones para los dos) ----
    mM = metricas(t, im, abs(a), abs(b), tolMin);
    mS = metricas(t, is, abs(a), abs(b), tolMin);
    w  = t >= 0;
    rms = sqrt(mean((im(w) - is(w)).^2));
    fit = 100*(1 - norm(im(w) - is(w))/norm(im(w) - mean(im(w))));
    disp1 = max(max(I(w,:), [], 2) - min(I(w,:), [], 2));       % dispersión entre repeticiones

    R = [R; table(a, b, numel(info(e).arch), mM.tr, mS.tr, mM.t90, mS.t90, mM.os, mS.os, ...
                  mM.ts2, mS.ts2, mM.ts5, mS.ts5, mM.ess, fit, rms, disp1, max(um), max(us), ...
        'VariableNames', {'a','b','rep','tr_med','tr_mod','t90_med','t90_mod','OS_med','OS_mod', ...
                          'ts2_med','ts2_mod','ts5_med','ts5_mod','ess_med','fit_pct','rms_mA', ...
                          'dispersion_mA','umax_med','umax_mod'})]; %#ok<AGROW>

    % ---- gráficas ----
    figure(fI); subplot(nr, nc, e); hold on; grid on;
    plot(t, I, 'Color', [0.78 0.78 0.78]);
    h1 = plot(t, im, 'b', 'LineWidth', 1.5);
    h2 = plot(t, is, '--', 'Color', [0.92 0.41 0.20], 'LineWidth', 1.5);
    stairs(t, abs(a) + (abs(b) - abs(a))*(t >= 0), 'k:');
    title(sprintf('%g\\rightarrow%g  fit %.0f%%  ts5 %.2f/%.2f ms', a, b, fit, mM.ts5, mS.ts5), 'FontSize', 8);
    xlim([-1 min(10, max(t))]); xlabel('t [ms]'); ylabel('|i| [mA]');
    if e == 1, legend([h1 h2], 'medido (prom.)', 'modelo', 'Location', 'southeast'); end

    figure(fU); subplot(nr, nc, e); hold on; grid on;
    plot(t, um, 'b', 'LineWidth', 1.3); stairs(t, us, '--', 'Color', [0.92 0.41 0.20], 'LineWidth', 1.3);
    title(sprintf('%g\\rightarrow%g', a, b), 'FontSize', 8); xlim([-1 min(10, max(t))]);
    xlabel('t [ms]'); ylabel('|PWM|');
    if e == 1, legend('medido', 'modelo', 'Location', 'best'); end
end

fprintf("Tiempos en ms, OS en %%. med = medido (promedio de repeticiones), mod = modelo.\n\n");
disp(R);
fprintf("\nResumen: fit medio %.1f %%, error RMS medio %.1f mA\n", mean(R.fit_pct), mean(R.rms_mA));
fprintf("ts5 medido/modelo: %.2f / %.2f ms (promedio),  ts2: %.2f / %.2f ms\n", ...
        mean(R.ts5_med(isfinite(R.ts5_med))), mean(R.ts5_mod(isfinite(R.ts5_mod))), ...
        mean(R.ts2_med(isfinite(R.ts2_med))), mean(R.ts2_mod(isfinite(R.ts2_mod))));
fprintf("Escalones con ts2 medido < 1 ms: %d de %d\n", nnz(R.ts2_med < 1), height(R));

writetable(R, fullfile(carpeta, 'comparacion_modelo.csv'));
dirA = fullfile(aqui, '..', 'analisis'); if ~exist(dirA, 'dir'), mkdir(dirA); end
exportgraphics(fI, fullfile(dirA, 'simc_medido_vs_modelo_corriente.png'), 'Resolution', 150);
exportgraphics(fU, fullfile(dirA, 'simc_medido_vs_modelo_pwm.png'), 'Resolution', 150);
fprintf("Guardado datos/comparacion_modelo.csv y analisis/simc_medido_vs_modelo_*.png\n");


%% ===================== FUNCIONES =====================
function v = num(meta, campo)
    t = regexp(meta, [campo '=([-\d.]+)'], 'tokens', 'once');
    v = str2double(t{1});
end

function x = leerArreglo(h, nombre)
    t = regexp(h, [nombre '\[MAP_N\]\s*=\s*\{([^}]*)\}'], 'tokens', 'once');
    x = str2double(strsplit(strtrim(t{1}), ','));
    x = x(~isnan(x));
end

function m = metricas(t, y, a, b, tolMin)
    % desde referencia 0 se normaliza con la corriente real de reposo (L298N, ~20-30 mA)
    if a == 0, a = mean(y(t < 0)); end
    k = t >= 0; t = t(k); y = y(k); d = b - a;
    yn = (y - a)/d;
    k10 = find(yn >= 0.1, 1); k90 = find(yn >= 0.9, 1);
    m.tr  = NaN; m.t90 = NaN;
    if ~isempty(k10) && ~isempty(k90), m.tr = t(k90) - t(k10); m.t90 = t(k90); end
    m.os  = max(0, (max(yn) - 1)*100);
    m.ts2 = tasent(t, y, b, max(0.02*abs(d), tolMin));
    m.ts5 = tasent(t, y, b, max(0.05*abs(d), tolMin));
    m.ess = mean(y(t > 0.8*max(t))) - b;
end

function ts = tasent(t, y, r, tol)
    fuera = find(abs(y - r) > tol, 1, 'last');
    if isempty(fuera), ts = 0;
    elseif fuera == numel(y), ts = Inf;
    else, ts = t(fuera + 1);
    end
end
