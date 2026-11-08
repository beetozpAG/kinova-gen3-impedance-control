%% ANALISIS_PI  Métricas de los escalones del lazo de corriente (ensayo_PI.m)
%
%  ts2 / ts5 : tiempo de asentamiento a ±2 % / ±5 % del escalón
%              (con un mínimo de ±tolMin mA, por el ruido del INA226 ~1 mA)
%  OS        : sobrepaso [%]     ess : error final [mA]     upico: PWM máximo
%
%  Para el plan ZN_EXP calcula además Ku, Pu y las ganancias de Z-N.

clear; clc; close all;

%% ===================== LO QUE CAMBIAS =====================
patron = "COMPARA_*.csv";      % p.ej. "COMPARA_*20261027_1530*" o "ZN_EXP_*"
tolMin = 2;                    % mA
meta_ts = 1.0;                 % ms (objetivo)
%% ===========================================================

carpeta = fullfile(fileparts(mfilename('fullpath')), 'datos');
A = dir(fullfile(carpeta, patron));
A = A(~contains({A.name}, '_meta'));
if isempty(A), error("No hay archivos %s", patron); end

R = table();
for f = 1:numel(A)
    T = leer_escalon(fullfile(carpeta, A(f).name));
    % nombre: <plan>_<config>_<a>_<b>_<yyyyMMdd>_<HHmmss>.csv (config puede tener "_")
    tok = regexp(A(f).name, '^(SIMC|COMPARA|ZN_EXP|MANUAL)_(.+)_[+-][\d.]+_[+-][\d.]+(_r\d+)?_\d{8}_\d{6}\.csv$', 'tokens', 'once');
    if isempty(tok), cfg = string(A(f).name); else, cfg = string(tok{2}); end
    a = T.ref(find(T.t_ms < 0, 1, 'last')); b = T.ref(end);
    if isempty(a), a = T.ref(1); end
    % ref guarda la magnitud; el signo está en u
    m = T.t_ms >= 0; t = T.t_ms(m); y = T.i(m);
    d = b - a;
    os  = max(0, (max((y - a)/d) - 1)*100);
    ts2 = tasent(t, y, b, max(0.02*abs(d), tolMin));
    ts5 = tasent(t, y, b, max(0.05*abs(d), tolMin));
    ess = mean(y(t > 0.8*max(t))) - b;
    up  = max(abs(T.u));
    signo = sign(median(T.u(T.u ~= 0))); if isnan(signo), signo = 1; end
    R = [R; table(string(A(f).name), cfg, signo*a, signo*b, ts2, ts5, os, ess, up, ...
         'VariableNames', {'archivo','config','a','b','ts2_ms','ts5_ms','OS_pct','ess_mA','u_max'})]; %#ok<AGROW>
end
R = sortrows(R, {'config','a','b'});
disp(R(:, 2:end));

% ---------- resumen por configuración ----------
cfgs = unique(R.config, 'stable');
fprintf("\n%-14s %8s %8s %8s %8s   (peor caso de los escalones)\n", "config", "ts2", "ts5", "OS", "|ess|");
for c = cfgs'
    m = R.config == c;
    fprintf("%-14s %8.2f %8.2f %8.1f %8.2f %s\n", c, max(R.ts2_ms(m)), max(R.ts5_ms(m)), ...
            max(R.OS_pct(m)), max(abs(R.ess_mA(m))), ...
            ternario(max(R.ts2_ms(m)) <= meta_ts, "  <- cumple ts2 < 1 ms", ""));
end
writetable(R, fullfile(carpeta, "metricas_" + erase(patron, ["*", ".csv"]) + ".csv"));

% ---------- gráficas: una columna por escalón, una fila por config ----------
esc = unique(R(:, {'a','b'}), 'rows', 'stable');
figure('Name', 'Escalones', 'Position', [40 40 300*height(esc) 200*numel(cfgs)]);
for ic = 1:numel(cfgs)
    for ie = 1:height(esc)
        k = find(R.config == cfgs(ic) & R.a == esc.a(ie) & R.b == esc.b(ie), 1);
        if isempty(k), continue; end
        T = leer_escalon(fullfile(carpeta, R.archivo(k)));
        subplot(numel(cfgs), height(esc), (ic-1)*height(esc) + ie); hold on; grid on;
        stairs(T.t_ms, T.ref, 'k:'); plot(T.t_ms, T.rd, 'Color', [0.6 0.6 0.6]);
        plot(T.t_ms, T.i, 'b.-', 'MarkerSize', 6);
        xline(R.ts2_ms(k), 'r:');
        title(sprintf('%s  %g\\rightarrow%g  ts2 %.2f ms OS %.0f%%', cfgs(ic), esc.a(ie), esc.b(ie), ...
              R.ts2_ms(k), R.OS_pct(k)), 'FontSize', 8, 'Interpreter', 'none');
        xlim([-1 8]); if ic == numel(cfgs), xlabel('t [ms]'); end
        if ie == 1, ylabel('i [mA]'); end
    end
end

% ---------- Ziegler-Nichols experimental ----------
if any(contains(R.archivo, "ZN_EXP"))
    kp = []; ratio = []; per = []; sat = [];
    for k = find(contains(R.archivo, "ZN_EXP"))'
        T = leer_escalon(fullfile(carpeta, R.archivo(k)));
        m = T.t_ms > 0.5; t = T.t_ms(m); e = T.i(m) - mean(T.i(T.t_ms > 0.6*max(T.t_ms)));
        [pk, tp] = picos(t, e);
        if numel(pk) >= 4
            u = T.u(T.t_ms > 3);
            kp(end+1) = sscanf(R.config(k), "KP%f");                 %#ok<AGROW>
            ratio(end+1) = mean(pk(3:end) ./ pk(2:end-1));           %#ok<AGROW> (sin el 1er pico)
            per(end+1) = mean(diff(tp(2:end)));                      %#ok<AGROW>
            sat(end+1) = mean(u == 0) > 0.05;                        %#ok<AGROW> PWM llega a 0
        end
    end
    [kp, o] = sort(kp); ratio = ratio(o); per = per(o); sat = sat(o);
    fprintf("\nZN experimental: KP vs razón entre picos sucesivos de la oscilación\n");
    disp(table(kp', ratio', per', sat', 'VariableNames', {'KP','razon_picos','periodo_ms','PWM_toca_0'}));
    j = find(ratio >= 0.95 | sat, 1);
    if isempty(j)
        fprintf("No hubo oscilación sostenida. Amplía kpZN en ensayo_PI.m.\n");
    else
        Ku = kp(j); Pu = per(j);
        fprintf("Ku ~ %.2f (mA/mA, con LIN)   Pu ~ %.3f ms   [modelo continuo: Ku 3.82, Pu 1.68 ms;\n", Ku, Pu);
        fprintf("  simulación discreta: Ku ~ 4.25, Pu ~ 1.7 ms]\n");
        fprintf("Z-N PI:  KP = 0.45 Ku = %.3f   TI = Pu/1.2 = %.3f ms\n", 0.45*Ku, Pu/1.2);
    end
end


%% ===================== FUNCIONES =====================
function ts = tasent(t, y, r, tol)
    fuera = find(abs(y - r) > tol, 1, 'last');
    if isempty(fuera), ts = 0;
    elseif fuera == numel(y), ts = Inf;
    else, ts = t(fuera + 1);
    end
end

function [pk, tp] = picos(t, e)
    % máximos locales de |e| (uno por semiciclo), sin toolbox
    s = sign(e); s(s == 0) = 1;
    cruces = [1; find(diff(s) ~= 0) + 1; numel(e)];
    pk = []; tp = [];
    for j = 1:numel(cruces) - 1
        idx = cruces(j):cruces(j+1)-1;
        if numel(idx) < 2, continue; end
        [v, k] = max(abs(e(idx)));
        if e(idx(k)) > 0, pk(end+1) = v; tp(end+1) = t(idx(k)); end %#ok<AGROW>
    end
end

function r = ternario(c, a, b), if c, r = a; else, r = b; end, end
