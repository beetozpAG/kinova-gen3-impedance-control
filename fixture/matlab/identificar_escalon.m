%% IDENTIFICAR_ESCALON  Función de transferencia PWM -> corriente
%
%            I(s)        K
%   G(s) = ------- = --------- e^(-theta s)
%            U(s)     tau s + 1
%
%  Método de los dos puntos (Smith) sobre la respuesta al escalón:
%    1) i0   = promedio antes del escalón,  ifin = promedio del final
%    2) K    = (ifin - i0) / (b - a)                      [mA / cuenta PWM]
%    3) y(t) = (i - i0) / (ifin - i0)   (respuesta normalizada 0 -> 1)
%    4) t28 = tiempo en que y = 0.283,  t63 = tiempo en que y = 0.632
%    5) tau   = 1.5 (t63 - t28)
%       theta = t63 - tau           (retardo; si sale ~0 se desprecia)
%
%  Usa los CSV que genera adquirir_escalon.m (carpeta datos/).

clear; clc; close all;

%% ===================== LO QUE CAMBIAS =====================
patron  = "escalon_*.csv";   % qué archivos analizar (p.ej. "escalon_*_20261027*.csv")
Ts_lazo = 250e-6;            % periodo del lazo PI (para el modelo discreto)
%% ===========================================================

carpeta  = fullfile(fileparts(mfilename('fullpath')), 'datos');
archivos = dir(fullfile(carpeta, patron));
archivos = archivos(~contains({archivos.name}, '_meta'));
if isempty(archivos), error("No hay archivos %s en %s", patron, carpeta); end

R = table();   % resultados por escalón
nA = numel(archivos);
figure('Name', 'Escalones', 'Position', [80 80 1100 700]);
nc = ceil(sqrt(nA)); nr = ceil(nA / nc);

for f = 1:nA
    T = readtable(fullfile(carpeta, archivos(f).name));
    t = T.t_ms * 1e-3;               % [s]
    i = T.i_mA;                      % magnitud medida por el INA226
    a = T.pwm(1); b = T.pwm(end);

    % ---- 1) niveles inicial y final ----
    pre  = t < 0;
    post = t > 0.75 * max(t);        % último 25 % del registro
    i0   = mean(i(pre));
    ifin = mean(i(post));

    % ---- 2) ganancia (incremental, en magnitud) ----
    du = abs(b) - abs(a);
    K  = (ifin - i0) / du;

    % ---- 3) respuesta normalizada ----
    y = (i - i0) / (ifin - i0);

    % ---- 4) tiempos al 28.3 % y 63.2 % ----
    t28 = cruce(t, y, 0.283);
    t63 = cruce(t, y, 0.632);

    % ---- 5) tau y theta ----
    tau   = 1.5 * (t63 - t28);
    theta = max(t63 - tau, 0);

    % ---- calidad: modelo vs datos ----
    ym  = (t >= theta) .* (1 - exp(-(t - theta) / tau));
    im  = i0 + (ifin - i0) * ym;
    % el fit se evalúa solo en la parte eléctrica (0 a 5 tau): después el rotor
    % se mueve un poco (juego del reductor) y la fcem baja la corriente; eso no
    % es parte del modelo R-L y bajaría el % sin que el modelo esté mal.
    we  = t >= 0 & t <= 5*tau;
    fit = 100 * (1 - norm(i(we) - im(we)) / norm(i(we) - mean(i(we))));

    sentido = "IN1 (+)"; if b < 0, sentido = "IN2 (-)"; end
    R = [R; table(string(archivos(f).name), a, b, string(sentido), i0, ifin, K, ...
                  tau*1e3, theta*1e3, fit, ...
        'VariableNames', {'archivo','a','b','sentido','i0_mA','ifin_mA','K_mA_por_pwm', ...
                          'tau_ms','theta_ms','fit_0a5tau_pct'})]; %#ok<AGROW>

    % ---- gráfica ----
    subplot(nr, nc, f);
    plot(t*1e3, i, '.', 'Color', [0.6 0.6 0.6]); hold on;
    plot(t*1e3, im, 'b', 'LineWidth', 1.4);
    yline(i0 + 0.632*(ifin - i0), ':r');
    xline(t63*1e3, ':r');
    grid on; xlabel('t [ms]'); ylabel('i [mA]');
    title(sprintf('%d\\rightarrow%d: K=%.3f, \\tau=%.2f ms, fit %.0f%%', a, b, K, tau*1e3, fit), ...
          'FontSize', 9);
end

disp(R(:, 2:end));

%% ---------- Modelo final: promedio por sentido y global ----------
fprintf("\n================ MODELO IDENTIFICADO ================\n");
for sdo = ["IN1 (+)", "IN2 (-)"]
    m = R.sentido == sdo;
    if any(m)
        fprintf("%s : K = %.4f mA/PWM   tau = %.3f ms   theta = %.3f ms   (%d escalones)\n", ...
            sdo, mean(R.K_mA_por_pwm(m)), mean(R.tau_ms(m)), mean(R.theta_ms(m)), nnz(m));
    end
end
K     = mean(R.K_mA_por_pwm);
tau   = mean(R.tau_ms) * 1e-3;
theta = mean(R.theta_ms) * 1e-3;

fprintf("\nGlobal:\n");
fprintf("          %.4f\n", K);
fprintf("  G(s) = ------------------ e^(-%.3g s)      [mA / cuenta PWM]\n", theta);
fprintf("          %.4g s + 1\n", tau);
fprintf("  Polo eléctrico: %.0f rad/s (%.0f Hz)\n", 1/tau, 1/(2*pi*tau));
fprintf("  Ganancia DC: %.1f mA por cada 100 cuentas de PWM\n", 100*K);
% OJO: K NO es 1/R. El INA226 mide corriente de la FUENTE (antes del L298N),
% que solo circula en el tiempo encendido: i_fuente ~ D * i_motor ~ D^2 V/R.
% Por eso la curva estática es ~cuadrática y K crece con el PWM: K(u) = 2c(u - u0).

% Discreto (retenedor de orden cero) al periodo del lazo PI
a_d = exp(-Ts_lazo / tau);
fprintf("\nDiscreto (Ts = %.0f us, ZOH, sin retardo):\n", Ts_lazo*1e6);
fprintf("  G(z) = %.5f z^-1 / (1 - %.5f z^-1)\n", K*(1 - a_d), a_d);
fprintf("  retardo theta = %.2f muestras\n", theta / Ts_lazo);

if exist('tf', 'file')     % Control System Toolbox
    G  = tf(K, [tau 1], 'InputDelay', theta);
    Gd = c2d(tf(K, [tau 1]), Ts_lazo, 'zoh');
    disp(G);
else
    G = []; Gd = [];
end

save(fullfile(carpeta, 'resultados_identificacion.mat'), 'R', 'K', 'tau', 'theta', 'G', 'Gd', 'Ts_lazo');
writetable(R, fullfile(carpeta, 'resultados_identificacion.csv'));
fprintf("\nGuardado en datos/resultados_identificacion.mat y .csv\n");

%% ---------- Estática (si existe) ----------
est = dir(fullfile(carpeta, 'estatica_*.csv'));
if ~isempty(est)
    E = readtable(fullfile(carpeta, est(end).name));
    figure('Name', 'Estatica');
    plot(E.pwm, E.i_mA, 'o-'); grid on; hold on;
    xlabel('PWM'); ylabel('i [mA]');
    title('Corriente estable vs PWM (rotor bloqueado)');
    % recta con la pendiente K identificada, pasando por el punto medio
    pm = median(E.pwm(E.i_mA > max(E.i_mA)*0.3));
    im = interp1(E.pwm, E.i_mA, pm);
    plot(E.pwm, im + K*(E.pwm - pm), '--');
    % Ajuste cuadrático i = iq + c (u - u0)^2  (zona de PWM >= 150)
    m  = E.pwm >= 150;
    fq = @(p, u) p(1) + p(2) * max(u - p(3), 0).^2;
    pq = fminsearch(@(p) sum((fq(p, E.pwm(m)) - E.i_mA(m)).^2), [30 1e-3 50]);
    uu = linspace(0, max(E.pwm), 200);
    plot(uu, fq(pq, uu), 'LineWidth', 1.4);
    legend('medido', sprintf('pendiente K = %.3f', K), ...
           sprintf('i = %.1f + %.2e (u - %.0f)^2', pq), 'Location', 'northwest');
    fprintf("\nEstática: i = %.1f + %.3e (u - %.0f)^2  ->  K(u) = %.3e (u - %.0f) mA/cuenta\n", ...
            pq(1), pq(2), pq(3), 2*pq(2), pq(3));
    save(fullfile(carpeta, 'resultados_identificacion.mat'), 'pq', '-append');
end


%% ===================== FUNCIONES =====================
function tc = cruce(t, y, nivel)
    % primer instante (t >= 0) en que y cruza "nivel", con interpolación lineal
    k = find(t >= 0 & y >= nivel, 1);
    if isempty(k), tc = NaN; return; end
    if k == 1 || t(k-1) < 0
        tc = t(k);
    else
        tc = t(k-1) + (nivel - y(k-1)) * (t(k) - t(k-1)) / (y(k) - y(k-1));
    end
end
