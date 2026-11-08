%% SINTONIZAR_PI  Sintonización del PI de corriente con reglas clásicas
%
%  Planta (01_identificacion_corriente):  G(s) = K e^(-theta s) / (tau s + 1)
%
%  1) Retardo efectivo del control digital: theta_e = theta + Ts/2
%     (el retenedor de orden cero agrega medio periodo de retardo).
%  2) Reglas: Ziegler-Nichols (curva de reacción y lazo cerrado), Cohen-Coon,
%     Chien-Hrones-Reswick (0 % y 20 % de sobrepaso), AMIGO y SIMC/IMC.
%  3) Dos versiones de la planta:
%       - SIN LIN: PI clásico con salida en PWM, K = 0.7435 mA/PWM (promedio).
%         Pero la K real cambia de ~0.4 a ~1.4 con el PWM.
%       - CON LIN: el PI calcula w [mA] y u = mapa^-1(w). La planta vista por
%         el PI tiene K = 1 en todo el rango.
%  4) Márgenes de fase/ganancia (respuesta en frecuencia, sin toolbox) y
%     simulación con el algoritmo del ESP32 (simular_PI.m): ts 2 %, ts 5 %, OS.
%  5) Estructura de 2 grados de libertad (FF + modelo de referencia).

clear; clc; close all;

%% ===================== DATOS DE LA PLANTA =====================
K     = 0.7435;      % mA / cuenta PWM (promedio, solo para SIN LIN)
tau   = 0.985e-3;    % s
theta = 0.366e-3;    % s
Ts    = 250e-6;      % periodo del lazo (4 kHz)
taud  = 0.10e-3;     % trayectoria deseada del modo FF (2 GDL)

% Mapa estático (IN1). Debe coincidir con control_PI_corriente/mapa_corriente.h
mapU = [0 50 100 150 200 250 300 350 400 450 500 550 600 650 700 750 800 850 900 950 1000 1023];
mapI = [30.22 30.88 41.28 51.11 66.46 88.86 117.73 152.36 193.58 240.64 293.35 ...
        351.05 421.87 492.39 571.18 656.26 747.65 845.34 949.34 1059.63 1176.23 1231.98];

% Escalones de prueba [a b] en mA
escalones = [100 300; 200 250; 40 300; 300 150; 150 500];
%% ===============================================================

the = theta + Ts/2;
P = struct('tau', tau, 'theta', theta, 'Ts', Ts, 'mapU', mapU, 'mapI', mapI, 'gain', 1);

% ---------- punto crítico (para ZN en lazo cerrado) ----------
% fase: -atan(w tau) - w the = -pi
w180 = fzero(@(w) atan(w*tau) + w*the - pi, [10 1e5]);
Pu   = 2*pi/w180;
Ku1  = sqrt(1 + (w180*tau)^2);          % con K = 1
fprintf("Retardo efectivo theta_e = %.3f ms (theta %.3f + Ts/2 %.3f)\n", the*1e3, theta*1e3, Ts/2*1e3);
fprintf("Punto crítico: Pu = %.3f ms;  Ku = %.3f (K=1)  |  %.3f PWM/mA (K=%.4f)\n\n", Pu*1e3, Ku1, Ku1/K, K);

% ---------- reglas (Kp con K = 1; para SIN LIN se divide entre K) ----------
r = the/tau;
reglas = {
  'Ziegler-Nichols (curva)',   0.9/r,                              3.33*the
  'Ziegler-Nichols (Ku, Pu)',  0.45*Ku1,                           Pu/1.2
  'Cohen-Coon',                (1/r)*(0.9 + r/12),                 the*(30 + 3*r)/(9 + 20*r)
  'CHR 0% sobrepaso',          0.35/r,                             1.17*tau
  'CHR 20% sobrepaso',         0.6/r,                              tau
  'AMIGO',                     0.15 + (0.35 - the*tau/(the+tau)^2)/r, 0.35*the + 13*the*tau^2/(tau^2 + 12*the*tau + 7*the^2)
  'SIMC / IMC (tc = theta)',   1/(2*r),                            min(tau, 8*the)
};
nR = size(reglas, 1);

T = table();
figure('Name', 'Reglas clásicas', 'Position', [60 60 1250 650]);
for j = 1:nR
    nombre = reglas{j,1}; Kp1 = reglas{j,2}; Ti = reglas{j,3};
    [PM, GM]   = margenes(Kp1, Ti, 1, tau, the);
    [~, GM15]  = margenes(Kp1, Ti, 1.15, tau, the);     % mapa 15 % mal

    % simulación CON LIN y SIN LIN
    resL = zeros(size(escalones,1), 3); resR = resL;
    for e = 1:size(escalones,1)
        a = escalones(e,1); b = escalones(e,2);
        SL = simular_PI(struct('Kp',Kp1,'Ti',Ti,'lin',true,'ff',false,'taud',taud), a, b, P);
        SR = simular_PI(struct('Kp',Kp1/K,'Ti',Ti,'lin',false,'ff',false,'taud',taud), a, b, P);
        resL(e,:) = [SL.ts2*1e3 SL.ts5*1e3 SL.OS]; resR(e,:) = [SR.ts2*1e3 SR.ts5*1e3 SR.OS];
        if e == 1
            subplot(2, 4, j); hold on; grid on;
            plot(SR.t*1e3, SR.i, 'Color', [0.65 0.65 0.65], 'LineWidth', 1.2);
            plot(SL.t*1e3, SL.i, 'b', 'LineWidth', 1.4);
            stairs(SL.t*1e3, SL.ref, 'k:');
            title(nombre, 'FontSize', 9); xlim([-1 10]); ylim([0 500]);
            xlabel('t [ms]'); ylabel('i [mA]');
            if j == 1, legend('sin LIN', 'con LIN', 'ref', 'Location', 'southeast'); end
        end
    end
    T = [T; table(string(nombre), Kp1, Ti*1e3, Kp1/Ti, Kp1/K, PM, GM, GM15, ...
         max(resL(:,1)), max(resL(:,2)), max(resL(:,3)), max(resR(:,1)), max(resR(:,3)), ...
        'VariableNames', {'regla','Kp_LIN','Ti_ms','Ki_LIN','Kp_sinLIN_PWMpormA','PM_grados','GM','GM_mapa15', ...
                          'ts2_ms_LIN','ts5_ms_LIN','OS_LIN','ts2_ms_sinLIN','OS_sinLIN'})]; %#ok<AGROW>
end

% ---------- 2 grados de libertad (LIN + FF) con la regla SIMC ----------
jS = nR;
cFF = struct('Kp', reglas{jS,2}, 'Ti', reglas{jS,3}, 'lin', true, 'ff', true, 'taud', taud);
subplot(2, 4, 8); hold on; grid on;
res = zeros(size(escalones,1), 3);
for e = 1:size(escalones,1)
    S = simular_PI(cFF, escalones(e,1), escalones(e,2), P);
    res(e,:) = [S.ts2*1e3 S.ts5*1e3 S.OS];
    if e == 1
        plot(S.t*1e3, S.i, 'b', 'LineWidth', 1.4); stairs(S.t*1e3, S.ref, 'k:');
    end
end
title(sprintf('SIMC + LIN + FF (\\tau_d = %.2f ms)', taud*1e3), 'FontSize', 9);
xlim([-1 10]); ylim([0 500]); xlabel('t [ms]');

disp(T);
fprintf("\nSIMC + LIN + FF: ts2 max %.2f ms, ts5 max %.2f ms, OS max %.1f %%\n", max(res(:,1)), max(res(:,2)), max(res(:,3)));
fprintf("  por escalón (ts2/ts5/OS):\n");
for e = 1:size(escalones,1)
    fprintf("   %4d -> %4d : %.2f / %.2f / %.1f %%\n", escalones(e,:), res(e,:));
end

% ---------- sensibilidad del FF a errores del mapa ----------
fprintf("\nFF con mapa equivocado (gain = planta/mapa):\n");
for g = [0.9 0.95 1.05 1.1]
    P2 = P; P2.gain = g; rr = zeros(size(escalones,1), 3);
    for e = 1:size(escalones,1)
        S = simular_PI(cFF, escalones(e,1), escalones(e,2), P2); rr(e,:) = [S.ts2*1e3 S.ts5*1e3 S.OS];
    end
    fprintf("  gain %.2f: ts2 max %.2f ms, ts5 max %.2f ms, OS max %.1f %%\n", g, max(rr(:,1)), max(rr(:,2)), max(rr(:,3)));
end

% ---------- límites físicos ----------
wmax = mapI(end);
fprintf("\nLímite físico (PWM 1023 desde el inicio): 100->300 mA en %.2f ms\n", (theta + tau*log((wmax-100)/(wmax-300)))*1e3);
fprintf("Límite físico de bajada (PWM 0): 300->150 mA en %.2f ms (al 2 %%: %.2f ms)\n", ...
        (theta + tau*log((300-mapI(1))/(150-mapI(1))))*1e3, (theta + tau*log((300-mapI(1))/(153-mapI(1))))*1e3);

writetable(T, 'resultados_sintonizacion.csv');
save('resultados_sintonizacion.mat', 'T', 'reglas', 'K', 'tau', 'theta', 'the', 'Ts', 'Ku1', 'Pu', 'taud');
fprintf("\nGuardado resultados_sintonizacion.csv / .mat\n");


%% ===================== FUNCIONES =====================
function [PM, GM] = margenes(Kp, Ti, Kg, tau, the)
% márgenes del lazo L = Kp (1 + 1/(Ti s)) Kg e^(-the s)/(tau s + 1)
w  = logspace(1, 5.5, 40000); s = 1i*w;
L  = Kp*(1 + 1./(Ti*s)) .* Kg .* exp(-the*s) ./ (tau*s + 1);
ph = unwrap(angle(L));
kc = find(abs(L) < 1, 1);  PM = 180 + rad2deg(ph(kc));
kp = find(ph < -pi, 1);    GM = 1/abs(L(kp));
end
