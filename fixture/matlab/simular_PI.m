function S = simular_PI(c, a, b, P)
% SIMULAR_PI  Simula un escalón de referencia a -> b [mA] del lazo de corriente
% con el mismo algoritmo que control_PI_corriente.ino (4 kHz, ZOH).
%
%  c : struct del controlador
%      .Kp, .Ti (s; 0 = sin integral), .lin (true/false), .ff (true/false), .taud (s)
%  P : struct de la planta (ver sintonizar_PI.m)
%      .tau, .theta, .Ts, .mapU, .mapI (mapa estático), .gain (1 = mapa exacto)
%
%  Planta (Hammerstein): w = iq + gain*(mapa(u) - iq);  i = w * e^(-theta s)/(tau s + 1)
%  S: .t .ref .rd .i .u .w (vectores por muestra), .ts2 .ts5 [s] (2 % y 5 %), .OS [%]

dt   = 5e-6;
Tend = 12e-3;  Tpre = 1e-3;
if isfield(P, 'Tend'), Tend = P.Tend; end
Ts   = P.Ts;
ns   = round(Ts/dt);
nd   = round(P.theta/dt);
ad   = exp(-Ts/P.tau);
iq   = P.mapI(1);
mapa = @(u) iq + P.gain*(interp1(P.mapU, P.mapI, min(max(u,0),1023), 'linear') - iq);
inv  = @(w) interp1(P.mapI, P.mapU, min(max(w,P.mapI(1)),P.mapI(end)), 'linear');
wMin = P.mapI(1); wMax = P.mapI(end);
if ~c.lin, wMin = 0; wMax = 1023; end

% estado inicial: estable en la referencia a
uu = linspace(0,1023,4001); ii = mapa(uu);
[~,k0] = min(abs(ii - a)); u = uu(k0);
if a <= 0, u = 0; end                 % referencia 0: puente apagado
x = mapa(u); hist = x*ones(nd+1,1); hp = 1;   % corriente real (con a = 0 es la del L298N, ~30 mA)
ki = 0; if c.Ti > 0, ki = c.Kp/c.Ti; end
if c.ff && c.lin
    integ = 0; ffon = true;
    adr = exp(-Ts/c.taud);
else
    ffon = false;
    if c.lin, integ = a; else, integ = u; end
    if a <= 0, integ = 0; end          % como el ESP32: con referencia 0 el integrador está en 0
    if ki == 0, integ = 0; end
end
rnext = a; rb = a*ones(16,1);
dfr = P.theta/Ts; d0 = floor(dfr); fr = dfr - d0;

n  = round((Tend+Tpre)/dt); N = floor(n/ns)+1;
S.t = zeros(N,1); S.ref = S.t; S.rd = S.t; S.i = S.t; S.u = S.t; S.w = S.t;
m = 0;
for k = 0:n-1
    t = k*dt - Tpre;
    if mod(k, ns) == 0
        ref = a; if t >= 0, ref = b; end
        y = hist(hp);
        if ffon
            rk = rnext; rnext = adr*rk + (1-adr)*ref;
            wff = (rnext - ad*rk)/(1 - ad);
            wff = min(max(wff, wMin), 0.9*wMax);
            rnext = ad*rk + (1-ad)*wff;
            rb = [rk; rb(1:end-1)];
            rdel = (1-fr)*rb(d0+1) + fr*rb(d0+2);
        else
            wff = 0; rdel = ref;
        end
        if ref <= 0                    % referencia 0: puente apagado (igual que el ESP32)
            u = 0; w = 0; integ = 0;
            m = m + 1; S.t(m) = t; S.ref(m) = ref; S.rd(m) = 0; S.i(m) = y; S.u(m) = 0; S.w(m) = 0;
            wp = mapa(u); x = x + dt/P.tau*(wp - x); hist(hp) = x; hp = mod(hp, nd+1) + 1;
            continue;
        end
        e  = rdel - y;
        ic = integ + ki*Ts*e;
        wn = wff + c.Kp*e + ic;
        windup = (wn > wMax && e > 0) || (wn < wMin && e < 0);
        if ~windup, integ = min(max(ic, -wMax), wMax); end
        w = min(max(wff + c.Kp*e + integ, wMin), wMax);
        if c.lin, u = inv(w); else, u = w; end
        m = m + 1;
        S.t(m) = t; S.ref(m) = ref; S.rd(m) = rdel; S.i(m) = y; S.u(m) = u; S.w(m) = w;
    end
    wp = mapa(u);
    x = x + dt/P.tau*(wp - x);
    hist(hp) = x; hp = mod(hp, nd+1) + 1;        % buffer circular del retardo
end
S.t = S.t(1:m); S.ref = S.ref(1:m); S.rd = S.rd(1:m); S.i = S.i(1:m); S.u = S.u(1:m); S.w = S.w(1:m);
[S.ts2, S.ts5, S.OS] = metricas(S.t, S.i, a, b);
end

function [ts2, ts5, OS] = metricas(t, i, a, b)
m = t >= 0; tt = t(m); y = i(m); d = b - a;
OS = max(0, (max((y - a)/d) - 1)*100);
ts2 = tasent(tt, y, b, 0.02*abs(d));
ts5 = tasent(tt, y, b, 0.05*abs(d));
end

function ts = tasent(t, y, r, tol)
fuera = find(abs(y - r) > tol, 1, 'last');
if isempty(fuera), ts = 0;
elseif fuera == numel(y), ts = Inf;
else, ts = t(fuera + 1);
end
end
