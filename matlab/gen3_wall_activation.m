function gain = gen3_wall_activation(t, wall)
% Intervalos [inicio,fin). Retirada exacta al final; reentrada suave opcional.
gain = 0;
if ~wall.enabled, return; end
if ~isfield(wall,'active_intervals')
    gain = 1; return; % Compatibilidad con la demo de pared permanente.
end
for i = 1:size(wall.active_intervals,1)
    a = wall.active_intervals(i,1); b = wall.active_intervals(i,2);
    if t >= a && t < b
        gain = 1;
        if a > 0 && isfield(wall,'reentry_ramp_s') && wall.reentry_ramp_s > 0
            u = min((t-a)/wall.reentry_ramp_s,1);
            gain = u*u*(3-2*u); % Smoothstep: evita reinsertar 75 N de golpe.
        end
        return;
    end
end
end
