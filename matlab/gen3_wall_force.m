function [force, penetration] = gen3_wall_force(x, vx, wall, t)
% Fuerza que LA PARED ejerce SOBRE el EE, normal X de la base.
% Kelvin-Voigt unilateral: solo contacto penetrante y nunca traccion.
if nargin < 4, t = 0; end
gain = gen3_wall_activation(t,wall);
penetration = 0;
force = 0;
if gain > 0
    penetration = max(x-wall.x,0);
    if penetration > 0
        force = -gain*max(0,wall.stiffness*penetration+wall.damping*vx);
    end
end
end
