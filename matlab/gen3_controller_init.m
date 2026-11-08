function s = gen3_controller_init(cfg, q0, T0)
% Estado equivalente al constructor + configuracion + initialize() de C++.
validateattributes(q0, {'double'}, {'size',[7 1],'finite'});
validateattributes(cfg.stiffness, {'double'}, {'size',[6 1],'finite','nonnegative'});
validateattributes(cfg.damping_factors, {'double'}, {'size',[6 1],'finite','nonnegative'});
validateattributes(cfg.position_offset, {'double'}, {'size',[3 1],'finite'});
validateattributes(cfg.absolute_position, {'double'}, {'size',[3 1],'finite'});
validateattributes(cfg.nullspace_offset, {'double'}, {'size',[7 1],'finite'});
validateattributes(cfg.wrench, {'double'}, {'size',[6 1],'finite'});
validateattributes(cfg.delta_tau_max, {'double'}, {'scalar','finite','nonnegative'});
validateattributes(cfg.update_hz, {'double'}, {'scalar','finite','positive'});
filters = [cfg.pose_filter cfg.stiffness_filter cfg.wrench_filter cfg.nullspace_filter];
assert(all(isfinite(filters) & filters > 0 & filters <= 1), 'Filtros fuera de (0,1].');
p0 = T0(1:3,4); R0 = T0(1:3,1:3);
switch cfg.reference_mode
    case 'InitialPose'
        s.p = p0; s.R = R0;
    case 'RelativeOffset'
        s.p = p0 + cfg.position_offset;
        s.R = R0*rpyRotation(cfg.orientation_offset_rpy_deg);
    case 'AbsolutePose'
        s.p = cfg.absolute_position;
        s.R = rpyRotation(cfg.absolute_orientation_rpy_deg);
    otherwise
        error('Modo de referencia desconocido: %s', cfg.reference_mode);
end
assert(all(isfinite([s.p(:);s.R(:)])), 'Referencia NaN/Inf.');
s.p_target = s.p; s.R_target = s.R; % Sin rampa inicial de pose, igual que C++.
s.q_null = q0;
s.q_null_target = q0 + double(cfg.enable_nullspace)*cfg.nullspace_offset;
s.K = [200;200;200;20;20;20]; % Constructor C++; NO iniciar en cfg.stiffness.
s.D = 2*sqrt(s.K);
s.K_target = cfg.stiffness;
s.D_target = 2*cfg.damping_factors.*sqrt(cfg.stiffness);
s.Kn = 0; s.Dn = 0;
assert(isfinite(cfg.nullspace_stiffness) && cfg.nullspace_stiffness >= 0);
assert(isfinite(cfg.nullspace_damping_factor) && cfg.nullspace_damping_factor >= 0);
s.Kn_target = double(cfg.enable_nullspace)*cfg.nullspace_stiffness;
s.Dn_target = double(cfg.enable_nullspace)*cfg.nullspace_damping_factor*2*sqrt(s.Kn_target);
s.w = zeros(6,1);
s.w_target = double(cfg.enable_wrench)*cfg.wrench;
assert(all(isfinite([s.q_null_target;s.w_target])), 'Objetivos NaN/Inf.');
s.tau_last = zeros(7,1);
s.a = zeros(1,4);
for i = 1:4
    f = filters(i);
    if f >= 1
        s.a(i) = 1;
    else
        f = min(max(f,1e-9),0.999999);
        tf = -1/log(1-f);
        s.a(i) = 1/(tf*cfg.update_hz+1);
    end
end
s.delta_tau_max = cfg.delta_tau_max;
end

function R = rpyRotation(degrees)
validateattributes(degrees, {'double'}, {'size',[3 1],'finite'});
r = degrees(1)*pi/180; p = degrees(2)*pi/180; y = degrees(3)*pi/180;
Rx = [1 0 0;0 cos(r) -sin(r);0 sin(r) cos(r)];
Ry = [cos(p) 0 sin(p);0 1 0;-sin(p) 0 cos(p)];
Rz = [cos(y) -sin(y) 0;sin(y) cos(y) 0;0 0 1];
R = Rz*Ry*Rx;
end
