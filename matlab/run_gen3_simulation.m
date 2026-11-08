function result = run_gen3_simulation(cfg)
% RUN_GEN3_SIMULATION Control discreto + planta rigida continua, SOLO offline.
% Uso: result = run_gen3_simulation(gen3_sim_config());
if nargin == 0
    cfg = gen3_sim_config();
end
validateattributes(cfg.update_hz,{'double'},{'scalar','finite','positive'});
validateattributes(cfg.duration,{'double'},{'scalar','finite','positive'});
validateattributes(cfg.integration_substeps,{'double'},{'scalar','integer','positive'});
validateattributes(cfg.dq0,{'double'},{'size',[7 1],'finite'});
validateattributes(cfg.disturbance_tau,{'double'},{'size',[7 1],'finite'});
validateattributes(cfg.disturbance_interval,{'double'},{'size',[1 2],'finite','nonnegative'});
assert(cfg.disturbance_interval(2) >= cfg.disturbance_interval(1), ...
    'Intervalo de perturbacion invertido.');
validateattributes(cfg.gravity,{'double'},{'size',[1 3],'finite'});
validateattributes(cfg.blend_time,{'double'},{'scalar','finite','nonnegative'});
validateattributes(cfg.KTGR,{'double'},{'size',[7 1],'finite','positive'});
validateattributes(cfg.max_tau,{'double'},{'size',[7 1],'finite','positive'});
validateattributes(cfg.max_current,{'double'},{'size',[7 1],'finite','positive'});
assert(any(strcmp(cfg.mode,{'shadow','closed_loop'})), 'Modo de simulacion invalido.');
[robot,ee,q] = gen3_load_model(cfg);
dq = cfg.dq0;
T0 = getTransform(robot,q,ee);
if ~isfield(cfg,'contact')
    defaults = gen3_sim_config(); cfg.contact = defaults.contact;
end
validateattributes(cfg.contact.offset_x,{'double'},{'scalar','finite','positive'});
validateattributes(cfg.contact.stiffness,{'double'},{'scalar','finite','positive'});
validateattributes(cfg.contact.damping,{'double'},{'scalar','finite','nonnegative'});
wall = cfg.contact; wall.x = T0(1,4)+wall.offset_x; wall.ee = ee;
if isfield(wall,'active_intervals')
    intervals = wall.active_intervals;
    validateattributes(intervals,{'double'},{'2d','ncols',2,'finite','nonnegative'});
    assert(all(intervals(:,2)>intervals(:,1)), 'Intervalos de contacto vacios/invertidos.');
    assert(all(intervals(2:end,1)>=intervals(1:end-1,2)), 'Intervalos solapados/desordenados.');
    % Cambios abruptos alineados al reloj de control para integrar cada lado.
    assert(all(abs(intervals(:)*cfg.update_hz-round(intervals(:)*cfg.update_hz))<1e-8), ...
        'Alinea los eventos de contacto a los ciclos de control.');
end
if isfield(wall,'reentry_ramp_s')
    validateattributes(wall.reentry_ramp_s,{'double'},{'scalar','finite','nonnegative'});
end
s = gen3_controller_init(cfg,wrapQ(q),T0);
dt = 1/cfg.update_hz;
count = ceil(cfg.duration/dt);
result.t = (0:count-1)'*dt;
names7 = {'q','dq','tau_task','tau_null','tau_ext','tau_impedance', ...
    'tau_gravity','tau_virtual','tau_applied','current_virtual','current_cmd'};
for i = 1:numel(names7)
    result.(names7{i}) = zeros(count,7);
end
result.position = zeros(count,3);
result.pose_error = zeros(count,6);
result.cart_velocity = zeros(count,6);
result.stiffness = zeros(count,6);
result.blend = zeros(count,1);
result.saturated = false(count,1);
result.contact_force_x = zeros(count,1);
result.penetration = zeros(count,1);
result.contact_gain = zeros(count,1);
result.wall = wall;
result.cfg = cfg; result.ee = ee; result.base = robot.BaseName;
result.desired_position = s.p; result.desired_rotation = s.R;
fprintf('Simulacion %s: %.3f s, %.0f Hz; sin comunicacion con robot.\n', ...
    cfg.mode,cfg.duration,cfg.update_hz);
for k = 1:count
    t = result.t(k);
    T = getTransform(robot,q,ee);
    Jmatlab = geometricJacobian(robot,q,ee); % MATLAB: [angular; lineal].
    J = Jmatlab([4:6 1:3],:);               % Control C++: [lineal; angular].
    [o,s] = gen3_controller_step(s,wrapQ(q),dq,T,J);
    g = gravityTorque(robot,q);
    virtual = g+o.tau_impedance;
    if strcmp(cfg.mode,'shadow')
        blend = 0;
        applied = g; % Replica la rama shadow de main, sin saturacion de torque.
        saturated = false;
    else
        if cfg.blend_time > 0
            blend = min(t/cfg.blend_time,1);
        else
            blend = 1;
        end
        requested = g+blend*o.tau_impedance;
        applied = min(max(requested,-cfg.max_tau),cfg.max_tau);
        saturated = any(applied ~= requested);
    end
    current = cfg.KTGR.*applied;
    assert(all(isfinite([g;virtual;current])),'Torque/corriente NaN/Inf.');
    assert(all(abs(current) <= cfg.max_current), ...
        'Corriente simulada excede maxCurrent; se detiene antes de integrar.');
    values = {q,dq,o.tau_task,o.tau_null,o.tau_ext,o.tau_impedance, ...
        g,virtual,applied,cfg.KTGR.*virtual,current};
    for i = 1:numel(names7)
        result.(names7{i})(k,:) = values{i}';
    end
    result.position(k,:) = T(1:3,4)';
    result.pose_error(k,:) = o.pose_error';
    result.cart_velocity(k,:) = o.cart_velocity';
    result.stiffness(k,:) = s.K';
    result.blend(k) = blend; result.saturated(k) = saturated;
    [result.contact_force_x(k),result.penetration(k)] = ...
        gen3_wall_force(T(1,4),o.cart_velocity(1),wall,t);
    result.contact_gain(k) = gen3_wall_activation(t,wall);
    % RK4 con torque ZOH. compute() NO se llama en subetapas del integrador.
    x = [q;dq]; h = min(dt,cfg.duration-t)/cfg.integration_substeps;
    for j = 1:cfg.integration_substeps
        tj = t+(j-1)*h;
        a = plant(tj,x,robot,applied,cfg,wall);
        b = plant(tj+h/2,x+h*a/2,robot,applied,cfg,wall);
        c = plant(tj+h/2,x+h*b/2,robot,applied,cfg,wall);
        % Ultima etapa usa limite izquierdo del ciclo ante retirada abrupta.
        endTime = min(tj+h,t+dt-8*eps(max(1,t+dt)));
        d = plant(endTime,x+h*c,robot,applied,cfg,wall);
        x = x+h*(a+2*b+2*c+d)/6;
        assert(all(isfinite(x)),'Integracion NaN/Inf: revisa modelo/paso/ganancias.');
    end
    q = x(1:7); dq = x(8:14); % La planta conserva angulos continuos.
    if mod(k,max(1,round(cfg.update_hz))) == 0
        fprintf('  t=%.1f s | error posicion=%.5f m\n',t,norm(o.pose_error(1:3)));
    end
end
result.final_time = cfg.duration;
result.final_state = [q;dq]; % Registros anteriores corresponden al inicio de ciclo.
if cfg.save_results
    if ~isfolder(cfg.output_dir), mkdir(cfg.output_dir); end
    tag = datestr(now,'yyyymmdd_HHMMSS_FFF');
    path = fullfile(cfg.output_dir,['gen3_' cfg.mode '_' tag '.mat']);
    save(path,'result');
    fprintf('Resultados: %s\n',path);
end
if cfg.plot, plot_gen3_simulation(result); end
end

function dx = plant(t,x,robot,tau,cfg,wall)
disturbance = zeros(7,1);
if t >= cfg.disturbance_interval(1) && t < cfg.disturbance_interval(2)
    disturbance = cfg.disturbance_tau;
end
contact_tau = zeros(7,1);
if wall.enabled
    T = getTransform(robot,x(1:7),wall.ee);
    Jraw = geometricJacobian(robot,x(1:7),wall.ee);
    Jx = Jraw(4,:); % Lineal X en MATLAB: fila 4.
    force = gen3_wall_force(T(1,4),Jx*x(8:14),wall,t);
    contact_tau = Jx'*force;
end
% Fuerza de contacto sobre la planta; no pasa por saturacion del actuador.
ddq = forwardDynamics(robot,x(1:7),x(8:14),tau+disturbance+contact_tau);
assert(all(isfinite(ddq)),'Aceleracion NaN/Inf en subetapa de integracion.');
dx = [x(8:14);ddq];
end

function q = wrapQ(q)
q = mod(q+pi,2*pi)-pi; % Solo entrada al controlador, como wrapJoints en main.
end
