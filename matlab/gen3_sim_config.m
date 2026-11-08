function cfg = gen3_sim_config()
% GEN3_SIM_CONFIG Simulacion offline. No contiene conexiones al hardware.
% Ganancias/filtros copiados de include/controller_config.hpp (2026-09-07).
cfg.urdf = '';                  % Vacio: loadrobot('kinovaGen3').
cfg.end_effector = '';          % Obligatorio con URDF propio; nombre de body.
cfg.q0 = [];                    % Vacio: homeConfiguration del modelo importado.
cfg.dq0 = zeros(7,1);
cfg.gravity = [0 0 -9.81];       % Base fija, Z hacia arriba.
cfg.mode = 'closed_loop';       % 'closed_loop' o 'shadow', SOLO simulacion.
cfg.update_hz = 1000;
cfg.duration = 30;
cfg.integration_substeps = 2;   % RK4, torque retenido durante cada ciclo.
cfg.reference_mode = 'RelativeOffset';
cfg.position_offset = [0.05;0;0];
cfg.orientation_offset_rpy_deg = [0;0;0];
cfg.absolute_position = [0;0;0];
cfg.absolute_orientation_rpy_deg = [0;0;0];
cfg.stiffness = [50;700;100;10;10;10];
cfg.damping_factors = ones(6,1);
cfg.enable_nullspace = false;
cfg.nullspace_stiffness = 5;
cfg.nullspace_damping_factor = 1;
cfg.nullspace_offset = zeros(7,1);
cfg.enable_wrench = false;
cfg.wrench = zeros(6,1);         % Consigna [F;M] en base, NO fuerza medida.
cfg.pose_filter = 0.1;
cfg.stiffness_filter = 0.1;
cfg.wrench_filter = 0.1;
cfg.nullspace_filter = 0.1;
cfg.delta_tau_max = 1;          % Nm/ciclo, igual que C++.
cfg.blend_time = 2;             % Rampa de impedancia solo en closed_loop.
cfg.KTGR = [ones(4,1)/(100*0.1124); ones(3,1)/(100*0.0764)];
cfg.max_current = [5;5;5;5;3;3;3];
cfg.max_tau = 0.9*cfg.max_current./cfg.KTGR;
cfg.disturbance_tau = zeros(7,1); % Torque externo aplicado A la planta.
cfg.disturbance_interval = [5 6]; % Segundos, intervalo [inicio, fin).
cfg.contact.enabled = false;    % Pared normal a X, contacto puntual del EE.
cfg.contact.offset_x = 0.05;     % x_wall = x_inicial + offset_x [m].
cfg.contact.stiffness = 1500;    % N/m, modelo de penalizacion de la pared.
cfg.contact.damping = 30;       % N s/m. Fuerza unilateral, nunca atractiva.
cfg.plot = true;
cfg.save_results = true;
cfg.output_dir = fullfile(fileparts(mfilename('fullpath')), 'results');
end
