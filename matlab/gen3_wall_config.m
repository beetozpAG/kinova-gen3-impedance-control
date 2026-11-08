function cfg = gen3_wall_config()
% Escenario de contacto SOLO en simulacion. Ganancias propias de esta demo.
cfg = gen3_sim_config();
folder = fileparts(mfilename('fullpath'));
cfg.urdf = fullfile(folder,'URDFFiles','GEN3_FOR_URDF_ARM_V11_MOD', ...
    'urdf','GEN3_FOR_URDF_ARM_V11_MOD.urdf');
cfg.end_effector = 'EndEffector_Link';
cfg.q0 = [0;20;0;10;0;10;0]*pi/180;
cfg.mode = 'closed_loop';
cfg.position_offset = [0.1; 0.1; -0.1];
cfg.contact.enabled = true;
cfg.contact.offset_x = 0.05;
cfg.duration = 6;
cfg.stiffness = [50;700;700;0.1;0.01;0.01];
cfg.stiffness_filter = 1; % Ganancias constantes desde el primer compute.
% Se mantienen dampingRule, muestreo, saturaciones y slew-rate del controlador.
cfg.plot = false;
end
