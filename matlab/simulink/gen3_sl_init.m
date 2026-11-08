function gen3_sl_init(model)
% Inicializa parametros del Model Workspace al pulsar Run.
cfg = gen3_sl_config();
[robot,ee,q0] = gen3_load_model(cfg);
assert(strcmp(robot.BodyNames{end},ee),'El bloque URDF espera EE como ultimo body.');
T0=getTransform(robot,q0,ee);
s = gen3_controller_init(cfg,q0,T0);
assert(cfg.stiffness_filter==1 && ~cfg.enable_nullspace && ~cfg.enable_wrench, ...
    'Este diagrama didactico implementa ganancias constantes, sin nullspace/wrench.');
ws = get_param(model,'ModelWorkspace');
assignin(ws,'cfg',cfg); assignin(ws,'robot',robot); assignin(ws,'ee',ee);
assignin(ws,'q0',q0); assignin(ws,'pd',s.p); assignin(ws,'Rd',s.R);
assignin(ws,'wall_x',T0(1,4)+cfg.contact.offset_x);
end
