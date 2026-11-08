function validate_gen3_simulation(cfg)
% Ejecutar en MATLAB antes de una simulacion larga.
% Pruebas independientes: signo orientacion, potencia, nullspace, dinamica y FD.
if nargin == 0, cfg = gen3_sim_config(); end
[robot,ee,q] = gen3_load_model(cfg);
T = getTransform(robot,q,ee);
Jraw = geometricJacobian(robot,q,ee);
J = Jraw([4:6 1:3],:);
epsilon = 1e-6;
Jfd = zeros(6,7);
for i = 1:7
    delta = zeros(7,1); delta(i) = epsilon;
    Tp = getTransform(robot,q+delta,ee);
    Tm = getTransform(robot,q-delta,ee);
    Jfd(1:3,i) = (Tp(1:3,4)-Tm(1:3,4))/(2*epsilon);
    Rdot = (Tp(1:3,1:3)-Tm(1:3,1:3))/(2*epsilon);
    W = Rdot*T(1:3,1:3)';
    Jfd(4:6,i) = [W(3,2);W(1,3);W(2,1)];
end
assert(norm(J-Jfd,'fro') < 1e-5,'Jacobiano no coincide con diferencias finitas.');
dq = [0.1;-0.2;0.05;0.1;0.02;-0.03;0.07];
tau = [1;2;-1;0.5;0.2;0.1;-0.2];
M = massMatrix(robot,q);
c = velocityProduct(robot,q,dq); % Vector C*dq, no matriz C.
g = gravityTorque(robot,q);
ddq = forwardDynamics(robot,q,dq,tau);
assert(norm(M*ddq+c+g-tau) < 1e-8*(1+norm(tau)), 'Residual dinamico excesivo.');
assert(norm(forwardDynamics(robot,q,zeros(7,1),g)) < 1e-8, ...
    'La compensacion de gravedad no equilibra la planta.');
w = [1;2;3;0.2;0.1;0.3];
assert(abs(dq'*(J'*w)-(J*dq)'*w) < 1e-12,'Potencia no conservada por J transpose.');

% Planta ficticia J=[I6 0]: permite resultados esperados sin depender del URDF.
test = gen3_sim_config(); test.reference_mode = 'InitialPose';
test.stiffness_filter = 1; test.delta_tau_max = 1e6;
Jtest = [eye(6),zeros(6,1)]; qtest = zeros(7,1);
s = gen3_controller_init(test,qtest,eye(4));
Ttest = eye(4); Ttest(1:3,1:3) = axang2rotm([0 0 1 0.2]);
[o,~] = gen3_controller_step(s,qtest,qtest,Ttest,Jtest);
assert(abs(o.pose_error(6)-0.2) < 1e-12 && abs(o.tau_task(6)+2) < 1e-10, ...
    'Signo/magnitud incorrecto del torque restaurador de orientacion.');
test.enable_nullspace = true; test.nullspace_offset(7) = 0.1;
test.nullspace_filter = 1;
s = gen3_controller_init(test,qtest,eye(4));
[o,~] = gen3_controller_step(s,qtest,qtest,eye(4),Jtest);
assert(norm(Jtest*o.tau_null) < 1e-12 && abs(o.tau_null(7)-0.5) < 1e-12, ...
    'Proyeccion de nullspace incorrecta.');
test = gen3_sim_config();
s = gen3_controller_init(test,qtest,eye(4));
[o,~] = gen3_controller_step(s,qtest,qtest,eye(4),Jtest);
assert(o.tau_task(1) > 9.99 && abs(o.tau_impedance(1)-1) < 1e-12, ...
    'No se conserva arranque del constructor C++ / limite de 1 Nm por ciclo.');

% Integracion con gravedad sola y estado inicial en reposo.
short = cfg; short.mode = 'shadow'; short.duration = 0.005;
short.dq0 = zeros(7,1); short.disturbance_tau = zeros(7,1);
short.plot = false; short.save_results = false;
r = run_gen3_simulation(short);
assert(norm(r.final_state-[q;zeros(7,1)]) < 1e-8, ...
    'Shadow ideal en reposo deberia mantener el estado.');
fprintf('Validaciones MATLAB superadas: Jacobiano, dinamica, orientacion, nullspace y shadow.\n');
end
