function plot_gen3_simulation(r)
% Graficas de estado y comandos; r es la estructura guardada en el .mat.
t = r.t;
figure('Name',['Gen3: ' r.cfg.mode]);
subplot(3,2,1);
plot(t,r.position); hold on;
plot(t,repmat(r.desired_position',numel(t),1),'--');
xlabel('Tiempo [s]'); ylabel('Posicion [m]'); grid on;
legend('x','y','z','x_d','y_d','z_d');
subplot(3,2,2);
plot(t,r.pose_error(:,1:3)); grid on;
xlabel('Tiempo [s]'); ylabel('Error posicion [m]'); legend('x','y','z');
subplot(3,2,3);
plot(t,r.pose_error(:,4:6)); grid on;
xlabel('Tiempo [s]'); ylabel('Error rotacion [rad]');
subplot(3,2,4);
plot(t,r.tau_impedance); grid on;
xlabel('Tiempo [s]'); ylabel('Impedancia limitada [Nm]');
subplot(3,2,5);
plot(t,r.tau_applied); grid on;
xlabel('Tiempo [s]'); ylabel('Torque aplicado a planta [Nm]');
subplot(3,2,6);
plot(t,r.current_cmd); hold on;
plot(t,repmat(r.cfg.max_current',numel(t),1),'k:');
plot(t,-repmat(r.cfg.max_current',numel(t),1),'k:'); grid on;
xlabel('Tiempo [s]'); ylabel('Corriente equivalente [A]');
figure('Name','Trayectoria cartesiana');
plot3(r.position(:,1),r.position(:,2),r.position(:,3)); hold on;
plot3(r.desired_position(1),r.desired_position(2),r.desired_position(3),'rx');
xlabel('X base [m]'); ylabel('Y base [m]'); zlabel('Z base [m]');
axis equal; grid on;
end
