% Experimento reproducible, exclusivamente en simulacion.
folder = fileparts(mfilename('fullpath'));
addpath(folder);
cfg = gen3_sim_config();
cfg.urdf = fullfile(folder,'URDFFiles','GEN3_FOR_URDF_ARM_V11_MOD', ...
    'urdf','GEN3_FOR_URDF_ARM_V11_MOD.urdf');
cfg.end_effector = 'EndEffector_Link';
% Postura de prueba EN EL MODELO URDF, no consigna de hardware.
cfg.q0 = [10;30;-20;45;15;-30;20]*pi/180;
cfg.plot = false;
% La corrida de 30 s en closed_loop divergio con las ganancias originales.
% Ventana corta reproducible para diagnosticarla, sin retocar ganancias.
cfg.duration = 2;
validate_gen3_simulation(cfg);
cfg.mode = 'shadow';
shadow = run_gen3_simulation(cfg);
cfg.mode = 'closed_loop';
closed = run_gen3_simulation(cfg);
previousVisibility = get(groot,'defaultFigureVisible');
visibilityCleanup = onCleanup(@()set(groot,'defaultFigureVisible',previousVisibility));
set(groot,'defaultFigureVisible','off');
plot_gen3_simulation(closed);
figures = findall(groot,'Type','figure');
for i = 1:numel(figures)
    exportgraphics(figures(i),fullfile(cfg.output_dir, ...
        sprintf('local_urdf_closed_%d.png',i)),'Resolution',150);
end
close all;
figure;
plot(shadow.t,vecnorm(shadow.pose_error(:,1:3),2,2)*1000, ...
    closed.t,vecnorm(closed.pose_error(:,1:3),2,2)*1000);
grid on; xlabel('Tiempo [s]'); ylabel('Norma error posicion [mm]');
legend('Shadow','Impedancia en planta simulada');
exportgraphics(gcf,fullfile(cfg.output_dir,'local_urdf_comparison.png'),'Resolution',150);
close all;
% Sensibilidad numerica: mismo controlador, doble resolucion de la planta.
cfg.save_results = false;
coarse = closed;
cfg.integration_substeps = 4;
fine = run_gen3_simulation(cfg);
[robot,ee,q0] = gen3_load_model(cfg);
M = massMatrix(robot,q0);
Jraw = geometricJacobian(robot,q0,ee); J = Jraw([4:6 1:3],:);
Dinitial = diag(2*sqrt([200;200;200;20;20;20]));
initialDampingRates = eig(M\(J'*Dinitial*J));
fid = fopen(fullfile(cfg.output_dir,'local_urdf_summary.txt'),'w');
assert(fid >= 0,'No se pudo guardar resumen.');
cleanup = onCleanup(@()fclose(fid));
fprintf(fid,'MATLAB %s\nURDF: %s\nEE: %s\n',version,cfg.urdf,cfg.end_effector);
fprintf(fid,'Postura inicial [deg]: %s\n',mat2str(cfg.q0'*180/pi));
fprintf(fid,'Ventana de diagnostico [s]: %.3f\n',cfg.duration);
fprintf(fid,'Autovalores M(q0) [kg m^2]: %s\n',mat2str(eig(M)',6));
fprintf(fid,'Max autovalor M^-1 J^T D_inicial J [1/s]: %.9g\n',max(real(initialDampingRates)));
fprintf(fid,'NOTA: corrida previa closed_loop de 30 s aborto por NaN tras crecer el error.\n');
fprintf(fid,'Error posicion ultimo registro shadow [m]: %.9g\n',norm(shadow.pose_error(end,1:3)));
fprintf(fid,'Error posicion ultimo registro closed [m]: %.9g\n',norm(closed.pose_error(end,1:3)));
fprintf(fid,'Error orientacion ultimo registro closed [rad]: %.9g\n',norm(closed.pose_error(end,4:6)));
fprintf(fid,'Max corriente equivalente por joint [A]: %s\n',mat2str(max(abs(closed.current_cmd)),6));
fprintf(fid,'Ciclos saturados closed: %d de %d\n',sum(closed.saturated),numel(closed.t));
fprintf(fid,'Diferencia q final subpasos 2 vs 4, a 2 s [rad]: %.9g\n',norm(coarse.final_state(1:7)-fine.final_state(1:7)));
fprintf(fid,'Diferencia dq final subpasos 2 vs 4, a 2 s [rad/s]: %.9g\n',norm(coarse.final_state(8:14)-fine.final_state(8:14)));
fprintf(fid,'Max velocidad por joint [rad/s]: %s\n',mat2str(max(abs(closed.dq)),6));
fprintf(fid,'Max diferencia posicion trayectorias subpasos 2 vs 4 [m]: %.9g\n',max(vecnorm(coarse.position-fine.position,2,2)));
save(fullfile(cfg.output_dir,'local_urdf_integration_comparison.mat'),'coarse','fine','initialDampingRates','M');
clear cleanup;
type(fullfile(cfg.output_dir,'local_urdf_summary.txt'));
clear visibilityCleanup;
