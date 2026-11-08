function [r,out] = run_gen3_simulink(openUI)
% Ejecuta realmente el .slx; NO llama al integrador run_gen3_simulation.
if nargin<1, openUI=true; end
m=build_gen3_release_simulink(openUI);
out=sim(m);
cfg=gen3_sl_config(); ws=get_param(m,'ModelWorkspace');
r.t=out.posicion.Time;
r.position=squeeze(out.posicion.Data);
r.pose_error=squeeze(out.error_pose.Data);
r.q=squeeze(out.articulaciones.Data);
r.current_cmd=squeeze(out.corriente.Data);
r.tau_applied=squeeze(out.torque_motor.Data);
r.contact_force_x=squeeze(out.fuerza_pared.Data);
r.contact_gain=squeeze(out.pared_presente.Data);
r.cfg=cfg; r.desired_position=evalin(ws,'pd'); r.desired_rotation=evalin(ws,'Rd');
r.position_error=r.position-r.desired_position(:)';
r.position_error_norm=sqrt(sum(r.position_error.^2,2));
r.wall=cfg.contact; r.wall.x=evalin(ws,'wall_x'); r.wall.ee=cfg.end_effector;
r.penetration=max(r.position(:,1)-r.wall.x,0).*r.contact_gain;
% Se grafica posicion/fuerza; no se estima velocidad por diferencia.
if ~isfolder(cfg.output_dir), mkdir(cfg.output_dir); end
save(fullfile(cfg.output_dir,'simulink_release.mat'),'r','out');
f=figure('Name','Simulink: pared retirada a 2 s','Color','w','Position',[100 100 1000 700]);
subplot(3,1,1);
plot(r.t,100*(r.position(:,1)-r.position(1,1)),'LineWidth',1.5); hold on;
wallLine=100*cfg.contact.offset_x*ones(size(r.t)); wallLine(r.contact_gain==0)=NaN;
plot(r.t,wallLine,'k--');
yline(100*(r.desired_position(1)-r.position(1,1)),'k:','Objetivo');
ylabel('X relativa [cm]'); grid on; legend('Efector','Pared presente');
subplot(3,1,2); plot(r.t,-r.contact_force_x,'LineWidth',1.5);
ylabel('Fuerza sobre pared [N]'); grid on;
subplot(3,1,3); plot(r.t,r.contact_gain,'LineWidth',1.5); ylim([-0.1 1.1]);
ylabel('Pared presente'); xlabel('Tiempo [s]'); grid on;
set(findall(f,'Type','axes'),'Color','w','XColor','k','YColor','k');
set(findall(f,'Type','text'),'Color','k');
set(findall(f,'Type','legend'),'Color','w','TextColor','k');
exportgraphics(f,fullfile(cfg.output_dir,'simulink_release.png'),'Resolution',150);
plot_gen3_position_error(r);
plot_gen3_motor_torques(r);
fprintf('SIMULINK: error final = %.6f mm; fuerza maxima = %.6f N\n', ...
    1000*r.position_error_norm(end),max(-r.contact_force_x));
end
