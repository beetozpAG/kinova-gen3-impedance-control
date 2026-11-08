function plot_gen3_wall(r)
% Guarda graficas, ademas de mostrarlas en MATLAB.
x0 = r.position(1,1); t = r.t;
f = figure('Name','Impedancia: objetivo +10 cm, pared +5 cm', ...
    'Color','w','Position',[100 100 1000 750]);
subplot(3,1,1);
plot(t,100*(r.position(:,1)-x0),'LineWidth',1.5); hold on;
yline(100*(r.wall.x-x0),'--','Pared','Color','k');
yline(100*(r.desired_position(1)-x0),':','Objetivo','Color','k');
ylim([0 11]);
ylabel('Desplazamiento X [cm]'); grid on;
subplot(3,1,2);
plot(t,-r.contact_force_x,'LineWidth',1.5); hold on;
plot(t,r.stiffness(:,1).*(r.desired_position(1)-r.position(:,1)),':');
ylabel('Fuerza [N]'); grid on;
legend('Fuerza EE sobre pared','Resorte cartesiano Kx*(xd-x)');
subplot(3,1,3);
plot(t,1000*r.penetration,'LineWidth',1.5);
ylabel('Penetracion elastica [mm]'); xlabel('Tiempo [s]'); grid on;
set(findall(f,'Type','axes'),'Color','w','XColor','k','YColor','k');
set(findall(f,'Type','text'),'Color','k');
set(findall(f,'Type','legend'),'Color','w','TextColor','k');
if ~isfolder(r.cfg.output_dir), mkdir(r.cfg.output_dir); end
exportgraphics(f,fullfile(r.cfg.output_dir,'wall_contact.png'),'Resolution',150);
end
