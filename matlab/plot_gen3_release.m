function plot_gen3_release(r)
f = figure('Name','Impedancia: sujetar y soltar', 'Color','w', ...
    'Position',[100 100 1100 850]);
t = r.t; x0 = r.position(1,1);
subplot(4,1,1);
plot(t,100*(r.position(:,1)-x0),'LineWidth',1.5); hold on;
wallLine = 100*(r.wall.x-x0)*ones(size(t)); wallLine(r.contact_gain==0)=NaN;
plot(t,wallLine,'k--','LineWidth',1.5);
yline(100*(r.desired_position(1)-x0),'k:','Objetivo');
ylabel('X relativa [cm]'); legend('Efector','Pared presente'); grid on;
subplot(4,1,2);
plot(t,-r.contact_force_x,'LineWidth',1.5); ylabel('Fuerza sobre mano [N]'); grid on;
subplot(4,1,3);
plot(t,r.cart_velocity(:,1),'LineWidth',1.2); ylabel('Velocidad X [m/s]'); grid on;
subplot(4,1,4);
plot(t,r.contact_gain,'LineWidth',1.5); ylim([-0.05 1.05]);
ylabel('Activacion mano'); xlabel('Tiempo [s]'); grid on;
set(findall(f,'Type','axes'),'Color','w','XColor','k','YColor','k');
set(findall(f,'Type','text'),'Color','k');
set(findall(f,'Type','legend'),'Color','w','TextColor','k');
if ~isfolder(r.cfg.output_dir), mkdir(r.cfg.output_dir); end
exportgraphics(f,fullfile(r.cfg.output_dir,'release_response.png'),'Resolution',150);
end
