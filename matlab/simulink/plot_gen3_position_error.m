function f = plot_gen3_position_error(r)
% Posicion en frame base y distancia al objetivo fijo; admite resultados previos.
pd=r.desired_position(:)';
e=r.position-pd;
distance=sqrt(sum(e.^2,2));
f=figure('Name','Simulink: posicion y error respecto al objetivo', ...
    'Color','w','Position',[120 80 1100 850]);
labels={'X','Y','Z'};
for k=1:3
    subplot(3,2,2*k-1);
    plot(r.t,100*r.position(:,k),'LineWidth',1.5); hold on;
    yline(100*pd(k),'k--','Referencia');
    ylabel([labels{k} ' base [cm]']); xlabel('Tiempo [s]'); grid on;
    legend('Efector','Referencia','Location','best');
end
subplot(3,2,2);
plot(r.t,1000*e,'LineWidth',1.2); yline(0,'k:');
ylabel('Error p - pd [mm]'); xlabel('Tiempo [s]'); grid on;
legend('e_x','e_y','e_z','Location','best');
subplot(3,2,4);
plot(r.t,1000*abs(e),'LineWidth',1.2);
ylabel('Error absoluto por eje [mm]'); xlabel('Tiempo [s]'); grid on;
legend('|e_x|','|e_y|','|e_z|','Location','best');
subplot(3,2,6);
plot(r.t,1000*distance,'LineWidth',1.5);
ylabel('Distancia al objetivo [mm]'); xlabel('Tiempo [s]'); grid on;
title(sprintf('Distancia final: %.6f mm',1000*distance(end)));
set(findall(f,'Type','axes'),'Color','w','XColor','k','YColor','k');
set(findall(f,'Type','text'),'Color','k');
set(findall(f,'Type','legend'),'Color','w','TextColor','k');
if ~isfolder(r.cfg.output_dir), mkdir(r.cfg.output_dir); end
exportgraphics(f,fullfile(r.cfg.output_dir,'simulink_position_error.png'),'Resolution',150);
fprintf('Referencia XYZ [m]: [%.6f %.6f %.6f]\n',pd);
fprintf('Posicion final XYZ [m]: [%.6f %.6f %.6f]\n',r.position(end,:));
fprintf('Error final XYZ [mm]: [%.6f %.6f %.6f]\n',1000*e(end,:));
end
