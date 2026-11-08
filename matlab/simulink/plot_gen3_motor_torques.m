function f = plot_gen3_motor_torques(r)
% Torque aplicado despues del blend/saturacion y limites de la misma corrida.
limits=r.cfg.max_tau(:)';
tau=r.tau_applied;
validateattributes(limits,{'double'},{'size',[1 7],'finite','positive'});
validateattributes(tau,{'double'},{'size',[numel(r.t) 7],'finite'});
peak=max(abs(tau),[],1);
f=figure('Name','Simulink: torques de motor y limites', ...
    'Color','w','Position',[100 50 1200 950]);
for k=1:7
    subplot(4,2,k);
    plot(r.t,tau(:,k),'LineWidth',1.2); hold on;
    yline(limits(k),'r--','Limite superior');
    yline(-limits(k),'r--','Limite inferior');
    yline(0,'k:','HandleVisibility','off');
    % Mantener visibles los limites y cualquier eventual exceso en los datos.
    extent=1.12*max(limits(k),peak(k)); ylim([-extent extent]);
    xlabel('Tiempo [s]'); ylabel('Torque [N m]'); grid on;
    title(sprintf('J%d: pico |torque| %.3f / limite %.3f N m',k,peak(k),limits(k)));
end
subplot(4,2,8);
bar(1:7,100*peak./limits); hold on;
yline(100,'r--','Limite');
ylim([0 max(115,112*max(peak./limits))]); xticks(1:7);
xlabel('Articulacion'); ylabel('Pico / limite [%]'); grid on;
title('Uso maximo del limite en las muestras registradas');
set(findall(f,'Type','axes'),'Color','w','XColor','k','YColor','k');
set(findall(f,'Type','text'),'Color','k');
if ~isfolder(r.cfg.output_dir), mkdir(r.cfg.output_dir); end
exportgraphics(f,fullfile(r.cfg.output_dir,'simulink_motor_torques.png'),'Resolution',150);
fprintf('Pico de torque / limite por articulacion [%%]:');
fprintf(' %.2f',100*peak./limits); fprintf('\n');
end
