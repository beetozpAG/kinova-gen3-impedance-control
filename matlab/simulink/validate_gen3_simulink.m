function metrics = validate_gen3_simulink(r)
% Compara el motor de Simulink con el integrador MATLAB independiente.
cfg=r.cfg; cfg.save_results=false; cfg.plot=false;
reference=run_gen3_simulation(cfg);
idx=1:numel(reference.t);
assert(max(abs(r.t(idx)-reference.t))<1e-10,'Tiempos desalineados.');
metrics.position_difference_m=max(vecnorm(r.position(idx,:)-reference.position,2,2));
metrics.torque_difference_nm=max(abs(r.tau_applied(idx,:)-reference.tau_applied),[],'all');
dforce=abs(r.contact_force_x(idx)-reference.contact_force_x);
metrics.force_pointwise_difference_n=max(dforce);
metrics.force_impulse_difference_ns=sum(dforce)/cfg.update_hz;
metrics.final_position_error_m=norm(r.pose_error(end,1:3));
assert(all(r.contact_force_x(r.t>=2)==0),'Fuerza despues de retirar pared.');
assert(all(r.contact_gain(r.t<2)==1) && all(r.contact_gain(r.t>=2)==0),'Step incorrecto.');
assert(all(isfinite(r.q(:))) && all(isfinite(r.current_cmd(:))));
assert(all(all(abs(r.current_cmd)<=cfg.max_current')),'Limites de corriente.');
save(fullfile(cfg.output_dir,'simulink_validation.mat'),'metrics','reference');
fid=fopen(fullfile(cfg.output_dir,'simulink_validation.txt'),'w'); assert(fid>=0);
cleanup=onCleanup(@()fclose(fid)); names=fieldnames(metrics);
for i=1:numel(names), fprintf(fid,'%s: %.10g\n',names{i},metrics.(names{i})); end
clear cleanup; disp(metrics);
assert(metrics.position_difference_m<2e-4,'Diferencia de posicion mayor a 0.2 mm.');
assert(metrics.force_impulse_difference_ns<0.02,'Diferencia de impulso mayor a 0.02 N s.');
assert(metrics.final_position_error_m<0.002,'Error final mayor a 2 mm.');
fprintf('Simulink vs MATLAB, retirada y recuperacion: PASS.\n');
end
