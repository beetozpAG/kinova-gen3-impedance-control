function metrics = validate_gen3_wall(baseline)
% Prueba fisica del contacto y convergencia de la planta (mismo controlador).
% baseline es resultado de run_gen3_wall_demo(false).
w = baseline.wall;
assert(gen3_wall_force(w.x-0.01,1,w) == 0,'Fuerza antes del contacto.');
assert(gen3_wall_force(w.x,1,w) == 0,'Fuerza sin penetracion.');
assert(abs(gen3_wall_force(w.x+0.001,0,w)+0.001*w.stiffness) < 1e-10);
assert(gen3_wall_force(w.x+0.001,-100,w) == 0,'Contacto atractivo al separarse.');
off = w; off.enabled = false;
assert(gen3_wall_force(w.x+0.001,1,off) == 0,'Pared deshabilitada ejerce fuerza.');
cfg = baseline.cfg; cfg.integration_substeps = 2*cfg.integration_substeps;
cfg.plot = false; cfg.save_results = false;
fine = run_gen3_simulation(cfg);
metrics.max_position_difference_m = max(vecnorm(baseline.position-fine.position,2,2));
metrics.max_force_difference_n = max(abs(baseline.contact_force_x-fine.contact_force_x));
metrics.final_displacement_m = baseline.position(end,1)-baseline.position(1,1);
metrics.final_force_n = -baseline.contact_force_x(end);
metrics.final_penetration_m = baseline.penetration(end);
metrics.contact_time_s = baseline.t(find(baseline.contact_force_x < 0,1));
metrics.saturated_cycles = sum(baseline.saturated);
gap = baseline.desired_position(1)-w.x;
k = cfg.stiffness(1);
metrics.equilibrium_force_n = k*w.stiffness/(k+w.stiffness)*gap;
metrics.equilibrium_penetration_m = k/(k+w.stiffness)*gap;
save(fullfile(cfg.output_dir,'wall_validation.mat'),'baseline','fine','metrics');
fid = fopen(fullfile(cfg.output_dir,'wall_validation.txt'),'w');
assert(fid >= 0); cleanup = onCleanup(@()fclose(fid));
names = fieldnames(metrics);
for i = 1:numel(names)
    fprintf(fid,'%s: %.10g\n',names{i},metrics.(names{i}));
end
clear cleanup;
disp(metrics);
assert(~isempty(metrics.contact_time_s),'El efector no toco la pared.');
assert(all(baseline.contact_force_x <= 0),'Fuerza atractiva.');
assert(metrics.max_position_difference_m < 2e-4,'Sin convergencia de posicion a 0.2 mm.');
assert(metrics.max_force_difference_n < 0.2,'Sin convergencia de fuerza a 0.2 N.');
assert(abs(metrics.final_force_n-metrics.equilibrium_force_n) < 0.2, ...
    'La fuerza final no se acerca al equilibrio esperado.');
fprintf('Contacto unilateral, equilibrio y comparacion de pasos: PASS.\n');
end
