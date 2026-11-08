function metrics = validate_gen3_release(r, fine)
% Comprueba eventos exactos, contactos repetidos y convergencia numerica.
w = r.wall;
assert(gen3_wall_activation(2-1e-9,w)==1);
assert(gen3_wall_activation(2,w)==0 && gen3_wall_activation(4,w)==0);
assert(gen3_wall_activation(4.25,w)>0 && gen3_wall_activation(4.5,w)==1);
assert(gen3_wall_force(w.x+0.05,1,w,2)==0);
assert(gen3_wall_force(w.x+0.05,1,w,6)==0);
assert(all(r.contact_force_x(r.contact_gain==0)==0));
assert(all(r.penetration(r.contact_gain==0)==0));
metrics.contact_peak_n = zeros(1,3);
metrics.release_error_m = zeros(1,3);
metrics.contact_error_m = zeros(1,3);
for i = 1:3
    a = w.active_intervals(i,1); b = w.active_intervals(i,2);
    active = r.t>=a & r.t<b;
    metrics.contact_peak_n(i) = max(-r.contact_force_x(active));
    metrics.contact_error_m(i) = abs(r.pose_error(find(active,1,'last'),1));
    if i < 3, endFree = w.active_intervals(i+1,1); else, endFree = r.cfg.duration; end
    free = r.t>=b & r.t<endFree;
    metrics.release_error_m(i) = abs(r.pose_error(find(free,1,'last'),1));
end
assert(all(metrics.contact_peak_n>1),'No hubo resistencia en todos los ciclos.');
assert(all(metrics.release_error_m<metrics.contact_error_m), 'No se recupero tras soltar.');
metrics.final_error_m = norm(r.pose_error(end,1:3));
metrics.saturated_cycles = sum(r.saturated);
cfg = r.cfg; cfg.integration_substeps = cfg.integration_substeps*2;
cfg.plot = false; cfg.save_results = false;
if nargin < 2
    fine = run_gen3_simulation(cfg);
else
    assert(isequaln(fine.cfg,cfg) && isequal(fine.t,r.t), ...
        'La corrida fina no corresponde al mismo experimento.');
end
metrics.max_position_difference_m = max(vecnorm(r.position-fine.position,2,2));
metrics.max_force_difference_n = max(abs(r.contact_force_x-fine.contact_force_x));
% Kelvin-Voigt unilateral tiene salto de damping al entrar en contacto.
% Conservar max puntual; evaluar tambien contacto sostenido e impulso L1.
sustained = r.penetration>1e-5 & fine.penetration>1e-5;
assert(any(sustained),'No hay contacto sostenido para comparar.');
metrics.sustained_force_difference_n = max(abs(r.contact_force_x(sustained)-fine.contact_force_x(sustained)));
metrics.absolute_force_impulse_difference_ns = sum(abs(r.contact_force_x-fine.contact_force_x))/cfg.update_hz;
metrics.peak_force_difference_n = abs(max(-r.contact_force_x)-max(-fine.contact_force_x));
save(fullfile(cfg.output_dir,'release_validation.mat'),'r','fine','metrics');
fid=fopen(fullfile(cfg.output_dir,'release_validation.txt'),'w'); assert(fid>=0);
cleanup=onCleanup(@()fclose(fid)); names=fieldnames(metrics);
for i=1:numel(names), fprintf(fid,'%s: %s\n',names{i},mat2str(metrics.(names{i}),9)); end
clear cleanup; disp(metrics);
assert(metrics.final_error_m<0.002,'Error final mayor a 2 mm.');
assert(metrics.max_position_difference_m<2e-4,'Convergencia de posicion insuficiente.');
assert(metrics.sustained_force_difference_n<0.2,'Convergencia de fuerza sostenida insuficiente.');
assert(metrics.absolute_force_impulse_difference_ns<0.02,'Convergencia de impulso insuficiente.');
assert(metrics.peak_force_difference_n<0.2,'Convergencia de pico global insuficiente.');
fprintf('Retirada a 2 s, resistencia repetida, recuperacion y convergencia: PASS.\n');
end
