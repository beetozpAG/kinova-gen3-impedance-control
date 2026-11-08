function result = run_gen3_wall_demo(animate)
% RUN_GEN3_WALL_DEMO Referencia +10 cm, pared +5 cm, sin shadow ni hardware.
% Uso: addpath('matlab'); r = run_gen3_wall_demo();
% Solo calculo/graficas sin animacion: r = run_gen3_wall_demo(false);
if nargin == 0, animate = true; end
cfg = gen3_wall_config();
result = run_gen3_simulation(cfg);
plot_gen3_wall(result);
if animate, animate_gen3_wall(result); end
end
