function r = run_gen3_release_demo(animate)
% Retirar pared a 2 s, repetir resistencia/liberacion, referencia +10 cm fija.
if nargin < 1, animate = true; end
cfg = gen3_release_config();
r = run_gen3_simulation(cfg);
plot_gen3_release(r);
if animate, animate_gen3_wall(r); end
end
