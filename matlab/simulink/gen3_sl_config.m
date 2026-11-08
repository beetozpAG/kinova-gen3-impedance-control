function cfg = gen3_sl_config()
% Una sola retirada a 2 s. Ganancias de la demo, no del hardware.
addpath(fileparts(fileparts(mfilename('fullpath'))));
cfg = gen3_release_config();
cfg.stiffness = [50;480;280;10;10;10];
cfg.q0 = [10; 30; -20; 45; 15; -30; 20]*pi/180;
cfg.reference_mode = 'AbsolutePose';
cfg.absolute_position = [0.08; 0.09; -0.1]; % Metros
cfg.duration = 6;
cfg.contact.active_intervals = [0 2];
cfg.contact.reentry_ramp_s = 0;
cfg.integration_substeps = 4;
cfg.output_dir = fullfile(fileparts(mfilename('fullpath')),'results');
end




