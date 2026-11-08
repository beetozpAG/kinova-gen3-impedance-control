function cfg = gen3_release_config()
% Misma demo validada: referencia fija, mano idealizada intermitente.
cfg = gen3_wall_config();
cfg.q0(4) = pi/2; % Mas flexion: +10 cm alcanzables sin acercarse a extension.
cfg.duration = 13;
cfg.contact.active_intervals = [0 2;4 6;8 10];
cfg.contact.reentry_ramp_s = 0.5;
cfg.output_dir = fullfile(cfg.output_dir,'release');
end
