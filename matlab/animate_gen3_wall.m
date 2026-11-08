function animate_gen3_wall(r)
% Animacion del brazo URDF y del punto de contacto; no colision de eslabones.
[robot,ee] = gen3_load_model(r.cfg); %#ok<ASGLU>
f = figure('Name','Gen3 simulado contra pared'); ax = axes(f);
xwall = r.wall.x; p0 = r.position(1,:); extent = 0.15;
yy = p0(2)+extent*[-1 1 1 -1]; zz = p0(3)+extent*[-1 -1 1 1];
stride = max(1,round(r.cfg.update_hz/20));
for k = 1:stride:numel(r.t)
    if ~isgraphics(f), break; end
    cla(ax);
    show(robot,r.q(k,:)','Parent',ax,'Frames','off','PreservePlot',false);
    hold(ax,'on');
    gain = gen3_wall_activation(r.t(k),r.wall);
    if gain > 0
        patch(ax,xwall*ones(1,4),yy,zz,[0.9 0.4 0.2], ...
            'FaceAlpha',0.35*gain,'Tag','Gen3ContactWall');
    end
    plot3(ax,r.position(1:k,1),r.position(1:k,2),r.position(1:k,3),'b-','LineWidth',2);
    plot3(ax,r.desired_position(1),r.desired_position(2),r.desired_position(3),'rx','MarkerSize',12);
    plot3(ax,r.position(k,1),r.position(k,2),r.position(k,3),'bo','MarkerFaceColor','b');
    title(ax,sprintf('t = %.2f s | Contacto habilitado: %.0f %% | Fuerza = %.2f N', ...
        r.t(k),100*gain,-r.contact_force_x(k)));
    axis(ax,'equal'); xlim(ax,[-1.2 1.2]); ylim(ax,[-1.2 1.2]); zlim(ax,[-0.2 1.6]);
    view(ax,135,25); grid(ax,'on'); drawnow; pause(0.05);
end
end
