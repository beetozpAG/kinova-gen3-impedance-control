function [robot, ee, q0] = gen3_load_model(cfg)
% Modelo dinamico de Robotics System Toolbox, sin ROS ni Kortex.
assert(exist('rigidBodyTree','class') == 8, 'Instala Robotics System Toolbox.');
if isempty(cfg.urdf)
    robot = loadrobot('kinovaGen3','DataFormat','column');
else
    assert(isfile(cfg.urdf), 'No existe el URDF: %s', cfg.urdf);
    auditUrdf(cfg.urdf);
    robot = importrobot(cfg.urdf);
    robot.DataFormat = 'column';
    assert(~isempty(cfg.end_effector), ...
        'Configura cfg.end_effector con el body/TCP de tu URDF. Usa showdetails(robot).');
end
robot.Gravity = cfg.gravity;
assert(numel(homeConfiguration(robot)) == 7, ...
    ['Se requieren exactamente 7 articulaciones moviles. Si hay dedos moviles, ' ...
     'prepara un modelo con pinza fija que conserve sus masas e inercias.']);
ee = cfg.end_effector;
if isempty(ee)
    ee = robot.BodyNames{end};
    fprintf('Modelo de biblioteca: se usa el ultimo body como EE: %s\n', ee);
end
assert(any(strcmp(robot.BodyNames,ee)), 'End-effector no encontrado: %s', ee);
q0 = cfg.q0;
if isempty(q0)
    q0 = homeConfiguration(robot);
end
validateattributes(q0, {'double'}, {'size',[7 1],'finite'});
fprintf('Base: %s | EE: %s | orden de articulaciones:\n',robot.BaseName,ee);
idx = 0;
for i = 1:robot.NumBodies
    b = robot.Bodies{i};
    assert(all(isfinite([b.Mass b.CenterOfMass b.Inertia])) && b.Mass >= 0, ...
        'Parametros inerciales invalidos: %s',b.Name);
    if ~strcmp(b.Joint.Type,'fixed')
        idx = idx+1;
        assert(strcmp(b.Joint.Type,'revolute'),'Se esperaba articulacion rotacional.');
        fprintf('  q(%d): %s -> %s\n',idx,b.Joint.Name,b.Name);
    end
end
M = massMatrix(robot,q0);
[~,flag] = chol((M+M')/2);
assert(all(isfinite(M(:))) && flag == 0 && rcond(M) > 1e-12, ...
    'M(q0) no es definida positiva o esta mal condicionada. Revisa inercias.');
fprintf('Masa de bodies importados: %.4f kg (no prueba equivalencia con tu montaje).\n', ...
    sum(cellfun(@(b)b.Mass,robot.Bodies)));
end

function auditUrdf(path)
% Evita aceptar silenciosamente un URDF puramente visual/cinematico.
doc = xmlread(path);
links = doc.getElementsByTagName('link');
joints = doc.getElementsByTagName('joint');
moving = {};
for i = 0:joints.getLength-1
    joint = joints.item(i);
    if ~strcmp(char(joint.getAttribute('type')),'fixed')
        child = joint.getElementsByTagName('child');
        if child.getLength > 0
            moving{end+1} = char(child.item(0).getAttribute('link')); %#ok<AGROW>
        end
    end
end
for i = 0:links.getLength-1
    link = links.item(i); name = char(link.getAttribute('name'));
    inertial = link.getElementsByTagName('inertial');
    if inertial.getLength == 0
        assert(~any(strcmp(moving,name)), ...
            'Falta <inertial> en link movil %s. Un URDF visual no basta.',name);
        warning('gen3:MissingFixedInertia', ...
            'Link fijo %s sin inertial: verifica que sea un frame sin masa.',name);
    else
        el = inertial.item(0);
        masses = el.getElementsByTagName('mass');
        tensors = el.getElementsByTagName('inertia');
        assert(masses.getLength == 1 && tensors.getLength == 1, ...
            'Inertial incompleto en %s.',name);
        mass = str2double(char(masses.item(0).getAttribute('value')));
        fields = {'ixx','iyy','izz','iyz','ixz','ixy'};
        v = zeros(1,6);
        for j = 1:6
            v(j) = str2double(char(tensors.item(0).getAttribute(fields{j})));
        end
        I = [v(1) v(6) v(5);v(6) v(2) v(4);v(5) v(4) v(3)];
        assert(isfinite(mass) && mass >= 0 && all(isfinite(v)), ...
            'Masa/inercia invalida en %s.',name);
        principal = eig(I);
        assert(min(principal) >= -1e-12 && ...
            2*max(principal) <= sum(principal)+1e-10, ...
            'Tensor de inercia no fisico en %s.',name);
        if any(strcmp(moving,name))
            assert(mass > 0 && min(principal) > 0, ...
                'Se requiere masa e inercia positiva en link movil %s.',name);
        end
    end
end
end
