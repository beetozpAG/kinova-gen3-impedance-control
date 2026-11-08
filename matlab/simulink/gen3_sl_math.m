function gen3_sl_math(block)
% Bloque Level-2 sin memoria: solo evaluaciones del modelo URDF/checks.
% Parametros: robot (o limite I), Rd (o KTGR), operacion.
block.NumDialogPrms = 3;
mode = block.DialogPrm(3).Data;
switch mode
    case 'kinematics'
        inputs = [7 7]; outputs = {3,3,6,[7 6],7,7};
    case 'dynamics'
        inputs = [7 7 7]; outputs = {7};
    case 'current_check'
        inputs = 7; outputs = {7};
    otherwise
        error('Operacion desconocida.');
end
block.NumInputPorts=numel(inputs); block.NumOutputPorts=numel(outputs);
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;
for i=1:numel(inputs)
    block.InputPort(i).Dimensions=inputs(i);
    block.InputPort(i).DatatypeID=0;
    block.InputPort(i).Complexity='Real';
    block.InputPort(i).DirectFeedthrough=true;
end
for i=1:numel(outputs)
    block.OutputPort(i).Dimensions=outputs{i};
    block.OutputPort(i).DatatypeID=0;
    block.OutputPort(i).Complexity='Real';
end
if strcmp(mode,'current_check'), block.SampleTimes=[-1 0];
else, block.SampleTimes=[0 0]; end
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('Outputs',@output);
end

function output(b)
for i=1:b.NumInputPorts
    assert(all(isfinite(b.InputPort(i).Data(:))),'Entrada NaN/Inf.');
end
mode=b.DialogPrm(3).Data;
if strcmp(mode,'current_check')
    current=b.InputPort(1).Data;
    assert(all(abs(current)<=b.DialogPrm(1).Data),'Corriente simulada excede maxCurrent.');
    b.OutputPort(1).Data=current; return;
end
robot=b.DialogPrm(1).Data;
q=b.InputPort(1).Data; dq=b.InputPort(2).Data;
if strcmp(mode,'kinematics')
    T=getTransform(robot,q,robot.BodyNames{end});
    Jraw=geometricJacobian(robot,q,robot.BodyNames{end});
    J=Jraw([4:6 1:3],:);
    aa=rotm2axang(T(1:3,1:3)*b.DialogPrm(2).Data');
    values={T(1:3,4),aa(1:3)'*aa(4),J*dq,J',gravityTorque(robot,q),J(1,:)'};
else
    % M(q)*ddq = tau_actuadores + tau_contacto - c(q,dq) - g(q).
    % c = C*dq ya multiplicado. No se usa inv(M).
    M=massMatrix(robot,q);
    c=velocityProduct(robot,q,dq);
    g=gravityTorque(robot,q);
    values={M\(b.InputPort(3).Data-c-g)};
end
for i=1:numel(values)
    assert(all(isfinite(values{i}(:))),'Modelo URDF produjo NaN/Inf.');
    b.OutputPort(i).Data=values{i};
end
end
