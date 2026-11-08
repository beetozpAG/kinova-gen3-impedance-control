function model = build_gen3_release_simulink(openUI,rebuild)
% Genera un .slx editable con bloques de resorte, damping, contacto e integracion.
if nargin<1, openUI=true; end
if nargin<2, rebuild=false; end
folder=fileparts(mfilename('fullpath')); addpath(folder); addpath(fileparts(folder));
model='gen3_pared_retira_2s'; file=fullfile(folder,[model '.slx']);
if isfile(file) && ~rebuild
    load_system(file); if openUI, open_system(model); end; return;
end
if isfile(file), copyfile(file,[file '.bak']); end
if bdIsLoaded(model), close_system(model,0); end
load_system('simulink'); new_system(model);
set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','1/(cfg.update_hz*cfg.integration_substeps)', ...
    'StopTime','cfg.duration','ReturnWorkspaceOutputs','on', ...
    'InitFcn','gen3_sl_init(bdroot);', ...
    'PostLoadFcn', ['addpath(fileparts(get_param(bdroot,''FileName''))); ' ...
                   'addpath(fileparts(fileparts(get_param(bdroot,''FileName''))));']);
gen3_sl_init(model);
ws=get_param(model,'ModelWorkspace'); robot=evalin(ws,'robot'); ee=evalin(ws,'ee');
assert(strcmp(robot.BodyNames{end},ee),'El bloque URDF espera EE como ultimo body.');

% Planta: integradores visibles, sin memoria oculta ni llamadas al runner MATLAB.
plant=[model '/Planta Gen3']; sub(plant,{'tau_total'},{'q','dq'});
sf(plant,'Dinamica M ddq = tau - c - g','robot,Rd,''dynamics''');
blk(plant,'simulink/Continuous/Integrator','Integral dq','InitialCondition','cfg.dq0');
blk(plant,'simulink/Continuous/Integrator','Integral q','InitialCondition','q0');
ln(plant,'tau_total/1','Dinamica M ddq = tau - c - g/3');
ln(plant,'Integral q/1','Dinamica M ddq = tau - c - g/1');
ln(plant,'Integral dq/1','Dinamica M ddq = tau - c - g/2');
ln(plant,'Dinamica M ddq = tau - c - g/1','Integral dq/1');
ln(plant,'Integral dq/1','Integral q/1'); ln(plant,'Integral q/1','q/1');
ln(plant,'Integral dq/1','dq/1');

sf(model,'Cinematica y gravedad','robot,Rd,''kinematics''');
ln(model,'Planta Gen3/1','Cinematica y gravedad/1');
ln(model,'Planta Gen3/2','Cinematica y gravedad/2');

ctrl=[model '/Controlador de impedancia'];
sub(ctrl,{'p','eR','vel_cart','JT'},{'tau_imp','error_pose'});
names={'p','eR','vel_cart','JT'};
for i=1:4
    blk(ctrl,'simulink/Discrete/Zero-Order Hold',['Muestreo ' names{i}],'SampleTime','1/cfg.update_hz');
    ln(ctrl,[names{i} '/1'],['Muestreo ' names{i} '/1']);
end
blk(ctrl,'simulink/Sources/Constant','Referencia pd','Value','pd');
blk(ctrl,'simulink/Math Operations/Sum','Error p menos pd','Inputs','+-');
blk(ctrl,'simulink/Signal Routing/Mux','Error cartesiano','Inputs','2');
ln(ctrl,'Muestreo p/1','Error p menos pd/1'); ln(ctrl,'Referencia pd/1','Error p menos pd/2');
ln(ctrl,'Error p menos pd/1','Error cartesiano/1'); ln(ctrl,'Muestreo eR/1','Error cartesiano/2');
blk(ctrl,'simulink/Math Operations/Gain','Resorte K e','Gain','diag(cfg.stiffness)','Multiplication','Matrix(K*u)');
blk(ctrl,'simulink/Math Operations/Gain','Amortiguador D v','Gain', ...
    'diag(2*cfg.damping_factors.*sqrt(cfg.stiffness))','Multiplication','Matrix(K*u)');
blk(ctrl,'simulink/Math Operations/Sum','Wrench -K e -D v','Inputs','--');
ln(ctrl,'Error cartesiano/1','Resorte K e/1'); ln(ctrl,'Muestreo vel_cart/1','Amortiguador D v/1');
ln(ctrl,'Resorte K e/1','Wrench -K e -D v/1'); ln(ctrl,'Amortiguador D v/1','Wrench -K e -D v/2');
blk(ctrl,'simulink/Math Operations/Product','Torques JT por wrench','Inputs','**','Multiplication','Matrix(*)');
ln(ctrl,'Muestreo JT/1','Torques JT por wrench/1'); ln(ctrl,'Wrench -K e -D v/1','Torques JT por wrench/2');
blk(ctrl,'simulink/Discrete/Unit Delay','Torque anterior','SampleTime','1/cfg.update_hz','InitialCondition','zeros(7,1)');
blk(ctrl,'simulink/Math Operations/Sum','Cambio solicitado','Inputs','+-');
blk(ctrl,'simulink/Discontinuities/Saturation','Limite cambio por ciclo', ...
    'UpperLimit','cfg.delta_tau_max','LowerLimit','-cfg.delta_tau_max');
blk(ctrl,'simulink/Math Operations/Sum','Torque limitado','Inputs','++');
ln(ctrl,'Torques JT por wrench/1','Cambio solicitado/1'); ln(ctrl,'Torque anterior/1','Cambio solicitado/2');
ln(ctrl,'Cambio solicitado/1','Limite cambio por ciclo/1'); ln(ctrl,'Limite cambio por ciclo/1','Torque limitado/1');
ln(ctrl,'Torque anterior/1','Torque limitado/2'); ln(ctrl,'Torque limitado/1','Torque anterior/1');
ln(ctrl,'Torque limitado/1','tau_imp/1'); ln(ctrl,'Error cartesiano/1','error_pose/1');
for i=1:4, ln(model,sprintf('Cinematica y gravedad/%d',i),sprintf('Controlador de impedancia/%d',i)); end

act=[model '/Gravedad blend y limites']; sub(act,{'tau_imp','g'},{'tau_motor','I_cmd'});
blk(act,'simulink/Sources/Clock','Tiempo');
blk(act,'simulink/Math Operations/Gain','Rampa 2 segundos','Gain','1/cfg.blend_time');
blk(act,'simulink/Discontinuities/Saturation','Alpha entre 0 y 1','LowerLimit','0','UpperLimit','1');
blk(act,'simulink/Discrete/Zero-Order Hold','Alpha muestreado','SampleTime','1/cfg.update_hz');
blk(act,'simulink/Discrete/Zero-Order Hold','Gravedad muestreada','SampleTime','1/cfg.update_hz');
blk(act,'simulink/Math Operations/Product','Alpha por tau','Inputs','**');
blk(act,'simulink/Math Operations/Sum','Sumar gravedad','Inputs','++');
blk(act,'simulink/Discontinuities/Saturation','Saturacion torque', ...
    'LowerLimit','-cfg.max_tau','UpperLimit','cfg.max_tau');
blk(act,'simulink/Math Operations/Gain','Torque a corriente', ...
    'Gain','diag(cfg.KTGR)','Multiplication','Matrix(K*u)');
sf(act,'Comprobar corriente finita y limites','cfg.max_current,cfg.KTGR,''current_check''');
ln(act,'Tiempo/1','Rampa 2 segundos/1'); ln(act,'Rampa 2 segundos/1','Alpha entre 0 y 1/1');
ln(act,'Alpha entre 0 y 1/1','Alpha muestreado/1'); ln(act,'Alpha muestreado/1','Alpha por tau/1');
ln(act,'tau_imp/1','Alpha por tau/2'); ln(act,'g/1','Gravedad muestreada/1');
ln(act,'Alpha por tau/1','Sumar gravedad/1'); ln(act,'Gravedad muestreada/1','Sumar gravedad/2');
ln(act,'Sumar gravedad/1','Saturacion torque/1'); ln(act,'Saturacion torque/1','tau_motor/1');
ln(act,'Saturacion torque/1','Torque a corriente/1');
ln(act,'Torque a corriente/1','Comprobar corriente finita y limites/1');
ln(act,'Comprobar corriente finita y limites/1','I_cmd/1');
ln(model,'Controlador de impedancia/1','Gravedad blend y limites/1');
ln(model,'Cinematica y gravedad/5','Gravedad blend y limites/2');

wall=[model '/Pared desaparece a 2 s']; sub(wall,{'p','vel_cart'},{'F_sobre_robot','presente'});
blk(wall,'simulink/Signal Routing/Demux','Separar posicion','Outputs','3');
blk(wall,'simulink/Signal Routing/Demux','Separar velocidad','Outputs','6');
ln(wall,'p/1','Separar posicion/1'); ln(wall,'vel_cart/1','Separar velocidad/1');
blk(wall,'simulink/Sources/Constant','Posicion pared','Value','wall_x');
blk(wall,'simulink/Math Operations/Sum','x menos pared','Inputs','+-');
ln(wall,'Separar posicion/1','x menos pared/1'); ln(wall,'Posicion pared/1','x menos pared/2');
blk(wall,'simulink/Discontinuities/Saturation','Penetracion positiva','LowerLimit','0','UpperLimit','inf');
ln(wall,'x menos pared/1','Penetracion positiva/1');
blk(wall,'simulink/Math Operations/Gain','Rigidez pared','Gain','cfg.contact.stiffness');
blk(wall,'simulink/Math Operations/Gain','Damping pared','Gain','cfg.contact.damping');
blk(wall,'simulink/Math Operations/Sum','Fuerza elastica viscosa','Inputs','++');
ln(wall,'Penetracion positiva/1','Rigidez pared/1'); ln(wall,'Separar velocidad/1','Damping pared/1');
ln(wall,'Rigidez pared/1','Fuerza elastica viscosa/1'); ln(wall,'Damping pared/1','Fuerza elastica viscosa/2');
blk(wall,'simulink/Discontinuities/Saturation','Sin traccion','LowerLimit','0','UpperLimit','inf');
ln(wall,'Fuerza elastica viscosa/1','Sin traccion/1');
blk(wall,'simulink/Logic and Bit Operations/Compare To Constant','Hay penetracion','const','0','relop','>');
ln(wall,'Penetracion positiva/1','Hay penetracion/1');
blk(wall,'simulink/Signal Attributes/Data Type Conversion','Boolean a double','OutDataTypeStr','double');
ln(wall,'Hay penetracion/1','Boolean a double/1');
blk(wall,'simulink/Sources/Step','Retirar pared','Time','2','Before','1','After','0','SampleTime','0');
blk(wall,'simulink/Math Operations/Product','Contacto unilateral activo','Inputs','***');
ln(wall,'Sin traccion/1','Contacto unilateral activo/1'); ln(wall,'Boolean a double/1','Contacto unilateral activo/2');
ln(wall,'Retirar pared/1','Contacto unilateral activo/3'); ln(wall,'Retirar pared/1','presente/1');
blk(wall,'simulink/Math Operations/Gain','Reaccion negativa X','Gain','-1');
ln(wall,'Contacto unilateral activo/1','Reaccion negativa X/1'); ln(wall,'Reaccion negativa X/1','F_sobre_robot/1');
for i=2:3, terminate(wall,sprintf('Separar posicion/%d',i),sprintf('No usar p%d',i)); end
for i=2:6, terminate(wall,sprintf('Separar velocidad/%d',i),sprintf('No usar v%d',i)); end
ln(model,'Cinematica y gravedad/1','Pared desaparece a 2 s/1');
ln(model,'Cinematica y gravedad/3','Pared desaparece a 2 s/2');
blk(model,'simulink/Math Operations/Product','Torque contacto Jx por F','Inputs','**');
ln(model,'Cinematica y gravedad/6','Torque contacto Jx por F/1');
ln(model,'Pared desaparece a 2 s/1','Torque contacto Jx por F/2');
blk(model,'simulink/Math Operations/Sum','Torque total en planta','Inputs','++');
ln(model,'Gravedad blend y limites/1','Torque total en planta/1');
ln(model,'Torque contacto Jx por F/1','Torque total en planta/2');
ln(model,'Torque total en planta/1','Planta Gen3/1');

sources={'Cinematica y gravedad/1','Controlador de impedancia/2','Pared desaparece a 2 s/1', ...
    'Pared desaparece a 2 s/2','Planta Gen3/1','Gravedad blend y limites/2','Gravedad blend y limites/1'};
vars={'posicion','error_pose','fuerza_pared','pared_presente','articulaciones','corriente','torque_motor'};
for i=1:numel(vars)
    blk(model,'simulink/Sinks/To Workspace',['Guardar ' vars{i}], ...
        'VariableName',vars{i},'SaveFormat','Timeseries','SampleTime','1/cfg.update_hz');
    ln(model,sources{i},['Guardar ' vars{i} '/1']);
end
blk(model,'simulink/Sinks/Scope','Scope posicion fuerza pared','NumInputPorts','3');
ln(model,sources{1},'Scope posicion fuerza pared/1'); ln(model,sources{3},'Scope posicion fuerza pared/2');
ln(model,sources{4},'Scope posicion fuerza pared/3');
for s={plant,ctrl,act,wall,model}, Simulink.BlockDiagram.arrangeSystem(s{1}); end
note=Simulink.Annotation(model, ...
    'SOLO SIMULACION | Referencia +10 cm | Pared +5 cm hasta t=2 s | Doble clic en subsistemas');
note.Position=[50 10];
save_system(model,file);
if openUI, open_system(model); end
end

function blk(sys,lib,name,varargin)
add_block(lib,[sys '/' name],varargin{:});
end
function sf(sys,name,params)
blk(sys,'simulink/User-Defined Functions/Level-2 MATLAB S-Function',name, ...
    'FunctionName','gen3_sl_math','Parameters',params);
end
function ln(sys,src,dst)
add_line(sys,src,dst,'autorouting','on');
end
function sub(path,inputs,outputs)
add_block('built-in/SubSystem',path);
for i=1:numel(inputs)
    blk(path,'simulink/Ports & Subsystems/In1',inputs{i},'Port',num2str(i));
end
for i=1:numel(outputs)
    blk(path,'simulink/Ports & Subsystems/Out1',outputs{i},'Port',num2str(i));
end
end
function terminate(sys,source,name)
blk(sys,'simulink/Sinks/Terminator',name); ln(sys,source,[name '/1']);
end
