function [o, s] = gen3_controller_step(s, q, dq, T, J)
% J debe tener orden [velocidad lineal; velocidad angular], frame base.
% Rotaciones matriciales: equivalentes al quaternion/AngleAxis de C++.
validateattributes(q, {'double'}, {'size',[7 1],'finite'});
validateattributes(dq, {'double'}, {'size',[7 1],'finite'});
validateattributes(J, {'double'}, {'size',[6 7],'finite'});
validateattributes(T, {'double'}, {'size',[4 4],'finite'});
ap = s.a(1); ak = s.a(2); aw = s.a(3); an = s.a(4);
s.p = (1-ap)*s.p + ap*s.p_target;
aa = rotm2axang(s.R'*s.R_target);
s.R = s.R*axang2rotm([aa(1:3), ap*aa(4)]); % Geodesica equivalente a slerp.
s.K = (1-ak)*s.K + ak*s.K_target;
s.D = (1-ak)*s.D + ak*s.D_target;
s.Kn = (1-ak)*s.Kn + ak*s.Kn_target;
s.Dn = (1-ak)*s.Dn + ak*s.Dn_target;
s.w = (1-aw)*s.w + aw*s.w_target;
s.q_null = (1-an)*s.q_null + an*s.q_null_target;
aa = rotm2axang(T(1:3,1:3)*s.R'); % Actual * deseada inversa.
eR = aa(1:3)'*aa(4);
if abs(aa(4)) < 1e-12
    eR = zeros(3,1);
end
o.pose_error = [T(1:3,4)-s.p; eR];
o.cart_velocity = J*dq;
o.tau_task = J'*(-s.K.*o.pose_error-s.D.*o.cart_velocity);
[U,S,V] = svd(J','econ');
singular = diag(S);
tol = 1e-6*max(size(J'))*max(abs(singular));
inv_s = zeros(size(singular));
keep = abs(singular) > tol;
inv_s(keep) = 1./singular(keep);
N = eye(7)-J'*(V*diag(inv_s)*U');
o.tau_null = N*(s.Kn*(s.q_null-q)-s.Dn*dq);
o.tau_ext = J'*s.w;
o.tau_requested = o.tau_task+o.tau_null+o.tau_ext;
o.tau_impedance = s.tau_last + min(max(o.tau_requested-s.tau_last, ...
    -s.delta_tau_max),s.delta_tau_max);
s.tau_last = o.tau_impedance;
assert(all(isfinite([o.pose_error;o.tau_requested;o.tau_impedance])), ...
    'Controlador produjo NaN/Inf.');
end
