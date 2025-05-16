%% 1) Robot Physical Parameters (symbolic, meters)
l0  = sym(0);        % Base offset along x
h1  = sym(0.030);    % Base height
l1  = sym(0.015);    % Shoulder link
l2  = sym(0.080);    % Upper arm
l3  = sym(0.080);    % Forearm
l4  = sym(0.080);    % Wrist link

global params
params = struct('l0',l0,'h1',h1,'l1',l1,'l2',l2,'l3',l3,'l4',l4);

%% 2) Symbolic Joint Variables & Dynamics Symbols
syms q1 q2 q3 q4 real           % Joint angles
syms m1 m2 m3 m4 g real         % Masses and gravity

%% 3) DH Parameters [theta, d, a, alpha]
DH = [ q1,  h1, l1, pi/2;
       q2,   0, l2,    0;
       q3,   0, l3,    0;
       q4,   0, l4,    0];

%% 4) DH-to-Homogeneous Transform Function
dhTransform = @(theta,d,a,alpha) [ ...
    cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta); ...
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta); ...
         0    ,         sin(alpha)      ,        cos(alpha)       ,     d      ; ...
         0    ,             0           ,             0          ,     1    ];

%% 5) Symbolic Forward Kinematics
T = eye(4);
for i = 1:4
    T = T * dhTransform(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
end
T04   = simplify(T);
P_sym = simplify(T04(1:3,4));      % End-effector position
R_sym = simplify(T04(1:3,1:3));    % End-effector orientation

disp('Forward Kinematics P(q):'); pretty(P_sym)

%% 6) Geometric Jacobian and Pseudo-Inverse
J_sym    = simplify(jacobian(P_sym, [q1; q2; q3; q4]));
Jinv_sym = simplify(pinv(J_sym));

disp('Jacobian J(q):');     pretty(J_sym)
disp('Pseudo-inverse J#(q):'); pretty(Jinv_sym)

%% 7) Symbolic Inverse Kinematics
% Solve for q1, q2, q3 given target position x,y,z (ignore q4)
syms x y z real
% 1) Base rotation
q1_sol = atan2(y, x);

% 2) Wrist center projection
d = sqrt((x - params.l0)^2 + y^2);
r = d - params.l1;
s = z - params.h1;

% 3) Law of cosines for elbow angle D
D = (r^2 + s^2 - params.l2^2 - params.l3^2)/(2*params.l2*params.l3);
q3_sol = atan2(+sqrt(1 - D^2), D);    % elbow-down
% q3_alt = atan2(-sqrt(1 - D^2), D); % elbow-up (optional)

% 4) Shoulder angle
theta = atan2(s, r);
phi   = atan2(params.l3*sin(q3_sol), params.l2 + params.l3*cos(q3_sol));
q2_sol = theta - phi;

% 5) Wrist joint does not affect position
q4_sol = sym(0);

disp('Inverse Kinematics Solutions:');
pretty([q1_sol; q2_sol; q3_sol; q4_sol])

%% 8) Static Torque under External Force
syms Fx Fy Fz real
F_ext     = [Fx; Fy; Fz];
tau_static = simplify(J_sym.' * F_ext);

disp('Static joint torques tau = J^T * F_ext:'); pretty(tau_static)

%% 9) Dynamics via Lagrangian Method
q    = [q1; q2; q3; q4];
qd   = sym('qd',  [4,1]);
qdd  = sym('qdd', [4,1]);
M    = diag([m1, m2, m3, m4]);
C    = sym(zeros(4));
G    = [0; -g*(m2*(l2/2) + m3*(l2 + l3/2) + m4*(l2 + l3));
            -g*(m3*(l3/2) + m4*l3); 0];
tau_dyn = simplify(M*qdd + C*qd + G);

disp('Dynamic torque tau_dyn = M qdd + C qd + G:'); pretty(tau_dyn)

%% 10) Computed-Torque PD Control Law
qd_des     = sym('qd_des',    [4,1]);
qd_dot_des = sym('qd_dot_des',[4,1]);
qd_ddot_des= sym('qd_ddot_des',[4,1]);
error      = qd_des - [q1; q2; q3; q4];
error_dot  = qd_dot_des - qd;
Kp = diag([50,50,50,10]);
Kd = diag([ 5, 5, 5, 1]);
tau_pd = simplify(M*qd_ddot_des + C*qd_dot_des + G + Kp*error + Kd*error_dot);

disp('Computed-Torque PD Control tau_pd:'); pretty(tau_pd)