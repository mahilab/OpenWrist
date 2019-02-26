clear all
close all
clc

%% System Definitions

syms m1 m2 l1 l2 c1 c2 b1 b2 fk1 fk2 tau1 tau2 q1 q2 q1d q2d q1dd q2dd g
syms Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2

% joint torques
Tau = [tau1; tau2];

% mass and inertia
m = [m1;m2];
Ic = {diag([Ixx1 Iyy1 Ixx1]) diag([Ixx2 Iyy2 Ixx2])};
% Ic = {zeros(3) zeros(3)};

% centers of mass
Pc1 = [0 c1 0].';
Pc2 = [0 c2 0].';
Pc = {Pc1 Pc2};

% gravity
g0 = [0;0;-g];

% state variables
Q = [q1;q2];
Qd = [q1d;q2d];
Qdd = [q1dd;q2dd];

B = [b1;b2];
Fk = [fk1;fk2];

DH_table = [0 0 0 q1; 0 -pi/2 l1 q2 + pi];
[T02, Ti, T0i] = dh2tf(DH_table);

%% EOMs (Newtonian)
[MVG_newton,w,~,~,~,~,~,~,~] = dynamics_newtonian(m,Pc,Ic,Ti,Qd,Qdd,g0);
MVG_newton = simplify(expand(MVG_newton));

%% EOMs (Lagrangian)
[MVG_lagrange, L, K_tot, U_tot, k, u] = dynamics_lagrangian(m,Pc,Ic,Ti,Q,Qd,Qdd,g0,0);

%% Separate MVG into M, V, and G
[M,V,G] = separate_mvg(MVG_lagrange,Qdd,g);

%% Create EOMS with and without damping
EOM = M*Qdd + V + G + B.*Qd;

%% Solve q1dd and q2dd
Qdd_sol = simplify( inv(M)*(Tau - V - G - B.*Qd) );

%% System Properties
syms rho r_link r_mass
m1        = pi*r_link^2*l1*rho;
m2_link   = pi*r_link^2*l2*rho;
m2_mass   = 4/3*pi*r_mass^3*rho;
m2        = m2_link + m2_mass;
c1        = l1/2;
c2_link   = l2/2;
c2_mass   = l2 + r_mass;
c2        = (m2_link * c2_link + m2_mass * (c2_mass)) / m2;
Ixx1      = 1/12*m1*(3*r_link^2+l1^2);
Iyy1      = 1/2*m1*r_link^2;
Izz1      = Ixx1;
Ixx2_link = 1/12*m2_link*(3*r_link^2+l2^2);
Ixx2_mass = 2/5*m2_mass*r_mass^2;
Ixx2      = (Ixx2_link + m2_link*(c2_link - c2)^2) + (Ixx2_mass + m2_mass*(c2_mass - c2)^2);
Iyy2      = 1/2*m2_link*r_link^2 + 2/5*m2_mass*r_mass^2;
Izz2      = Ixx2;

%% Linearize / Create State Space
X = [q1 q2 q1d q2d];
lin_point = [0 0 0 0];

A34 = simplify(subs(jacobian(Qdd_sol,X),X,lin_point));
B34 = simplify(subs(jacobian(Qdd_sol,tau1),X,lin_point));

A = [0   0   1   0;
     0   0   0   1;
     A34];

B = [0 ; 0; B34];

C = [1 0 0 0; 0 1 0 0];
D = 0;

%% Define values
rho      = 10;
r_link = .025;
r_mass = .1;
l1 = 1;
l2 = 1;
g = 9.81;
b1 = 0.01;
b2 = 0.01;