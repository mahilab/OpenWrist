% Evan Pezent | evanpezent.com | epezent@rice.edu
% 02/04/2017

% =========================================================================
% This script computes the OpenWrist dynamic equations symbolically using
% the Newton-Euler approach, and rearranges all terms in the form:
% Tau = M(Q)Q" + V(Q,Q') + G(Q) + B.*Q' + Fk.*sign(Q')
%
% Requires adding functions from Evan's CraigRobotics Toolbox to PATH:
% https://github.com/epezent/CraigRobotics
% =========================================================================

%% Define Symbolic Symbols
syms tau1 eta1 Jm1 q1 q1d q1dd m1 b1 fk1 ...
     tau2 eta2 Jm2 q2 q2d q2dd m2 b2 fk2 ...
     tau3 eta3 Jm3 q3 q3d q3dd m3 b3 fk3 ...
     Pc1x Pc1y Pc1z   Ic1xx Ic1xy Ic1xz Ic1yy Ic1yz Ic1zz ...
     Pc2x Pc2y Pc2z   Ic2xx Ic2xy Ic2xz Ic2yy Ic2yz Ic2zz ...
     Pc3x Pc3y Pc3z   Ic3xx Ic3xy Ic3xz Ic3yy Ic3yz Ic3zz ...
     g

Tau = [tau1; tau2; tau3];

Eta = [eta1;eta2;eta3];
Jm = [Jm1;Jm2;Jm3];

Q = [q1;q2;q3];
Qd = [q1d;q2d;q3d];
Qdd = [q1dd;q2dd;q3dd];

B = [b1;b2;b3];
Fk = [fk1;fk2;fk3];

Pc1 = [Pc1x Pc1y Pc1z].';
Pc2 = [Pc2x Pc2y Pc2z].';
Pc3 = [Pc3x Pc3y Pc3z].';

Ic1 = [Ic1xx -Ic1xy -Ic1xz;
    -Ic1xy Ic1yy -Ic1yz;
    -Ic1xz -Ic1yz Ic1xx];

Ic2 = [Ic2xx -Ic2xy -Ic2xz;
    -Ic2xy Ic2yy -Ic2yz;
    -Ic2xz -Ic2yz Ic2xx];

Ic3 = [Ic3xx -Ic3xy -Ic3xz;
    -Ic3xy Ic3yy -Ic3yz;
    -Ic3xz -Ic3yz Ic3xx];

%% Forward Kinematics
DH_table = [0 0 0 q1;
    0 pi/2 0 q2-pi/2;
    0 pi/2 0 q3];

[~,T_array] = dh2tf(DH_table);

%% Newton-Euler Dynamics
m = [m1;m2;m3];
Pc = {Pc1 Pc2 Pc3};
Ic = {Ic1 Ic2 Ic3};
g0 = [0; g; 0];
MVG = dynamics_newtonian(m,Pc,Ic,T_array,Qd,Qdd,g0);
MVG = simplify(expand(MVG));

%% Separate MVG into M, V, and G
[M,V,G] = separate_mvg(MVG,Qdd,g);

%% Get Equation of Motion
EOM = Tau == M*Qdd + Jm.*Eta.^2.*Qdd + V + G + B.*Qd + Fk.*tanh(10 * Qd);

%% Solved for acclerations ( i.e inv(M) * (Tau - V - G) )
Qdd_solved = inv(MM + diag(Jm.*Eta.^2)) * (Tau - V - G - B.*Qd - Fk.*tanh(10 * Qd));

%% Numerical Evaluation
openWrist = OpenWristInit();

m1 = openWrist.PS.m;
m2 = openWrist.FE.m;
m3 = openWrist.RU.m;

eta1 = openWrist.PS.eta;
eta2 = openWrist.FE.eta;
eta3 = openWrist.RU.eta;

Jm1 = openWrist.PS.motor.J;
Jm2 = openWrist.FE.motor.J;
Jm3 = openWrist.RU.motor.J;

b1 = openWrist.PS.B;
b2 = openWrist.FE.B;
b3 = openWrist.RU.B;

fk1 = openWrist.PS.fk;
fk2 = openWrist.FE.fk;
fk3 = openWrist.RU.fk;

Pc1x = openWrist.PS.Xc;
Pc1y = openWrist.PS.Yc;
Pc1z = openWrist.PS.Zc;

Pc2x = openWrist.FE.Xc;
Pc2y = openWrist.FE.Yc;
Pc2z = openWrist.FE.Zc;

Pc3x = openWrist.RU.Xc;
Pc3y = openWrist.RU.Yc;
Pc3z = openWrist.RU.Zc;

Ic1xx = openWrist.PS.Icxx;
Ic1xy = openWrist.PS.Icxy;
Ic1xz = openWrist.PS.Icxz;
Ic1yy = openWrist.PS.Icyy;
Ic1yz = openWrist.PS.Icyz;
Ic1zz = openWrist.PS.Iczz;

Ic2xx = openWrist.FE.Icxx;
Ic2xy = openWrist.FE.Icxy;
Ic2xz = openWrist.FE.Icxz;
Ic2yy = openWrist.FE.Icyy;
Ic2yz = openWrist.FE.Icyz;
Ic2zz = openWrist.FE.Iczz;

Ic3xx = openWrist.RU.Icxx;
Ic3xy = openWrist.RU.Icxy;
Ic3xz = openWrist.RU.Icxz;
Ic3yy = openWrist.RU.Icyy;
Ic3yz = openWrist.RU.Icyz;
Ic3zz = openWrist.RU.Iczz;

g = openWrist.g;

M_num = simplify(expand(eval(M)));
V_num = simplify(expand(eval(V)));
G_num = simplify(expand(eval(G)));
B_num = eval(B);
Fk_num = eval(Fk);
Jm_num = eval(Jm);
Eta_num = eval(Eta);

%% Numerical Model
EOM_num = M_num*Qdd + Jm_num.*Eta_num.^2.*Qdd + V_num + G_num + B_num.*Qd + Fk_num.*tanh(10*Qd) - Tau;

%% Solve q1dd
LHS1 = simplify(expand(solve(EOM_num(1) == 0, q2dd)));
MHS1 = simplify(expand(solve(EOM_num(2) == 0, q2dd)));
RHS1 = simplify(expand(solve(EOM_num(3) == 0, q2dd)));
LHS2 = (solve(LHS1==MHS1,q3dd));
RHS2 = (solve(MHS1==RHS1,q3dd));
q1dd_solved = solve(LHS2==RHS2,q1dd);
%% Solve q2dd
LHS1 = simplify(expand(solve(EOM_num(1) == 0, q3dd)));
MHS1 = simplify(expand(solve(EOM_num(2) == 0, q3dd)));
RHS1 = simplify(expand(solve(EOM_num(3) == 0, q3dd)));
LHS2 = (solve(LHS1==MHS1,q1dd));
RHS2 = (solve(MHS1==RHS1,q1dd));
q2dd_solved = solve(LHS2==RHS2,q2dd);
%% Solve q3dd
LHS1 = simplify(expand(solve(EOM_num(1) == 0, q1dd)));
MHS1 = simplify(expand(solve(EOM_num(2) == 0, q1dd)));
RHS1 = simplify(expand(solve(EOM_num(3) == 0, q1dd)));
LHS2 = (solve(LHS1==MHS1,q2dd));
RHS2 = (solve(MHS1==RHS1,q2dd));
q3dd_solved = solve(LHS2==RHS2,q3dd);
