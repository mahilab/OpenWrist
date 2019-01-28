% Evaluate necessar items
A_eval = eval(eval(A));
B_eval = eval(eval(B));

%% Controller Design

states = {'q1' 'q2' 'q1d' 'q2d'};
inputs = {'tau1';};
outputs = {'q1'; 'q2'};

sys_ss = ss(A_eval,B_eval,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

co = ctrb(sys_ss);
ob = obsv(sys_ss);
controllability = rank(co);
observability = rank(ob);

poles = eig(A_eval); % should have a positive pole (i.e. unstable)

% LQR
LQR_Q = C'*C;
LQR_R = 1;
K = lqr(A_eval,B_eval,LQR_Q,LQR_R);

Ac = [(A_eval-B_eval*K)];
Bc = [B_eval];
Cc = [C];
Dc = [D];

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

% Simulte System
t = 0:0.001:10;
r = 0.0*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t,[0 0.2 0 0],'zoh');
plot(t,rad2deg(y(:,1)),t,rad2deg(y(:,2)));
xlabel('t')
ylabel('deg')
legend('q1','q2')