%Rishab Sareen - 20505101
%Conrad Montor - 20460296

clear all
close all

A = [0 1 0 0;
    0 0 1 0;
    -3 1 2 3;
    2 1 0 0];

B = [0 0;
    0 0;
    1 2;
    0 2];

C = [1 0 0 0;
    0 0 0 1];

D = 0;

X_0 = [1 1 1 1]';

Q = ctrb(A,B);
R = ctrb(A', C')';

if (rank(Q) == size(A))
    K = place(A, B, [-4, -4.2, -4.4, -4.6]);
end
if (rank(R) == size(A))
    F = place(A', C', [-10, -10.1, -10.2, -10.3]);
    F = F';
end

%% Simulate only state feedback

sys = ss(A-B*K,B,C,D);

t = linspace(0,10,100);
u = ones(max(size(t)),2);

figure(1)
lsim(sys,u,t,X_0);
title('Only state feedback - no noise');


%% Simulate combined system
A_estimator = [ A -B*K;
       F*C  A-F*C-B*K];

B_estimator = [B eye(4) zeros(4,2);
    B  zeros(4,4) F];

C_estimator = [C zeros(2,4)];
   
D_estimator = [zeros(2,6) eye(2)];

sys = ss(A_estimator,B_estimator,C_estimator,D_estimator);

t = linspace(0,10,100);
v = zeros(2,max(size(t)));
n = 0.1*randn(4,100);
w = 0.1*randn(2,100);

u = [v;
    n;
    w];

figure(2)
lsim(sys,u,t,[X_0; 0; 0; 0; 0]);
title('State Estimator - with noise');

%% Kalman Filter Design

Q = [0.01 0 0 0;
    0 0.01 0 0;
    0 0 0.01 0;
    0 0 0 0.01];
    
R = [0.01 0;
    0 0.01];

[F,P,ev] = lqr(A',C',Q,R);

F = F';

A_kalman = [ A -B*K;
       F*C  A-F*C-B*K];

B_kalman = [B eye(4) zeros(4,2);
    B  zeros(4,4) F];

C_kalman = [C zeros(2,4)];
   
D_kalman = [zeros(2,6) eye(2)];

sys = ss(A_kalman,B_kalman,C_kalman,D_kalman);

t = linspace(0,10,100);
v = zeros(2,max(size(t)));
n = 0.1*randn(4,100);
w = 0.1*randn(2,100);

u = [v;
    n;
    w];

figure(3)
lsim(sys,u,t,[X_0; 0; 0; 0; 0]);
title('Kalman Filter - with noise');


