%Rishab Sareen - 20505101
%Conrad Montor - 20460296
%Adam Sequeira - 20511490

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

D = [0 0;
    0 0];

Q = ctrb(A,B);
R = ctrb(A', C')';

if (rank(Q) == size(A))
    K = place(A, B, [-1 -2 -3 -3.5]);
end
if (rank(R) == size(A))
    F = place(A', C', [-1.5 -3.1 -2.1 -4]);
    F = F';
end

%% Simulate only state feedback

sys = ss(A-B*K,B,C,D);

t = linspace(0,10,100);
u = ones(max(size(t)),2);

figure(1)
lsim(sys,u,t);
title('Only state feedback');


%% Simulate combined system
A = [ A -B*K;
       F*C  A-F*C-B*K];

B = [B;
    B];

C = [C -D*K];
   
sys = ss(A,B,C,D);

t = linspace(0,10,100);
u = ones(max(size(t)),2);

figure(2)
lsim(sys,u,t);
title('Combined system');

figure(3)
lsim(sys,u,t,[0 0 0 0 0.1 0.1 0.1 0.1]');
title('Combined system with initial estimate error');


