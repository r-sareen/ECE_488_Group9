%Rishab Sareen - 20505101
%Conrad Montor - 20460296
%Adam Sequeira - 20511490

clear all
close all

A_11 = [0 1;
        0 -1];
A_12 = [0 1;
        0 -10];
A_22 = A_11;
A_21 = A_12;

A = blkdiag(A_11,A_21,A_12,A_22);

B = [0 0;
    1 0;
    0 0;
    1 0;
    0 0;
    0 1;
    0 0;
    0 1];

C = [1 0 0 0 1 0 0 0;
    0 0 1 0 0 0 1 0];

D = [0 0;
    0 0];

% Controllability
Q = ctrb(A,B);

T_1 = orth(Q);

T_2 = [1 0;
       0 1;
       0 0;
       0 0;
       0 0;
       0 1;
       0 0;
       0 0];

T = [T_1 T_2];

A_ = inv(T)*A*T;

B_ = inv(T)*B;

C_ = C*T;

A_cont = A_(1:rank(Q), 1:rank(Q));

B_cont = B_(1:rank(Q),1:2);

C_cont = C_(1:2,1:rank(Q));

%Observability
R = ctrb(A_cont', C_cont')';

%We are already observable since orth(R) is 6x6 and rank(R) = 6
%Therefore the minimal realization is (A_cont, B_cont, C_cont, D)







         
           


