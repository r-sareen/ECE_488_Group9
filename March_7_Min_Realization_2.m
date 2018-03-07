%Rishab Sareen - 20505101
%Conrad Montor - 20460296
%Adam Sequeira - 20511490

clear all
close all

A = [-0.5 -1 -1.5 -2;
      0 1 -2 -2;
     -0.5 -1 0.5 2;
     0.5 1 0.5 -1];
 
B = [0.5;
    1;
    -0.5;
    0.5];

C = [1 3 0 -1];

% Controllability
Q = ctrb(A,B);

T_1 = orth(Q);

T_2 = [0;
       1;
       1;
       0];
   
T = [T_1 T_2];

A_ = inv(T)*A*T;

B_ = inv(T)*B;

C_ = C*T;

A_cont = A_(1:rank(Q), 1:rank(Q));

B_cont = B_(1:rank(Q),:);

C_cont = C_(:,1:rank(Q));

%Observability
R = ctrb(A_cont', C_cont')';

T_1 = orth(R')';
T_2 = [1 0 0];

T = [T_1;
     T_2];
 
A_ = T*A_cont*inv(T);

B_ = T*B_cont;

C_ = C_cont*inv(T);

dim = rank(R);

A_min = A_(1:dim,1:dim);

B_min = B_(1:dim,:);

C_min = C_(:,1:dim);
