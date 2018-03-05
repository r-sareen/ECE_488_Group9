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

A_aug = [A zeros(4,2);
        C zeros(2,2)];

B_aug = [B;
        zeros(2,2)];
    
Q = ctrb(A_aug,B_aug);
R = ctrb(A', C')';

if (rank(R) == size(A))
    F = place(A', C', [-1.5 -3.1 -2.1 -4]);
    F = F';
end
if (rank(Q) == size(A_aug))
    K_aug = place(A_aug, B_aug, [-1 -2 -3 -3.5 -2.5 -1.5]);
    K_1 = K_aug(:,1:4);
    K_2 = K_aug(:,5:6);
    
    A_final = [A -B*K_1 -B*K_2;
        F*C A-F*C-B*K_1 -B*K_2;
        C zeros(2,4) zeros(2,2)];
    
    B_final = [zeros(8,2);
        -1*eye(2,2)];
    
    C_final = [C zeros(2,6)];
    
    sys = ss(A_final,B_final,C_final,D);

    t = linspace(0,10,100);
    u = ones(max(size(t)),2);

    figure(1)
    lsim(sys,u,t);
end