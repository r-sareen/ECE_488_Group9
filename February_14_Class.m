%Rishab Sareen - 20505101
%Conrad Montor - 20460296

clear all
close all


B_prime = [1 1 1]';

A_prime = [-2    -1     1;
      0    -1     0;
     -2     0     0];
       
Q_prime = ctrb(A_prime,B_prime);

if (rank(Q_prime) == size(A_prime))
    poly = charpoly(A_prime);
    A = [0 1 0;
        0 0 1;
        poly(3) poly(2) poly(1)];
    B = [0 0 1]';
    K = place(A,B,[-2 -2.0001 -1.99999]);
end

step(ss(A-B*K,B,[1 1 0],0));