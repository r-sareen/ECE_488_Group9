clear all 
close all

A = [0 1 0;
    0 0 1;
    -1 -1 -1];

B = [0;
    0;
    1];

T = [1 0 1;
     0 1 2;
     1 0 3]

A_ = T*A*inv(T);

B_ = T*B;

K = place(A_,B_,[-2 -1.9999 -2.001]);