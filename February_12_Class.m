clear all
close all

A = [0 1;
    -2 -2];

B = [1;
    0];

Con = ctrb(A,B);


if (rank(Con) == rank(A))
    K = place(A,B,[-1 -1.0001]);
end

