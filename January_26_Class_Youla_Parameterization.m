%Rishab Sareen - 20505101
%Conrad Montor - 20460296

clear all
close all

s = tf('s');
P = 100/((s+1)*(s-2)*(s+4));

[A,B,C,D] = ssdata(P);

K = place(A,B,[-4 -2 -3]);

F = place(A',C',[-1 -6 -5]);
F = F';

[num, den] = ss2tf(A-F*C, -1*F, -1*K, 0, 1);
x_p = tf(num, den);
[num, den] = ss2tf(A-B*K, B, C-D*K, D, 1);
n_p = tf(num, den);
[num, den] = ss2tf(A-F*C, -B+F*D, -K, 1, 1);
y_p = tf(num, den);
[num, den] = ss2tf(A-B*K, B, -K, 1, 1);
d_p = tf(num, den);

r = 1/(s+1);
C = (x_p + r*d_p)/(y_p - r*n_p);

step(feedback(C*P,1));



