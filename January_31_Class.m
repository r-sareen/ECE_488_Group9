%Rishab Sareen - 20505101
%Conrad Montor - 20460296
%Adam Sequeira - 20511490

clear all
close all

%% Step 2
s = tf('s');
b = 0.1;
a = 3+b;
K_d = 3 + a*b;
K_p = 1;

P = 1/(s*(s-b));
C = (K_d*s + K_p)/(s+a);
figure(1);
step(feedback(P*C, 1));


b = 0.5;
a = 3+b;
K_d = 3 + a*b;

P = 1/(s*(s-b));
C = (K_d*s + K_p)/(s+a);
hold on
step(feedback(P*C, 1));


b = 1;
a = 3+b;
K_d = 3 + a*b;

P = 1/(s*(s-b));
C = (K_d*s + K_p)/(s+a);
hold on
step(feedback(P*C, 1));


%% Step 3
[A,B,C,D] = ssdata(P);

K = place(A,B,[-1 -2]);

F = place(A',C',[-5 -3]);
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

hold on
step(feedback(C*P,1));
legend('b=0.1', 'b=0.5', 'b=1', 'Youla');