clear all
close all

syms th thd thdd T eq;

eq = thdd + thd + sin(th) - T;

xdot = [thd;
        solve(eq , thdd)];
    
y = th;

A = jacobian(xdot, [th thd]);

A = subs(A, th, 0);
A = double(A);

B = jacobian(xdot, T);

B = subs(B, T, 0);
B = double(B);

[t,x] = ode45(@(t,x) linear_pendulum(t,x,A,B), [0 25], [0 0]');

figure(1);
title("Linearized System");
plot(x(:,1));



