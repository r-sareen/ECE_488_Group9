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

B = subs(B, T, 2);
B = double(B);

[t,x] = ode45(@(t,x) linear_pendulum(t,x,A,B), [0 50], [0 0]');

figure(1);
title("Linearized System");
plot(x(:,1));


[t,x] = ode45(@(t,x) non_linear_pendulum(t,x), [0 50], [0 0]');
figure(2);
title("Non-Linearized System");
plot(x(:,1));

