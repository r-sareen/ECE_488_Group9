clear all
close all

s = tf('s');
K_d = 1;
K_p = 100;


P = 1/(s*(s+0.1));
C = K_d*s + K_p;

S = 1/(1+P*C);
T = (P*C)/(1 + P*C);

figure(1);
bode(S);
figure(2);
bode(T);

G = (P*C)/(1+P*C);

dt = 0.01;
time = 10;
u = zeros(900,1);
t_input = zeros(900,1);

n=1;
for t=1:dt:time
    u(n) = sin((sqrt(K_p))*t);
    t_input(n) = t;
    n=n+1;
end

figure(3);
subplot(3,1,1);
lsim(G,u,t_input); 

n=1;
for t=1:dt:time
    u(n) = sin(t);
    t_input(n) = t;
    n=n+1;
end

subplot(3,1,2);
lsim(G,u,t_input); 

n=1;
for t=1:dt:time
    u(n) = sin(1.5*(sqrt(K_p))*t);
    t_input(n) = t;
    n=n+1;
end

subplot(3,1,3);
lsim(G,u,t_input); 