clear all
close all

M=10;
B=10;
K=10;

[t,x] = ode45 (@(t,x) mass_spring_damper_system(t,x,M,B,K), [0 10], [0 0]');

plot(t,x(:,1));