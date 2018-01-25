%Rishab Sareen - 20505101
%Conrad Montor - 20460296
%Adam Sequeira - 20511490

clear all
close all

s = tf('s');
%% Define Plant
G = s/(s+1);

%% i)
D = 0.9;

figure(1);
nyquist(G*D);
figure(2);
step(feedback(G,D));

%% ii)
a = 0.1;
D = (0.9*a)/(s+a);

figure(3);
nyquist(G*D);
figure(4);
step(feedback(G,D));

a = 1;
figure(5);
nyquist(G*D);
figure(6);
step(feedback(G,D));

a = 10;
figure(7);
nyquist(G*D);
figure(8);
step(feedback(G,D));

%% iii)
T = 0.1;
D = 0.9*exp(-s*T);

figure(9);
nyquist(G*D);
figure(10);
step(feedback(G,D));

T = 1;
figure(11);
nyquist(G*D);
figure(12);
step(feedback(G,D));


