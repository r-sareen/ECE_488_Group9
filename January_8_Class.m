%Rishab Sareen - 20505101
%Conrad Montor - 20460296
%Adam Sequeira - 20511490

clear all;
close all;

s = tf('s');
% G = 10/(s*(s+2));
% nyquist1(G);

G = 2/(s^4 + 3*s^3 + 6*s^2 + s);
k = linspace(0.9,1.1,1000);
rlocus(G,k);