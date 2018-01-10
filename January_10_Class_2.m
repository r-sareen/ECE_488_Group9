%Rishab Sareen - 20505101
%Conrad Montor - 20460296
%Adam Sequeira - 20511490

%Design with gain of 1000

clear all
close all

s = tf('s');
P = 1/((s+1)*(s+5)*(s+10));

k = 1000;
t_2 = k*0.5/50;

%% Iteration 1
[Gm,Pm,Wgm,Wpm] = margin(k*P);
figure(1);
bode(k*P);

alpha = (1+sin(deg2rad(50-Pm)))/(1-sin(deg2rad(50-Pm)));
gain_c = 10*log(alpha);
% Look at plot to find w_c:
w_c = 1.6;

%% Calculate Controller
t_1 = 1/(sqrt(alpha)*w_c);

z_1 = 1/(alpha*t_1);
t_d = t_1;
z_2 = alpha/t_2;

C = k*((s+z_1)*(s+z_2))/(s*(s+1)/t_d);
figure(2);
bode(C*P);
[Gm_comp,Pm_comp,Wgm_comp,Wpm_comp] = margin(C*P);
