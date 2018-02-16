%Rishab Sareen - 20505101
%Conrad Montor - 20460296
%Adam Sequeira - 20511490

clear all
close all

s = tf('s');
P = [1/(s*(s+1)) (1/(s*(s+10)));
    1/(s*(s+10)) (1/(s*(s+1)))];
I = [1 0;
     0 1];
T = [100/99 -10/99;
    -10/99 100/99];
 
%% Poles @ -0.5
K_p = 0.25;
K_d = 0;
C = [K_p+K_d*(s/(0.01*s+1)) 0;
     0 K_p+K_d*(s/(0.01*s+1))];
figure(1)
step(feedback(P*C,I));
title('Step Response for Poles @ -0.5');
hold on
step(feedback(P*T*C,I));
legend('No Coupling', 'With Coupling');
 
%% Poles @ -1
K_p = 1;
K_d = 1;
C = [K_p+K_d*(s/(0.01*s+1)) 0;
     0 K_p+K_d*(s/(0.01*s+1))];
figure(2)
step(feedback(P*C,I));
title('Step Response for Poles @ -1');
hold on
step(feedback(P*T*C,I));
legend('No Coupling', 'With Coupling');

 
%% Poles @ -5
K_p = 25;
K_d = 9;
C = [K_p+K_d*(s/(0.01*s+1)) 0;
     0 K_p+K_d*(s/(0.01*s+1))];
figure(3)
step(feedback(P*C,I));
title('Step Response for Poles @ -5');
hold on
step(feedback(P*T*C,I));
legend('No Coupling', 'With Coupling');


