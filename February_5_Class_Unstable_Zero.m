%Rishab Sareen - 20505101
%Conrad Montor - 20460296
%Adam Sequeira - 20511490

clear all
close all

s = tf('s');
P = 1/(s*(s+10));

K_p = 1;
K_d = 2*sqrt(K_p) - 10;


%% Low T_d
T_d = 0.01;
C = K_d*s + K_p;
delay = (1-T_d*(s/2))/(1 + T_d*(s/2));

L = C*delay*P;
S = 1/(1+L);
T = L/(1+L);

% figure(1);
% step(feedback(L,1));
% hold on
% 
% figure(2);
% bode(S);
% hold on
% bode(T);
% legend('S','T');
% title('T_d = 0.01');

%% High T_d
T_d = 1;
C = K_d*s + K_p;
delay = (1 - T_d*(s/2))/(1 + T_d*(s/2));

L = C*delay*P;
S = 1/(1+L);
T = L/(1+L);

% figure(1);
% step(feedback(L,1));
% legend('T_d = 0.01','T_d = 1');
% 
% figure(3);
% bode(S);
% hold on
% bode(T);
% legend('S','T');
% title('T_d = 1');

%% Part 2
T_d = 0.2;
delay = (1 - T_d*(s/2))/(1 + T_d*(s/2));
b = 0.1;
a = 3+b;
K_d = 3 + a*b;
K_p = 1;

P = 1/(s*(s-b));
C = (K_d*s + K_p)/(s+a);
figure(1);
step(feedback(P*delay*C, 1));


b = 0.5;
a = 3+b;
K_d = 3 + a*b;

P = 1/(s*(s-b));
C = (K_d*s + K_p)/(s+a);
hold on
step(feedback(P*delay*C, 1));


b = 1;
a = 3+b;
K_d = 3 + a*b;

P = 1/(s*(s-b));
C = (K_d*s + K_p)/(s+a);
hold on
step(feedback(P*delay*C, 1));

legend('b=0.1', 'b=0.5', 'b=1');
