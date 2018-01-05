clear all
close all

P = tf(10, [1 10 0]);


%% Critically Damped
k_p = 10;
k_d = 1;
s = tf('s');
C = k_p+k_d*s;

G = feedback(P*C,1);
figure
step(G);
figure
bode(G);


%% Underdamped
k_p = 15;
k_d = 1;
s = tf('s');
C = k_p+k_d*s;

G = feedback(P*C,1);
figure
step(G);
figure
bode(G);

%% Overdamped
k_p = 5;
k_d = 1;
s = tf('s');
C = k_p+k_d*s;

G = feedback(P*C,1);
figure
step(G);
figure
bode(G);
