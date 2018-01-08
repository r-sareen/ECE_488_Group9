clear all
close all

s = tf('s');
k_p = 10;
k_d = 1;

P = [10/((s+1)*s) 1/(s*(s+100)); 1/(s*(s+100)) 10/((s+1)*s)];
C = k_p+k_d*s;



% G = [feedback(P(1,1)*C,1) P(1,2);
%     P(2,1) feedback(P(2,2)*C,1)];




