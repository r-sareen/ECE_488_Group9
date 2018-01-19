%Rishab Sareen - 20505101
%Conrad Montor - 20460296
%Adam Sequeira - 20511490

clear all
close all

A = [-2 1;
     0 -2];

B = [0 1]';

C = [1 1];

x = [0 0]';

dt = 0.1;
n = 1;
u = 1;
time = 10;
y_hist_approx = zeros(1,time/dt - 1/dt);
y_hist_exact = zeros(1,time/dt - 1/dt);
for t = 1:dt:time
    K_1 = A*x + B*u;
    
    x_2 = x + (1/2)*dt*K_1;
    K_2 = A*x_2 + B*u;
    
    x_3 = x + (1/2)*dt*K_2;
    K_3 = A*x_3 + B*u;
    
    x_4 = x + dt*K_3;
    K_4 = A*x_4 + B*u;
    
    x = x+(dt/6)*(K_1 + 2*K_2 + 2*K_3 + K_4);
    y_hist_approx(n) = C*x;
    y_hist_exact(n) = (3/4) - (2*t)*exp(-2*t) - (3/4)*(exp(-2*t)); 
    n = n+1;
end

plot(y_hist_approx);
hold on 
plot(y_hist_exact);
legend('Numerical', 'Analytical');