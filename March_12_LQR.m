%Rishab Sareen - 20505101
%Conrad Montor - 20460296
%Adam Sequeira - 20511490

clear all
close all

A = [0 1 0 0;
    0 0 1 0;
    -3 1 2 3;
    2 1 0 0];

B = [0 0;
    0 0;
    1 2;
    0 2];

C = [1 0 0 0;
    0 0 0 1];

D = 0;

%% First choice for Q and R
Q = [1 0 0 0
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
    
R = [1 0
    0 1];

[K,P,ev] = lqr(A,B,Q,R);

sys = ss(A-B*K,B,C,D);

t = linspace(0,10,100);
u = zeros(max(size(t)),2);

[y,t,x] = lsim(sys,u,t,[1 1 1 1]);

figure(1)
subplot(6,1,1);
plot(x(:,1));
title('x(1)');
subplot(6,1,2);
plot(x(:,2));
title('x(2)');
subplot(6,1,3);
plot(x(:,3));
title('x(3)');
subplot(6,1,4);
plot(x(:,4));
title('x(4)');
u = -K*(transpose(x));
u = transpose(u);
subplot(6,1,5);
plot(u(:,1));
title('u(1)');
subplot(6,1,6);
plot(u(:,2));
title('u(2)');

%% Second choice for Q and R
Q = [1 0 0 0
    0 1 0 0;
    0 0 1 0;
    0 0 0 25];
    
R = [10 0
    0 1];

[K,P,ev] = lqr(A,B,Q,R);

sys = ss(A-B*K,B,C,D);

t = linspace(0,10,100);
u = zeros(max(size(t)),2);

[y,t,x] = lsim(sys,u,t,[1 1 1 1]);

figure(2)
subplot(6,1,1);
plot(x(:,1));
title('x(1)');
subplot(6,1,2);
plot(x(:,2));
title('x(2)');
subplot(6,1,3);
plot(x(:,3));
title('x(3)');
subplot(6,1,4);
plot(x(:,4));
title('x(4)');
u = -K*(transpose(x));
u = transpose(u);
subplot(6,1,5);
plot(u(:,1));
title('u(1)');
subplot(6,1,6);
plot(u(:,2));
title('u(2)');

%% Third choice for Q and R
Q = [10 0 0 0
    0 5 0 0;
    0 0 1 0;
    0 0 0 25];
    
R = [10 0
    0 25];

[K,P,ev] = lqr(A,B,Q,R);

sys = ss(A-B*K,B,C,D);

t = linspace(0,10,100);
u = zeros(max(size(t)),2);

[y,t,x] = lsim(sys,u,t,[1 1 1 1]);

figure(3)
subplot(6,1,1);
plot(x(:,1));
title('x(1)');
subplot(6,1,2);
plot(x(:,2));
title('x(2)');
subplot(6,1,3);
plot(x(:,3));
title('x(3)');
subplot(6,1,4);
plot(x(:,4));
title('x(4)');
u = -K*(transpose(x));
u = transpose(u);
subplot(6,1,5);
plot(u(:,1));
title('u(1)');
subplot(6,1,6);
plot(u(:,2));
title('u(2)');
