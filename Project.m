clear all 
close all

%% Define constants
m_1_real = 1;
m_2_real = 1;
l_1_real = 1;
l_2_real = 1;
c_1_real = 1;
c_2_real = 1;
g_real = 9.81;

%% Solve for state space form

syms q_1 q_1_d q_1_dd q_2 q_2_d q_2_dd t_1 t_2 eq_1 eq_2;

syms m_1 l_1 m_2 l_2 c_1 c_2 g;

eq_1 = ((m_1*l_1^2)/3 + (m_2*l_2^2)/12 + m_2*(l_1^2 + (l_2^2)/4 + l_1*l_2*cos(q_2)))*q_1_dd ...
    +  ((m_2*l_2^2)/3 + (m_2*l_1*l_2/2)*cos(q_2))*q_2_dd ...
    -  (m_2*l_1*l_2*sin(q_2)*q_1_d*q_2_d) - ((m_2*l_1*l_2*sin(q_2))/2)*q_2_d^2 ...
    +  (m_1*l_1/2 + m_2*l_1)*g*cos(q_1) + (m_2*l_2/2)*g*cos(q_1+q_2) + c_1*q_1_d ...
    - t_1;

eq_2 = ((m_2*l_2^2)/3 + (m_2*l_1*l_2/2)*cos(q_2))*q_1_dd + ((m_2*l_2^2)/3)*q_2_dd ...
    +  ((m_2*l_1*l_2*sin(q_2))/2)*q_1_d^2 + (m_2*l_2/2)*g*cos(q_1 + q_2) + c_2*q_2_d ...
    -  t_2;

[s_1, s_2] = solve (eq_1, eq_2, q_1_dd, q_2_dd);


xdot = [q_1_d;
        s_1;
        q_2_d;
        s_2];
    
%% Linearization Code

%Operating Point:
q_1_bar = pi/2;
q_1_d_bar = 0;
q_2_bar = 0;
q_2_d_bar = 0;

t_1_bar = ((m_1_real*l_1_real)/2 + m_2_real*l_1_real)*g_real*cos(q_1_bar) ...
        +  (m_2_real*l_2_real/2)*g_real*cos(q_1_bar + q_2_bar);
t_2_bar = (m_2_real*l_2_real/2)*g_real*cos(q_1_bar + q_2_bar);

A = jacobian(xdot, [q_1 q_1_d q_2 q_2_d]);
A = subs(A, q_1, q_1_bar);
A = subs(A, q_2, q_2_bar);
A = subs(A, q_1_d, q_1_d_bar);
A = subs(A, q_2_d, q_2_d_bar);
A = subs(A, m_1, m_1_real);
A = subs(A, m_2, m_2_real);
A = subs(A, l_1, l_1_real);
A = subs(A, l_2, l_2_real);
A = subs(A, c_1, c_1_real);
A = subs(A, c_2, c_2_real);
A = subs(A, g, g_real);
A = double(A);

B = jacobian(xdot, [t_1 t_2]);
B = subs(B, q_2, q_2_bar);
B = subs(B, t_1, t_1_bar);
B = subs(B, t_2, t_2_bar);
B = subs(B, m_1, m_1_real);
B = subs(B, m_2, m_2_real);
B = subs(B, l_1, l_1_real);
B = subs(B, l_2, l_2_real);
B = subs(B, c_1, c_1_real);
B = subs(B, c_2, c_2_real);
B = subs(B, g, g_real);
B = double(B);

C = [1 0 1 0];

m_1 = 1;
m_2 = 1;
l_1 = 1;
l_2 = 1;
c_1 = 1;
c_2 = 1;
g = 9.81;

%% Closed Loop PD Controller
%Linear Controller
Kp_1 = -100;
Kd_1 = -10;
Kp_2 = -100;
Kd_2 = -10;
K = [Kp_1 Kd_1 0 0; 
     0 0 Kp_2 Kd_2];
x_des = [0; 0; 0; 0]; 
[t,x] = ode45(@(t,x) linear_robot(t,x,A,B,K,x_des), [0 15], [0.1 0 0 0]');
figure(1);
plot(x(:,1));
hold on;
plot(x(:,3));
legend('Link 1', 'Link 2');
title('Linear Simulation - PD Controller');

%Non-Linear Controller
x_des = [0; 0; 1; 0];
[t,x] = ode45(@(t,x) non_linear_robot(t,x,x_des,K,m_1_real,m_2_real,l_1_real,l_2_real,g_real,c_1_real,c_2_real), [0 50], [0 0 0 0]');
figure(2);
plot(x(:,1));
hold on;
plot(x(:,3));
legend('Link 1', 'Link 2');
title('Non-Linear Simulation - PD Controller');

%% Open Loop Simulations
% [t,x] = ode45(@(t,x) non_linear_robot(t,x,m_1_real,m_2_real,l_1_real,l_2_real,g_real,c_1_real,c_2_real), [0 50], [0 0 0 0]');
% figure(1);
% plot(x(:,1));
% hold on;
% plot(x(:,2));
% legend('Link 1', 'Link 2');
% title('Non-Linear Simulation');

% [t,x] = ode45(@(t,x) linear_robot(t,x,A,B), [0 50], [0.1 0 0 0]');
% figure(2);
% plot(x(:,1));
% hold on;
% plot(x(:,2));
% legend('Link 1', 'Link 2');
% title('Linear Simulation');

% %% TF of Linear System
% s = tf('s');
% I = [1 0;
%     0 1];
% G = C*(inv(s*I-A))*B;


