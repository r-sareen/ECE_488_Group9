%% Solve for state space form
%
% syms q_1 q_1_d q_1_dd q_2 q_2_d q_2_dd t_1 t_2 eq_1 eq_2;
% 
% syms m_1 l_1 m_2 l_2 c_1 c_2 g;
% 
% eq_1 = ((m_1*l_1^2)/3 + (m_2*l_2^2)/12 + m_2*(l_1^2 + (l_2^2)/4 + l_1*l_2*cos(q_2)))*q_1_dd ...
%     +  ((m_2*l_2^2)/3 + (m_2*l_1*l_2/2)*cos(q_2))*q_2_dd ...
%     -  (m_2*l_1*l_2*sin(q_2)*q_1_d*q_2_d) - ((m_2*l_1*l_2*sin(q_2))/2)*q_2_d^2 ...
%     +  (m_1*l_1/2 + m_2*l_1)*g*cos(q_1) + (m_2*l_2/2)*g*cos(q_1+q_2) + c_1*q_1_d ...
%     - t_1;
% 
% eq_2 = ((m_2*l_2^2)/3 + (m_2*l_1*l_2/2)*cos(q_2))*q_1_dd + ((m_2*l_2^2)/3)*q_2_dd ...
%     +  ((m_2*l_1*l_2*sin(q_2))/2)*q_1_d^2 + (m_2*l_2/2)*g*cos(q_1 + q_2) + c_2*q_2_d ...
%     -  t_2;
% 
% [s_1, s_2] = solve (eq_1, eq_2, q_1_dd, q_2_dd);
% 
% 
% xdot = [q_1_d;
%         s_1;
%         q_2_d;
%         s_2];
    
clear all 
close all

m_1 = 1;
m_2 = 1;
l_1 = 1;
l_2 = 1;
c_1 = 1;
c_2 = 1;
g = 9.81;

[t,x] = ode45(@(t,x) non_linear_robot(t,x,m_1,m_2,l_1,l_2,g,c_1,c_2), [0 50], [0 0 0 0]');
plot(x(:,1));

plot(x(:,2));