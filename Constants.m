%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%You NEED these constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
c1=8;%link 1 friction coeffecient
c2=8;%link 2 friction coeffecient
l1=0.4412; %link 1 length
l2=0.4412; %link 2 length
m1=l1*0.85;%link 1 mass
m2=l2*0.85;%link 2 mass
g=3.7;%acceleration due to gravity m/s^2 on mars
%Calculate initial robot state (Starting at point A)
Y = sqrt(1-((0.1^2 + 0.2^2 - l1^2 - l2^2)/(2*l1*l2))^2);
X = (0.1^2 + 0.2^2 - l1^2 - l2^2)/(2*l1*l2);
q_2_0 = atan2(Y,X);
q_1_0 = atan2(0.2, 0.1) - atan2(l2*sin(q_2_0),l1 + l2*cos(q_2_0));
x_0=[q_1_0,0,q_2_0,0]'; %initial robot state
tau_0=[0,0]'; %initial torque
energy = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Declare all your variables here, prefix with my_ %Feel Free to add to or remove these constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i=1; %counter used in linearization code
j=1; %counter used to step through target points when controlling arm
x_hat_old_d = [0 0 0 0]';
wait_count = 0; %counter used to wait at corner target points
dist_hist_real = zeros(10,000);
dist_hist = zeros(10,000);
target_points=[0.1, 0.2;
               0.11, 0.2; 
               0.12, 0.2;
               0.13, 0.2;
               0.14, 0.2;
               0.15, 0.2;
               0.16, 0.2;
               0.17, 0.2;
               0.18, 0.2;
               0.19, 0.2;
               0.2, 0.2;
               0.2, 0.19;
               0.2, 0.18;
               0.2, 0.17;
               0.2, 0.16;
               0.2, 0.15;
               0.2, 0.14;
               0.2, 0.13;
               0.2, 0.12;
               0.2, 0.11;
               0.2, 0.1;
               0.19, 0.1;
               0.18, 0.1;
               0.17, 0.1;
               0.16, 0.1;
               0.15, 0.1;
               0.14, 0.1;
               0.13, 0.1;
               0.12, 0.1;
               0.11, 0.1;
               0.1, 0.1;
               0.1, 0.11;
               0.1, 0.12;
               0.1, 0.13;
               0.1, 0.14;
               0.1, 0.15;
               0.1, 0.16;
               0.1, 0.17;
               0.1, 0.18;
               0.1, 0.19;
               0.1, 0.2;];
           
%% Symbollically solve for A and B jacobian matrices using given dynamic equations
syms q_1 q_1_d q_1_dd q_2 q_2_d q_2_dd t_1 t_2 eq_1 eq_2;
syms m_1 l_1 m_2 l_2 c_1 c_2 g_;
    
eq_1 = ((m_1*l_1^2)/3 + (m_2*l_2^2)/12 + m_2*(l_1^2 + (l_2^2)/4 + l_1*l_2*cos(q_2)))*q_1_dd ...
+  ((m_2*l_2^2)/3 + (m_2*l_1*l_2/2)*cos(q_2))*q_2_dd ...
-  (m_2*l_1*l_2*sin(q_2)*q_1_d*q_2_d) - ((m_2*l_1*l_2*sin(q_2))/2)*q_2_d^2 ...
+  (m_1*l_1/2 + m_2*l_1)*g_*cos(q_1) + (m_2*l_2/2)*g_*cos(q_1+q_2) + c_1*q_1_d ...
- t_1;

eq_2 = ((m_2*l_2^2)/3 + (m_2*l_1*l_2/2)*cos(q_2))*q_1_dd + ((m_2*l_2^2)/3)*q_2_dd ...
    +  ((m_2*l_1*l_2*sin(q_2))/2)*q_1_d^2 + (m_2*l_2/2)*g_*cos(q_1 + q_2) + c_2*q_2_d ...
    -  t_2;

[s_1, s_2] = solve (eq_1, eq_2, q_1_dd, q_2_dd);


xdot = [q_1_d;
        s_1;
        q_2_d;
        s_2];     
        
A_jac = jacobian(xdot, [q_1 q_1_d q_2 q_2_d]);    
B_jac = jacobian(xdot, [t_1 t_2]);
C = [1 0 0 0;
     0 0 1 0];

%% Kalman Filter covariance matrices
Q_kal = [1 0 0 0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

R_kal = [0.000033846 0;
        0 0.000033846];

%% LQR Q and R matrices
Q_LQR = [400000 0 0 0
        0 1000 0 0;
        0 0 400000 0;
        0 0 0 1000];
    
R_LQR = [1 0;
        0 1];
    
%% Iterate through all operating points and store linearized versions of all parameters
for i=1:length(target_points)
    %% Calculate Operating Point
    Y = sqrt(1-((target_points(i,1)^2 + target_points(i,2)^2 - l1^2 - l2^2)/(2*l1*l2))^2);
    X = (target_points(i,1)^2 + target_points(i,2)^2 - l1^2 - l2^2)/(2*l1*l2);
    q_2_bar = atan2(Y,X);
    q_2_d_bar = 0;
    
    q_1_bar = atan2(target_points(i,2), target_points(i,1)) - atan2(l2*sin(q_2_bar),l1 + l2*cos(q_2_bar));
    q_1_d_bar = 0;
    
    t_1_bar = ((m1*l1)/2 + m2*l1)*g*cos(q_1_bar) ...
        +  (m2*l2/2)*g*cos(q_1_bar + q_2_bar);
    t_2_bar = (m2*l2/2)*g*cos(q_1_bar + q_2_bar);
    
    %% Linearize about Operating Point
    A = A_jac;
    A = subs(A, q_1, q_1_bar);
    A = subs(A, q_2, q_2_bar);
    A = subs(A, q_1_d, q_1_d_bar);
    A = subs(A, q_2_d, q_2_d_bar);
    A = subs(A, m_1, m1);
    A = subs(A, m_2, m2);
    A = subs(A, l_1, l1);
    A = subs(A, l_2, l2);
    A = subs(A, c_1, c1);
    A = subs(A, c_2, c2);
    A = subs(A, g_, g);
    A = subs(A, t_1, t_1_bar);
    A = subs(A, t_2, t_2_bar);
    A = double(A);
   
    B = B_jac;
    B = subs(B, q_2, q_2_bar);
    B = subs(B, t_1, t_1_bar);
    B = subs(B, t_2, t_2_bar);
    B = subs(B, m_1, m1);
    B = subs(B, m_2, m2);
    B = subs(B, l_1, l1);
    B = subs(B, l_2, l2);
    B = subs(B, c_1, c1);
    B = subs(B, c_2, c2);
    B = subs(B, g_, g);
    B = double(B);
    
    %% Solve for estimator F matrix (Kalman Filter)
    [F,P,ev] = lqr(A',C',Q_kal,R_kal);
    F = F';
    
    %% Solve for controller gains (LQR)
    [K,P,ev] = lqr(A,B,Q_LQR,R_LQR);
    
    
    %% Store all relevant values
    A_list(:,:,i) = A;
    B_list(:,:,i) = B;
    F_list(:,:,i) = F;
    K_list(:,:,i) = K;
    t_list(:,:,i) = [t_1_bar t_2_bar];
    q1_list(i) = q_1_bar;
    q2_list(i) = q_2_bar;
    
end          
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%