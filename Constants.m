%put constant values in this file%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%You NEED these constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
c1=0;%link 1 friction coeffecient
c2=0;%link 2 friction coeffecient
l1=0; %link 1 length
l2=0; %link 2 length
m1=0;%link 1 mass
m2=0;%link 2 mass
g=3.7;%acceleration due to gravity m/s^2 on mars
x_0=[0,0,0,0]';%x_0=[q1_0,q1dot_0,q2_0,q2dot_0] initial conditions for the robot
tau_0=[0,0]'; %initial torque
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Declare all your variables here, prefix with my_ %Feel Free to add to or remove these constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
my_time=0;
my_angle_vector=[0 0]';
my_state_estimate_vector=[0 0 0 0]';
my_some_variable_a=0;
my_some_variable_b=0;
i=1;
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
               0.1, 0.1];
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
   C = [1 0 1 0];
           
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%