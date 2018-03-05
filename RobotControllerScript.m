x_pos = cos(x(1))*l1 + cos(x(1)+x(3))*l2;
y_pos = sin(x(1))*l1 + sin(x(1)+x(3))*l2;

if (x_pos-target_points(i,1) < 0.01 && y_pos-target_points(i,2))
    i+=1;
    
    % Calculate operating point
    q_1_bar = 
    q_1_d_bar = 
    q_2_bar =
    q_2_d_bar = 
    
    t_1_bar = ((m1*l1)/2 + m2*l1)*g*cos(q_1_bar) ...
        +  (m2*l2/2)*g*cos(q_1_bar + q_2_bar);
    t_2_bar = (m2*l2/2)*g*cos(q_1_bar + q_2_bar);
    
    A = subs(A_jac, q_1, q_1_bar);
    A = subs(A_jac, q_2, q_2_bar);
    A = subs(A_jac, q_1_d, q_1_d_bar);
    A = subs(A_jac, q_2_d, q_2_d_bar);
    A = subs(A_jac, m_1, m1);
    A = subs(A_jac, m_2, m2);
    A = subs(A_jac, l_1, l1);
    A = subs(A_jac, l_2, l2);
    A = subs(A_jac, c_1, c1);
    A = subs(A_jac, c_2, c2);
    A = subs(A_jac, g_, g);
    A = double(A);
   
    B = subs(B_jac, q_2, q_2_bar);
    B = subs(B_jac, t_1, t_1_bar);
    B = subs(B_jac, t_2, t_2_bar);
    B = subs(B_jac, m_1, m1);
    B = subs(B_jac, m_2, m2);
    B = subs(B_jac, l_1, l1);
    B = subs(B_jac, l_2, l2);
    B = subs(B_jac, c_1, c1);
    B = subs(B_jac, c_2, c2);
    B = subs(B_jac, g_, g);
    B = double(B);
    
    % Get K
    
    % Calculate U (inputs)
    
    