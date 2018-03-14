%% Implement estimater here
x_pos = cos(q(1))*l1 + cos(q(1)+q(3))*l2;
y_pos = sin(q(1))*l1 + sin(q(1)+q(3))*l2;

x_delta = x_pos-target_points(i,1);
y_delta = y_pos-target_points(i,2);

if (x_delta < 0.001 && y_delta < 0.001)
    i+=1;
    
    %% Calculate Operating Point
    Y = sqrt(1-((x_delta^2 + y_delta^2 - l1^2 - l2^2)/(2*l1*l2))^2);
    X = (x_delta^2 + y_delta^2 - l1^2 - l2^2)/(2*l1*l2);
    q_2_bar_delta = atan2(Y,X)
    q_2_d_bar_delta = 0
    
    q_1_bar_delta = atan2(y_delta, x_delta) - atan2(l2*sin(q_2_bar),l1 + l2*cos(q_2_bar))
    q_1_d_bar_delta = 0
    
    t_1_bar = ((m1*l1)/2 + m2*l1)*g*cos(q_1_bar) ...
        +  (m2*l2/2)*g*cos(q_1_bar + q_2_bar);
    t_2_bar = (m2*l2/2)*g*cos(q_1_bar + q_2_bar);
    
    %% Linearize about Operating Point
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
    
    C = [1 0 1 0];
    
    % Get K
    % Just state feedback is required for pull method
    % If we did push, regulator is required.
    
    % Calculate U (inputs)
    
    