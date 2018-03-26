U_old = U;

%% Estimate state using the state estimator equations
x_hat = x_hat_old + ((A_list(:,:,j)-F_list(:,:,j)*C)*x_hat_old + F_list(:,:,j)*C*qout(end,:)' + B_list(:,:,j)*U_old)*0.001;
x_hat = x_hat - [q1_list(j) 0 q2_list(j) 0]';
x_hat_old = x_hat;

%% No Estimator
% x_hat = qout(end,:)' - [q1_list(j) 0 q2_list(j) 0]';
% x_hat_old = x_hat;

if (abs(x_hat(1)) < 0.05 && abs(x_hat(3)) < 0.05)
    j = j+1; 
    if (j == 32)
        j = 31;
    end
end

U = -K_list(:,:,j)*(x_hat) + t_list(:,:,j);
    
% Just state feedback is required for pull method
% If we did push, regulator is required.

    
    