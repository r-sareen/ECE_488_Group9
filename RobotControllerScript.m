U_old = U;

%% Add noise to ouput
q = q + randn(2,1)*0.0058178;

%% Estimate state using the state estimator equations
x_hat_d = x_hat_old_d + ((A_list(:,:,j)-F_list(:,:,j)*C)*x_hat_old_d + F_list(:,:,j)*(q-[q1_list(j) q2_list(j)]')...
            + B_list(:,:,j)*(U_old-t_list(:,:,j)'))*0.001;
x_hat_old_d = x_hat_d;

% Calculate distance to target
x_hat = x_hat_d + [q1_list(j) 0 q2_list(j) 0]';
Y = sin(x_hat(1))*l1 + sin(x_hat(1)+x_hat(3))*l2;
X = cos(x_hat(1))*l1 + cos(x_hat(1)+x_hat(3))*l2;

% x_hat_d = x_hat - [q1_list(j) 0 q2_list(j) 0]';
% error_list(:,:,count) = x_hat - qout(end,:)' 

%% No Estimator
% x_hat = qout(end,:)';
% 
% % Calculate distance to target
% Y = sin(x_hat(1))*l1 + sin(x_hat(1)+x_hat(3))*l2;
% X = cos(x_hat(1))*l1 + cos(x_hat(1)+x_hat(3))*l2;
% 
% x_hat_d = x_hat - [q1_list(j) 0 q2_list(j) 0]';
% x_hat_old = x_hat;



if (sqrt((X-target_points(j,1))^2 + (Y-target_points(j,2))^2)<0.01)
    if (j==1 && wait_count<500)
        wait_count = wait_count+1;
        j = 1;
    end
    if (j==11 && wait_count<500)
        wait_count = wait_count+1;
        j = 11;
    end
    if (j==21 && wait_count<500)
        wait_count = wait_count+1;
        j = 21;
    end
    if (j==31 && wait_count<500)
        wait_count = wait_count+1;
        j = 31;
    end
    if (j==41 && wait_count<500)
        wait_count = wait_count+1;
        j = 41;
    end
    if (wait_count>=500 || wait_count == 0)
        if (j<41)
            j = j+1;
        end
        wait_count = 0;
    end
end

U = -K_list(:,:,j)*(x_hat_d) + t_list(:,:,j)';
    
% Just state feedback is required for pull method
% If we did push, regulator is required.

    
    