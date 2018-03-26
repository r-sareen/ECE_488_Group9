%% Estimate state using the state estimator equations
U_old = U;

x_hat = x_hat_old + ((A_list(:,:,j)-F_list(:,:,j)*C)*x_hat_old + F_list(:,:,j)*C*qout(end,:)' + B_list(:,:,j)*U_old)*0.001;
x_hat_old = x_hat;

if (x_hat(1) < 0.001 && x_hat(3) < 0.001)
    j = j+1;
   
    U = -K_list(:,:,j)*(x_hat);
end
    
% Just state feedback is required for pull method
% If we did push, regulator is required.

    
    