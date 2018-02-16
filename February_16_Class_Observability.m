%Rishab Sareen - 20505101
%Conrad Montor - 20460296

clear all
close all

A = [0 0 -1;
    -1 0 0;
     3 -1 -3];
 
 B = [0;
     0;
     1];
 
 C = [0 1 0];
 
 R = ctrb(A', C')';
 
 if (rank(R) == size(A))
    poly = charpoly(A);
    A_ = [0 0 -poly(3);
          1 0 -poly(2);
          0 1 -poly(1)];
    C_ = [0 0 1];
    R_ = ctrb(A_',C_')';
    T = inv(R_)*R;
%     syms s f1 f2 f3;
%     F_ = [f1;
%          f2;
%          f3];
%     char_poly = det(s*eye(3) - (A_-F_*C_));
    
    F_ = place(A_', C_',[-2 -2.00001 -1.99999]);
    F = inv(T)*F_'; 
 end
 
 [t,x,x_hat] = ode45 (@(t,x) observable_system(t,x,A,B,C,F), [0 10], [0 0 0 1 1 1]');
 
 figure(1);
 plot(x(:,1));
 hold on
 plot(x(:,4));
 plot(x(:,1)-x(:,4));
 title('x(1) vs estimated x(1)');
 legend('real', 'estimated', 'error');
 
 figure(2);
 plot(x(:,2));
 hold on
 plot(x(:,5));
 plot(x(:,2)-x(:,5));
 title('x(2) vs estimated x(2)');
 legend('real', 'estimated', 'error');
 
 figure(3);
 plot(x(:,3));
 hold on
 plot(x(:,6));
 plot(x(:,3)-x(:,6));
 title('x(3) vs estimated x(3)');
 legend('real', 'estimated', 'error');
