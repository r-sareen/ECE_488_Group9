function xdot = linear_robot(t,x,A,B,K,x_des)
u = K*(x-x_des);
xdot = A*[x(1);x(2);x(3);x(4)] + B*u;
end

