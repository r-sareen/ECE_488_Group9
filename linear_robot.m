function xdot = linear_robot(t,x,A,B)
t_1 = 0;
t_2 = 0;
xdot = A*[x(1);x(2);x(3);x(4)] + B*[t_1; t_2];
end

