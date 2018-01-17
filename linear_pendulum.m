function xdot = linear_pendulum(t,x,A,B)
    u=2;
    xdot = A*[x(1);x(2)] + B*u;
end

