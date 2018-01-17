function xdot = non_linear_pendulum(t,x)
    T = 1;
    xdot = [x(2);
        T - x(2) - sin(x(1))];
end

