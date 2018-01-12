function xdot = mass_spring_damper_system(t,x,M,B,K)
    if (t<=0.2)
        u = 1;
    else
        u = 0;
    end
    xdot(1) = x(2);
    xdot(2) = u/M - (B/M)*x(2) - (K/M)*x(1);
    xdot = xdot';
end

