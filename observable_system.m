function x_s = observable_system(t,x,A,B,C,F)
A = [A zeros(3,3);
    F*C A-F*C];

B = [B;
    B];

C = [C 0 0 0];

x_s = A*x + B*1;
end

