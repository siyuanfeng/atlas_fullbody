function x1 = lipm_int(x0, u0, dt, m, g, z)
    x1(1:2) = x0(1:2) + dt*x0(3:4);
    x1(3) = x0(3) + dt*(x0(1)-u0(1))*g/z;
    x1(4) = x0(4) + dt*(x0(2)-u0(2))*g/z;
%     x1(1) = x0(1) + dt*x0(2);
%     x1(2) = x0(2) + dt*(x0(1)-u0(1))*g/z;
end