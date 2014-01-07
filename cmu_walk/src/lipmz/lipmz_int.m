function x1 = lipmz_int(x0, u0, dt, m, g, z)
    x1(1:3) = x0(1:3) + dt*x0(4:6);
    
    x = x0(1);
    y = x0(2);
    z = x0(3);
    F = u0(3);
    px = u0(1);
    py = u0(2);
    xdd = (x-px)*F/(m*z);
    ydd = (y-py)*F/(m*z);
    zdd = (F/m)-g;

    % xdd
    x1(4) = x0(4) + dt*xdd;
    % ydd
    x1(5) = x0(5) + dt*ydd;
    % zdd
    x1(6) = x0(6) + dt*zdd;
    
%     x1(1) = x0(1) + dt*x0(2);
%     x1(2) = x0(2) + dt*(x0(1)-u0(1))*g/z;
end