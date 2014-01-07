function [Ks, dus] = backward_pass(ABfunc, X, U, Xref, Uref, Q, R, Vxx, Vx, dt, m, g, z)
    for i = size(X,2):-1:1
        x = X(:,i);
        u = U(:,i);
        xref = Xref(:,i);
        uref = Uref(:,i);
        
        [A, B] = ABfunc(x, u, dt, m, g, z);
%         [A, B] = getAB(x, u, dt, m, g, z);
        
        Lx = Q*(x-xref);
        Lu = R*(u-uref);
        
        Qx = Lx + A'*Vx;
        Qu = Lu + B'*Vx;
        
        Qxx = Q + A'*Vxx*A;
        Qxu = A'*Vxx*B;
        Quu = R + B'*Vxx*B;
        Qux = B'*Vxx*A;
%         Qux = Qxu';

        Ks{i} = -Quu\Qux;
        dus{i} = -Quu\Qu;
%         Ks{i} = -inv(Quu)*Qux;
%         dus{i} = -inv(Quu)*Qu;
        
        Vx = Qx + Ks{i}'*Qu;
        Vxx = Qxx + Qxu*Ks{i};
    end
end