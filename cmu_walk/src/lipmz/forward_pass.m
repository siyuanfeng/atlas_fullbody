function [X, U] = forward_pass(dyn, alpha, x0, Xref, Uref, Ks, dus, dt, m, g, z)
    X = zeros(size(Xref));
    U = zeros(size(Uref));
    N = size(Xref, 2);
    X(:,1) = x0;
    
    for i = 1 : N
        zz = X(:,i) - Xref(:,i);
        U(:,i) = Ks{i}*zz + Uref(:,i) + alpha*dus{i};
        if (i < N)
            X(:,i+1) = dyn(X(:,i), U(:,i), dt, m, g, z);
%             X(:,i+1) = lipmz_int(X(:,i), U(:,i), dt, m, g, z);
        end
    end
end