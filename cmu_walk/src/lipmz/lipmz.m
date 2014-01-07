clear
% close all

syms x y z xd yd zd F px py dt m g z0

xdd = (x-px)*F/(m*(z-z0));
ydd = (y-py)*F/(m*(z-z0));
zdd = (F/m)-g;

x1 = x + xd*dt;
y1 = y + yd*dt;
z1 = z + zd*dt;
xd1 = xd + xdd*dt;
yd1 = yd + ydd*dt;
zd1 = zd + zdd*dt;

A = [diff(x1, x), diff(x1, y), diff(x1, z), diff(x1, xd), diff(x1, yd), diff(x1, zd); ... 
     diff(y1, x), diff(y1, y), diff(y1, z), diff(y1, xd), diff(y1, yd), diff(y1, zd); ... 
     diff(z1, x), diff(z1, y), diff(z1, z), diff(z1, xd), diff(z1, yd), diff(z1, zd); ... 
     diff(xd1, x), diff(xd1, y), diff(xd1, z), diff(xd1, xd), diff(xd1, yd), diff(xd1, zd); ... 
     diff(yd1, x), diff(yd1, y), diff(yd1, z), diff(yd1, xd), diff(yd1, yd), diff(yd1, zd); ... 
     diff(zd1, x), diff(zd1, y), diff(zd1, z), diff(zd1, xd), diff(zd1, yd), diff(zd1, zd)]

B = [diff(x1, px), diff(x1, py), diff(x1, F); ...
     diff(y1, px), diff(y1, py), diff(y1, F); ...
     diff(z1, px), diff(z1, py), diff(z1, F); ...
     diff(xd1, px), diff(xd1, py), diff(xd1, F); ...
     diff(yd1, px), diff(yd1, py), diff(yd1, F); ...
     diff(zd1, px), diff(zd1, py), diff(zd1, F)]

% x = [x y z xd xd zd]
% u = [F px py]
% Q = 1e-6*eye(6);
% Q(3,3) = 1e-2;
% R = eye(3);
% R(3,3) = 1e-6;
Q(1:3,1:3) = 1e-4*eye(3);
Q(4:6,4:6) = 1e-2*eye(3);
R = eye(3);
Q(3,3) = 1e1;
R(3,3) = 1e-6;

dt = 1e-3;
m = 90;
g = 9.81;

x = 0;
y = 0;
z = 0.88;
xd = 0;
yd = 0;
zd = 0;

F = m*g;
px = 0;
py = 0;

% get Vxx
As = subs(A);
Bs = subs(B);

[a,b] = getAB([x, y, z, 0, 0, 0], [px, py, F], dt, m, g, z)

[k,s,e] = dlqr( a, b, Q, R );

Vxx = s;
Vx = zeros(6,1);

N = 3600;
py_d = zeros(1,N);
px_d = zeros(1,N);
z_d = 0.93*ones(1,N);

time = linspace(1,N-1,N)*dt;

px_d(1:600) = 0; 
px_d(601:1200) = 0.4;
px_d(1201:1800) = 0.8;
px_d(1801:end) = 0.8;

py_d(1:600) = 0; 
py_d(601:1200) = -0.1;
py_d(1201:1800) = 0.1;

z_d(1:600) = 0.88;
z_d(601:1200) = 0.98;
z_d(1201:1800) = 0.78; %-0.1;

% py_d(1,2001:3000) = -0.1;
% py_d(1,3001:4000) = 0.1;
% py_d(1,4001:5000) = -0.1;
% py_d(1,5001:6000) = 0.1;
% py_d(1,6001:7000) = -0.1;
% py_d(1,7001:8000) = 0.1;
% 
% px_d(1,2001:3000) = 0.1;
% px_d(1,3001:4000) = 0.2;
% px_d(1,4001:5000) = 0.3;
% px_d(1,5001:6000) = 0.4;
% px_d(1,6001:7000) = 0.5;
% px_d(1,7001:8000) = 0.6;
% px_d(1,8001:end) = 0.6;
% 
% z_d(1,3001:4000) = 0.9;
% z_d(1,4001:5000) = 0.93;
% z_d(1,5001:6000) = 1;

xx(:,1) = [ px_d(1,1) py_d(1,1) z 0 0 0 ]';
uu(:,N) = [ F; 0; 0 ];

xxref = zeros(6,N);
xxref(1,:) = px_d;
xxref(2,:) = py_d;
xxref(3,:) = z_d;
uuref = zeros(3,N);
uuref(1,:) = px_d;
uuref(2,:) = py_d;
uuref(3,:) = F*ones(1,N);

% forwardpass
for i = 1:N
 zz = xx(:,i) - xxref(:,i);
 uu(:,i) = - k*zz + uuref(:,i);
 if (i < N)
    xx(:,i+1) = lipmz_int(xx(:,i), uu(:,i), dt, m, g, z);
 end
end

hFig = figure(1);
set(hFig, 'Position', [0 0 800 800]);
set(gca, 'FontSize', 14);
subplot(3,1,1)
hold on
plot(time, xx(1,:), 'b-.', 'LineWidth', 2);
plot(time, uu(1,:), 'g-.', 'LineWidth', 2);
plot(time, px_d(1,:), 'r', 'LineWidth', 2);

subplot(3,1,2)
hold on
plot(time, xx(2,:), 'b-.', 'LineWidth', 2);
plot(time, uu(2,:), 'g-.', 'LineWidth', 2);
plot(time, py_d(1,:), 'r', 'LineWidth', 2);

subplot(3,1,3)
hold on
plot(time, xx(3,:), 'b-.', 'LineWidth', 2);
plot(time, uu(3,:) ./ (m*g), 'g-.', 'LineWidth', 2);
plot(time, z_d, 'r', 'LineWidth', 2);

% figure
% plot(px_d(1,:),py_d(1,:),'b',uu(1,:),uu(1,:),'c',xx(1,:),xx(2,:),'r')
% legend('desired ZMP', 'actual ZMP', 'actual COM');

% figure
% plot(xx(3,:));

x0 = xx(:,1);
X = xx;
U = uu;
ii = 1 : N;

alpha = 1;
for it = 1 : 5
    [Ks, dus] = backward_pass(@getAB, X, U, xxref, uuref, Q, R, Vxx, Vx, dt, m, g, z);
    [X, U] = forward_pass(@lipmz_int, alpha, x0, X, U, Ks, dus, dt, m, g, z);
    
%     if (mod(it, 5) == 0) 
%         figure
%         subplot(2,1,1)
%         plot(px_d(1,:),py_d(1,:),'b',U(1,:),U(2,:),'c',X(1,ii),X(2,ii),'r')
%         legend('desired ZMP', 'actual ZMP', 'actual COM');
% 
%         subplot(2,1,2)
%         plot(ii,z_d(1,ii),'b',ii,X(3,:),'r');
%         legend('desired z', 'actual z');
%         drawnow
%     end
end


    subplot(3,1,1)
    hold on    
    plot(time, U(1,:), 'g', 'LineWidth', 2);
    plot(time, X(1,:), 'b', 'LineWidth', 2);    
    xlabel('time [s]');
    ylabel('x [m]');
    legend('x0', 'px0', 'px_d', 'px1', 'x1');
    axis([0, 3.5, -0.5, 1]);

    subplot(3,1,2)
    hold on
    plot(time, U(2,:), 'g', 'LineWidth', 2);
    plot(time, X(2,:), 'b', 'LineWidth', 2);    
    xlabel('time [s]');
    ylabel('y [m]');
    legend('y0', 'py0', 'py_d', 'py1', 'y1');
    axis([0, 3.5, -0.2, 0.2]);

    subplot(3,1,3)
    hold on
    plot(time, U(3,:) ./ (m*g), 'g', 'LineWidth', 2);
    plot(time, X(3,:), 'b', 'LineWidth', 2);    
    xlabel('time [s]');
    ylabel('z [m]');
    legend('z0', 'Fz0/mg', 'z_d', 'Fz1/mg', 'z1');
    axis([0, 3.5, 0.5, 1.5]);
    
    
% figure
% plot(X(3,:))

% figure
% subplot(3,1,1)
% hold on
% plot(xxref(1,:), 'b');
% plot(U(1,:), 'c');
% plot(X(1,:), 'r');
% 
% subplot(3,1,2)
% hold on
% plot(xxref(2,:), 'b');
% plot(U(2,:), 'c');
% plot(X(2,:), 'r');
% 
% subplot(3,1,3)
% hold on
% plot(xxref(3,:), 'b');
% plot(X(3,:), 'r');
