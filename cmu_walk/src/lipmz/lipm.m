clear
close all

syms x y px py z xd yd dt m g

xdd = (x-px)*g/z;
ydd = (y-py)*g/z;

x1 = x + xd*dt;
xd1 = xd + xdd*dt;
y1 = y + yd*dt;
yd1 = yd + ydd*dt;

% [x y xd yd]
% u = [px py]
A = [diff(x1, x), diff(x1, y), diff(x1, xd), diff(x1, yd); ...  
     diff(y1, x), diff(y1, y), diff(y1, xd), diff(y1, yd); ...  
     diff(xd1, x), diff(xd1, y), diff(xd1, xd), diff(xd1, yd); ... 
     diff(yd1, x), diff(yd1, y), diff(yd1, xd), diff(yd1, yd)]

B = [diff(x1, px),diff(x1, py); ...
     diff(y1, px),diff(y1, py); ...
     diff(xd1, px), diff(xd1, py); ...
     diff(yd1, px), diff(yd1, py)]

Q = 1e-6*eye(4);
R = 1e0*eye(2);

dt = 1e-3;
g = 9.81;
m = 90;
F = m*g;

x = 0;
p = 0;
z = 0.93;
xd = 0;

% get Vxx
As = subs(A)
Bs = subs(B)

[a,b] = getAB_lipm([x, z, 0, 0], [F, p], dt, m, g, z)

[k,s,e] = dlqr( As, Bs, Q, R );

Vxx = s;
Vx = zeros(size(Vxx,1),1);

N = 10000;
py_d = zeros(1,N);
py_d(1,1201:1800) = -0.1;
py_d(1,1801:2400) = 0.1;
py_d(1,2401:3000) = -0.1;
py_d(1,3001:3600) = 0.1;
py_d(1,3601:4200) = -0.1;
py_d(1,4201:4800) = 0.1;

px_d = zeros(1,N);
px_d(1,1201:1800) = 0.4;
px_d(1,1801:2400) = 0.8;
px_d(1,2401:3000) = 1.2;
px_d(1,3001:3600) = 1.6;
px_d(1,3601:4200) = 2.0;
px_d(1,4201:4800) = 2.4;
px_d(1,4801:end) = 2.4;

z_d = 0.93*ones(1,N);

xx(:,1) = [ px_d(1); py_d(1); 0; 0 ];
uu(:,N) = [ px_d(1); py_d(1) ];

xxref = zeros(4,N);
xxref(1,:) = px_d;
xxref(2,:) = py_d;
uuref = zeros(2,N);
uuref(1,:) = px_d;
uuref(2,:) = py_d;

% forwardpass
for i = 1:N
	zz = xx(:,i) - xxref(:,i);
    uu(:,i) = - k*zz + uuref(:,i);
    if (i < N)
        xx(:,i+1) = lipm_int(xx(:,i), uu(:,i), dt, m, g, z);
    end
end

figure
ii = 1:N;
plot(ii,px_d(1,ii),'b',ii,uu(1,ii),'c',ii,xx(1,ii),'r')
legend('desired ZMP', 'actual ZMP', 'actual COM');

% figure
% plot(xx(2,:));

x0 = xx(:,1);
X = xx;
U = uu;
for it = 1 : 2
    [Ks, dus] = backward_pass(@getAB_lipm, X, U, xxref, uuref, Q, R, Vxx, Vx, dt, m, g, z);
    [X, U] = forward_pass(@lipm_int, 1, x0, X, U, Ks, dus, dt, m, g, z);

    figure
    plot(ii,px_d(1,ii),'b',ii,U(1,ii),'c',ii,X(1,ii),'r')
    legend('desired ZMP', 'actual ZMP', 'actual COM');
end

figure
plot(px_d(1,ii),py_d(1,ii),'b',U(1,ii),U(2,ii),'c',X(1,ii),X(2,ii),'r')
legend('desired ZMP', 'actual ZMP', 'actual COM');