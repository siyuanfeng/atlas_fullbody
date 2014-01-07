
%WPA model
% Survey of Maneuvering Target Tracking. Part I: Dynamic Models
% EQ 1
%% IMU FILTER
N_STATE = 9; 
N_OBSV = 6; 
dt = 1e-3; % Time interval
Quat_P0 =1e-1*eye(N_STATE);
dlmwrite('Quat_P0', Quat_P0, 'delimiter', '\t', 'precision', 15);

varQ = 1e3;
Quat_Q = eye(N_STATE); 
Quat_Q(7:9,7:9) = (dt^5)/20*eye(3); % q
Quat_Q(1:3,1:3) = (dt^3)/3*eye(3); %w
% Quat_Q(4:6,4:6) = dt*eye(3); %a
Quat_Q(4:6,4:6) = dt*eye(3); %a
Quat_Q(1:3,7:9) = (dt^4)/8*eye(3); %qw
Quat_Q(7:9,1:3) = (dt^4)/8*eye(3); %qw
Quat_Q = varQ*Quat_Q; 
dlmwrite('Quat_Q', Quat_Q, 'delimiter', '\t', 'precision', 15);
% 
% Quat_R = (0.017^2)*eye(N_OBSV);

Quat_R = 1e5*eye(N_OBSV);
Quat_R(1:3,1:3) = 4e-8*eye(3);

dlmwrite('Quat_R', Quat_R, 'delimiter', '\t', 'precision', 15);

%% Bias Filter
N_STATE = 6; 
N_OBSV = 6; 
P0b =1e-4*eye(N_STATE);
dlmwrite('Bias_P0', P0b, 'delimiter', '\t', 'precision', 15);

Qb = 1e-10*eye(N_STATE);
dlmwrite('Bias_Q', Qb, 'delimiter', '\t', 'precision', 15);
% Perfect measurement (0)
Rb = 1e0*eye(N_OBSV);
% Rb(1:3,1:3) = (7.5e-6)^2; 
% Rb(4:6,4:6) = 1e-6; 
Rb(1:3,1:3) = 7.5*7.5e-6*eye(3); 
Rb(4:6,4:6) = 1e0*eye(3); 
dlmwrite('Bias_R', Rb, 'delimiter', '\t', 'precision', 15);

%% Kinematic filter 2 for velocity and position
N_STATE = 6; 
N_OBSV = 12; 

% qv = 5e-7;
% qx = 1e-10;
% % rv = 5e-5;
% rv = 1e-4;
% % rx = 1e-7;
% rx = 1e-6;
% % rx = 5e5;
qv = 1e-2;
qx = 1e-6;
% rv = 5e-5;
rv = 1e-4;
% rx = 1e-7;
rx = 1e-4;
% rx = 5e5;

P0kc =1e-2*eye(N_STATE);
dlmwrite('KC_P02', P0kc, 'delimiter', '\t', 'precision', 15);

Q_DSc = qv*eye(N_STATE);
Q_DSc(1:3,1:3) = qx*eye(3);
dlmwrite('DSc_Q2', Q_DSc, 'delimiter', '\t', 'precision', 15);

% Perfect measurement (0)

R_DSc = rv*eye(N_OBSV);
R_DSc(1:3,1:3) = rx*eye(3);
R_DSc(7:9,7:9) = rx*eye(3);

% R_DSc(3,3) = 1e-7; % z is known for now
% R_DSc(9,9) = 1e-7;


dlmwrite('DSc_R2', R_DSc, 'delimiter', '\t', 'precision', 15);
R_SS = rv*eye(N_OBSV);
R_SS(1:3,1:3) = rx*eye(3);

% R_SS(3,3) = 1e-7;

R_SS(7:12,7:12) = 0.0;
dlmwrite('SS_R2', R_SS, 'delimiter', '\t', 'precision', 15);
% Rb(1:3,1:3) = (7.5e-6)^2; 
% Rb(4:6,4:6) = 1e-6; 

alpha = [0.1, 1e-0, 1e-1];
dlmwrite('alpha_Rxv', alpha, 'delimiter', '\t', 'precision', 15);

%% Kinematic filter 3 for velocity and position
N_STATE = 6; 
N_OBSV = 6; 
% Working Parameters
% qv = 1e-4;
% qx = 1e-10;
% rv = 1e-4;
% rx = 1e-9;

qv = 1e-0;
qx = 1e-1;
rv = 1e-0;
rx = 1e-0;

P0kc =1e-2*eye(N_STATE);
dlmwrite('KC_P03', P0kc, 'delimiter', '\t', 'precision', 15);

Q_DSc = qv*eye(N_STATE);
Q_DSc(1:3,1:3) = qx*eye(3);
dlmwrite('DSc_Q3', Q_DSc, 'delimiter', '\t', 'precision', 15);

% Perfect measurement (0)

R_DSc = rv*eye(N_OBSV);
R_DSc(1:3,1:3) = rx*eye(3);
% R_DSc(6,6) = 1e-3; % Vertical velcotiy
dlmwrite('DSc_R3', R_DSc, 'delimiter', '\t', 'precision', 15);
cd ../../

