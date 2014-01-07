%IMU KF Parameters
clear; clc;

N_STATE = 27; 
N_OBSV = 12; 
dt = 1e-3; % Time interval
Quat_P0 =1e-1*eye(N_STATE);
dlmwrite('Imu_P0', Quat_P0, 'delimiter', '\t', 'precision', 15);
% Quat_P0(1:3,1:3) = 1e-4*eye(3);
% Quat_P0(4:6,4:6) = 1e-4*eye(3);
% Quat:_P0(7:9,7:9) = 1e-4*eye(3);

varQ = 1e3;
Quat_Q = eye(N_STATE); 
% Quat_Q(1:3,1:3) = (dt^5)/20*eye(3); %x
Quat_Q(1:3,1:3) = 1e-7/varQ*eye(3); %x
Quat_Q(4:6,4:6) = (dt^5)/20*eye(3); %q
% Quat_Q(7:9,7:9) = (dt^3)/3*eye(3); %v
Quat_Q(7:9,7:9) = dt*eye(3); %v
Quat_Q(10:12,10:12) = (dt^3)/3*eye(3); %w
% Quat_Q(13:15,13:15) = (1.7e-2)^2/varQ*eye(3); %a
Quat_Q(13:15,13:15) =dt*eye(3); %a

Quat_Q(1:3,7:9) = (dt^4)/8*eye(3); %xv
Quat_Q(7:9,1:3) = (dt^4)/8*eye(3); %xv
Quat_Q(4:6,10:12) = (dt^4)/8*eye(3); %qw
Quat_Q(10:12,4:6) = (dt^4)/8*eye(3);%qw

Quat_Q(16:18,16:18) = (8e-7)^2/varQ*eye(3); %w_bias
% Quat_Q(19:21,19:21) = 1e-6/varQ*eye(3); % a_bias
Quat_Q(19:21,19:21) = 1e-15*eye(3); % a_bias

Quat_Q(22:24,22:24) = 1e-7/varQ*eye(3); % Left foot position
Quat_Q(25:27,25:27) = 1e-7/varQ*eye(3); % Right Foot position



Quat_Q = varQ*Quat_Q; 
dlmwrite('Imu_Q', Quat_Q, 'delimiter', '\t', 'precision', 15);
% 
Quat_R = 1e-4*eye(N_OBSV);
Quat_R(4:6,4:6) = 1e4*eye(3);

dlmwrite('Imu_R', Quat_R, 'delimiter', '\t', 'precision', 15);
