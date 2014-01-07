% The following is for quaternion KF

% N_STATE = 9; 
% N_OBSV = 6; 
% Quat_P0 =1e-1*eye(N_STATE);
% dlmwrite('Quat_P0', Quat_P0, 'delimiter', '\t', 'precision', 15);
% 
% Quat_Q = 1e-2*eye(N_STATE); 
% Quat_Q(1:3,1:3) = 1e-10*eye(3);
% Quat_Q(4:6,4:6) = 1e-4*eye(3);
% dlmwrite('Quat_Q', Quat_Q, 'delimiter', '\t', 'precision', 15);
% 
% Quat_R = 1e1*eye(N_OBSV);
% Quat_R(1:3,1:3) = 1e-4*eye(3);
% 
% dlmwrite('Quat_R', Quat_R, 'delimiter', '\t', 'precision', 15);


% N_STATE = 9; 
% N_OBSV = 6; 
% Quat_P0 =1e-1*eye(N_STATE);
% dlmwrite('Quat_P0', Quat_P0, 'delimiter', '\t', 'precision', 15);
% 
% Quat_Q = 1e-4*eye(N_STATE); 
% Quat_Q(1:3,1:3) = 1e-14*eye(3);
% Quat_Q(4:6,4:6) = 1e-6*eye(3);
% dlmwrite('Quat_Q', Quat_Q, 'delimiter', '\t', 'precision', 15);
% 
% Quat_R = 1e-4*eye(N_OBSV);
% Quat_R(1:3,1:3) = 1e-4*eye(3);
% 
% dlmwrite('Quat_R', Quat_R, 'delimiter', '\t', 'precision', 15);

%WPA model
% Survey of Maneuvering Target Tracking. Part I: Dynamic Models
% EQ 1
% The following works for iter = 80
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
% clear; clc;
% 
% N_STATE = 27; 
% N_OBSV = 12; 
% dt = 1e-3; % Time interval
% Quat_P0 =1e-1*eye(N_STATE);
% dlmwrite('Imu_P0', Quat_P0, 'delimiter', '\t', 'precision', 15);
% Quat_P0(1:3,1:3) = 1e-4*eye(3);
% Quat_P0(4:6,4:6) = 1e-4*eye(3);
% Quat_P0(7:9,7:9) = 1e-4*eye(3);
% 
% varQ = 1e3;
% Quat_Q = eye(N_STATE); 
% Quat_Q(1:3,1:3) = (dt^5)/20*eye(3); %x
% Quat_Q(4:6,4:6) = (dt^5)/20*eye(3); %q
% Quat_Q(7:9,7:9) = (dt^3)/3*eye(3); %v
% Quat_Q(10:12,10:12) = (dt^3)/3*eye(3); %w
% Quat_Q(13:15,13:15) = (1.7e-2)^2/varQ*eye(3); %a
% 
% Quat_Q(1:3,7:9) = (dt^4)/8*eye(3); %xv
% Quat_Q(7:9,1:3) = (dt^4)/8*eye(3); %xv
% Quat_Q(4:6,10:12) = (dt^4)/8*eye(3); %qw
% Quat_Q(10:12,4:6) = (dt^4)/8*eye(3);%qw
% 
% Quat_Q(16:18,16:18) = (8e-7)^2/varQ*eye(3); %w_bias
% Quat_Q(19:21,19:21) = 1e-6/varQ*eye(3); % a_bias
% 
% Quat_Q(22:24,22:24) = 1e-12/varQ*eye(3); % Left foot position
% Quat_Q(25:27,25:27) = 1e-12/varQ*eye(3); % Right Foot position
% 
% 
% 
% Quat_Q = varQ*Quat_Q; 
% dlmwrite('Imu_Q', Quat_Q, 'delimiter', '\t', 'precision', 15);
% % 
% Quat_R = 1e-4*eye(N_OBSV);
% % Quat_R(1:end,1:end) = 4e-2*eye(3);
% 
% dlmwrite('Imu_R', Quat_R, 'delimiter', '\t', 'precision', 15);
% 
% % For velocity EKF filter
% 
N_STATE = 3; 
N_OBSV = 3; 
P0v =1e-0*eye(N_STATE);
dlmwrite('P0v', P0v, 'delimiter', '\t', 'precision', 15);

Qv = 1e-12*eye(N_STATE);
dlmwrite('Qv', Qv, 'delimiter', '\t', 'precision', 15);
% Perfect measurement (0)
Rv = 3e-10*eye(N_OBSV);
dlmwrite('Rv', Rv, 'delimiter', '\t', 'precision', 15);

% N_STATE = 28; 
% N_OBSV = 28; 
% P0v =1e-1*eye(N_STATE);
% dlmwrite('P0_Jd_EKF', P0v, 'delimiter', '\t', 'precision', 15);
% 
% Qv = 4e-2*eye(N_STATE);
% dlmwrite('Q_Jd_EKF', Qv, 'delimiter', '\t', 'precision', 15);
% % Perfect measurement (0)
% Rv = 1e-0*eye(N_OBSV);
% dlmwrite('R_Jd_EKF', Rv, 'delimiter', '\t', 'precision', 15);
% Bias Filter
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

%     <noise>
%       <type>gaussian</type>
%       <rate>
%         <mean>0.0</mean>
%         <stddev>2e-4</stddev>
%         <bias_mean>0.0000075</bias_mean>
%         <bias_stddev>0.0000008</bias_stddev>
%       </rate>
%       <accel>
%         <mean>0.0</mean>
%         <stddev>1.7e-2</stddev>
%         <bias_mean>0.1</bias_mean>
%         <bias_stddev>0.001</bias_stddev>
%       </accel>
%     </noise>

% % Kinematic filter
% N_STATE = 15; 
% N_OBSV = 12; 
% P0kc =1e-4*eye(N_STATE);
% dlmwrite('KC_P0', P0kc, 'delimiter', '\t', 'precision', 15);
% 
% Q_DSc = 1e-6*eye(N_STATE);
% Q_DSc(4:6,4:6) = 1e-3*eye(3);
% dlmwrite('DSc_Q', Q_DSc, 'delimiter', '\t', 'precision', 15);
% Q_SSL = 1e-6*eye(N_STATE);
% Q_SSL(4:6,4:6) = 1e-3*eye(3);
% Q_SSL(10:12,10:12) = 1e6*eye(3);
% dlmwrite('SSL_Q', Q_SSL, 'delimiter', '\t', 'precision', 15);
% Q_SSR = 1e-6*eye(N_STATE);
% Q_SSR(4:6,4:6) = 1e-3*eye(3);
% Q_SSR(7:9,7:9) = 1e6*eye(3);
% dlmwrite('SSR_Q', Q_SSR, 'delimiter', '\t', 'precision', 15);
% % Perfect measurement (0)
% R_DSc = 1e-0*eye(N_OBSV);
% dlmwrite('DSc_R', R_DSc, 'delimiter', '\t', 'precision', 15);
% R_SS = 1e-0*eye(N_OBSV);
% R_SS(7:12,7:12) = 0.0;
% dlmwrite('SS_R', R_SS, 'delimiter', '\t', 'precision', 15);
% % Rb(1:3,1:3) = (7.5e-6)^2; 
% % Rb(4:6,4:6) = 1e-6; 

% Kinematic filter for velocity only
N_STATE = 3; 
N_OBSV = 6; 
P0kc =1e-4*eye(N_STATE);
dlmwrite('KC_P0', P0kc, 'delimiter', '\t', 'precision', 15);

Q_DSc = 1e-6*eye(N_STATE);
% Q_DSc(4:6,4:6) = 1e-3*eye(3);
dlmwrite('DSc_Q', Q_DSc, 'delimiter', '\t', 'precision', 15);
Q_SSL = 1e-6*eye(N_STATE);
% Q_SSL(4:6,4:6) = 1e-3*eye(3);
% Q_SSL(10:12,10:12) = 1e6*eye(3);
dlmwrite('SSL_Q', Q_SSL, 'delimiter', '\t', 'precision', 15);
Q_SSR = 1e-6*eye(N_STATE);
% Q_SSR(4:6,4:6) = 1e-3*eye(3);
% Q_SSR(7:9,7:9) = 1e6*eye(3);
dlmwrite('SSR_Q', Q_SSR, 'delimiter', '\t', 'precision', 15);
% Perfect measurement (0)
R_DSc = 5e-5*eye(N_OBSV);
dlmwrite('DSc_R', R_DSc, 'delimiter', '\t', 'precision', 15);
R_SS = 5e-5*eye(N_OBSV);
R_SS(4:6,4:6) = 0.0;
dlmwrite('SS_R', R_SS, 'delimiter', '\t', 'precision', 15);
% Rb(1:3,1:3) = (7.5e-6)^2; 
% Rb(4:6,4:6) = 1e-6; 


%     Eigen::Matrix<double,15,15> DSc_Q; 
%     Eigen::Matrix<double,15,15> SSL_Q; 
%     Eigen::Matrix<double,15,15> SSR_Q;
%     Eigen::Matrix<double,12,12> DSc_R; 
%     Eigen::Matrix<double,12,12> SS_R; 
