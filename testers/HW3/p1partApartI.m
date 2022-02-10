%% AME 532a HW 3 Problem 1 
clc
clear
close all

% Add all custom libraries to the workspace
addpath('fv_sim/fv_sim/user_defined_libraries');

%% Part a 

% Radius of Earth
re_m = 6378000;

% Starting vector of body in ECEF frame and CS
ExE_BfromE_0_m = [re_m; 0; 0];

% lat lon and euler angles for DCMs
ll_NfromE_deg = [0, 0]; %Note: N is North East Down; Need inv of this DCM
euler_angles_NfromB_0_deg = [0; 0; 0]; % Body frame same as NED
% C_EfromB is found in simulink

% Velocity of the body in ECEF frame in the body CS
EvB_BfromE_mps = [150; 0; 0];

% Time used to set up simulink simulation
flight_time_s = 1000;

% Run the simulink model
simout = sim('translation', 'TimeOut', flight_time_s);
tout = simout.tout;
ExE_BfromE_m_part_i = simout.ExE_BfromE_m_part_i;
ExE_BfromE_m_part_ii = simout.ExE_BfromE_m_part_ii;

figure(1)
subplot(1,3,1);
plot(tout, ExE_BfromE_m_part_i(:,1));
title("P1a Part I: X position with constant C_E_B");
subplot(1,3,2);
plot(tout, ExE_BfromE_m_part_i(:,2));
title("P1a Part I: Y position with constant C_E_B");
subplot(1,3,3);
plot(tout, ExE_BfromE_m_part_i(:,3));
title("P1a Part I: Z position with constant C_E_B");

figure(2)
subplot(1,3,1);
plot(tout, ExE_BfromE_m_part_ii(:,1));
title("X position with lat/lon-based C_E_B");
subplot(1,3,2);
plot(tout, ExE_BfromE_m_part_ii(:,2));
title("Y position with lat/lon-based C_E_B");
subplot(1,3,3);
plot(tout, ExE_BfromE_m_part_ii(:,3));
title("Z position with lat/lon-based C_E_B");

%% Part b

% roll pitch yaw, or phi theta psi
omega_BwrtN_rps = deg2rad([0; 0; 0.01]); % Rotate about the down axis

% Run the simulink model
simout_b = sim('rotation_b', 'TimeOut', flight_time_s);
tout_b = simout_b.tout;
% yaw pitch roll, or psi theta phi
euler_angles_NfromB_euler_kin_deg = rad2deg(simout_b.euler_angles_NfromB_euler_kin_deg);

figure()
plot(tout_b, euler_angles_NfromB_euler_kin_deg(:,1));
title("P1b: angular rotation (H method)");

%% Part c

% Run the simulink model
simout_c = sim('rotation_c', 'TimeOut', flight_time_s);
tout_c = simout_c.tout;
% yaw pitch roll, or psi theta phi
euler_angles_NfromB_strapdown_deg = rad2deg(simout_c.euler_angles_NfromB_strapdown_deg);

figure()
plot(tout_c, euler_angles_NfromB_strapdown_deg(:,1));
title("P1c: angular rotation (strapdown eqns)");

%% Part d

% Run the simulink model
simout_d = sim('rotation_d', 'TimeOut', flight_time_s);
tout_d = simout_d.tout;
% yaw pitch roll, or psi theta phi
euler_angles_NfromB_quat_deg = rad2deg(simout_d.euler_angles_NfromB_quat_deg);

figure()
plot(tout_d, euler_angles_NfromB_quat_deg(:,1));
title("P1d: angular rotation (quaternion method)");