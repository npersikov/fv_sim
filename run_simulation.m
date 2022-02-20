%% AME 532a HW 3 Problem 2 
clc
clear
close all

% Add all custom libraries to the workspace
addpath('C:\Users\Nikita\Documents\USC\Homework\Spring 2022\Flight Vehicle Stability and Control\fv_sim/fv_sim/user_defined_libraries');

%% Simulation Setup

% Constants. TODO: move these out into their own folder
re_m            = 6378000; % Radius of Earth
g_mps2          = 0;

% Simulation Parameters
flight_time_s   = 1000; % Time used to set up simulink simulation %TODO connect this to Simulink

% ===== Vehicle Parameters =====

m = 1000;
J = [900, 0, 0;
    0, 200, 0;
    0, 0, 600];

% ===== Initial Conditions =====

% Vehicle State
ExE_BfromE_0_m  = [re_m; 0; 0]; % Starting vector of body in ECEF frame and CS
EvB_BfromE_mps  = [0; 0; 0]; % Velocity of the body in ECEF frame in the body CS
omega_BwrtN_dps = [0; 0; 0.01]; % roll pitch yaw rates, or phi theta psi rates. (IE, rotate about the down axis)
omega_BwrtN_rps = deg2rad(omega_BwrtN_dps);
omega_pure_quat = [0; omega_BwrtN_rps]';

% Frames and Coordinate Systems
ll_NfromE_deg               = [0, 0]; % Lat lon and euler angles for DCMs. Note: N is North East Down; Need inv of this DCM
ll_NfromE_rad               = deg2rad(ll_NfromE_deg); %TODO: figure out where to handle degree changes
euler_angles_NfromB_0_deg   = [0; 0; 0]; % Body frame same as NED. Note: C_EfromB is found in simulink. NOTE: must be ZYX rotation!
euler_angles_NfromB_0_rad   = deg2rad(euler_angles_NfromB_0_deg);
omega_EwrtI_rps_0           = [0; 0; 0]; % rotation of Earth
omega_BwrtI_rps_0           = omega_BwrtN_dps + omega_EwrtI_rps_0; % since initial N to E velocity is 0;0;0

% ===== Simulation =====

% Run the simulink model
simout = sim('fv_sim', 'TimeOut', flight_time_s);

% Get simulation outputs
tout                    = simout.tout;
ExE_BfromE_m            = simout.ExE_BfromE_m;
euler_angles_NfromB_deg = rad2deg(simout.euler_angles_NfromB_rad);


% ===== Post Processing =====
problem = "Problem 2b: ";
figure(1)
title("Unique initial conditions");
subplot(1,3,1);
plot(tout, ExE_BfromE_m(:,1));
title("X position vs time");
subplot(1,3,2);
plot(tout, ExE_BfromE_m(:,2));
title("Y position vs time");
subplot(1,3,3);
plot(tout, ExE_BfromE_m(:,3));
title("Z position vs time");

figure(2)
title("Angular position");
subplot(1,3,1);
plot(tout, euler_angles_NfromB_deg(:,1));
title("Rotation about Z (degrees) vs time (s)");
subplot(1,3,2);
plot(tout, euler_angles_NfromB_deg(:,2));
title("Rotation about Y (degrees) vs time (s)");
subplot(1,3,3);
plot(tout, euler_angles_NfromB_deg(:,3));
title("Rotation about X (degrees) vs time (s)");
