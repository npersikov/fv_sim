%% AME 532a HW 3 Problem 2 
clc
clear
close all

% Add all custom libraries to the workspace
addpath('fv_sim/fv_sim/user_defined_libraries');

%% Simulation Setup

% Constants. TODO: move these out into their own folder
re_m = 6378000; % Radius of Earth


% ===== Initial Conditions =====

% Vehicle State
ExE_BfromE_0_m  = [re_m; 0; 0]; % Starting vector of body in ECEF frame and CS
EvB_BfromE_mps  = [150; 0; 0]; % Velocity of the body in ECEF frame in the body CS
omega_BwrtN_dps = [0; 0; 0.11]; % roll pitch yaw rates, or phi theta psi rates. (IE, rotate about the down axis)
omega_BwrtN_rps = deg2rad(omega_BwrtN_dps);

% Frames and Coordinate Systems
ll_NfromE_deg               = [0, 0]; % Lat lon and euler angles for DCMs. Note: N is North East Down; Need inv of this DCM
ll_NfromE_rad               = deg2rad(ll_NfromE_deg); %TODO: figure out where to handle degree changes
euler_angles_NfromB_0_deg   = [0; 0; 0]; % Body frame same as NED. Note: C_EfromB is found in simulink
euler_angles_NfromB_0_rad   = deg2rad(euler_angles_NfromB_0_deg);

% ===== Simulation =====

% Simulation Parameters
flight_time_s = 1000; % Time used to set up simulink simulation %TODO connect this to Simulink

% Run the simulink model
simout = sim('fv_sim', 'TimeOut', flight_time_s);

% Get simulation outputs
tout                    = simout.tout;
ExE_BfromE_m            = simout.ExE_BfromE_m;
euler_angles_NfromB_deg = simout.euler_angles_NfromB_deg;


% ===== Post Processing =====



