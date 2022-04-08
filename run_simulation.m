%% AME 532a Simulation Setup and Runner Script
clc
clear
close all

% Add all custom libraries to the workspace
run_sim_script_path = mfilename('fullpath');
run_sim_script_path = run_sim_script_path(1:(length(run_sim_script_path) - 14));
path_to_libraries = [run_sim_script_path,'user_defined_libraries'];
addpath(path_to_libraries); %C:\Users\Nikita\Documents\USC\Homework\Spring 2022\Flight Vehicle Stability and Control\fv_sim/
%% Simulation Setup

% Constants. TODO: move these out into their own folder
re_m            = 6378000; % Radius of Earth
g_mps2          = 9.81;

% Simulation Parameters
flight_time_s   = 1000; % Time used to set up simulink simulation %TODO connect this to Simulink

% Post-Processing Parameters
plots_on        = false; % True if plots are needed
rl_plots_on     = false; % True if root locus plots are needed

% ===== Vehicle Parameters =====

[cg_m, m_kg, J_kgm2] = get_mass_props();

%% ===== Initial Conditions =====

% Vehicle State
lat_d           = 35.28; % N35.28
lon_d           = -115;  % W115;
ground_level_m  = 995;  % 995 for N35.28 W115 7968 Found through trial and error for N35.28 W-115
altitude_m      = 100; 
thrust_N        = [10*0;0;0];

ExE_BfromE_0_m  = lla2ecef([lat_d, lon_d, ground_level_m + altitude_m])'; % SoCal
EvB_BfromE_mps  = [50; 0.00001; 10]; % Velocity of the body in ECEF frame in the body CS
omega_BwrtN_dps = [0.00001; 0.00001; 10*0.0000001]; % roll pitch yaw rates, or phi theta psi rates. (IE, rotate about the down axis)
omega_BwrtN_rps = deg2rad(omega_BwrtN_dps);
omega_pure_quat = [0; omega_BwrtN_rps]';

% Frames and Coordinate Systems
ll_NfromE_deg               = [lat_d, lon_d]; % Lat lon and euler angles for DCMs. Note: N is North East Down; Need inv of this DCM
ll_NfromE_rad               = deg2rad(ll_NfromE_deg); %TODO: figure out where to handle degree changes
euler_angles_BfromN_0_deg   = [0.017*0; 0.017*0; 0.017*0]; % Body frame same as NED. Note: C_EfromB is found in simulink. NOTE: must be ZYX rotation!
euler_angles_BfromN_0_rad   = deg2rad(euler_angles_BfromN_0_deg);
omega_EwrtI_rps_0           = [0; 0; 0]; % rotation of Earth
omega_BwrtI_rps_0           = omega_BwrtN_rps + omega_EwrtI_rps_0; % since initial N to E velocity is 0;0;0

%% Control Inputs
aoa_step_time = 10;
aoa_step_value = -5*pi/180*0;
bank_step_time = 10;
bank_step_value = 10*pi/180*0;

% Select which control systems are active:
is_controlling_pitch = false;
is_controlling_roll = true;

%% ===== Simulation =====

%% Linearize Model
% Linearize the model while ignoring translational motion. This linearized
% ABCD model will allow for stabilizing control system design for steady
% level flight.

% Set the control system gains
pitch_gain = -10*0;
roll_rate_gain = 10*0;
aoa_gain = 500*0; % The gain for angle of attack. Used to command the elevator deflection.
bank_angle_gain = 100*0; 

% Trim the simulink model
% [x,u,y,dx] = trim('fv_sim_linearized');
% The order that I think the states are in in the x vector
states = ["vel_x", "vel_y", "vel_z", "avel_x", "avel_y", "avel_z", "psi", "theta", "phi"];

% Get a state space model of the linearized system (ABCD matrices)
% argout = linmod('fv_sim_linearized', x, u);

% Get eigenvalues for linearized system
% lin_eigs = eig(argout.a);

% Get transfer functions from the state space model
% [num_coeff, den_coeff] = ss2tf(argout.a, argout.b, argout.c, argout.d);

% Plot the root locus for each state
if rl_plots_on
%     for rl_plot_num = 1:1:length(x)
%         sys = tf(num_coeff, den_coeff(rl_plot_num,:)); 
%         figure()
%         rlocus(sys);
%         title(states(rl_plot_num));
%         disp("Eigenvalue for " + states(rl_plot_num) + ": " + lin_eigs);
%     end

    % Pitch Plots
%     sys = tf(num_coeff(8,:), den_coeff); 
%     figure()
%     rlocus(sys);
%     figure()
%     rlocus(sys, pitch_gain);

    % Roll Plots
    sys = tf(num_coeff(6,:), den_coeff); 
    figure()
    rlocus(sys);
    title("Roll Rate (General Plot)");
    figure()
    rlocus(sys, roll_rate_gain);
    title("Roll Rate (With Gain)");

    sys = tf(num_coeff(9,:), den_coeff); 
    figure()
    rlocus(sys);
    title("Bank Angle (General Plot)");
    figure()
    rlocus(sys, bank_angle_gain);
    title("Bank Angle (With Gain)");

%     roll_step_response_info = stepinfo(sys);
    figure()
    step(sys);
end

disp("Model Was Trimmed.");

%% Run the nonlinear simulink model
simout = sim('fv_sim', 'TimeOut', flight_time_s);

%% Get simulation outputs
tout                    = simout.tout;
ExE_BfromE_m            = simout.ExE_BfromE_m;
euler_angles_NfromB_deg = rad2deg(simout.euler_angles_NfromB_rad);

%% ===== Post Processing =====
if plots_on
    
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

end