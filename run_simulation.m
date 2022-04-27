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

m_kg = 7;
cg_m = [1.54-1.75; 0; -0.018755961]; % My aircraft
J_kgm2 = 20*J_kgm2;

% Actuator models
% order: elevator, rudder, right aileron, left aileron
time_constant_matrix = 1*eye(4); % TODO figure out a better way to calculate these

%% ===== Initial Conditions =====

% Vehicle State
lat_d           = 35.28; % N35.28
lon_d           = -115;  % W115;
ground_level_m  = 995;  % 995 for N35.28 W115 7968 Found through trial and error for N35.28 W-115
altitude_m      = 100; 
thrust_N        = [10*0;0;0];

ExE_BfromE_0_m  = lla2ecef([lat_d, lon_d, ground_level_m + altitude_m])'; % SoCal
EvB_BfromE_mps  = [20; 0; -2]; % Velocity of the body in ECEF frame in the body CS
omega_BwrtN_dps = [0; 0; 0]; % roll pitch yaw rates, or phi theta psi rates. (IE, rotate about the down axis)
omega_BwrtN_rps = deg2rad(omega_BwrtN_dps);
omega_pure_quat = [0; omega_BwrtN_rps]';

% Frames and Coordinate Systems
ll_NfromE_deg               = [lat_d, lon_d]; % Lat lon and euler angles for DCMs. Note: N is North East Down; Need inv of this DCM
ll_NfromE_rad               = deg2rad(ll_NfromE_deg); %TODO: figure out where to handle degree changes
euler_angles_BfromN_0_deg   = [0.017*0; 0.017*0; 0.017*0]; % Body frame same as NED. Note: C_EfromB is found in simulink. NOTE: must be ZYX rotation!
euler_angles_BfromN_0_rad   = deg2rad(euler_angles_BfromN_0_deg);
omega_EwrtI_rps_0           = [0; 0; 0]; % rotation of Earth
omega_BwrtI_rps_0           = omega_BwrtN_rps + omega_EwrtI_rps_0; % since initial N to E velocity is 0;0;0

% Other states:
% Moment applied by the two paylaod release stepper motors. It is applied
% to the y axis, and they spin forwards, which adds a positive reaction
% moment. Turning them on and switching direction will be handled in the
% simulink model.
payload_release_motor_moment_Nm = [0; 1.08*0; 0]; 

%% Control Inputs
aoa_step_time = 10;
aoa_step_value = -5*pi/180*0;
bank_step_time = 10;
bank_step_value = 10*pi/180*0;

% Select which control systems are active:
is_controlling_pitch = true;
is_controlling_roll = false;

%% ===== Simulation =====

%% Linearize Model
% Linearize the model while ignoring translational motion. This linearized
% ABCD model will allow for stabilizing control system design for steady
% level flight.

% Set the control system gains
pitch_gain = -50*0;
roll_rate_gain = 10*0;
aoa_gain = 500*0; % The gain for angle of attack. Used to command the elevator deflection.
bank_angle_gain = 100*0; 

% Trim the simulink model
[x,u,y,dx] = trim('fv_sim_linearized', [EvB_BfromE_mps; euler_angles_BfromN_0_deg; omega_BwrtN_dps], 0, [0, atan2(EvB_BfromE_mps(1), EvB_BfromE_mps(3))]');
% The order that I think the states are in in the x vector
states = ["vel_x", "vel_y", "vel_z", "avel_x", "avel_y", "avel_z", "psi", "theta", "phi"];

% Get a state space model of the linearized system (ABCD matrices)
lin_sys = linmod('fv_sim_linearized', x, u);

% Get eigenvalues for linearized system
lin_eigs = eig(lin_sys.a);

% Get transfer functions from the state space model
[num_coeff, den_coeff] = ss2tf(lin_sys.a, lin_sys.b, lin_sys.c, lin_sys.d);

% Plot the root locus for each state
if rl_plots_on
%     for rl_plot_num = 1:1:length(x)
%         sys = tf(num_coeff, den_coeff(rl_plot_num,:)); 
%         figure()
%         rlocus(sys);
%         title(states(rl_plot_num));
%         disp("Eigenvalue for " + states(rl_plot_num) + ": " + lin_eigs);
%     end

    figure()
    plot(lin_eigs,'O','color','green')
    grid on 

    sys = tf(num_coeff(1,:), den_coeff); 
    figure()
    rlocus(sys);

    sys = tf(num_coeff(2,:), den_coeff); 
    figure()
    rlocus(sys);

    % Pitch Plots
%     sys = tf(num_coeff(8,:), den_coeff); 
%     figure()
%     rlocus(sys);
%     figure()
%     rlocus(sys, pitch_gain);

    % Roll Plots
%     sys = tf(num_coeff(6,:), den_coeff); 
%     figure()
%     rlocus(sys);
%     title("Roll Rate (General Plot)");
%     figure()
%     rlocus(sys, roll_rate_gain);
%     title("Roll Rate (With Gain)");
% 
%     sys = tf(num_coeff(9,:), den_coeff); 
%     figure()
%     rlocus(sys);
%     title("Bank Angle (General Plot)");
%     figure()
%     rlocus(sys, bank_angle_gain);
%     title("Bank Angle (With Gain)");
% 
% %     roll_step_response_info = stepinfo(sys);
%     figure()
%     step(sys);
end

% A = magic(9);
% B = magic(9);
% B = B(:,1);
% Weight Matrices for LQR controller
Q = 1*eye(length(x)) + 0.01 * ones(length(x)); % Make this larger for aggressive correction
R = 10*eye(length(u));
N = zeros(length(x),length(u));
% LQR control gains
% [K_lqr,~,~] = lqr(A, B, Q, R, N);

% [K_lqr,~,~] = lqr(lin_sys.a, lin_sys.b, Q, R, N);

% With thrust on, K_lqr can be determined. It is:
% K_lqr = [-3.02453481973423	0.200987337098554	0.297266261308987 ...
% -190130702.677928	-129205.574640224	-38024779.7004798 ...	
% -1564090.56175558 -29.0228747350303	-199250.589036954];
% The corresponding states for this K matrix are, in order:
% xyz velocity, angular position, angular velocity.

disp("Model Was Trimmed.");

%% Run the nonlinear simulink model
simout = sim('fv_sim', 'TimeOut', flight_time_s);

%% Get simulation outputs
tout                    = simout.tout;
ExE_BfromE_m            = simout.ExE_BfromE_m;
euler_angles_NfromB_deg = rad2deg(simout.euler_angles_NfromB_rad);

%% ===== Post Processing =====
if plots_on

    % This is the position of the plane in E frame and CS wrt E
    positions = squeeze(simout.ExE_BfromE_m.signals.values); 
    velocities = squeeze(simout.EvE_BfromE_mps.signals.values);
    
    % Get the time indeces of the (phugoid) steady state which is 
    % expected to start at 10 seconds
    time_ss = 30;
    time_array = simout.tout;

    positions_from_velocities = cumtrapz(time_array, velocities');
    initial_position = positions(:,1);

    simulation_time = time_array(end);
    t_index_ss = find(time_array > time_ss);
    i_t_0_ss = t_index_ss(1); % index of initial time of steady state
    i_t_f_ss = t_index_ss(length(t_index_ss)); % ^ for final time

    % Since end may work, delete i_t_f_ss?
    avg_velocity_from_velocities = mean(velocities(:, i_t_0_ss:end),2);
    avg_velocity_from_positions = (positions(:,end) - positions(:,i_t_0_ss))/(simulation_time - time_array(i_t_0_ss));
    % These should start at the initial position, not at (0,0,0);
%     avg_positions = avg_velocity_from_velocities*time_array(i_t_0_ss, end)' + initial_position;
    avg_positions = avg_velocity_from_positions*time_array(i_t_0_ss:end)';

    % plot the difference between actual position and would-be position if
    % the plane always flew at its average speed (as calculated after the
    % assumed steady-state time, generally 10 seconds)
%     plot3(positions(1,i_t_0_ss:end) - avg_positions(1,:), positions(2,i_t_0_ss:end) - avg_positions(2,:), time_array(i_t_0_ss:end));
    plot3(positions(1,i_t_0_ss:end) - initial_position(1) - avg_positions(1,:), positions(3,i_t_0_ss:end) - initial_position(3) - avg_positions(3,:), time_array(i_t_0_ss:end));
%     plot(positions(1,i_t_0_ss:end)- initial_position(1)-avg_positions(1,:), positions(3,i_t_0_ss:end)- initial_position(3) - avg_positions(3,:));
%     axis equal
    xlabel("real x position offset from time-averaged x position");
    ylabel("real z position offset from time-averaged z position");
    zlabel("time (s)");
    title("motion of plane relative to expected position " + ...
        "based on average velocity");
    grid on

    figure()
    relevant_times = time_array(i_t_0_ss:end);
    x_downrange = positions(1,i_t_0_ss:end) - initial_position(1);
%     plot(relevant_times, x_downrange);
%     hold on 
%     plot(relevant_times, 3.771*relevant_times - 118.2);
%     grid on
%     xlabel("time");
%     ylabel("x position, real and trendline");
% %     zlabel("time (s)");
%     title("motion of plane relative to expected position " + ...
%         "based on average velocity");

%     figure()
    plot(relevant_times, positions(3,i_t_0_ss:end)- initial_position(3) - avg_positions(3,:));
    title("diff in averaged and real y vs time")
        
end