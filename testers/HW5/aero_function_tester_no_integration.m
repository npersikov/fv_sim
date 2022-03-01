clc
clear
close all

% Add all custom libraries to the workspace
addpath('C:\Users\Nikita\Documents\USC\Homework\Spring 2022\Flight Vehicle Stability and Control\fv_sim/fv_sim/');

%% As of 2/28/2022
% X force (drag) is positive most of the time. This is wrong.


% Use these if comparing against velocity magnitude
% velocities = -100:10:100;
% vB_BfromE_mps_list = [velocities; zeros(1,length(velocities)); zeros(1,length(velocities))];

% Use these if comparing for angle of attack
velocities = -20:1:20;
vB_BfromE_mps_list = [100*ones(1,length(velocities)); zeros(1,length(velocities)); velocities];

vB_AfromN = [0;0;0]; % zero gust
C_BfromN = angle2dcm(0,0,0); % no rotation
rho = 1.225;
mass = 0.48; %kg
beta = 0;

total_time = 10; %seconds
dt = 0.01; % timestep
time = 0:dt:total_time;

%% Multiple runs for trends
forces = zeros(3,length(velocities));
moments = zeros(3,length(velocities));
alphas = zeros(4,length(velocities));
for velocity_index = 1:1:length(velocities)

vB_BfromE_mps = vB_BfromE_mps_list(:,velocity_index);

[total_force_b_N, total_moment_b_Nm, alpha_surfs] =...
    aero_forces_and_moments(vB_BfromE_mps, vB_AfromN, C_BfromN, rho, beta);

forces(:,velocity_index) = total_force_b_N;
moments(:,velocity_index) = total_moment_b_Nm;
alphas(:,velocity_index) = alpha_surfs;

end

%% Multiple runs for trends
forces = zeros(3,length(velocities));
moments = zeros(3,length(velocities));
alphas = zeros(4,length(velocities));
for velocity_index = 1:1:length(velocities)

vB_BfromE_mps = vB_BfromE_mps_list(:,velocity_index);

[total_force_b_N, total_moment_b_Nm, alpha_surfs] =...
    aero_forces_and_moments(vB_BfromE_mps, vB_AfromN, C_BfromN, rho, beta);

forces(:,velocity_index) = total_force_b_N;
moments(:,velocity_index) = total_moment_b_Nm;
alphas(:,velocity_index) = alpha_surfs;

end

%% post processing
figure();
plot(velocities(1,:), forces(:,:));
title("Body force components vs z velocity");
legend(["x","y","z"]);

figure();
plot(velocities(1,:), moments(:,:));
title("Body moment components vs z velocity");
legend(["x","y","z"]);

figure();
plot(velocities(1,:), rad2deg(alphas(:,:)));
title("Angles of Attack vs z velocity");
legend(["elevator", "rudder", "left wing", "right wing"]);

figure();
plot(rad2deg(alphas(4,:)), forces(:,:));
title("Body force components vs wing AoA");
legend(["x","y","z"]);

figure();
plot(rad2deg(alphas(4,:)), moments(:,:));
title("Body moment components vs wing AoA");
legend(["x","y","z"]);


%% IF want to run only once
% [total_force_b_N, total_moment_b_Nm, alpha_surfs] =...
%     aero_forces_and_moments([0;0;0], vB_AfromN, C_BfromN, rho, beta);


%% Functions
% vB_BfromE_mps -> velocity of the body in ECEF
% vB_AfromN -> velocity of the air/gust in NED
% C_BfromN -> DCM from body frame to NED frame
% rho -> local air density
% v_inf
function [total_force_b_N, total_moment_b_Nm, alpha_surfs] =...
    aero_forces_and_moments(vB_BfromE_mps, vB_AfromN, C_BfromN, rho, beta)

    % Get mass properties
    [cg_m, ~, ~] = get_mass_props;
    
    % TODO think of a better way to do this
    % Get aerodynamic design parameters
    [surfs, aero] = const.aero_design_params;

    % Incidence angles of each surface
    i_surfs = aero(:, 3)';

    % Get parameters of the planform
    % These are lists with the order: elevator, rudder, left wing, right
    % wing.
    [surf_pos_m, c_m, ~, S_m2, AR] = get_planform_params;

    % The airspeed of the plane, taking gust speed into account. Used to
    % calculate angle of attack of each individual control surface.
%     vB_BfromA_mps = vB_BfromE_mps + C_BfromN*vB_AfromN; 

    % The airspeed of the plane, NOT taking gust speed into account. Used 
    % to calculate angle of attack of each individual control surface.
    vB_BfromA_mps = vB_BfromE_mps;
    v_inf = norm(vB_BfromA_mps); % QUESTION: is this true??

    % Allocate array for angles of attack
    alpha_surfs = zeros(1,4); 

    %TODO: try this without the for loop
    % use the formula from lecture 6 to calculate each surface's AoA
    for i = 1:1:length(aero(:,1)) 
        dotprod = dot(vB_BfromA_mps,surfs(:,i));
        if norm(vB_BfromA_mps) == 0 % If the plane is not miving WRT the air (no gusts or velocity), forces are 0.
            total_force_b_N     = [0;0;0];
            total_moment_b_Nm   = [0;0;0];
            alpha_surfs = i_surfs;
            return;
        else
            alpha_surfs(i) = i_surfs(i) - asin(dotprod/norm(vB_BfromA_mps));
        end
    end

    % Coefficient slopes
    C_La_surfs = 2*pi.*AR./(2+AR); % 3D lift slope approx. for each surface
    C_Da_surfs = aero(:,5)';
    C_Ma_surfs = aero(:,8)';

    % Coefficients at 0 AoA for each surface
    C_L0_surfs = aero(:,1);
    C_D0_surfs = aero(:,4);
    C_M0_surfs = aero(:,7);

    % AoA of 0 drag
    a0_surfs = aero(:,6)';

    % Calculate all coefficients for each aerodynamic surface
    C_Ls_surfs = C_L0_surfs' + C_La_surfs.*alpha_surfs;
    C_Ds_surfs = C_D0_surfs' + C_Da_surfs.*(alpha_surfs - a0_surfs).^2 +...
        C_Ls_surfs.^2./(pi.*aero(:,2)'.*AR);
    C_Ms_surfs = C_M0_surfs' + C_Ma_surfs.*alpha_surfs;

    % Calculate lift and drag of each surface. These are scalars.
    lifts_surfs_N       = 0.5*rho*v_inf^2.*C_Ls_surfs.*S_m2;
    drags_surfs_N       = 0.5*rho*v_inf^2.*C_Ds_surfs.*S_m2;

    % Moments due to the surfaces themselves (different from moments_s_Nm)
    moments_surfs_Nm    = 0.5*rho*v_inf^2.*C_Ms_surfs.*S_m2.*c_m; 

    % Initialize total force and moment
    total_force_b_N       = [0;0;0];
    total_moment_b_Nm     = [0;0;0];
    utility_matrix  = [0,0,0; 0,0,-1; 0,1,0]; % for moment calculation
    for i = 1:1:length(aero(:,1)) 
        % Calculate net force vector on the surface in wind frame
        force_surf_w = lifts_surfs_N(i)*surfs(:,i) - [drags_surfs_N(i);0;0];

        C_BfromW = get_C_BW(alpha_surfs(i), beta);

        % Calculate net force vector on the surface in body frame
        force_surf_b = C_BfromW*force_surf_w;

        % Calculate net moment vector on the surface in the wind frame.
        % Moments on the surfaces (different from moments_surfs_Nm)
        moment_due_to_moment = ...
            moments_surfs_Nm(i)*utility_matrix*surfs(:,i);
        moment_arm = (surf_pos_m(i,:) - cg_m);
        moment_due_to_force = ...
            cross(moment_arm, force_surf_b)';
        moment_s_Nm = moment_due_to_moment + moment_due_to_force;

        % Calculate net moment vector on the surface in the body frame
        % NOTE: Theyre the same because of surface vector definition
        moment_b_Nm = moment_s_Nm; 

        % Total the body forces and moments
        total_force_b_N       = total_force_b_N + force_surf_b;
        total_moment_b_Nm     = total_moment_b_Nm + moment_b_Nm;
    end

    C_D_fuse        = 0.5;
    S_fuse          = 0.001;
    total_force_b_N = total_force_b_N - [0.5*rho*v_inf^2*C_D_fuse*S_fuse;0;0];
end

function C_BfromW = get_C_BW(alpha, beta)
    % Get DCM from wind frame to body frame
    % DCM from stability to body coordinate system
    C_SfromB = [cos(alpha)   0   sin(alpha) ;
                0            1   0         ;
                -sin(alpha)  0   cos(alpha)];
        
    % DCM from wind to stability coordinate system
    C_WfromS = [cos(beta)   sin(beta)   0 ;
                -sin(beta)  cos(beta)   0 ;
                0           0           1];
     
    C_WfromB = C_WfromS * C_SfromB;  % DCM from wind to body
    C_BfromW = C_WfromB';
end

