%% AME 532a HW 3 Problem 1 
clc
clear
close all

% Add all custom libraries to the workspace
addpath('fv_sim/fv_sim/user_defined_libraries');

%% Part b

% Radius of Earth
re_m = 6378000;

euler_angles_NfromB_deg = [0; 0; 0]; % Body frame same as NED