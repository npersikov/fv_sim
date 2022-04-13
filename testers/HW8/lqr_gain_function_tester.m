clc
clear
close all

%% Tester for get_lqr_k.m

% Some test system
A = [3, 4; 5, 1];
B = [1; 1.5]; 
C = eye(2); % assume perfect sensors
x = [1, 1]; % current state
x_0 = [0, 0]; % desired state (trim)
dx = x_0 - x; % offset from trim point

% Classical controls outputs
[num, den] = ss2tf(A, B, C, B*0);
rlocus(num(1, :), den);

% Weight Matrices for LQR controller
Q = 3*eye(2); % Make this larger for aggressive correction
R = eye(2);

% Initial Guesses for P and S
P_0 = ones(length(x));
S_0 = [1];

K = get_lqr_k(A, B, C, Q, R, x, P_0, S_0);
