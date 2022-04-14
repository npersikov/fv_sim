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
% rlocus(num(1, :), den);

% Weight Matrices for LQR controller
Q = 3*eye(2); % Make this larger for aggressive correction
R = eye(1);
N = zeros(2,1);

% Initial Guesses for P and S
% P_0 = ones(length(x));
% S_0 = [1];
P_0 = [2, 3; 4, 5];
S_0 = [1, 5.5;3, 5.6];

% [K,S,e] = lqr(A,B,Q,R,N);
K = get_lqr_k(A, B, C, Q, R, x, P_0, S_0);

function ps = set_ps(P, S)
    ps = [];
    n = length(P);
    m = length(S);
    for i = 1:1:n
        ps = [ps, P(i, 1:n)];
    end
    for i = 1:1:m
        ps = [ps, S(i, 1:m)];
    end
end

function [P, S] = get_ps(ps, n, m)
    % Throw error if ps has the wrong number of elements based on the
    % dimensions of P and S, namely nxn and mxm
    if length(ps) ~= n^2 + m^2
        error("ps has an improper amount of elements");
    end
    
    % Allocate P and S and set them to their proper dimensions
    P = zeros(n);
    S = zeros(m);
    for i = 1:1:length(ps)
        if i <= n^2
            % Get the row of P we are on for current i
            row = floor((i-1)/n)+1; 
            % i minus number of full rows passed should give the column
            col = i - floor((i-1)/n)*n ;
            P(row, col) = ps(i); % Set the value in the right place in P
        else
            % Get the row of S we are on for current i
            row = floor((i-n^2-1)/m)+1; 
            % i minus number of full rows passed should give the column
            col = i-n^2 - floor((i-n^2-1)/m)*m ;
            S(row, col) = ps(i); % Set the value in the right place in S
        end
    end
end