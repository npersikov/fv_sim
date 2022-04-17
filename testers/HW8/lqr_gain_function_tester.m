clc
clear
close all

%% Tester for get_lqr_k.m

% Some test system
A = [3, 4; 5, 1];
B = [1; 1.5]; 
C = eye(2); % assume perfect sensors
D = B*0;
x = [1, 1]; % current state
x_0 = [0, 0]; % desired state (trim)
dx = x_0 - x; % offset from trim point

% real system
A = [0	-4.52773960631005e-13	-4.10454005853045e-14	-4.77973728873867e-07	73.2495196713506	0.214899103398165	-3.23815048848549e-09	14439.1801713786	14447.9901234538;
4.52773960631005e-13	0	-1.40354644224315e-12	13.9662331560387	-0.417236114491431	155.891574152558	-14438.3782007635	-1.84696500295748e-09	-2882.93499895245;
4.10454005853045e-14	1.40354644224315e-12	0	-0.0386716300776773	-3774.30591095152	-10.2686645098357	-14447.9892536653	2823.79777091024	-0.0125108679897326;
0	0	0	0	0	0	1.00000000000000	0	0;
0	0	0	0	0	0	0	1	0;
0	0	0	0	0	0	0	0	1;
0	0	0	5.14984806974018	-0.494741884873432	182.303083898453	-125.153406709857	-4.54103269655850e-09	8.32539820771092;
0	0	0	-0.393780192976239	-72015.8715460462	-200.837333863055	0.0259000368902293	-1250.27891924836	-0.372561455130791;
0	0	0	-40.4241149467574	3.88351337481608	-1431.00159827211	2.55103639491530	1.69890810070191e-08	-38.1984171042037];
B = [-0.0445748612321127;
0;
-1.60857423525465;
0;
0;
0;
0;
-47.7847294114776;
0];
% A = magic(9);
% B = magic(9);
% B = B(:,1);
C = [6.65982746987130e-05	0	1.33172509320504e-05	0	0	0	0	0	0;
0	0	0	0	1	0	0	0	0];
D = [0;0];

% Classical controls outputs
[num, den] = ss2tf(A, B, C, D);
% rlocus(num(1, :), den);

% Weight Matrices for LQR controller
Q = 3*eye(length(A)); % Make this larger for aggressive correction
R = eye(length(B(1,:)));
N = zeros(length(A), length(B(1,:)));

% Q = 3*magic(length(A)); % Make this larger for aggressive correction
% R = magic(length(B(1,:)));
% N = zeros(length(A), length(B(1,:)));

% Initial Guesses for P and S
% P_0 = ones(length(x));
% S_0 = [1];
P_0 = [2, 3; 4, 5];
S_0 = [1, 5.5;3, 5.6];

[K,S,e] = lqr(A,B,Q,R,N);
% K = get_lqr_k(A, B, C, Q, R, x, P_0, S_0);

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