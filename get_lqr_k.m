% Get the K gain matrix for an LQR controller using a linear system (or 
% linearized model) defined by matrices A, B, and C.
% 
% The Q and R (state and control weight) matrices are needed as well. These
% should be defined by the designer based on desired controller behavior.
% For example, if you want state to be controlled quickly regardless of
% actuator use, make the state weight matrix Q larger. If you don't want to
% use the actuator that much (for example, to save power), and are OK with
% the system reaching the set point slowly, make R larger.
%
% Finally, you need x, which is the deviation from the trim state.
function K = get_lqr_k(A, B, C, Q, R, x, P_0, S_0)
    ps_0 = set_ps(P_0, S_0);
    n = length(P_0);
    m = length(S_0);
    % Find P and S matrices using glorified trial and error
    ps = fminsearch(@(ps)lyap_sys(A, B, C, Q, R, x, n, m, ps), ps_0);
    [P, S] = get_ps(ps, n, m);
    % Calculate K now that all matrices are known
    K = calc_K(B, C, P, R, S);
end

% The functions that must be solved for P and S
function [f1, f2] = lyap_sys(A, B, C, Q, R, x, n, m, ps)
    [P, S] = get_ps(ps, n, m);
    K = calc_K(B, C, P, R, S);
    f1 = A'*P + P*A + Q + C'*K'*R*K*C;
    f2 = A*S + S*A' + x;
end

% Utility function to calculate K from other matrices
function K = calc_K(B, C, P, R, S)
    K = R\B'*P*S*C'/(C*S*C'); % Definition of the LQR gain matrix
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