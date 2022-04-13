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
    % Find P and S matrices using glorified trial and error
    [P, S] = fminsearch(@(P, S)lyap_sys(A, B, C, P, Q, R, S, x), [P_0, S_0]);
    % Calculate K now that all matrices are known
    K = calc_K(B, C, P, R, S);
end

% The functions that must be solved for P and S
function [f1, f2] = lyap_sys(A, B, C, P, Q, R, S, x)
    K = calc_K(B, C, P, R, S);
    f1 = A'*P + P*A + Q + C'*K'*R*K*C;
    f2 = A*S + S*A' + x;
end

% Utility function to calculate K from other matrices
function K = calc_K(B, C, P, R, S)
    K = R\B'*P*S*C'/(C*S*C'); % Definition of the LQR gain matrix
end