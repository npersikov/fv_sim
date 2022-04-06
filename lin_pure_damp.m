%% Get pure damping feedback response for a linearized model
% Input is the model name and a gain value
% Output is the initial TF numerator and denominator coefficients, and the
% final ones.
function [num_coeff, den_coeff, lin_eigs] = lin_pure_damp(model_name, pitch_kd)
    % Trim the simulink model
    pitch_gain = pitch_kd;
    [x,u,y,dx] = trim(model_name);

    % Get a state space model of the linearized system (ABCD matrices)
    argout = linmod('fv_sim_linearized', x, u);
    
    % Get eigenvalues for linearized system
    lin_eigs = eig(argout.a);
    
    % Get transfer functions from the state space model
    [den_coeff, num_coeff] = ss2tf(argout.a, argout.b, argout.c, argout.d);
end