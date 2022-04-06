%% NOTE: alpha and delta_e are both in degrees!
function C_L = elevator_contrib(alpha_deg, delta_e_deg)
    % interp1(x values, y values, desired x test values)
    % Interpolate all three tables for the desired alpha, then use those
    % three C_L values to interpolate with delta_e

    % These are the angles of attack corresponding to lift coefficients.
    alphas_down     = [-15.03311258, -3.774834437, 10.52980132, 15.16556291];
    alphas_neutral  = [-15.36423841, -3.377483444, 11.39072848, 14.83443709];
    alphas_up       = [-14.96688742, -2.649006623, 12.38410596, 14.70198675];

    cls_down        = [-0.316498316, -0.414141414, 1.168350168, 0.922558923];
    cls_neutral     = [-0.400673401, -0.464646465, 1.053872054, 0.784511785];
    cls_up          = [-0.474747475, -0.565656566, 0.929292929, 0.67003367];

    % In order, these are d_E = -10, 0, 10
    d_Es = [-10, 0, 10];
    cl_down = interp1(alphas_down, cls_down, alpha_deg, 'linear', 'extrap');
    cl_neutral = interp1(alphas_neutral, cls_neutral, alpha_deg, 'linear', 'extrap');
    cl_up = interp1(alphas_up, cls_up, alpha_deg, 'linear', 'extrap');
    C_Ls = [cl_up, cl_neutral, cl_down];
    
    C_L = interp1(d_Es, C_Ls, delta_e_deg, 'linear', 'extrap');
end