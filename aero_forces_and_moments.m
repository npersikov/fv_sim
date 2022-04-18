% This function will calculate aerodynamic forces on my plane. I am
% starting over because the last function was written all at once without
% testing. I will not make that mistake this time!

% ========== Useful geometric definitions ==========
forward = [1; 0; 0];
right   = [0; 1; 0];
down    = [0; 0; 1];
% NOTE The (0,0,0) point in the aircraft geometric coordinate system shall
% be the midpoint of the frontmost leading edge.

% TODO define these constants elsewhere
% ========== Physical Constants ==========
rho_kgpm3 = 1.225;

% ========== Overall Aircraft Characteristics ==========
% Vector from aircraft frame center to the cg
cg_m = [-0.496987516, 0, -0.018755961]; % Real aircraft cm

% NOTE later these will be function inputs
% ========== State Vectors ==========
% NOTE this is actually v_B_BfromA, which must be properly determined when 
% inputting it into this function
v_mps = [100; 0; -2]; % Placeholder. 

% ========== Allocate arrays ==========
% This array stores net force vectors for each control surface
aero_forces     = zeros(3, 4);
aero_moments    = zeros(3, 4);

% ========== Calculate forces and moments for each control surface ========
for aero_surface = 1:1:4 % There are assumed to be only 4 aero surfaces
    % Quantities that must be determined for each control surface in this
    % layer of the code. Stuff like incidence angle will be determined
    % where it is directly required (for organizational purposes).
    [C_L, C_D, C_M]     = get_coeffs(aero_surface, v_mps); % v_mps is needed to determine alpha
    S_m2                = get_S(aero_surface);
    chord_m             = get_c(aero_surface);
    position_m          = get_pos(aero_surface);
    
    % The force is the vector sum of the lift and drag vectors. The lift
    % coefficient is multiplied by the upward direction vector, and the 
    % drag coefficient is multiplied by the backward direction vector
    aero_forces(:,aero_surface) = 0.5*rho_kgpm3*(norm(v_mps)^2)*S_m2*(C_L*get_surf_vec(aero_surface) + C_D*-forward);

    % Utility matrix used to figure out about which axis the moment turns 
    % for  given aero surface.
    surf_to_moment_dir_matrix = [0, 0, 0;
                                 0, 0, -1;
                                 0, 1, 0];

    % Aerodynamic moment applied directly to the current surface
    pure_moment = 0.5*rho_kgpm3*(norm(v_mps)^2)*S_m2*C_M*chord_m*surf_to_moment_dir_matrix*get_surf_vec(aero_surface);

    % Aero moment due to aero force offset from cg
    dist_to_cg = position_m - cg_m;
    moment_force_contrib = cross(dist_to_cg, aero_forces(:, aero_surface));

    % The moment is calculated in two parts: the first is similar to the
    % lift/drag calculation, and the second is a moment due to forces on
    % individual surfaces acting on moment arms.
    aero_moments(:, aero_surface) = pure_moment + moment_force_contrib;
end

% ========== Final outputs: ==========
total_force_b_N     = sum(aero_forces,2);
total_moment_b_Nm   = sum(aero_moments,2);


% ========== Functions ==========

% NOTE: surface order is horizontal stabilizer, vertical stabilizer, right
% wing, left wing

function [C_L, C_D, C_M] = get_coeffs(aero_surface, v_mps)

    alpha = get_alpha(aero_surface, v_mps);

%     [C_L_alpha, C_D_alpha, C_M_alpha] = get_slopes(aero_surface);
%     [C_L0, C_D0, C_M0]                = get_coeff_offset(aero_surface);
% 
%     C_L = C_L_alpha*alpha + C_L0;
%     C_D = C_D_alpha*alpha + C_D0;
%     C_M = C_M_alpha*alpha + C_M0;

    % No, these tables are not ugly.
    cl_vs_a_4412 =  [-9.457659372	-0.34322453;
                    -8.80114177	    -0.380217606;
                    -7.602283539	-0.449851632;
                    -6.117982873	-0.484668645;
                    -5.604186489	-0.449851632;
                    -4.8905804	    -0.321463897;
                    -2.692673644	0.037586548;
                    -0.294957184	0.398813056;
                    2.502378687	    0.742631058;
                    5.756422455	    1.073392681;
                    8.26831589	    1.304055391;
                    8.839200761	    1.325816024;
                    9.410085633	    1.33016815;
                    10.4376784	    1.308407517;
                    11.29400571	    1.33016815;
                    13.00666032	    1.443323442;
                    14.26260704	    1.465084075;
                    15.23311132	    1.408506429;
                    15.97526166	    1.33016815;
                    17.05994291	    1.186547972];

    

end

function S_m2 = get_S(aero_surface)
    Ss = [0.18, 0.09, 0.55, 0.55]; % Real aircraft aero surface areas
    S_m2 = Ss(aero_surface);
end

function chord_m = get_c(aero_surface)
    cs = [0.2, 0.2, 0.72, 0.72]; %placeholders
    chord_m = cs(aero_surface);
end

function position_m = get_pos(aero_surface)
% h. stab, v. stab, R wing, L wing
    poss = [-1.75, -1.75,  -0.34,    -0.34;   % Forward. 
             0,    0,    0.24, -0.24; % Right
             0,   -0.125,  0,    0];  % Down. Real aircraft positions

    position_m = poss(:, aero_surface);
end

function alpha = get_alpha(aero_surface, v_B_BfromA)
    surf_vec        = get_surf_vec(aero_surface);
    incidence_angle = get_i(aero_surface);

    alpha = incidence_angle - asin(dot(v_B_BfromA, surf_vec)/norm(v_B_BfromA));
end

function incidence_angle = get_i(aero_surface)
    is = [-2.5*pi/180, 0, 0, 0]; % Real aircraft incidence angles
    incidence_angle = is(aero_surface);
end

function surf_vec = get_surf_vec(aero_surface)
    % h. stab, v. stab, R wing, L wing
    surf_vecs = [0,  0,  0,  0;  % Forward
                 0,  1,  0,  0;  % Right
                -1,  0, -1, -1]; % Down, placeholders

    surf_vec = surf_vecs(:, aero_surface);
end

function [C_L_alpha, C_D_alpha, C_M_alpha] = get_slopes(aero_surface)
    % h. stab, v. stab, R wing, L wing
    slopes = [0.2, 0.2, 0.2, 0.2;          % lift slopes
              0.002, 0.002, 0.002, 0.002;  % drag slopes
              0.002, 0.002, 0.002, 0.002]; % moment slopes, placeholders

    % One column is the three coefficients corresponding to one aero
    % surface
    surf_slopes = slopes(:, aero_surface);
    C_L_alpha = surf_slopes(1);
    C_D_alpha = surf_slopes(2);
    C_M_alpha = surf_slopes(3);
end

function [C_L0, C_D0, C_M0] = get_coeff_offset(aero_surface)
    % NOTE that the intercepts for the elevator and rudder are zero because
    % they are symmetrical airfoils.
    % h. stab, v. stab, R wing, L wing
    inters = [0, 0, 0.002, 0.002;    % lift slope intercepts
              0, 0, 0.0002, 0.0002;  % drag slope intercepts
              0, 0, 0.0002, 0.0002]; % moment slope intercepts, placeholders

    % One column is the three coefficient intercepts corresponding to one 
    % aero surface
    surf_inters = inters(:, aero_surface);
    C_L0 = surf_inters(1);
    C_D0 = surf_inters(2);
    C_M0 = surf_inters(3);
end
