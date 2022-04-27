clc
clear
close all

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
% cg_m = [-0.496987516; 0; -0.018755961]; % Real aircraft cm
cg_m = [1.54-1.75; 0; -0.018755961]; % Real aircraft cm

alpha_test_range = -3:0.25:3;
alphas          = zeros(4, length(alpha_test_range));
pitch_moments   = zeros(4, length(alpha_test_range));
total_pitch_moments = zeros(1, length(alpha_test_range));
up_vel_index = 1;

for up_vel = alpha_test_range


% NOTE later these will be function inputs
% ========== State Vectors ==========
% NOTE this is actually v_B_BfromA, which must be properly determined when 
% inputting it into this function
v_mps = [20; 0; up_vel]; % Placeholder. 
omega_BfromE_radps = [0; 0; 0];

% ========== Allocate arrays ==========
% This array stores net force vectors for each control surface
aero_forces     = zeros(3, 4);
aero_moments    = zeros(3, 4);

% ========== Calculate forces and moments for each control surface ========
for aero_surface = 1:1:4 % There are assumed to be only 4 aero surfaces
    % Quantities that must be determined for each control surface in this
    % layer of the code. Stuff like incidence angle will be determined
    % where it is directly required (for organizational purposes).
    S_m2                        = get_S(aero_surface);
    chord_m                     = get_c(aero_surface);
    position_m                  = get_pos(aero_surface) + 0.25*get_c(aero_surface)*[1;0;0]; % Apply quarter chord rule
    [C_L, C_D, C_M, alpha]      = get_coeffs_linearized(aero_surface, v_mps, omega_BfromE_radps, position_m); % v_mps is needed to determine alpha

    alphas(aero_surface, up_vel_index) = alpha; % Save off angles of attack for plotting

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

pitch_moments(:, up_vel_index) = aero_moments(2,:)'; % save off pitch moment for plotting
total_pitch_moments(up_vel_index) = total_moment_b_Nm(2);

up_vel_index = up_vel_index + 1;

end

% plot data for debugging
surfaces = ["a_{el}", "a_{ru}", "a_{rw}", "a_{lw}"];
figure()
for plotnum = 1:1:4
    subplot(2,2, plotnum)
    plot(alphas(plotnum, :), pitch_moments(plotnum, :));
    xlabel("surface alpha");
    ylabel("surface pitch moment");
    title(surfaces(plotnum));
    hold on
    grid on
end

figure()
plot(mean(alphas), total_pitch_moments)
title("total pitching moment vs average alpha");
xlabel("average alpha");
ylabel("total pitch moment");
grid on



% ========== Functions ==========

% NOTE: surface order is horizontal stabilizer, vertical stabilizer, right
% wing, left wing

function [C_L, C_D, C_M] = get_coeffs(aero_surface, v_mps)

    if norm(v_mps) == 0
        alpha = 0;
    else
        alpha = get_alpha(aero_surface, v_mps)*180/pi;
    end

    % Ratios of 4412 to 4424 airfoils in the main wing
    ratio_4412 = 0.39;
    ratio_4424 = 0.71;

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
    
    cd_vs_a_4412 = [-9.476102941	1.070291262	;
                    -8.979779412	1.054757282	;
                    -8.483455882	0.977087379	;
                    -7.766544118	0.942912621	;
                    -6.884191176	0.859029126	;
                    -6.443014706	0.793786408	;
                    -5.78125	    0.728543689	;
                    -5.395220588	0.672621359	;
                    -5.174632353	0.36815534	;
                    -4.126838235	0.281165049	;
                    -2.362132353	0.228349515	;
                    -1.09375	    0.187961165	;
                    1.773897059	    0.175533981	;
                    5.799632353	    0.212815534	;
                    8.612132353	    0.231456311	;
                    9.273897059	    0.250097087	;
                    9.715073529	    0.274951456	;
                    11.03860294	    0.377475728	;
                    12.80330882	    0.504854369	;
                    14.18198529	    0.644660194	;
                    15.28492647	    0.809320388	;
                    15.89154412	    0.980194175	;
                    16.55330882	    1.200776699	;
                    17.02205882	    1.408932039	;];

    cm_vs_a_4412 = [-9.428571429	-0.038095238	;
                    -9.028571429	-0.039047619	;
                    -8.628571429	-0.035238095	;
                    -7.828571429	-0.02952381	;
                    -7.428571429	-0.032857143	;
                    -7.085714286	-0.022619048	;
                    -6.8	        -0.02047619	;
                    -6.4	        -0.026904762	;
                    -5.942857143	-0.022380952	;
                    -5.714285714	-0.031190476	;
                    -5.457142857	-0.03202381	;
                    -5.228571429	-0.060714286	;
                    -4.4	        -0.072380952	;
                    -3.6	        -0.080952381	;
                    -2.771428571	-0.088571429	;
                    -1.257142857	-0.099047619	;
                    -0.685714286	-0.102619048	;
                    0.114285714	    -0.104761905	;
                    0.8	            -0.10547619	;
                    1.771428571	    -0.10547619	;
                    2.8	            -0.104404762	;
                    3.942857143	    -0.101904762	;
                    5.142857143	    -0.099047619	;
                    6.514285714	    -0.095	;
                    7.657142857	    -0.088571429	;
                    8.514285714	    -0.081190476	;
                    9.171428571	    -0.072380952	;
                    9.771428571	    -0.060238095	;
                    10.17142857	    -0.053809524	;
                    10.68571429	    -0.047619048	;
                    11.51428571	    -0.041785714	;
                    12.4	        -0.038809524	;
                    12.91428571	    -0.035714286	;
                    13.25714286	    -0.035238095	;
                    13.88571429	    -0.02952381	;
                    14.28571429	    -0.029166667	;
                    14.62857143	    -0.026190476	;
                    15.25714286	    -0.024404762	;
                    15.74285714	    -0.025833333	;
                    16.17142857	    -0.03047619	;
                    16.54285714	    -0.037142857	;
                    17.02857143	    -0.05	;];


    cl_vs_a_4424 = [-19.7338403	    -0.99112426	;
                    -18.93536122	-1.01183432	;
                    -15.43726236	-0.875739645	;
                    -12.20532319	-0.618343195	;
                    -9.69581749	    -0.316568047	;
                    -7.034220532	-0.133136095	;
                    -4.144486692	-0.026627219	;
                    3.688212928	    0.482248521	;
                    4.942965779	    0.795857988	;
                    6.80608365	    1.053254438	;
                    12.73764259	    1.281065089	;
                    16.3878327	    1.331360947	;
                    19.23954373	    1.322485207	;];

    cd_vs_a_4424 = [-19.7053407	    0.081532493	;
                    -17.9373849	    0.059010669	;
                    -16.24309392	0.041551891	;
                    -15.21178637	0.033608147	;
                    -13.55432781	0.024616877	;
                    -12.11786372	0.019379243	;
                    -10.64456722	0.016062076	;
                    -7.771639042	0.012919496	;
                    -3.94106814	    0.011522793	;
                    4.198895028	    0.012046557	;
                    5.856353591	    0.014839961	;
                    7.845303867	    0.017633366	;
                    10.71823204	    0.024093113	;
                    13.00184162	    0.034219205	;
                    15.69060773	    0.052463628	;
                    19.33701657	    0.089039767	;];

    cm_vs_a_4424 = [-19.04489016	-0.090576541	;
                    -15.95033429	-0.12332008	;
                    -14.38395415	-0.131550696	;
                    -12.70296084	-0.137276342	;
                    -11.02196753	-0.143359841	;
                    -9.570200573	-0.148369781	;
                    -8.500477555	-0.143717694	;
                    -7.659980898	-0.134771372	;
                    -6.131805158	-0.114373757	;
                    -3.075453677	-0.07	;
                    0.05730659	    -0.038866799	;
                    3.495702006	    -0.009522863	;
                    3.954154728	    -0.012385686	;
                    5.100286533	    -0.045666004	;
                    5.711556829	    -0.047992048	;
                    6.781279847	    -0.058906561	;
                    7.927411652	    -0.044592445	;
                    9.761222541	    -0.028489066	;
                    11.97707736	    -0.012027833	;
                    14.30754537	    0.000854871	;
                    16.25596944	    0.00693837	;
                    18.01337154	    0.009801193	;
                    19.23591213	    0.009801193	;];

    cl_vs_a_0006 = [-9.981343284	-0.57231365	;
                    -9.720149254	-0.592449177	;
                    -9.253731343	-0.699322362	;
                    -8.470149254	-0.721006776	;
                    -7.201492537	-0.711713456	;
                    -5.970149254	-0.628073572	;
                    -3.097014925	-0.380251694	;
                    -2.649253731	-0.29661181	;
                    -2.462686567	-0.222265247	;
                    0               0;
                    2.611940299	    0.217618587	;
                    3.02238806	    0.357018393	;
                    4.067164179	    0.44375605	;
                    6.156716418	    0.645111326	;
                    6.902985075	    0.688480155	;
                    8.320895522	    0.713262343	;
                    9.179104478	    0.691577928	;
                    9.888059701	    0.570764763	;];

    cd_vs_a_0006 = [-9.485294118	0.114191675	;
                    -6.599264706	0.057153921	;
                    -5.422794118	0.032061955	;
                    -4.0625	        0.018121975	;
                    -2.775735294	0.010919652	;
                    0.165441176	    0.009757986	;
                    3.216911765	    0.011151985	;
                    4.724264706	    0.022303969	;
                    5.790441176	    0.036708616	;
                    6.893382353	    0.064123911	;
                    9.246323529	    0.109893514	;];

    cm_vs_a_0006 = [-9.50617284	    0.018498024	;
                    -5.897435897	-0.010434783	;
                    -2.744539411	0.007233202	;
                    -2.32668566	    -0.007826087	;
                    -0.009496676	-1.39E-17	;
                    0               0;
                    2.57359924	    0.008063241	;
                    2.877492877	    -0.007114625	;
                    5.764482431	    0.010316206	;
                    9.449192783	    -0.015177866	;];

    if aero_surface == 1 % Horizontal Stabilizer, 0006
        C_L = interp1(cl_vs_a_0006(:,1), cl_vs_a_0006(:,2), alpha);
        C_D = interp1(cd_vs_a_0006(:,1), cd_vs_a_0006(:,2), alpha);
        C_M = interp1(cm_vs_a_0006(:,1), cm_vs_a_0006(:,2), alpha);

    elseif aero_surface == 2 % Vertical Stabilizer, 0006
        C_L = interp1(cl_vs_a_0006(:,1), cl_vs_a_0006(:,2), alpha);
        C_D = interp1(cd_vs_a_0006(:,1), cd_vs_a_0006(:,2), alpha);
        C_M = interp1(cm_vs_a_0006(:,1), cm_vs_a_0006(:,2), alpha);

    elseif aero_surface == 3 || aero_surface == 4 % Wing, 4412 and 4424

        % For these, since the wing uses two different airfoils, the net
        % coefficients are averages weighted by the areas.

        C_L = interp1(cl_vs_a_4412(:,1), cl_vs_a_4412(:,2), alpha)*...
            ratio_4412 + interp1(cl_vs_a_4424(:,1), ...
            cl_vs_a_4424(:,2), alpha)*ratio_4424;

        C_D = interp1(cd_vs_a_4412(:,1), cd_vs_a_4412(:,2), alpha)*...
            ratio_4412 + interp1(cd_vs_a_4424(:,1), ...
            cd_vs_a_4424(:,2), alpha)*ratio_4424;

        C_M = interp1(cm_vs_a_4412(:,1), cm_vs_a_4412(:,2), alpha)*...
            ratio_4412 + interp1(cm_vs_a_4424(:,1), ...
            cm_vs_a_4424(:,2), alpha)*ratio_4424;

    else % In case someone inputs an aero surface that doesn't exist
        error("The aero surface does not exist");
    end  
end


function [C_L, C_D, C_M, alpha] = get_coeffs_linearized(aero_surface, v_mps, omega_BfromE, x_surf_from_cg)

    if norm(v_mps) == 0
        alpha = 0;
    else
        alpha = get_alpha(aero_surface, v_mps, omega_BfromE, x_surf_from_cg)*180/pi;
    end

    % Ratios of 4412 to 4424 airfoils in the main wing
    ratio_4412 = 0.6357;
    ratio_4424 = 0.3643;

    % No, these tables are not ugly.
    cl_vs_a_4412 =  [-4.8905804	    -0.321463897;
                    -2.692673644	0.037586548;
                    -0.294957184	0.398813056;
                    2.502378687	    0.742631058;
                    5.756422455	    1.073392681;];
    
    cd_vs_a_4412 = [-9.476102941	1.070291262	;
                    -8.979779412	1.054757282	;
                    -8.483455882	0.977087379	;
                    -7.766544118	0.942912621	;
                    -6.884191176	0.859029126	;
                    -6.443014706	0.793786408	;
                    -5.78125	    0.728543689	;
                    -5.395220588	0.672621359	;
                    -5.174632353	0.36815534	;
                    -4.126838235	0.281165049	;
                    -2.362132353	0.228349515	;
                    -1.09375	    0.187961165	;
                    1.773897059	    0.175533981	;
                    5.799632353	    0.212815534	;
                    8.612132353	    0.231456311	;
                    9.273897059	    0.250097087	;
                    9.715073529	    0.274951456	;
                    11.03860294	    0.377475728	;
                    12.80330882	    0.504854369	;
                    14.18198529	    0.644660194	;
                    15.28492647	    0.809320388	;
                    15.89154412	    0.980194175	;
                    16.55330882	    1.200776699	;
                    17.02205882	    1.408932039	;];

    cm_vs_a_4412 = [-1.257142857	-0.099047619	;
                    -0.685714286	-0.102619048	;
                    0.114285714	    -0.104761905	;
                    0.8	            -0.10547619	;];


    cl_vs_a_4424 = [-4.144486692	-0.026627219	;
                    3.688212928	    0.482248521	;
                    4.942965779	    0.795857988	;];

    cd_vs_a_4424 = [-19.7053407	    0.081532493	;
                    -17.9373849	    0.059010669	;
                    -16.24309392	0.041551891	;
                    -15.21178637	0.033608147	;
                    -13.55432781	0.024616877	;
                    -12.11786372	0.019379243	;
                    -10.64456722	0.016062076	;
                    -7.771639042	0.012919496	;
                    -3.94106814	    0.011522793	;
                    4.198895028	    0.012046557	;
                    5.856353591	    0.014839961	;
                    7.845303867	    0.017633366	;
                    10.71823204	    0.024093113	;
                    13.00184162	    0.034219205	;
                    15.69060773	    0.052463628	;
                    19.33701657	    0.089039767	;];

    cm_vs_a_4424 = [-6.131805158	-0.114373757	;
                    -3.075453677	-0.07	;
                    0.05730659	    -0.038866799	;
                    3.495702006	    -0.009522863	;
                    3.954154728	    -0.012385686	;];

    cl_vs_a_0006 = [-5.970149254	-0.628073572	;
                    -3.097014925	-0.380251694	;
                    -2.649253731	-0.29661181	;
                    -2.462686567	-0.222265247	;
                    0               0;
                    2.611940299	    0.217618587	;
                    3.02238806	    0.357018393	;
                    4.067164179	    0.44375605	;];

    cd_vs_a_0006 = [-9.485294118	0.114191675	;
                    -6.599264706	0.057153921	;
                    -5.422794118	0.032061955	;
                    -4.0625	        0.018121975	;
                    -2.775735294	0.010919652	;
                    0.165441176	    0.009757986	;
                    3.216911765	    0.011151985	;
                    4.724264706	    0.022303969	;
                    5.790441176	    0.036708616	;
                    6.893382353	    0.064123911	;
                    9.246323529	    0.109893514	;];

    cm_vs_a_0006 = [-2.32668566	    -0.007826087	;
                    -0.009496676	-1.39E-17	;
                    0               0;
                    2.57359924	    0.008063241	;];

    if aero_surface == 1 || aero_surface == 2 % Horizontal or Vertical Stabilizer, 0006
        C_L = interp1(cl_vs_a_0006(:,1), cl_vs_a_0006(:,2), alpha, 'linear', 'extrap');
        C_D = interp1(cd_vs_a_0006(:,1), cd_vs_a_0006(:,2), alpha, 'linear', 'extrap');
        C_M = interp1(cm_vs_a_0006(:,1), cm_vs_a_0006(:,2), alpha, 'linear', 'extrap');

    elseif aero_surface == 3 || aero_surface == 4 % Wing, 4412 and 4424

        % For these, since the wing uses two different airfoils, the net
        % coefficients are averages weighted by the areas.

        C_L = interp1(cl_vs_a_4412(:,1), cl_vs_a_4412(:,2), alpha, 'linear', 'extrap')*...
            ratio_4412 + interp1(cl_vs_a_4424(:,1), ...
            cl_vs_a_4424(:,2), alpha, 'linear', 'extrap')*ratio_4424;

        C_D = interp1(cd_vs_a_4412(:,1), cd_vs_a_4412(:,2), alpha, 'linear', 'extrap')*...
            ratio_4412 + interp1(cd_vs_a_4424(:,1), ...
            cd_vs_a_4424(:,2), alpha, 'linear', 'extrap')*ratio_4424;

        C_M = interp1(cm_vs_a_4412(:,1), cm_vs_a_4412(:,2), alpha, 'linear', 'extrap')*...
            ratio_4412 + interp1(cm_vs_a_4424(:,1), ...
            cm_vs_a_4424(:,2), alpha, 'linear', 'extrap')*ratio_4424;

    else % In case someone inputs an aero surface that doesn't exist
        error("The aero surface does not exist");
    end  
end



function S_m2 = get_S(aero_surface)
    Ss = [0.18, 0.09, 0.55, 0.55]; % Real aircraft aero surface areas
    S_m2 = Ss(aero_surface);
end

function chord_m = get_c(aero_surface)
    cs = [0.2, 0.2, 0.72, 0.72]; % Real aircraft chords
    chord_m = cs(aero_surface);
end

function position_m = get_pos(aero_surface)
% h. stab, v. stab, R wing, L wing
    poss = [-1.75, -1.75,  -0.34,    -0.34;   % Forward. 
             0,    0,    0.24, -0.24; % Right
             0,   -0.125,  0,    0];  % Down. Real aircraft positions

    position_m = poss(:, aero_surface);
end

function alpha = get_alpha(aero_surface, v_B_BfromA, omega_BfromE, x_surf_from_cg)
    surf_vec        = get_surf_vec(aero_surface);
    incidence_angle = get_i(aero_surface);

    v_eff = v_B_BfromA + cross(omega_BfromE, x_surf_from_cg);

    alpha = incidence_angle - asin(dot(v_eff, surf_vec)/norm(v_eff));
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
