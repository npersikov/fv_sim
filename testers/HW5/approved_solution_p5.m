%% Tester I wrote myself
clc
clear
close all

vB_BfromE_mps = [100;0;0]; % velocity of 100 m/s forward
vB_AfromN = [0;0;0]; % zero gust
C_BfromN = angle2dcm(0,0,0); % no rotation
rho = 1.225;
mass = 0.48; %kg
total_time = 10; %seconds
dt = 0.01; % timestep
time = 0:dt:total_time;

drags = time;
lifts = time;
side_forces = time;
alphas = [time;time;time;time];
C_Ls = [time;time;time;time];
lifts_surfs_b = [time;time;time;time];
lifts_surfs_w = [time;time;time;time];
airspeeds = [time;time;time];

% Extra inputs needed to run his function


for ii = 1:1:length(time)

[cg_m, ~, ~] = get_mass_props;

% TODO think of a better way to do this
% Get aerodynamic design parameters
[surfs, aero] = const.aero_design_params;

% Incidence angles of each surface
i_surfs = aero(:, 3)';
u_in = surfs;

[surf_pos_m, c_m, ~, S_m2, AR] = get_planform_params;

% The airspeed of the plane, taking gust speed into account. Used to
% calculate angle of attack of each individual control surface.
vB_BfromA_mps = vB_BfromE_mps + C_BfromN*vB_AfromN;
v_inf = norm(vB_BfromA_mps); % QUESTION: is this true??
v_B_B_BA = vB_BfromA_mps;
om_B_B_BE = [0;0;0];
x_CM_B = c_m;

% Variable structure descriptions:
% AeroControlSurfs = [hingepointx, y, z, ,,,,,,,,,,,,,,i_surf]; % four rows, one for each control surface


    [total_force_b_N, M_ext_B, VisualizationData] = ExternalForcesAndMoments( ...
    u_in, ...
    v_B_B_BA, ...
    om_B_B_BE, ...
    rho, ...
    x_CM_B, ...
    AeroControlSurfs, ...
    AeroSurfs, ...
    StallModel_AeroControlSurfs, ...
    StallModel_AeroSurfs, ...
    TiltModel, ...
    Props, ...
    PropTables, ...
    MaxTurbulence, ...
    SeedVal, ...
    blownWingFlags, ...
    blownWingOption);

acceleration = total_force_b_N./mass;
vB_BfromE_mps = vB_BfromE_mps + acceleration*dt;

drags(ii) = total_force_b_N(1); % seems correct
lifts(ii) = total_force_b_N(3); % seems wrong
side_forces(ii) = total_force_b_N(2); % seems correct
alphas(:, ii) = alpha_surfs'; % seems wrong
C_Ls(:, ii) = C_Ls_surfs'; % seems wrong
airspeeds(:,ii) = vB_BfromA_mps;

end




%% Approved function
function [F_ext_B, M_ext_B, VisualizationData] = ExternalForcesAndMoments( ...
    u_in, ...
    v_B_B_BA, ...
    om_B_B_BE, ...
    rho, ...
    x_CM_B, ...
    AeroControlSurfs, ...
    AeroSurfs, ...
    StallModel_AeroControlSurfs, ...
    StallModel_AeroSurfs, ...
    TiltModel, ...
    Props, ...
    PropTables, ...
    MaxTurbulence, ...
    SeedVal, ...
    blownWingFlags, ...
    blownWingOption)
 
% Initialize the random function's seed value as a persistent variable
persistent rng_seed
if isempty(rng_seed)
    rng_seed = SeedVal;
    rng(rng_seed);
end
 
% Air velocity components needed for alpha/beta calculations
vx = v_B_B_BA(1);
vy = v_B_B_BA(2);
vz = v_B_B_BA(3);
 
% Determine Freestream velocity and angle of attack and sideslip
Vinf = norm(v_B_B_BA);
alpha = atan2(vz, vx);
beta = asin( max([-1, min([vy / Vinf, 1])]) ); % This means beta is constrained to +/- 90, which is not correct. Need atan2 with a projection.
 
% Set alpha and beta to zero if there is no air velocity
if Vinf == 0
    alpha = 0;
    beta = 0;
end
 
% DCM from stability to body CS
C_SB = [cos(alpha)   0   sin(alpha) ;
        0           1   0         ;
        -sin(alpha)  0   cos(alpha)];
    
% DCM from wind to stability CS
C_WS = [cos(beta)   sin(beta)   0 ;
        -sin(beta)  cos(beta)   0 ;
        0           0           1];
 
C_WB = C_WS * C_SB;  % DCM from wind to body
C_BW = C_WB';        % DCM from body to wing is the transpose of wind-body
 
% Simulink is weird and transposes 1-D vectors sometimes
if (size(AeroControlSurfs,1) == 1 || size(AeroControlSurfs,2) == 1)
    AeroControlSurfs = AeroControlSurfs';
end
nAeroControlSurfs = size(AeroControlSurfs,1);  % Number of aerodynamic surfaces with control surfaces
deflections = u_in(1:nAeroControlSurfs); % input vector of control surface deflection signals
 
TiltFlag = TiltModel(1); % Flag for whether there is a tilt wing mechanism
NumTiltControlSurfaces = TiltModel(2); % Number of control surfaces that are tilt-mech
NumTiltRotors = TiltModel(3); % Number of propellers affected by the tilt-mech
HingePoint = TiltModel(4:6); % Location of hinge point surfaces rotate about
 
% Simulink is weird and transposes 1-D vectors sometimes when Matlab
% doesn't.  This makes it work both ways.
sizeHinge = size(HingePoint);
if sizeHinge(1) == 1
    HingePoint = HingePoint';
end
 
% Determine if tilt-wing model or not
if TiltFlag == 1
    WingTilt = u_in(nAeroControlSurfs+1); % The tilt signal is between the surface signals and prop signals
else
    TiltFlag = 0;
    WingTilt = 0;
end
 
% Rotation Matrix moves components to proper location based on tilt angle
Tilt_DCM = [cos(WingTilt) 0  sin(WingTilt)
            0             1  0
           -sin(WingTilt) 0  cos(WingTilt)];
       
% Initialize force and moment vectors
F_ACsurf_B = zeros(3, nAeroControlSurfs);
M_ACsurf_B = zeros(3, nAeroControlSurfs);
dx_ACsurf_B = zeros(3, nAeroControlSurfs);
 
F_aeroControls_B = [0; 0; 0];
M_aeroControls_B = [0; 0; 0];
 
% Simulink is weird and transposes 1-D vectors sometimes
if (size(Props,1) == 1 || size(Props,2) == 1)
    Props2=Props';
else
    Props2 = Props;
end
nProps = size(Props2,1);
om_motors = u_in(nAeroControlSurfs + TiltFlag + 1:end);
 
% For now, all prop variables are just for slip stream calculations. They
% are initialized here
T_prop = zeros(nProps);
Q_prop = zeros(nProps);
P_prop = zeros(nProps);
F_prop_B = zeros(3, nProps);
M_prop_B = zeros(3, nProps);
dx_prop_B = zeros(3, nProps);
 
F_props_B = [0; 0; 0];
M_props_B = [0; 0; 0];
 
LiftSlip = 0;
LiftTest = 0;
 
for iACsurf = 1:nAeroControlSurfs
    
    % Incidence Due to Wing Tilt
    if iACsurf <= NumTiltControlSurfaces
        x_B_AeroRefHinge = AeroControlSurfs(iACsurf,1:3)' - HingePoint;
        X_B_AeroRefHinge_Tilt = Tilt_DCM * x_B_AeroRefHinge;
        x_AeroRef_B = X_B_AeroRefHinge_Tilt + HingePoint;
        
        i_surf = AeroControlSurfs(iACsurf,18) + WingTilt;
    else
        x_AeroRef_B = AeroControlSurfs(iACsurf,1:3)';
        
        i_surf = AeroControlSurfs(iACsurf,18);
    end
    
    % Geometry Terms
    Sref = AeroControlSurfs(iACsurf,9);
    AR = AeroControlSurfs(iACsurf,10);
    n_B = AeroControlSurfs(iACsurf,4:6)';
    u = deflections(iACsurf);
    dx_ACsurf_B(:,iACsurf) = x_AeroRef_B - x_CM_B;
    
    % Lift Terms
    CL_0 = AeroControlSurfs(iACsurf,11);
    Su_S_ratio = AeroControlSurfs(iACsurf,21)/AeroControlSurfs(iACsurf,9);
    dCL_du = AeroControlSurfs(iACsurf,19);
    
    % Drag Terms
    CD_0 = AeroControlSurfs(iACsurf,12);
    CD_a = AeroControlSurfs(iACsurf,13);
    a0 = AeroControlSurfs(iACsurf,14);
    e_surf = AeroControlSurfs(iACsurf,15);
    
    % Moment Terms
    CM_0 = AeroControlSurfs(iACsurf,16);
    CM_a = AeroControlSurfs(iACsurf,17);
    Cref = AeroControlSurfs(iACsurf,7);
    C_Moment = [0, 0,  0;
                0, 0, -1;
                0, 1,  0];
 
    % Wing damping: Induced velocity due to body rates changes the
    % effective aoa
    RandomDisturbance = [rand;rand;rand] .* 2 - 1;
    %    v_B_B_SnA = v_B_B_BA + cross(om_B_B_BE,dx_ACsurf_B(:,iACsurf)); %
    %    Added the random disturbance (turbulence) to the wing surfs
    v_B_B_SnA = v_B_B_BA + RandomDisturbance * MaxTurbulence + cross(om_B_B_BE,dx_ACsurf_B(:,iACsurf));
    %    a_surf = i_surf - asin( dot(v_B_B_SnA, n_B) / max([0.001, norm(v_B_B_SnA)]));
    a_surf = i_surf - atan2( dot(v_B_B_SnA, n_B) , dot(v_B_B_SnA, [1;0;0]) ); % The old way did not allow a_surf > 90 or < -90
    
    blownWingFlag = blownWingFlags(iACsurf);
    iProp=0;
    if (blownWingFlag == 1 && blownWingOption == 3)
        if iACsurf == 1
            iProp = 1;
        elseif iACsurf == 2
            iProp = 4;
        end
        
        % Deternime Tilt
        if iProp <= NumTiltRotors
            x_B_AeroRefHinge = Props2(iProp,1:3)' - HingePoint;
            X_B_AeroRefHinge_Tilt = Tilt_DCM * x_B_AeroRefHinge;
            x_AeroRef_B = X_B_AeroRefHinge_Tilt + HingePoint;
            
            n_B_prop = Tilt_DCM * Props2(iProp,4:6)';
        else
            x_AeroRef_B = Props2(iProp,1:3)';
            n_B_prop = Props2(iProp,4:6)';
        end
        
        % Geometry Terms
        Rref = Props2(iProp,7);
        prop_num = max(1, round(Props2(iProp,8))); % max & round needed for linearization for some reason
        dir_flag = Props2(iProp,9);
        dx_prop_B(:,iProp) = x_AeroRef_B - x_CM_B;
        
        % Left-right spinning direction flag
        if dir_flag == 1
            motor_dir = -1;
        else
            motor_dir = 1;
        end
        om = motor_dir .* om_motors(iProp);
        
        % om_notched is non-zero so AR isn't inf for 0 RPM
        if om == 0
            om_notched = 0.001;
        elseif (om < 0.001 && om > -0.001)
            om_notched = 0.001 * sign(om);
        else
            om_notched = om;
        end
        
        % Velocity into prop with turbulence
        % Random factors currently don't affect the slipstream model.
        % Need to figure out how to work the same random turbulence
        % factors into the slipstream model.
        RandomDisturbance = zeros(3,1); %[rand;rand;rand] .* 2 - 1;
        
        V_B_B_PA = v_B_B_BA + RandomDisturbance * MaxTurbulence + cross(om_B_B_BE,dx_prop_B(:,iProp));
        V_in = dot(V_B_B_PA, n_B_prop);
        
        AR_Prop = V_in / ( Rref * om_notched ); % Propeller Advance Ratio
        % Extrap limit based on +or- AR_Prop
        if (AR_Prop > max(PropTables(3*(prop_num-1)+1,:)))
            CTextrapVal = PropTables(3*(prop_num-1)+2,end);
            CQextrapVal = PropTables(3*(prop_num-1)+3,end);
        elseif (AR_Prop < min(PropTables(3*(prop_num-1)+1,:)))
            CTextrapVal = PropTables(3*(prop_num-1)+2,1);
            CQextrapVal = PropTables(3*(prop_num-1)+3,1);
        else
            CTextrapVal = 0;
            CQextrapVal = 0;
        end
        
        % Determine Propeller Thrust
        CT = interp1(PropTables(3*(prop_num-1)+1,:), PropTables(3*(prop_num-1)+2,:), AR_Prop,'linear',CTextrapVal);
        T =  CT * 0.5 * rho * om^2 * Rref^4;
        
        vj = sign(Vinf^2 + T / (rho * pi * Rref^2)) * sqrt(abs(Vinf^2 + T / (rho * pi * Rref^2))); % from a momentum flux to produce a thrust assuming constant flow
        vj_B_B_SnA = n_B_prop * vj; % from a momentum flux to produce a thrust assuming constant flow
        
        v_B_B_SnA = v_B_B_SnA + vj_B_B_SnA;
        a_surf = i_surf - atan2( dot(v_B_B_SnA, n_B) , dot(v_B_B_SnA, [1;0;0]) ); % The old way did not allow a_surf > 90 or < -90
    end
    
    CL_a = 2 * pi * AR / (2 + AR); % 2D CLa simulation method
    %     CL_a = 6.37 * AR / (2 + AR); % 2D CLa from Colin's slipstream code
    
    % Stall model terms
    alpha_Stall = StallModel_AeroControlSurfs(2*(iACsurf-1)+1,1);
    Stall_Alpha_Lookup = StallModel_AeroControlSurfs(2*(iACsurf-1)+1,:);
    Stall_CL_Lookup = StallModel_AeroControlSurfs(2*(iACsurf-1)+2,:);
    alpha_PostStall = StallModel_AeroControlSurfs(2*(iACsurf-1)+1,end);
    
    blendRegionSize = 10*pi/180;
    
    % Stall, post-stall, or non-stall lift if-structure
    if a_surf >= alpha_PostStall+blendRegionSize
        CL_fromWindTunnelTests = 0.8 * max(Stall_CL_Lookup) * sin(2*a_surf);
        CL = CL_fromWindTunnelTests;
    elseif a_surf >= alpha_PostStall
        CL_fromWindTunnelTests = 0.8 * max(Stall_CL_Lookup) * sin(2*a_surf);
        CL_lookup = interp1(Stall_Alpha_Lookup, Stall_CL_Lookup, a_surf, 'linear', Stall_CL_Lookup(end));
        
        fraction = (a_surf - alpha_PostStall)/blendRegionSize;
        
        CL = fraction*CL_fromWindTunnelTests + (1-fraction)*CL_lookup;
    elseif a_surf >= alpha_Stall
        CL_lookup = interp1(Stall_Alpha_Lookup, Stall_CL_Lookup, a_surf, 'linear', Stall_CL_Lookup(end));
        CL = CL_0 + CL_lookup         + Su_S_ratio * dCL_du * u;
    elseif a_surf >= -alpha_Stall
        CL = CL_0 + CL_a *  a_surf         + Su_S_ratio * dCL_du * u;
    elseif a_surf >= -alpha_PostStall
        CL_lookup = -interp1(Stall_Alpha_Lookup, Stall_CL_Lookup, -a_surf, 'linear', Stall_CL_Lookup(end));
        CL = CL_0 + CL_lookup         + Su_S_ratio * dCL_du * u;
    elseif a_surf >= -alpha_PostStall - blendRegionSize
        CL_fromWindTunnelTests = -0.8 * max(Stall_CL_Lookup) * sin(-2*a_surf);
        CL_lookup = -interp1(Stall_Alpha_Lookup, Stall_CL_Lookup, -a_surf, 'linear', Stall_CL_Lookup(end));
        
        fraction = (-a_surf - alpha_PostStall)/blendRegionSize;
        
        CL = fraction*CL_fromWindTunnelTests + (1-fraction)*CL_lookup;
    else
        CL_fromWindTunnelTests = 0.8 * max(Stall_CL_Lookup) * sin(-2*a_surf);
        CL = -CL_fromWindTunnelTests;
    end
    
    % Stall or non-stall drag if-structure
    if a_surf >= alpha_PostStall
        CD_fromWindTunnelTests = 2 * sin(a_surf);
        CD = CD_fromWindTunnelTests;
    elseif a_surf >= alpha_Stall
        CD_fromWindTunnelTests = 2 * sin(a_surf);
        CD_curve = CD_0 + CD_a * (a_surf - a0)^2 + CL^2 / (pi * e_surf * AR);
        
        fraction = (a_surf - alpha_Stall)/(alpha_PostStall - alpha_Stall);
        
        CD = fraction*CD_fromWindTunnelTests + (1-fraction)*CD_curve;
    elseif a_surf >= -alpha_Stall
        CD = CD_0 + CD_a * (a_surf - a0)^2 + CL^2 / (pi * e_surf * AR);
    elseif a_surf >= -alpha_PostStall
        CD_fromWindTunnelTests = 2 * abs(sin(a_surf));
        CD_curve = CD_0 + CD_a * (a_surf - a0)^2 + CL^2 / (pi * e_surf * AR);
        
        fraction = (-a_surf - alpha_Stall)/(alpha_PostStall - alpha_Stall);
        
        CD = fraction*CD_fromWindTunnelTests + (1-fraction)*CD_curve;
    else
        CD_fromWindTunnelTests = 2 * abs(sin(a_surf));
        CD = CD_fromWindTunnelTests;
    end
    
    % Old Lift and Drag coefficient methods commented out
    %     CL = CL_0 + CL_a *  a_surf         + Su_S_ratio * dCL_du * u;
    %     CD = CD_0 + CD_a * (a_surf - a0)^2 + CL^2 / (pi * e_surf * AR);
    CD = max(CD, CD_0);
    CM = CM_0 + CM_a * a_surf;
    
    CL_Slip_tot = 0;
    q = 1/2*rho*Vinf^2;
    qs = q;
    Vj = Vinf;
    
    % Find the thrust of the propellers for each surface. Currently assumes
    % 3 propellers per tilt surface.
    iProp=0;
    blownWingFlag = blownWingFlags(iACsurf);
    if blownWingFlag == 1 && blownWingOption == 2 %blownWingOption==2 means additiveCL
        CL_Slip_tot = 0;
        CD_Slip_tot = 0;
        Ss_tot = 0;
        for iProp = 3*iACsurf-2:3*iACsurf
            % Deternime Tilt
            if iProp <= NumTiltRotors
                x_B_AeroRefHinge = Props2(iProp,1:3)' - HingePoint;
                X_B_AeroRefHinge_Tilt = Tilt_DCM * x_B_AeroRefHinge;
                x_AeroRef_B = X_B_AeroRefHinge_Tilt + HingePoint;
                
                n_B_prop = Tilt_DCM * Props2(iProp,4:6)';
            else
                x_AeroRef_B = Props2(iProp,1:3)';
                n_B_prop = Props2(iProp,4:6)';
            end
            
            % Geometry Terms
            Rref = Props2(iProp,7);
            prop_num = max(1, round(Props2(iProp,8))); % max & round needed for linearization for some reason
            
            dir_flag = Props2(iProp,9);
            dx_prop_B(:,iProp) = x_AeroRef_B - x_CM_B;
            
            % Left-right spinning direction flag
            if dir_flag == 1
                motor_dir = -1;
            else
                motor_dir = 1;
            end
            om = motor_dir .* om_motors(iProp);
            
            % om_notched is non-zero so AR isn't inf for 0 RPM
            if om == 0
                om_notched = 0.001;
            elseif (om < 0.001 && om > -0.001)
                om_notched = 0.001 * sign(om);
            else
                om_notched = om;
            end
            
            % Velocity into prop with turbulence
            % Random factors currently don't affect the slipstream model.
            % Need to figure out how to work the same random turbulence
            % factors into the slipstream model.
            RandomDisturbance = zeros(3,1); %[rand;rand;rand] .* 2 - 1;
            
            V_Prop = v_B_B_BA + RandomDisturbance * MaxTurbulence + cross(om_B_B_BE,dx_prop_B(:,iProp));
            V_in = dot(V_Prop, n_B_prop);
            
            
            AR_Prop = V_in / ( Rref * om_notched ); % Propeller Advance Ratio
            % Extrap limit based on +or- AR_Prop
            if (AR_Prop > max(PropTables(3*(prop_num-1)+1,:)))
                CTextrapVal = PropTables(3*(prop_num-1)+2,end);
                CQextrapVal = PropTables(3*(prop_num-1)+3,end);
            elseif (AR_Prop < min(PropTables(3*(prop_num-1)+1,:)))
                CTextrapVal = PropTables(3*(prop_num-1)+2,1);
                CQextrapVal = PropTables(3*(prop_num-1)+3,1);
            else
                CTextrapVal = 0;
                CQextrapVal = 0;
            end
            
            % Determine Propeller Thrust
            CT = interp1(PropTables(3*(prop_num-1)+1,:), PropTables(3*(prop_num-1)+2,:), AR_Prop,'linear',CTextrapVal);
            T =  CT * 0.5 * rho * om^2 * Rref^4;
            %             CQ = interp1(PropTables(3*(prop_num-1)+1,:), PropTables(3*(prop_num-1)+3,:), AR_Prop,'linear',CQextrapVal);
            
            
            V0 = Vinf; % Air Velocity
            S = Sref; % Planform Area
            
            df = u; % Flap deflection
            
            c = Cref; % Wing chord
            cf = AeroControlSurfs(iACsurf,20); % Control Surface Chord
            
            D = Rref * 2; % Prop Diameter
            a0 = 6.37;  % 2D CL_alpha
            
            CD0 = CD_0; % CD at zero lift
            
            alpha_w  = a_surf; % Surface aoa
            alpha_LO = -4.55 * pi/180; % Surface zero lift aoa
            
            i_w = 0; % angle between thrust and alpha_wing
            alpha_T = alpha_w-i_w; % difference in wing and thrust angle
            
            q = 1/2*rho*V0^2; % Free-stream dynamic pressure
            
            U = -V0*cos(alpha_T) + sqrt((V0*cos(alpha_T))^2+T/((rho/2)*(pi*D^2/4))); % Induced Velocity by propeller thrust
            d = D*sqrt(1/2*(2*V0*cos(alpha_T)+U)/(V0*cos(alpha_T)+U)); % Fully developed slip stream diameter
            r0 = D/2; % Fully developed slip stream radius
            Vj = sqrt(V0^2 + T/((rho/2)*(pi*D^2/4))); % Velocity of the slip stream
            qs = 1/2*rho*Vj^2; % Slip stream dynamic pressure
            
            Ss = d*c; % S/3; %
            
            phi = atan(V0*sin(alpha_T)/(V0*cos(alpha_T)+U)); % angle between thrust line and induced velocity vector
            alpha_s = i_w+phi-alpha_LO+df; % Effective angle of attack of wing portion
            mu = V0*cos(alpha_T-phi)/Vj;   % Propeller axis inclined to the free-stream
            
            dCl_S = Ss/S*cos(alpha_T-phi)*(1.87)*alpha_s*(r0/c)*(1-mu^2); % CL change due to slipstream
            CL_Slip = dCl_S*qs/max(q,0.1); % Reference the cl change to the freestream q
            
            dCX_S = Ss/S*CD0 + ((1.87*alpha_s*r0/c*(1-mu^2))^2/(pi*AR))*cos(alpha_T-phi)...
                + (1.87*alpha_s*r0/c*(1-mu^2))^2*Ss/S*sin(alpha_T-phi); % Drag change due to slipstream
            CD_Slip = dCX_S*qs/max(q,0.1); % Reference CD change to the freestream q
            
            if q == 0
                CL_Slip = 0;
                CD_Slip = 0;
            end
            
            CL_Slip_tot = CL_Slip_tot + CL_Slip;
            CD_Slip_tot = CD_Slip_tot + CD_Slip;
            
            Ss_tot = Ss_tot + Ss;
        end
        % Blown wing CL method from paper
        %         a = a0*AR/(AR*sqrt(1+(a0/(pi*AR))^2)+a0/pi);
        %         theta_h = 1-2*cos(atan(sin(df)/(c/cf)-cos(df)))*(cf/c);
        %         alpha = alpha_w - alpha_LO + df*(1-2*theta_h/a+2*sin(theta_h)/a);
        %
        %         Cl_w = (q/qs)*a*alpha;
        %         CL = Cl_w + CL_prop_tot
        
        % Use CL calculated from Aero_Control_Surfs Method
        CL = CL_Slip_tot + CL;
        CD = CD_Slip_tot + CD;
    end
    % Calc Aero Forces and Moments
    RandomDisturbance = [rand;rand;rand] .* 2 - 1;
    L_ACsurf_W = CL * 0.5 * rho * (norm(v_B_B_BA + RandomDisturbance * MaxTurbulence))^2 * Sref;
    D_ACsurf_W = CD * 0.5 * rho * (norm(v_B_B_BA + RandomDisturbance * MaxTurbulence))^2 * Sref;
    M_ACsurf_W = CM * 0.5 * rho * (norm(v_B_B_BA + RandomDisturbance * MaxTurbulence))^2 * Sref * Cref;
    
    F_ACsurf_W = [-D_ACsurf_W; 0; 0] + L_ACsurf_W * n_B; % Forces in Wind CS
    F_ACsurf_B(:,iACsurf) = C_BW * F_ACsurf_W; % Forces in Body
    
    M_ACsurf_B(:,iACsurf) = M_ACsurf_W * C_Moment * n_B;  % Aerodynamic moments in the Body
    M_ACsurf_B(:,iACsurf) = M_ACsurf_B(:,iACsurf) + cross(dx_ACsurf_B(:,iACsurf), F_ACsurf_B(:,iACsurf)); % Add Moment due to force and moment arm
    
    F_aeroControls_B = F_aeroControls_B + F_ACsurf_B(:,iACsurf);
    M_aeroControls_B = M_aeroControls_B + M_ACsurf_B(:,iACsurf);
    
    % Plotting data for slip stream stuff
    Wing_CL = CL - CL_Slip_tot;
    CL_Slip_plot = CL_Slip_tot * q/qs;
    LiftSlip = LiftSlip + CL_Slip_plot * 0.5 * rho * Vj^2 * Sref;
    LiftTest = LiftTest + CL * q/qs *0.5 * rho *Vj^2 * Sref;
end

% Matlab and Simulink treat 1D arrays differently. In Simulink, need to only check the 2 dimension for size 1 prior to changing it
if (size(AeroSurfs,2) == 1)
    AeroSurfs2 = AeroSurfs';
else
    AeroSurfs2 = AeroSurfs;
end
nAeroSurfs = size(AeroSurfs2,1);
 
F_surf_B = zeros(3, nAeroSurfs);
M_surf_B = zeros(3, nAeroSurfs);
dx_surf_B = zeros(3, nAeroSurfs);
 
F_aero_B = [0; 0; 0];
M_aero_B = [0; 0; 0];
 
for iSurf = 1:nAeroSurfs
    % Geometry Terms
    x_AeroRef_B = AeroSurfs2(iSurf,1:3)';
    n_B = AeroSurfs2(iSurf,4:6)';
    Cref = AeroSurfs2(iSurf,7)';
    Sref = AeroSurfs2(iSurf,9);
    AR = AeroSurfs2(iSurf,10);
    i_surf = AeroSurfs2(iSurf,18);
    dx_surf_B(:,iSurf) = x_AeroRef_B - x_CM_B;
    
    % Lift Terms
    CL_0 = AeroSurfs2(iSurf,11);
    
    % Drag Terms
    CD_0 = AeroSurfs2(iSurf,12);
    CD_a = AeroSurfs2(iSurf,13);
    a0 = AeroSurfs2(iSurf,14);
    e_surf = AeroSurfs2(iSurf,15);
    
    % Moment Terms
    CM_0 = AeroSurfs2(iSurf,16);
    CM_a = AeroSurfs2(iSurf,17);
    C_Moment = [0, 0,  0;
                0, 0, -1;
                0, 1,  0];
 
    RandomDisturbance = [rand;rand;rand] .* 2 - 1;
    %    v_B_B_SnA = v_B_B_BA + cross(om_B_B_BE,dx_surf_B(:,iSurf)); % Local velocity at surface
    %    Added the random disturbance (turbulence) to the wing surfs
    v_B_B_SnA = v_B_B_BA + RandomDisturbance * MaxTurbulence + cross(om_B_B_BE,dx_surf_B(:,iSurf));
    %    a_surf = i_surf - asin( dot(v_B_B_SnA, n_B) / max([0.001, norm(v_B_B_SnA)]));
    a_surf = i_surf - atan2( dot(v_B_B_SnA, n_B) , dot(v_B_B_SnA, [1;0;0]) );
    
    CL_a = 2 * pi * AR / (2 + AR); % 3D CLalpha
    
    % Stall model terms
    alpha_Stall = StallModel_AeroSurfs(2*(iSurf-1)+1,1);
    Stall_Alpha_Lookup = StallModel_AeroSurfs(2*(iSurf-1)+1,:);
    Stall_CL_Lookup = StallModel_AeroSurfs(2*(iSurf-1)+2,:);
    alpha_PostStall = StallModel_AeroSurfs(2*(iSurf-1)+1,end);
    
    blendRegionSize = 10*pi/180;
    % Stall, post-stall, or non-stall lift if-structure
    if a_surf >= alpha_PostStall+blendRegionSize
        CL_fromWindTunnelTests = 0.8 * max(Stall_CL_Lookup) * sin(2*a_surf);
        CL = CL_fromWindTunnelTests;
    elseif a_surf >= alpha_PostStall
        CL_fromWindTunnelTests = 0.8 * max(Stall_CL_Lookup) * sin(2*a_surf);
        CL_lookup = interp1(Stall_Alpha_Lookup, Stall_CL_Lookup, a_surf, 'linear', Stall_CL_Lookup(end));
        
        fraction = (a_surf - alpha_PostStall)/blendRegionSize;
        
        CL = fraction*CL_fromWindTunnelTests + (1-fraction)*CL_lookup;
        
    elseif a_surf >= alpha_Stall
        CL_lookup = interp1(Stall_Alpha_Lookup, Stall_CL_Lookup, a_surf, 'linear', Stall_CL_Lookup(end));
        CL = CL_0 + CL_lookup;
    elseif a_surf >= -alpha_Stall
        CL = CL_0 + CL_a *  a_surf;
    elseif a_surf >= -alpha_PostStall
        CL_lookup = -interp1(Stall_Alpha_Lookup, Stall_CL_Lookup, -a_surf, 'linear', Stall_CL_Lookup(end));
        CL = CL_0 + CL_lookup;
        
    elseif a_surf >= -alpha_PostStall - blendRegionSize
        CL_fromWindTunnelTests = -0.8 * max(Stall_CL_Lookup) * sin(-2*a_surf);
        CL_lookup = -interp1(Stall_Alpha_Lookup, Stall_CL_Lookup, -a_surf, 'linear', Stall_CL_Lookup(end));
        
        fraction = (-a_surf - alpha_PostStall)/blendRegionSize;
        
        CL = fraction*CL_fromWindTunnelTests + (1-fraction)*CL_lookup;
    else
        CL_fromWindTunnelTests = 0.8 * max(Stall_CL_Lookup) * sin(-2*a_surf);
        CL = -CL_fromWindTunnelTests;
        
    end
    
    % Stall or non-stall drag if-structure
    if a_surf >= alpha_PostStall
        CD_fromWindTunnelTests = 2 * sin(a_surf);
        CD = CD_fromWindTunnelTests;
    elseif a_surf >= alpha_Stall
        CD_fromWindTunnelTests = 2 * sin(a_surf);
        CD_curve = CD_0 + CD_a * (a_surf - a0)^2 + CL^2 / (pi * e_surf * AR);
        
        fraction = (a_surf - alpha_Stall)/(alpha_PostStall - alpha_Stall);
        
        CD = fraction*CD_fromWindTunnelTests + (1-fraction)*CD_curve;
    elseif a_surf >= -alpha_Stall
        CD = CD_0 + CD_a * (a_surf - a0)^2 + CL^2 / (pi * e_surf * AR);
    elseif a_surf >= -alpha_PostStall
        CD_fromWindTunnelTests = 2 * abs(sin(a_surf));
        CD_curve = CD_0 + CD_a * (a_surf - a0)^2 + CL^2 / (pi * e_surf * AR);
        
        fraction = (-a_surf - alpha_Stall)/(alpha_PostStall - alpha_Stall);
        
        CD = fraction*CD_fromWindTunnelTests + (1-fraction)*CD_curve;
    else
        CD_fromWindTunnelTests = 2 * abs(sin(a_surf));
        CD = CD_fromWindTunnelTests;
    end
    
    %     CL = CL_0 + CL_a *  a_surf;
    %     CD = CD_0 + CD_a * (a_surf - a0)^2 + CL^2 / (pi * e_surf * AR);
    CD = max(CD, CD_0);
    CM = CM_0 + CM_a * a_surf;
    
    % Calculate forces and moments
    L_surf_W = CL * 0.5 * rho * Vinf^2 * Sref;
    D_surf_W = CD * 0.5 * rho * Vinf^2 * Sref;
    M_surf_W = CM * 0.5 * rho * Vinf^2 * Sref * Cref;
    
    F_surf_W = [-D_surf_W; 0; 0] + L_surf_W * n_B; % Forces in Wind CS
    F_surf_B(:,iSurf) = C_BW * F_surf_W; % Forces in Body
    
    M_surf_B(:,iSurf) = M_surf_W * C_Moment * n_B; % Aerodynamic Moments
    M_surf_B(:,iSurf) = M_surf_B(:,iSurf) + cross(dx_surf_B(:,iSurf), F_surf_B(:,iSurf)); % Moments due to forces and moment arm
    
    F_aero_B = F_aero_B + F_surf_B(:,iSurf);
    M_aero_B = M_aero_B + M_surf_B(:,iSurf);
end
 
if (size(Props,1) == 1 || size(Props,2) == 1)
    Props2=Props';
else
    Props2 = Props;
end
nProps = size(Props2,1);
om_motors = u_in(nAeroControlSurfs + TiltFlag + 1:end); % Motor rate signals
 
F_prop_B = zeros(3, nProps);
M_prop_B = zeros(3, nProps);
dx_prop_B = zeros(3, nProps);
 
F_props_B = [0; 0; 0];
M_props_B = [0; 0; 0];
 
for iProp = 1:nProps
    % Deternime Tilt
    if iProp <= NumTiltRotors
        x_B_AeroRefHinge = Props2(iProp,1:3)' - HingePoint;
        X_B_AeroRefHinge_Tilt = Tilt_DCM * x_B_AeroRefHinge;
        x_AeroRef_B = X_B_AeroRefHinge_Tilt + HingePoint;
        
        n_B = Tilt_DCM * Props2(iProp,4:6)';
    else
        x_AeroRef_B = Props2(iProp,1:3)';
        n_B = Props2(iProp,4:6)';
    end
    
    % Geometry Terms
    Rref = Props2(iProp,7);
    prop_num = max(1, round(Props2(iProp,8))); % max & round needed for linearization for some reason
    dir_flag = Props2(iProp,9);
    dx_prop_B(:,iProp) = x_AeroRef_B - x_CM_B;
    
    % Left-right spinning direction flag
    if dir_flag == 1
        motor_dir = -1;
    else
        motor_dir = 1;
    end
    om = motor_dir .* om_motors(iProp);
    
    % om_notched is non-zero so AR isn't inf for 0 RPM
    if om == 0
        om_notched = 0.001;
    elseif (om < 0.001 && om > -0.001)
        om_notched = 0.001 * sign(om);
    else
        om_notched = om;
    end
    
    % Velocity into prop with turbulence
    RandomDisturbance = [rand;rand;rand] .* 2 - 1;
    V_Prop = v_B_B_BA + RandomDisturbance * MaxTurbulence + cross(om_B_B_BE,dx_prop_B(:,iProp));
    V_in = dot(V_Prop, n_B);
    
    % Propeller advance ratio
    AR = V_in / ( Rref * om_notched );
    if (AR > max(PropTables(3*(prop_num-1)+1,:)))
        CTextrapVal = PropTables(3*(prop_num-1)+2,end);
        CQextrapVal = PropTables(3*(prop_num-1)+3,end);
    elseif (AR < min(PropTables(3*(prop_num-1)+1,:)))
        CTextrapVal = PropTables(3*(prop_num-1)+2,1);
        CQextrapVal = PropTables(3*(prop_num-1)+3,1);
    else
        CTextrapVal = 0;
        CQextrapVal = 0;
    end
    
    % Thrust and Torque Coefficients
    CT = interp1(PropTables(3*(prop_num-1)+1,:), PropTables(3*(prop_num-1)+2,:), AR,'linear',CTextrapVal);
    CQ = interp1(PropTables(3*(prop_num-1)+1,:), PropTables(3*(prop_num-1)+3,:), AR,'linear',CQextrapVal);
    
    % Thrust and Torque Calculations
    T_prop(iProp) =  CT * 0.5 * rho * om^2 * Rref^4;
    Q_prop(iProp) = -CQ * 0.5 * rho * om^2 * Rref^5 * motor_dir;
    P_prop(iProp) = abs(om) * abs(Q_prop(iProp));
    
    F_prop_B(:,iProp) = n_B * T_prop(iProp); % Prop Force in the body
    
    M_prop_B(:,iProp) = Q_prop(iProp) * n_B; % Prop Torque in the body
    M_prop_B(:,iProp) = M_prop_B(:,iProp) + cross(dx_prop_B(:,iProp), F_prop_B(:,iProp)); % Prop Torque withmoment due to thrust
    
    F_props_B = F_props_B + F_prop_B(:,iProp);
    M_props_B = M_props_B + M_prop_B(:,iProp);
end
 
F_ext_B = F_aeroControls_B + F_aero_B + F_props_B;
M_ext_B = M_aeroControls_B + M_aero_B + M_props_B;
 
% Output vector of data that may be needed for potting
VisualizationData = [Wing_CL CL_Slip_plot Vj LiftSlip LiftTest -F_props_B(3) T_prop(1) P_prop(1)];
%AR,F_props_B,M_props_B om_notched

end
