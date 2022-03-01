clc
clear
close all


componentMassesAndGeom = ... % Stands for component masses and geometries. Shortened for concise coding
    [90     0.1     0.96    0.01    -0.23    0.44   0;        % RightWing+Servo (s4) 
     90     0.1     0.96    0.01    -0.23   -0.44   0;        % LeftWing+Servo (s5) 
     13     0.075   0.35    0.002   -0.76   0       -0.16;    % Hor. Stab. (s2) 
     0      0.08    0.002   0.18    -0.76   0       -0.09;    % Vert. Stab. (s3) 
     72     0.065   0.035   0.015   -0.05   0       0.03;     % Battery 
     106    0.87    0.07    0.07    -0.4    0       0;        % Fuselage 
     27     0.05    0.03    0.005   -0.05   0       0.02;     % Motor Controller 
     10     0.04    0.02    0.005   -0.1    0       0.02;     % Radio 
     20     0.05    0.01    0.01    -0.014  0       0;        % 2 Servos 
     40     0.03    0.02    0.02    0.02    0       0.01;     % Motor 
     12     0       0.26    0.025   0.05    0       0.01];    % Propeller

%% Define Aircraft Aero Properties
% Elevator  = surface2 = s2
% Rudder    = surface3 = s3
% RightWing = surface4 = s4
% LeftWing  = surface5 = s5
x_s2 = [componentMassesAndGeom(3,5) + (1/4)*componentMassesAndGeom(3,2); 
componentMassesAndGeom(3,6);componentMassesAndGeom(3,7)];
x_s3 = [-0.76; 0; -0.09];
x_s4 = [componentMassesAndGeom(1,5) + (1/4)*componentMassesAndGeom(1,2); 
componentMassesAndGeom(1,6);componentMassesAndGeom(1,7)];
x_s5 = [componentMassesAndGeom(2,5) + (1/4)*componentMassesAndGeom(2,2); 
componentMassesAndGeom(2,6);componentMassesAndGeom(2,7)];
 
n_s2 = [0; 0; -1];
n_s3 = [0; 1; 0];
n_s4 = [0; 0; -1];
n_s5 = [0; 0; -1];
 
c_s2 = componentMassesAndGeom(3,2);
c_s3 = 0.08;
c_s4 = componentMassesAndGeom(1,2);
c_s5 = componentMassesAndGeom(2,2);
 
b_s2 = componentMassesAndGeom(3,3);
b_s3 = 0.18; % his was 0.08
b_s4 = componentMassesAndGeom(1,3);
b_s5 = componentMassesAndGeom(2,3);
 
S_s2 = c_s2 * b_s2;
S_s3 = c_s3 * b_s3;
S_s4 = c_s4 * b_s4;
S_s5 = c_s5 * b_s5;
 
AR_s2 = b_s2 / c_s2;
AR_s3 = b_s3 / c_s3;
AR_s4 = b_s4 / c_s4;
AR_s5 = b_s5 / c_s5;
 
CL0_s2 = 0;
CL0_s3 = 0;
CL0_s4 = 0.05;
CL0_s5 = 0.05;
 
e_s2 = 0.8;
e_s3 = 0.8;
e_s4 = 0.9;
e_s5 = 0.9;
 
i_s2 = -0.;
i_s3 = 0;
i_s4 = 0.05;
i_s5 = 0.05;
 
CD0_s2 = 0.01;
CD0_s3 = 0.01;
CD0_s4 = 0.01;
CD0_s5 = 0.01;
 
CDa_s2 = 1;
CDa_s3 = 1;
CDa_s4 = 1;
CDa_s5 = 1;
 
a0_s2 = 0;
a0_s3 = 0;
a0_s4 = 0.05;
a0_s5 = 0.05;
 
CM0_s2 = 0;
CM0_s3 = 0;
CM0_s4 = -0.05;
CM0_s5 = -0.05;
 
CMa_s2 = 0;
CMa_s3 = 0;
CMa_s4 = 0;
CMa_s5 = 0;

componentMassesAndGeom_AeroControls = ...
    [90     0.1     0.48    0.01    -0.23   0.44    0;  % Wing01+Servo
     90     0.1     0.48    0.01    -0.23   -0.44   0;  % Wing02+Servo
     13     0.075   0.35    0.002   -0.76   0      -0.16]; % Hor. Stab.
StallRegion_AeroControls = [5*pi/180   22*pi/180;
                            5*pi/180   22*pi/180;
                            7.5*pi/180 16*pi/180];
 
%AeroSurfs format
%  1,2,3 = pos in body, (x,y,z)
%  4,5,6 = normal vector in body (x,y,z)
%  7,8,9 = Cref, Bref, Sref
%  10,11,12 = AR, CL0, CD0
%  13,14,15 = CDa, a0, e
%  16,17,18 = CM0, CMa, i
 
%For Surfaces with Controls
%  19,20,21 = dCLdu, C_u, S_u
 
nACsurfs = size(componentMassesAndGeom_AeroControls,1);
 
for i_ACsurf = 1:nACsurfs
 
 %% Assign aero geometric parameters from table above
% HorStab   = surface2 = s2
% RightWing = surface4 = s4
% LeftWing  = surface5 = s5
    %xPos of Aerodynamic Center
    AeroControlSurfs(i_ACsurf,1) = componentMassesAndGeom_AeroControls(i_ACsurf,5) + (1/4)*componentMassesAndGeom_AeroControls(i_ACsurf,2);
    %yPos of Aerodynamic Center
    AeroControlSurfs(i_ACsurf,2) = componentMassesAndGeom_AeroControls(i_ACsurf,6); %yPos
    %zPos of Aerodynamic Center
    AeroControlSurfs(i_ACsurf,3) = componentMassesAndGeom_AeroControls(i_ACsurf,7); %zPos
    %Cref
    AeroControlSurfs(i_ACsurf,7) = componentMassesAndGeom_AeroControls(i_ACsurf,2); %Cref
    %Bref
    AeroControlSurfs(i_ACsurf,8) = max(componentMassesAndGeom_AeroControls(i_ACsurf,3),componentMassesAndGeom_AeroControls(i_ACsurf,4));
    %Sref
    AeroControlSurfs(i_ACsurf,9) = AeroControlSurfs(i_ACsurf,7) * AeroControlSurfs(i_ACsurf,8);
 
    %dCLdu 
    AeroControlSurfs(i_ACsurf,19) = 3; %lift coefficient increase per surface deflection in radians
    %dCD0
    AeroControlSurfs(i_ACsurf,12) = 0.01;
    %dCDa
    AeroControlSurfs(i_ACsurf,13) = 1;
    %Cu
    AeroControlSurfs(i_ACsurf,20) = 0.2*AeroControlSurfs(i_ACsurf,7);
    %Su
    AeroControlSurfs(i_ACsurf,21) = AeroControlSurfs(i_ACsurf,20) * AeroControlSurfs(i_ACsurf,8);
    %CMa
    AeroControlSurfs(i_ACsurf,17) = 0;
    % alpha_stall
    AeroControlSurfs(i_ACsurf,21) = StallRegion_AeroControls(i_ACsurf,1);
end
 
%n_B (x,y,z) surf 1
AeroControlSurfs(1,4)=0;
AeroControlSurfs(1,5)=0;
AeroControlSurfs(1,6)=-1;
 
%n_B (x,y,z) surf 2
AeroControlSurfs(2,4)=0;
AeroControlSurfs(2,5)=0;
AeroControlSurfs(2,6)=-1;
 
%n_B (x,y,z) surf 3
AeroControlSurfs(3,4)=0;
AeroControlSurfs(3,5)=0;
AeroControlSurfs(3,6)=-1;
 
%AR
AeroControlSurfs(1,10) = (AeroControlSurfs(1,8) + AeroControlSurfs(2,8)) / AeroControlSurfs(1,7);
AeroControlSurfs(2,10) = (AeroControlSurfs(1,8) + AeroControlSurfs(2,8)) / AeroControlSurfs(2,7);
AeroControlSurfs(3,10) = AeroControlSurfs(3,8) / AeroControlSurfs(3,7);
 
CL_a_AeroControlSurfs(1) = 2 * pi * AeroControlSurfs(1,10) / (2 + AeroControlSurfs(1,10));
CL_a_AeroControlSurfs(2) = 2 * pi * AeroControlSurfs(2,10) / (2 + AeroControlSurfs(2,10));
CL_a_AeroControlSurfs(3) = 2 * pi * AeroControlSurfs(3,10) / (2 + AeroControlSurfs(3,10));
 
% CL0 - CL @ 0 alpha
AeroControlSurfs(1,11) = 0.05;
AeroControlSurfs(2,11) = 0.05;
AeroControlSurfs(3,11) = 0;
 
%e - Oswald
AeroControlSurfs(1,15) = 0.9;
AeroControlSurfs(2,15) = 0.9;
AeroControlSurfs(3,15) = 0.8;
 
%i - incidence angle
AeroControlSurfs(1,18) = 0.05;
AeroControlSurfs(2,18) = 0.05;
AeroControlSurfs(3,18) = 0;
 
%a0
AeroControlSurfs(1,14) = 0.05;
AeroControlSurfs(2,14) = 0.05;
AeroControlSurfs(3,14) = 0;
 
%CM0 - Moment Coeff @ 0 alpha
AeroControlSurfs(1,16) = -0.05;
AeroControlSurfs(2,16) = -0.05;
AeroControlSurfs(3,16) = 0;
%    mass   xSize   ySize   zSize   xLoc    yLoc    zLoc
%    1      2       3       4       5       6       7    
componentMassesAndGeom_Aero = ...
    [0      0.08    0.002   0.18    -0.76   0       -0.09];    % Ver. Stab.
 
StallRegion_Aero = [7.5*pi/180 16*pi/180];
 
n_surfs = size(componentMassesAndGeom_Aero,1);
 
for i_surf = 1:n_surfs
 
 %% Assign aero geometric parameters from table above
% VerStab   = surface3 = s3
    AeroSurfs(i_surf,1) = componentMassesAndGeom_Aero(i_surf,5) + (1/4)*componentMassesAndGeom_Aero(i_surf,2); 
    AeroSurfs(i_surf,2) = componentMassesAndGeom_Aero(i_surf,6);
    AeroSurfs(i_surf,3) = componentMassesAndGeom_Aero(i_surf,7);
    AeroSurfs(i_surf,7) = componentMassesAndGeom_Aero(i_surf,2);
    AeroSurfs(i_surf,8) = max(componentMassesAndGeom_Aero(i_surf,3),componentMassesAndGeom_Aero(i_surf,4));
    AeroSurfs(i_surf,9) = AeroSurfs(i_surf,7) * AeroSurfs(i_surf,8);
 
    AeroSurfs(i_surf,12) = 0.01;
    AeroSurfs(i_surf,13) = 1;
    AeroSurfs(i_surf,17) = 0;
    
    AeroSurfs(i_surf,21) = StallRegion_Aero(i_surf,1);
end
 
AeroSurfs(1,4) = 0;
AeroSurfs(1,5) = -1;
AeroSurfs(1,6) = 0;
 
AeroSurfs(1,10) = AeroSurfs(1,8) / AeroSurfs(1,7);
CL_a_AeroSurfs(1) = 2 * pi * AeroSurfs(1,10) / (2 + AeroSurfs(1,10));
 
AeroSurfs(1,11) = 0;
 
AeroSurfs(1,15) = 0.8;
 
AeroSurfs(1,18) = 0;
 
AeroSurfs(1,14) = 0;
 
AeroSurfs(1,16) = 0;
%% Define Aircraft Mass Elements (using grams)
% These are components of the vehicle that only have static and non-rotating mass properties
%    mass   xSize   ySize   zSize   xLoc    yLoc    zLoc
%    1      2       3       4       5       6       7
componentMassesAndGeom_Masses = ...
    [72     0.065   0.035   0.015   -0.05   0       0.03;     % Battery
     106    0.87    0.07    0.07    -0.4    0       0;        % Fuselage
     27     0.05    0.03    0.005   -0.05   0       0.02;     % Motor Controller
     10     0.04    0.02    0.005   0.1     0       0.02;     % Radio
     20     0.05    0.01    0.01    -0.014  0       0;        % 2 Servos
     40     0.03    0.02    0.02    0.02    0       0.01];    % Motor
clearvars componentMassesAndGeom TransInertia x_CM_B iComp componentInertias J RotInertia componentMoments
 
%% compute aircraft mass properties based on component masses and geometry
% assumes components have uniform density
 
% Convert first column to kg
componentMassesAndGeom = ...
   [componentMassesAndGeom_Masses;
    componentMassesAndGeom_AeroControls;
    componentMassesAndGeom_Aero];
%     componentMassesAndGeom_Prop];
 
componentMassesAndGeom(:,1) = componentMassesAndGeom(:,1)/1000;
 
TransInertia = sum(componentMassesAndGeom(:,1));
 
componentMoments(:,1) = componentMassesAndGeom(:,1).*componentMassesAndGeom(:,5);
componentMoments(:,2) = componentMassesAndGeom(:,1).*componentMassesAndGeom(:,6);
componentMoments(:,3) = componentMassesAndGeom(:,1).*componentMassesAndGeom(:,7);
 
x_CM_B = sum(componentMoments)/TransInertia;
x_CM_B=x_CM_B';
%x_cm=x_cm+[.01;0;0];
 
for iComp=1:size(componentMassesAndGeom,1)
    if (componentMassesAndGeom(iComp,2) ~= 0)
        componentInertias(iComp,1) = (1/12) * componentMassesAndGeom(iComp,1) .* (componentMassesAndGeom(iComp,3).^2 + componentMassesAndGeom(iComp,4).^2);
        componentInertias(iComp,2) = (1/12) * componentMassesAndGeom(iComp,1) .* (componentMassesAndGeom(iComp,4).^2 + componentMassesAndGeom(iComp,2).^2);
        componentInertias(iComp,3) = (1/12) * componentMassesAndGeom(iComp,1) .* (componentMassesAndGeom(iComp,2).^2 + componentMassesAndGeom(iComp,3).^2);
    else
        componentInertias(iComp,1) = (1/12) * componentMassesAndGeom(iComp,1) .* (componentMassesAndGeom(iComp,3).^2 + componentMassesAndGeom(iComp,4).^2);        
    end
end
 
J=zeros(3,3);
J(1,1)=sum(componentInertias(:,1)) + sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,6)-x_CM_B(2)).^2 + (componentMassesAndGeom(:,7)-x_CM_B(3)).^2));
J(2,2)=sum(componentInertias(:,2)) + sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,7)-x_CM_B(3)).^2 + (componentMassesAndGeom(:,5)-x_CM_B(1)).^2));
J(3,3)=sum(componentInertias(:,3)) + sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,5)-x_CM_B(1)).^2 + (componentMassesAndGeom(:,6)-x_CM_B(2)).^2));
J(1,2) = -sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,5)-x_CM_B(1)) .* (componentMassesAndGeom(:,6)-x_CM_B(2))));
J(1,3) = -sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,7)-x_CM_B(3)) .* (componentMassesAndGeom(:,5)-x_CM_B(1))));
J(2,3) = -sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,6)-x_CM_B(2)) .* (componentMassesAndGeom(:,7)-x_CM_B(3))));
 
J(1,2) = -J(1,2);
J(1,3) = -J(1,3);
J(2,3) = -J(2,3);
J(2,1) = J(1,2);
J(3,1) = J(1,3);
J(3,2) = J(2,3);
 
RotInertia = J;
