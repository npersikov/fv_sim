clear all
clc
%% Define Aircraft Mass and Geometry Properties (using grams)
%    mass   xSize   ySize   zSize   xLoc    yLoc    zLoc
%    1      2       3       4       5       6       7
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
 
% Convert first column to kg
componentMassesAndGeom(:,1) = componentMassesAndGeom(:,1)/1000;
 
m = sum(componentMassesAndGeom(:,1));
 
componentMoments(:,1) = componentMassesAndGeom(:,1).*componentMassesAndGeom(:,5);
componentMoments(:,2) = componentMassesAndGeom(:,1).*componentMassesAndGeom(:,6);
componentMoments(:,3) = componentMassesAndGeom(:,1).*componentMassesAndGeom(:,7);
 
x_cm = sum(componentMoments)/m;
x_cm=x_cm';
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
J(1,1)=sum(componentInertias(:,1)) + sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,6)-x_cm(2)).^2 + (componentMassesAndGeom(:,7)-x_cm(3)).^2));
J(2,2)=sum(componentInertias(:,2)) + sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,7)-x_cm(3)).^2 + (componentMassesAndGeom(:,5)-x_cm(1)).^2));
J(3,3)=sum(componentInertias(:,3)) + sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,5)-x_cm(1)).^2 + (componentMassesAndGeom(:,6)-x_cm(2)).^2));
J(1,2) = -sum(componentMassesAndGeom(:,1) .* (componentMassesAndGeom(:,5) .* componentMassesAndGeom(:,6)));
J(1,3) = -sum(componentMassesAndGeom(:,1) .* (componentMassesAndGeom(:,7) .* componentMassesAndGeom(:,5)));
J(2,3) = -sum(componentMassesAndGeom(:,1) .* (componentMassesAndGeom(:,6) .* componentMassesAndGeom(:,7)));
 
J(1,2) = -J(1,2);
J(1,3) = -J(1,3);
J(2,3) = -J(2,3);
J(2,1) = J(1,2);
J(3,1) = J(1,3);
J(3,2) = J(2,3);