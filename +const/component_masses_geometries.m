function cmg = component_masses_geometries()
%% Define Aircraft Mass and Geometry Properties (using grams) 
%    mass   xSize   ySize   zSize   xLoc    yLoc    zLoc 
%    1      2       3       4       5       6       7
cmg = ... % Stands for component masses and geometries. Shortened for concise coding
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
end