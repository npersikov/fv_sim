function [surfs, aero] = aero_design_params()
    %% Aerodynamic Design Parameters
    % Elevator
    n_s2    = [0; 0; -1];   % Unit vector in surface normal direction
    CL0_s2  = 0;            % Lift at 0 angle of attack
    e_s2    = 0.8;          % Oswald efficiency factor
    i_s2    = 0;            % Incidence angle (angle of surface mounting)
    CD0_s2  = 0.01;         % Drag at 0 angle of attack
    CDa_s2  = 1;            % Drag slope
    a0_s2   = 0;            % Minimum drag angle of attack
    CM0_s2  = 0;            % Moment at 0 angle of attack
    CMa_s2  = 0;            % Moment slope
    
    % Rudder
    n_s3    = [0; 1; 0];
    CL0_s3  = 0;
    e_s3    = 0.8;
    i_s3    = 0;
    CD0_s3  = 0.01;
    CDa_s3  = 1;
    a0_s3   = 0;
    CM0_s3  = 0;
    CMa_s3  = 0;
    
    % Left wing (something something politics)
    n_s4    = [0; 0; -1];
    CL0_s4  = 0.05;
    e_s4    = 0.9;
    i_s4    = 0.05;
    CD0_s4  = 0.01;
    CDa_s4  = 1;
    a0_s4   = 0.05;
    CM0_s4  = -0.05;
    CMa_s4  = 0;
    
    % Right wing
    n_s5    = [0; 0; -1];
    CL0_s5  = 0.05;
    e_s5    = 0.9;
    i_s5    = 0.05;
    CD0_s5  = 0.01;
    CDa_s5  = 1;
    a0_s5   = 0.05;
    CM0_s5  = -0.05;
    CMa_s5  = 0;

    surfs   = [n_s2, n_s3, n_s4, n_s5];
    aero    = [CL0_s2, e_s2, i_s2, CD0_s2, CDa_s2, a0_s2, CM0_s2, CMa_s2;
               CL0_s3, e_s3, i_s3, CD0_s3, CDa_s3, a0_s3, CM0_s3, CMa_s3;
               CL0_s4, e_s4, i_s4, CD0_s4, CDa_s4, a0_s4, CM0_s4, CMa_s4;
               CL0_s5, e_s5, i_s5, CD0_s5, CDa_s5, a0_s5, CM0_s5, CMa_s5;];
end