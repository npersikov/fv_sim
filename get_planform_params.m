% Each output is a list of the respective parameters in this order:
% elevator, rudder, left wing, right wing.
function [surf_pos_m, c_m, b_m, S_m2, AR] = get_planform_params()

%% Define Aircraft Mass and Geometry Properties (using grams) 
cmg = const.component_masses_geometries; 

%% Calculate Remaining Aerodynamic Properties

% Chords for each section in meters
c_el = cmg(3,2);
c_ru = cmg(4,2);
c_lw = cmg(2,2);
c_rw = cmg(1,2);

% Spans for each section in meters
b_el = cmg(3,3);
b_ru = cmg(3,4);
b_lw = cmg(2,3);
b_rw = cmg(1,3);

% Chord, span, area, aspect ratios for elevator, rudder, left wing, right
% wing. For the two wings, b, S, and AR need to be added together for the
% full wing. Chord is the same for a half wing and full wing.
c_m     = [c_el, c_ru, c_lw, c_rw];
b_m     = [b_el, b_ru, b_lw, b_rw];
S_m2    = c_m.*b_m; 
AR      = b_m./c_m;

% mid span quarter chord coordinates of elevator; rudder; left wing; right
% wing; WRT the reference point WRT which the data from problem 3 was
% given.
surf_pos_m      = [cmg(3,5) + c_el/4, cmg(3,6), cmg(3,7);
                   cmg(4,5) + c_ru/4, cmg(4,6), cmg(4,7);
                   cmg(2,5) + c_lw/4, cmg(2,6), cmg(2,7);
                   cmg(1,5) + c_rw/4, cmg(1,6), cmg(1,7);];

end