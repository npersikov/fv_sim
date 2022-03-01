function [cg_m, m_kg, J_kgm2] = get_mass_props()

%% Define Aircraft Mass and Geometry Properties (using grams) 
cmg = const.component_masses_geometries; 

%% Calculate Mass Properties

% Convert to kg
cmg = [cmg(:,1)/1000, cmg(:,2:7)]; 

% Total mass
m_kg = sum(cmg(:,1));

% Center of mass
cg_m = [sum(cmg(:,1).*cmg(:,5))/sum(cmg(:,1)), ...
        sum(cmg(:,1).*cmg(:,6))/sum(cmg(:,1)), ...
        sum(cmg(:,1).*cmg(:,7))/sum(cmg(:,1))];

% J' (prime). Assuming everything is box shaped with uniform density
J_xxp_list = cmg(:,1)/12.*(cmg(:,3).^2 + cmg(:,4).^2); 
J_yyp_list = cmg(:,1)/12.*(cmg(:,2).^2 + cmg(:,4).^2);
J_zzp_list = cmg(:,1)/12.*(cmg(:,2).^2 + cmg(:,3).^2);

% Parallel axis theorem terms
mx2_list = cmg(:,1).*((cmg(:,6) - cg_m(2)).^2 + (cmg(:,7) - cg_m(3)).^2);
my2_list = cmg(:,1).*((cmg(:,5) - cg_m(1)).^2 + (cmg(:,7) - cg_m(3)).^2);
mz2_list = cmg(:,1).*(((cmg(:,6) - cg_m(2)).^2 + (cmg(:,5) - cg_m(1)).^2));

% Calculate inertias of each component wrt the plane
J_xx_list = J_xxp_list + mx2_list;
J_yy_list = J_yyp_list + my2_list;
J_zz_list = J_zzp_list + mz2_list;

% Principal inertias
J_xx = sum(J_xx_list);
J_yy = sum(J_yy_list);
J_zz = sum(J_zz_list);

% Products of inertia
J_xy = sum(cmg(:,1).*cmg(:,5).*cmg(:,6));
J_yx = J_xy;
J_xz = sum(cmg(:,1).*cmg(:,5).*cmg(:,7));
J_zx = J_xz;
J_yz = sum(cmg(:,1).*cmg(:,6).*cmg(:,7));
J_zy = J_yz;

% Inertia matrix
J_kgm2 = [J_xx, J_xy, J_xz;
          J_yx, J_yy, J_yz;
          J_zx, J_zy, J_zz];

end