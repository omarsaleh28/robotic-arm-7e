function [p, phi] = al5d_fk(q, P)
%AL5D_FK  Forward kinematics (geometric form).
%   [p, phi] = al5d_fk(q, P)
%   q   = [th1 th2 th3 th4] in degrees (convention: see al5d_params)
%   p   = [x y z] position of the gripper centre point (TCP), mm
%   phi = absolute gripper pitch, deg (-90 = pointing straight down)

th1 = q(1); th2 = q(2); th3 = q(3); th4 = q(4);
a2  = th2;               % absolute angle of upper arm
a3  = th2 + th3;         % absolute angle of forearm
phi = th2 + th3 + th4;   % absolute angle of gripper

r = P.a1 + P.L2e*cosd(a2) + P.L3e*cosd(a3) + P.Lg*cosd(phi);   % horizontal distance from base axis
z = P.d1 + P.L2e*sind(a2) + P.L3e*sind(a3) + P.Lg*sind(phi);

p = [r*cosd(th1), r*sind(th1), z];
end
