function [q, ok, msg] = al5d_ik(p, phi, P)
%AL5D_IK  Closed-form inverse kinematics (geometric solution, Kinematics Part 1).
%   [q, ok, msg] = al5d_ik(p, phi, P)
%   p   = [x y z] target for the gripper centre point (mm)
%   phi = required absolute gripper pitch (deg). Use -90 to pick/place from above.
%   q   = [th1 th2 th3 th4] (deg), elbow-up solution
%   ok  = true if the target is reachable AND inside the joint limits
%   msg = reason if not ok
%
%   Method:
%   1) Base angle from the top view: th1 = atan2(y, x).
%   2) Step back from the TCP along the gripper to find the wrist point.
%   3) Solve the 2-link planar problem (upper arm + forearm) with the law of cosines.
%   4) Wrist angle makes the gripper pitch equal phi.

if nargin < 2 || isempty(phi), phi = -90; end
q = nan(1,4); ok = false; msg = '';

x = p(1); y = p(2); z = p(3);
th1 = atan2d(y, x);
r   = hypot(x, y);

% wrist point (side view), measured from the shoulder axis
rw = r - P.a1 - P.Lg*cosd(phi);
zw = z - P.d1 - P.Lg*sind(phi);

D = (rw^2 + zw^2 - P.L2e^2 - P.L3e^2) / (2*P.L2e*P.L3e);   % cos(th3)
if abs(D) > 1
    msg = sprintf('Out of reach: wrist point is %.1f mm from shoulder, max %.1f mm', ...
                  hypot(rw,zw), P.L2e + P.L3e);
    return
end

th3 = -acosd(D);                                             % elbow-up (forearm bends down)
th2 = atan2d(zw, rw) - atan2d(P.L3e*sind(th3), P.L2e + P.L3e*cosd(th3));
th4 = phi - th2 - th3;

q = [th1 th2 th3 th4];

% joint limit check
names = {'base','shoulder','elbow','wrist'};
for k = 1:4
    if q(k) < P.lim(k,1) - 1e-9 || q(k) > P.lim(k,2) + 1e-9
        msg = sprintf('%s angle %.1f deg outside limits [%g, %g]', ...
                      names{k}, q(k), P.lim(k,1), P.lim(k,2));
        return
    end
end
ok = true;
end
