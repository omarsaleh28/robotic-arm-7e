function tau = al5d_static_torque(q, P, with_ball)
%AL5D_STATIC_TORQUE  Gravity holding torque at shoulder, elbow and wrist (kg*cm).
%   tau = al5d_static_torque(q, P, with_ball)
%   tau = [shoulder elbow wrist], kg*cm (same unit as servo datasheets)
%   Simple lumped-mass model (each link mass at its midpoint, servos at the
%   joints). It is a first estimate for sizing; the FEA/hand calcs refine it.

if nargin < 3, with_ball = true; end
th2 = q(2); a3 = q(2)+q(3); phi = q(2)+q(3)+q(4);

% horizontal positions (mm) measured from the shoulder axis
x_elbow  = P.L2e*cosd(th2);
x_wrist  = x_elbow + P.L3e*cosd(a3);
x_tcp    = x_wrist + P.Lg*cosd(phi);
x_upmid  = 0.5*x_elbow;
x_fomid  = x_elbow + 0.5*P.L3e*cosd(a3);
x_grmid  = x_wrist + 0.5*P.Lg*cosd(phi);
% extension assumed spread along the link it is added to
if P.ext_fore > 0,  x_ext = x_fomid; else, x_ext = x_upmid; end

m_ball = with_ball * P.ball_mass_g;

ext_on_fore = P.ext_fore > 0;
m_ext = P.m_extension * ((P.ext_fore + P.ext_upper) > 0);   % no extension fitted -> no mass
% NB: abs() treats any mass behind the joint as adding torque (conservative).

% list of [mass_g, x_mm, carried_by_elbow, carried_by_wrist]
loads = [ P.m_upper_link,  x_upmid, 0, 0;
          P.m_elbow_servo, x_elbow, 0, 0;
          P.m_fore_link,   x_fomid, 1, 0;
          m_ext,           x_ext,   ext_on_fore, 0;
          P.m_wrist_servo, x_wrist, 1, 0;
          P.m_gripper_body + P.m_grip_servo, x_grmid, 1, 1;
          m_ball,          x_tcp,   1, 1 ];

% torque (g*mm) -> kg*cm : /1000 (g->kg) /10 (mm->cm)
conv = 1/10000;
m = loads(:,1); x = loads(:,2); e = loads(:,3) == 1; w = loads(:,4) == 1;
t_sh = sum(m .* abs(x))                    * conv;
t_el = sum(m(e) .* abs(x(e) - x_elbow))    * conv;
t_wr = sum(m(w) .* abs(x(w) - x_wrist))    * conv;
tau = [t_sh t_el t_wr];
end
