function P = al5d_params()
%AL5D_PARAMS  All arm dimensions, limits and calibration in ONE place.
%   Change values here only; every other file reads them from P.
%   Lengths in mm, angles in degrees.
%
%   Project: Robotic Arm 7E - Golf Ball Teeing Robot (UFMFV8-15-3 GDIP)
%
%   Angle convention used throughout (side view, arm pointing forward):
%     th1 = base yaw. 0 = pointing along +x, positive = anticlockwise from above.
%     th2 = shoulder. Upper arm angle ABOVE horizontal (0 = horizontal, 90 = straight up).
%     th3 = elbow. Forearm angle RELATIVE to the upper arm (0 = straight in line,
%           negative = forearm bends down).
%     th4 = wrist pitch. Gripper angle RELATIVE to the forearm (0 = in line).
%     Gripper pitch (absolute) phi = th2 + th3 + th4. phi = -90 means pointing straight down.

%% ---------------- Link lengths (mm) ----------------
% Measured from the Lynxmotion AL5D CAD model (AL5D_2017_12_17.STEP, SolidWorks 2016)
% between joint-axis centres (servo horns / bearings), 1 Nov 2026.
% Lynxmotion's code values in brackets. Still measure the real arm (V&V evidence).
P.d1  = 71.2;   % table (base underside) -> shoulder axis height   [69]
P.a1  = 11.9;   % shoulder axis sits 11.9 mm IN FRONT of the base rotation axis
                % (horizontal offset towards the gripper)            [0, ignored before]
P.L2  = 146.3;  % shoulder axis -> elbow axis (humerus)              [147]
P.L3  = 187;    % elbow axis -> wrist-pitch axis (ulna/forearm). Lynxmotion AL5D breakdown
                % sheet: 7.375 in. The CAD (6-DOF wrist-rotate model) gave 180.7. MEASURE.
P.Lg  = 86;     % wrist-pitch axis -> gripper tip, 3.375 in (Lynxmotion sheet, no wrist rotate).
                % Our arm has NO wrist rotate (confirmed 1 Nov): 5 servos.
                % CAD: stock fingertips are 93.8 mm from the wrist-pitch axis (the CAD
                % model includes a wrist-rotate servo). The team's curved claw tips
                % (ClawTipv6) change this: set it from the assembly once fixed.

% Reach extension (mechanical team). It isn't known yet where it will sit,
% so both options are provided. Set the one that matches the design.
P.ext_upper = 0;   % mm added to the upper arm (humerus)
P.ext_fore  = 0;   % mm added to the forearm (ulna)

%% ---------------- Golf-specific geometry ----------------
P.ball_d       = 42.67;  % R&A minimum ball diameter (mm)
P.ball_mass_g  = 45.93;  % R&A maximum ball mass (g)
P.z_pick       = 21.3;   % height of ball CENTRE when picked (mm). 21.3 = ball resting
                         % on the table. Raise this if picking from a raised rack.
P.z_tee        = 21.3 + 25;  % ball centre height on the tee (mm). PLACEHOLDER:
                             % set to (tee height above table) + ball radius.
P.phi_pick     = -90;    % gripper pitch used to pick the ball (deg). -90 = from above
                         % (simplest, self-centring), 0 = from the side (most reach).
                         % Section 2 of al5d_main compares several options.
P.r_required   = 500;    % brief: >= 50 cm from base centre to the picked object
P.r_margin     = 20;     % design margin on top of the requirement (mm)

%% ---------------- Joint limits (deg), in the convention above ----------------
% PLACEHOLDERS - measure the safe limits on the real arm with the SSC-32U and
% convert them to this convention.
P.lim = [ -90   90;    % th1 base
            0  180;    % th2 shoulder
         -180    0;    % th3 elbow (only bends downwards)
          -90   90 ];  % th4 wrist

%% ---------------- Servo calibration (for SSC-32U testing / STM32 later) ----------------
% pulse_us = centre_us + dir * (theta - theta_at_centre) * us_per_deg
% us_per_deg from Hitec datasheets (1 Nov): HS-485HB 0.102 deg/us -> 9.8 us/deg;
% HS-805BB / HS-755HB / HS-645MG "45 deg per 400 us" -> 8.9 us/deg. Neutral = 1500 us.
% centre/theta_at_centre/dir STILL NEED CALIBRATING on the real arm: they depend on
% how each horn was fitted during assembly.
%            ch  centre_us  theta_at_centre  dir  us_per_deg
P.cal = [     0    1500           0          +1      9.8 ;   % base     HS-485HB
              1    1500          90          +1      8.9 ;   % shoulder HS-805BB
              2    1500         -90          +1      8.9 ;   % elbow    HS-755HB
              3    1500           0          +1      8.9 ];  % wrist    HS-645MG
P.grip_ch        = 4;     % gripper HS-422
P.grip_open_us   = 1200;  % PLACEHOLDER
P.grip_closed_us = 1900;  % PLACEHOLDER
P.pulse_min_us   = 900;   % hard safety clamp: Hitec datasheet range 900-2100 us.
P.pulse_max_us   = 2100;  % Widen only after testing each joint on the real arm.

%% ---------------- Masses for the static torque check (g) ----------------
% Servo masses from Hitec datasheets; structure masses are ESTIMATES.
% Weigh the real parts (kitchen scale) and update.
P.m_elbow_servo  = 110;   % HS-755HB, sits at the elbow
P.m_wrist_servo  = 55;    % HS-645MG, sits at the wrist
P.m_grip_servo   = 46;    % HS-422, in the gripper
P.m_upper_link   = 60;    % upper-arm tube + brackets (estimate)
P.m_fore_link    = 50;    % forearm tube + brackets (estimate)
P.m_gripper_body = 5;     % claw tip ~5 g (team). CAD: ClawTipv6 2.8 cm3 (~3.4 g solid PLA) and
                          % ClawBracketv1 1.6 cm3 (~2.0 g); two of each = ~11 g solid. Excl. servo.
P.m_extension    = 40;    % reach extension (estimate, mech team to update)
% Stall torques at 6 V (team's AL5D servo datasheet sheet), kg*cm
P.stall_shoulder = 24.7;  % HS-805BB
P.stall_elbow    = 13.2;  % HS-755HB
P.stall_wrist    = 9.6;   % HS-645MG
P.torque_limit_frac = 0.5;  % keep continuous holding torque <= 50% of stall

%% ---------------- Derived ----------------
P.L2e = P.L2 + P.ext_upper;   % effective upper arm
P.L3e = P.L3 + P.ext_fore;    % effective forearm
end
