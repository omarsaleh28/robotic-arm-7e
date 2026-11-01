%% AL5D_MAIN  Kinematics, workspace and reach check for the golf-ball teeing robot
%  Project: Robotic Arm 7E (UFMFV8-15-3 GDIP), robotics/control workstream
%
%  Run this file (F5). It does five things:
%   1. Verifies forward kinematics two ways (geometry vs Denavit-Hartenberg)  -> V&V evidence
%   2. Checks the 50 cm working-envelope requirement and sizes the extension -> feeds mech team
%   3. Plots the workspace (side and top view)                               -> report figure
%   4. Verifies inverse kinematics by a round-trip test (IK -> FK)           -> V&V evidence
%   5. Generates a pick-and-place sequence as SSC-32U commands               -> test on real arm
%   6. Estimates static holding torque at the joints                         -> feeds mech/FEA
%
%  All numbers live in al5d_params.m. Update them after measuring the real arm.
%  No toolbox needed. Optional Robotics Toolbox (Peter Corke) check at the end.

clear; clc; close all;
P = al5d_params();

fprintf('=== AL5D model: d1=%g  a1=%g  L2=%g(+%g)  L3=%g(+%g)  Lg=%g mm ===\n', ...
        P.d1, P.a1, P.L2, P.ext_upper, P.L3, P.ext_fore, P.Lg);

%% 1. FK verification: geometric method vs DH method --------------------------
N = 2000;
qs = P.lim(:,1)' + rand(N,4).*(P.lim(:,2)-P.lim(:,1))';
err = zeros(N,1);
for i = 1:N
    p_geo = al5d_fk(qs(i,:), P);
    T     = al5d_fk_dh(qs(i,:), P);
    err(i) = norm(p_geo - T(1:3,4)');
end
fprintf('\n[1] FK check (geometry vs DH), %d random poses: max difference = %.2e mm  -> %s\n', ...
        N, max(err), pass_fail(max(err) < 1e-6));

%% 2. Working envelope: reach in the PICK pose --------------------------------
% The brief requires >= 500 mm from the base centre to the object picked.
% KEY POINT: the pick pitch matters. Picking from above (-90) wastes the gripper
% length (it hangs below the wrist); picking from the side (0) adds it to the reach.
r_target = P.r_required + P.r_margin;
r_scan   = 0:0.5:900;
bend     = 20;           % practical design: keep the elbow bent >= 20 deg at max reach
                         % (dead-straight arm = singularity + maximum load)

fprintf('\n[2] Reach check: ball centre at z = %.1f mm, target r = %g mm (%g + %g margin)\n', ...
        P.z_pick, r_target, P.r_required, P.r_margin);
fprintf('    %-26s %12s %10s %18s %18s\n', 'Pick pitch', 'max r (mm)', '>=520?', ...
        'ext. min (mm)', 'ext. practical');
for phi = unique([P.phi_pick -90 -45 0])
    ok_v = false(size(r_scan));
    for k = 1:numel(r_scan)
        [~, ok_v(k)] = al5d_ik([r_scan(k) 0 P.z_pick], phi, P);
    end
    rmax = max([0 r_scan(ok_v)]);
    % shoulder -> wrist distance needed to put the TCP at r_target with this pitch
    rw_t = r_target - P.a1 - P.Lg*cosd(phi);
    zw_t = P.z_pick - P.d1 - P.Lg*sind(phi);
    dist = hypot(rw_t, zw_t);
    ext_min = max(0, dist - (P.L2 + P.L3));
    ext_prac = NaN;
    for e = 0:0.5:500                        % extension added to the forearm
        L3t = P.L3 + e;
        D = (dist^2 - P.L2^2 - L3t^2) / (2*P.L2*L3t);
        if abs(D) <= 1 && acosd(D) >= bend, ext_prac = e; break; end
    end
    tag = sprintf('%+d deg', phi);
    if phi == -90, tag = [tag ' (from above)']; elseif phi == 0, tag = [tag ' (from side)']; end
    if phi == P.phi_pick, tag = [tag ' *']; end
    fprintf('    %-26s %12.1f %10s %18.0f %18.0f\n', tag, rmax, pass_fail(rmax >= r_target), ...
            ext_min, ext_prac);
end
fprintf('    * = pitch selected in al5d_params (P.phi_pick). Extension figures assume Lg = %g mm,\n', P.Lg);
fprintf('      joint limits ignored for the extension sizing; "practical" keeps the elbow bent >= %d deg.\n', bend);

% reach for the selected pitch (used by the plots and later sections)
reach_ok = false(size(r_scan));
for k = 1:numel(r_scan)
    [~, reach_ok(k)] = al5d_ik([r_scan(k) 0 P.z_pick], P.phi_pick, P);
end
r_max_pick = max([0 r_scan(reach_ok)]);

%% 3. Workspace plots ---------------------------------------------------------
Nw = 20000;
qw = P.lim(:,1)' + rand(Nw,4).*(P.lim(:,2)-P.lim(:,1))';
pw = zeros(Nw,3);
for i = 1:Nw, pw(i,:) = al5d_fk(qw(i,:), P); end
rw = hypot(pw(:,1), pw(:,2));
above = pw(:,3) >= 0;                        % ignore poses below the table

figure('Name','AL5D workspace','Color','w','Position',[100 100 1100 450]);
subplot(1,2,1); hold on; grid on; axis equal;
scatter(rw(above), pw(above,3), 2, [0.6 0.7 0.9], 'filled');
% pick-pose reach line
rp = r_scan(reach_ok);
plot(rp, P.z_pick*ones(size(rp)), 'b-', 'LineWidth', 3);
xlim([-20, max(r_target, max(rw))+60]); ylim([-20, max(pw(:,3))+40]);
xline_(P.r_required, 'r--', '500 mm requirement');
yline_(0, 'k-', 'table');
% draw the arm at the furthest pick pose
if any(reach_ok)
    qd = al5d_ik([r_max_pick 0 P.z_pick], P.phi_pick, P);
    draw_arm_side(qd, P);
end
xlabel('Horizontal distance from base axis r (mm)'); ylabel('Height z (mm)');
title('Side view: reachable TCP positions (blue line = pick reach)');

subplot(1,2,2); hold on; grid on; axis equal;
scatter(pw(above,1), pw(above,2), 2, [0.6 0.7 0.9], 'filled');
t = linspace(-pi, pi, 200);
plot(P.r_required*cos(t), P.r_required*sin(t), 'r--', 'LineWidth', 1.5);
plot(r_max_pick*cos(t(abs(t)<=pi/2)), r_max_pick*sin(t(abs(t)<=pi/2)), 'b-', 'LineWidth', 2);
lim_xy = max(r_target, max(rw)) + 60; xlim([-lim_xy lim_xy]); ylim([-lim_xy lim_xy]);
xlabel('x (mm)'); ylabel('y (mm)');
title('Top view (red = 500 mm, blue = pick reach)');

%% 4. IK verification: round trip IK -> FK on random reachable targets -------
Nt = 1000; n_ok = 0; maxe = 0; n_try = 0;
while n_ok < Nt && n_try < 50*Nt
    n_try = n_try + 1;
    th  = -90 + 180*rand;                    % within base limits
    r   = 80 + (r_max_pick - 80)*rand;
    z   = P.z_pick + 200*rand;
    tgt = [r*cosd(th) r*sind(th) z];
    [q, ok] = al5d_ik(tgt, P.phi_pick, P);
    if ~ok, continue; end
    [p, phi] = al5d_fk(q, P);
    maxe = max([maxe, norm(p - tgt), abs(phi - P.phi_pick)]);
    n_ok = n_ok + 1;
end
fprintf('\n[4] IK round-trip test: %d reachable targets, max position/pitch error = %.2e  -> %s\n', ...
        n_ok, maxe, pass_fail(maxe < 1e-6));

%% 5. Example pick-and-place sequence as SSC-32U commands ---------------------
% Positions are EXAMPLES inside the current reach. Replace with the rack and tee
% positions once the layout is fixed (later these come from teach mode / the AI).
rack = [min(250, r_max_pick-10)   0   P.z_pick];   % ball pick-up point
tee  = [0   min(250, r_max_pick-10)   P.z_tee];    % tee, 90 deg to the left
hover = 60;                                         % approach height above target (mm)

seq = { 'Above rack',  rack + [0 0 hover], 'open',   2000;
        'Down to ball',rack,               'open',   1500;
        'Grip',        rack,               'closed', 800;
        'Lift',        rack + [0 0 hover], 'closed', 1500;
        'Above tee',   tee  + [0 0 hover], 'closed', 2000;
        'Down to tee', tee,                'closed', 1500;
        'Release',     tee,                'open',   800;
        'Retreat',     tee  + [0 0 hover], 'open',   1500 };

fprintf('\n[5] Pick-and-place sequence (send each line to the SSC-32U, ending with CR):\n');
for s = 1:size(seq,1)
    [q, ok, msg] = al5d_ik(seq{s,2}, P.phi_pick, P);
    if ~ok
        fprintf('    %-13s UNREACHABLE: %s\n', seq{s,1}, msg); continue
    end
    cmd = al5d_angles_to_ssc32(q, P, seq{s,3}, seq{s,4});
    fprintf('    %-13s q = [%6.1f %6.1f %6.1f %6.1f]  ->  %s\n', seq{s,1}, q, cmd);
end
fprintf('    !! Calibration in al5d_params.m is PLACEHOLDER. Calibrate before sending to the arm.\n');

%% 6. Static holding torque -------------------------------------------------
% (a) current geometry at its furthest pick, (b) the arm WITH the practical
% extension holding a ball at the 500 mm requirement. (b) is the design case.
stall = [P.stall_shoulder P.stall_elbow P.stall_wrist];
names = {'Shoulder (HS-805BB)','Elbow (HS-755HB)','Wrist (HS-645MG)'};
Pe = P;                                     % model with the practical forearm extension
rw_t = r_target - P.a1 - P.Lg*cosd(P.phi_pick);
zw_t = P.z_pick - P.d1 - P.Lg*sind(P.phi_pick);
dist = hypot(rw_t, zw_t);
for e = 0:0.5:500
    L3t = P.L3 + e; D = (dist^2 - P.L2^2 - L3t^2) / (2*P.L2*L3t);
    if abs(D) <= 1 && acosd(D) >= bend, Pe.ext_fore = P.ext_fore + e; break; end
end
Pe.L3e = Pe.L3 + Pe.ext_fore;
Pe.lim(:,1) = -360; Pe.lim(:,2) = 360;      % limits ignored for this load case
cases = { P,  [r_max_pick 0 P.z_pick], sprintf('current geometry, ball at r = %.0f mm', r_max_pick);
          Pe, [r_target   0 P.z_pick], sprintf('with %.0f mm forearm extension, ball at r = %.0f mm', ...
                                               Pe.ext_fore, r_target) };
fprintf('\n[6] Static holding torque (masses partly ESTIMATED - weigh the parts):\n');
for c = 1:size(cases,1)
    [qd, okd] = al5d_ik(cases{c,2}, P.phi_pick, cases{c,1});
    if ~okd, continue; end
    tau = al5d_static_torque(qd, cases{c,1}, true);
    fprintf('    Case %d: %s\n', c, cases{c,3});
    for j = 1:3
        fprintf('      %-20s %5.1f kg*cm of %4.1f stall = %3.0f%%  -> %s\n', names{j}, tau(j), ...
                stall(j), 100*tau(j)/stall(j), pass_fail(tau(j) <= P.torque_limit_frac*stall(j)));
    end
end
fprintf('    (PASS = below %.0f%% of stall torque, a common rule for continuous holding)\n', ...
        100*P.torque_limit_frac);

%% Optional: cross-check with Peter Corke's Robotics Toolbox (as in Kinematics Part 2)
if exist('SerialLink', 'file') || exist('SerialLink', 'class')
    L(1) = Link('d', P.d1, 'a', 0,     'alpha', pi/2);
    L(2) = Link('d', 0,    'a', P.L2e, 'alpha', 0);
    L(3) = Link('d', 0,    'a', P.L3e, 'alpha', 0);
    L(4) = Link('d', 0,    'a', P.Lg,  'alpha', 0);
    arm = SerialLink(L, 'name', 'AL5D');
    q0  = deg2rad([20 60 -100 -50]);
    Tt  = arm.fkine(q0);
    if isobject(Tt), Tt = Tt.T; end
    fprintf('\n[opt] Robotics Toolbox FK vs ours: diff = %.2e mm\n', ...
            norm(Tt(1:3,4)' - al5d_fk(rad2deg(q0), P)));
    figure('Name','Robotics Toolbox model'); arm.plot(q0, 'workspace', [-600 600 -600 600 -100 600]);
else
    fprintf('\n[opt] Robotics Toolbox not found - skipped (not needed).\n');
end

%% ---------------- helpers (Octave/MATLAB compatible) ----------------
function s = pass_fail(c)
if c, s = 'PASS'; else, s = 'FAIL'; end
end

function xline_(x, style, label)
yl = ylim; plot([x x], yl, style, 'LineWidth', 1.5);
text(x, yl(2), [' ' label], 'Color', 'r', 'VerticalAlignment', 'top');
end

function yline_(y, style, label)
xl = xlim; plot(xl, [y y], style);
text(xl(1), y, [' ' label], 'VerticalAlignment', 'bottom');
end

function draw_arm_side(q, P)
[~, Ts] = al5d_fk_dh(q, P);
pts = squeeze(Ts(1:3,4,:))';
r = hypot(pts(:,1), pts(:,2));
plot([0; r], [0; pts(:,3)], 'k-o', 'LineWidth', 2.5, 'MarkerFaceColor', 'y');
end
