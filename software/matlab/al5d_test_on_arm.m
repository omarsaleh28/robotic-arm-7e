%% AL5D_TEST_ON_ARM  Accuracy test on the real arm through the SSC-32U
%  1. Connect the SSC-32U by USB, power the servos at 6 V, check the COM port
%     in Device Manager and set it below.
%  2. Run this script. The arm goes to each test point SLOWLY and waits.
%  3. Measure where the gripper centre actually is (ruler from the base centre)
%     and type it in. Results are saved to arm_test_log.csv (V&V evidence).
%
%  Keep a hand on the power switch. Stop with Ctrl+C at any time.
%  Do NOT run until the calibration in al5d_params.m has been checked
%  (step 1: centre pulse and direction of each joint).

clear; clc;
P = al5d_params();
PORT = "COM3";          % <-- change to your port (Mac/Linux: "/dev/tty...")
MOVE_MS = 3000;         % slow moves while testing

% Test points [x y z] in mm (gripper centre), all well inside the current reach.
% Add a 500 mm point once the extension is fitted.
targets = [ 200    0   80;
            250    0   80;
            300    0   80;
            200  150   80;
            150 -150   80 ];

s = serialport(PORT, 9600);          % SSC-32U default baud
configureTerminator(s, "CR");        % every command ends with carriage return
pause(1);

% safe start pose: arm folded up near the base
home = [150 0 150];
q = al5d_ik(home, P.phi_pick, P);
writeline(s, al5d_angles_to_ssc32(q, P, 'open', 4000));
pause(4.5);

log = [];
for k = 1:size(targets,1)
    tgt = targets(k,:);
    [q, ok, msg] = al5d_ik(tgt, P.phi_pick, P);
    if ~ok
        fprintf('Point %d [%g %g %g] skipped: %s\n', k, tgt, msg); continue
    end
    cmd = al5d_angles_to_ssc32(q, P, 'open', MOVE_MS);
    fprintf('\nPoint %d: target [%g %g %g] mm\n  sending %s\n', k, tgt, cmd);
    writeline(s, cmd);
    pause(MOVE_MS/1000 + 0.5);

    m = input('  Measured gripper centre [x y z] in mm (e.g. [205 2 78]): ');
    err = norm(m - tgt);
    fprintf('  Error = %.1f mm\n', err);
    log = [log; k, tgt, m, err]; %#ok<AGROW>
end

% back to the safe pose
q = al5d_ik(home, P.phi_pick, P);
writeline(s, al5d_angles_to_ssc32(q, P, 'open', 4000));
clear s

T = array2table(log, 'VariableNames', ...
    {'point','tx','ty','tz','mx','my','mz','error_mm'});
T.date = repmat(string(datetime('now','Format','yyyy-MM-dd HH:mm')), height(T), 1);
writetable(T, 'arm_test_log.csv');
disp(T);
fprintf('Mean error %.1f mm, max %.1f mm. Saved arm_test_log.csv\n', ...
        mean(T.error_mm), max(T.error_mm));
