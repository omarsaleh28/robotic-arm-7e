function [cmd, pulses] = al5d_angles_to_ssc32(q, P, grip, time_ms)
%AL5D_ANGLES_TO_SSC32  Convert joint angles to servo pulses and an SSC-32U command.
%   [cmd, pulses] = al5d_angles_to_ssc32(q, P, grip, time_ms)
%   q       = [th1 th2 th3 th4] (deg)
%   grip    = 'open' | 'closed' | [] (leave gripper unchanged)
%   time_ms = move duration (use slow moves, e.g. 2000, while testing)
%   cmd     = string to send to the SSC-32U, e.g. '#0P1500#1P1500...T2000'
%             (terminate with a carriage return, char(13), when sending)
%
%   The same angle->pulse maths will be ported to the STM32 firmware.

if nargin < 3, grip = []; end
if nargin < 4, time_ms = 2000; end

pulses = zeros(1,4);
cmd = '';
for k = 1:4
    ch = P.cal(k,1); c = P.cal(k,2); t0 = P.cal(k,3); s = P.cal(k,4); upd = P.cal(k,5);
    pw = c + s*(q(k) - t0)*upd;
    pw = min(max(round(pw), P.pulse_min_us), P.pulse_max_us);   % safety clamp
    pulses(k) = pw;
    cmd = [cmd sprintf('#%dP%d', ch, pw)]; %#ok<AGROW>
end
if strcmpi(grip,'open')
    cmd = [cmd sprintf('#%dP%d', P.grip_ch, P.grip_open_us)];
elseif strcmpi(grip,'closed')
    cmd = [cmd sprintf('#%dP%d', P.grip_ch, P.grip_closed_us)];
end
cmd = [cmd sprintf('T%d', time_ms)];
end
