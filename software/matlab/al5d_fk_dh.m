function [T, Ts] = al5d_fk_dh(q, P)
%AL5D_FK_DH  Forward kinematics using standard (distal) Denavit-Hartenberg
%   parameters and homogeneous transforms, as taught in Kinematics Part 2.
%   This is an independent check of al5d_fk (two methods must agree).
%
%   DH table (standard / distal convention), angles in degrees:
%     n |  a_n   | alpha_n |  d_n  | theta_n
%     1 |  a1    |   90    |  d1   | th1
%     2 |  L2e   |    0    |   0   | th2
%     3 |  L3e   |    0    |   0   | th3
%     4 |  Lg    |    0    |   0   | th4
%
%   T  = 4x4 base->TCP transform. Position = T(1:3,4).
%   Ts = 4x4xN transforms of every frame (for plotting the arm).

dh = [ P.a1   90  P.d1  q(1);
       P.L2e   0  0     q(2);
       P.L3e   0  0     q(3);
       P.Lg    0  0     q(4) ];

T  = eye(4);
Ts = zeros(4,4,size(dh,1)+1);
Ts(:,:,1) = T;
for n = 1:size(dh,1)
    T = T * dh_transform(dh(n,1), dh(n,2), dh(n,3), dh(n,4));
    Ts(:,:,n+1) = T;
end
end

function A = dh_transform(a, alpha, d, theta)
% Standard DH: A = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
ct = cosd(theta); st = sind(theta);
ca = cosd(alpha); sa = sind(alpha);
A = [ ct  -st*ca   st*sa   a*ct;
      st   ct*ca  -ct*sa   a*st;
       0     sa      ca      d ;
       0      0       0      1 ];
end
