function [v, x] = trapezoid_ref(D, T, ta, td, t)
%TRAPEZOID_REF Speed and position reference of a timed run, the same profile
%   as Start_Timed_Profile() in car_drive.c:
%     D = A1 + A2 + A3 = Sn*ta/2 + Sn*(T - ta - td) + Sn*td/2
%     =>  Sn = D / (T - (ta + td)/2)
%   D in cm, times in s, t a vector of times. Returns v (cm/s) and x (cm).

Sn = D / (T - (ta + td) / 2);
t  = min(max(t, 0), T);           % before the start: 0, after the end: hold
t2 = T - td;                      % start of the deceleration
x1 = Sn * ta / 2;                 % distance after the acceleration (area A1)
x2 = x1 + Sn * (t2 - ta);         % distance after the cruise (A1 + A2)

acc = t < ta;
cru = t >= ta & t < t2;
dec = t >= t2;

v = acc .* (Sn * t / ta) + cru .* Sn + dec .* (Sn * (T - t) / td);
x = acc .* (Sn * t.^2 / (2 * ta)) ...
  + cru .* (x1 + Sn * (t - ta)) ...
  + dec .* (x2 + Sn * (t - t2) - Sn * (t - t2).^2 / (2 * td));
end
