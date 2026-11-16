function S = joints_to_servo(q)
%JOINTS_TO_SERVO Joint angles [rad] -> servo command angles [deg], 0..180 (90 = joint zero).
S = max(0, min(180, round(rad2deg(q(:)') + 90)));
end
