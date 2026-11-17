function pick_and_place(esp, pickAngles, placeAngles, timeout)
%PICK_AND_PLACE Pick an object at one pose and drop it at another.
%   pick_and_place(esp, pickAngles, placeAngles)
%   esp         : serialport connected to firmware/esp32_main
%   pickAngles  : 1x5 servo angles of the pick pose (e.g. a chess square from the table)
%   placeAngles : 1x5 servo angles of the place pose (e.g. the box next to the board)
%
%   Sequence: open gripper -> go to pick pose -> close -> lift (HOME) -> go to place
%   pose -> open -> HOME. Each step waits for the ESP32 to confirm before the next.

arguments
    esp
    pickAngles (1,5) double
    placeAngles (1,5) double
    timeout (1,1) double = 15
end

steps = {
    "OPEN",            "# GRIP"
    packet(pickAngles),  "# OK"
    "CLOSE",           "# GRIP"
    "HOME",            "# HOME"
    packet(placeAngles), "# OK"
    "OPEN",            "# GRIP"
    "HOME",            "# HOME"
    };

for k = 1:size(steps, 1)
    writeline(esp, steps{k, 1});
    reply = waitFor(esp, steps{k, 2}, timeout);
    fprintf("%-22s -> %s\n", steps{k, 1}, reply);
end
end

function msg = packet(S)
S = max(0, min(180, round(S)));
msg = sprintf("%d,%d,%d,%d,%d", S);
end

function line = waitFor(esp, prefix, timeout)
% read lines until one starts with prefix; feedback lines (f1..f6) are skipped
t0 = tic;
while toc(t0) < timeout
    if esp.NumBytesAvailable > 0
        line = strtrim(readline(esp));
        if startsWith(line, prefix)
            return
        elseif startsWith(line, "# ERR")
            error("pick_and_place:esp32", "ESP32 rejected a command: %s", line);
        end
    else
        pause(0.02);
    end
end
error("pick_and_place:timeout", "No '%s' from the ESP32 within %g s.", prefix, timeout);
end
