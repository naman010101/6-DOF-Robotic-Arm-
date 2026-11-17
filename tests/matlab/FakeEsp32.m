classdef FakeEsp32 < handle
    %FAKEESP32 Stands in for the serialport in tests: replies to commands the way
    %firmware/esp32_main does (status lines only), and records what was sent.
    properties
        Sent string = string.empty
        Replies string = string.empty
        FailOn string = ""        % a command to reject, to test error handling
    end
    properties (Dependent)
        NumBytesAvailable
    end
    methods
        function n = get.NumBytesAvailable(obj)
            n = sum(strlength(obj.Replies) + 1);
        end
        function writeline(obj, msg)
            msg = strtrim(string(msg));
            obj.Sent(end+1) = msg;
            obj.Replies(end+1) = "90,90,90,90,90,90";   % periodic feedback noise
            u = upper(msg);
            if msg == obj.FailOn, obj.Replies(end+1) = "# ERR rejected: " + msg; return, end
            if u == "OPEN",  obj.Replies(end+1) = "# GRIP 40";  return, end
            if u == "CLOSE", obj.Replies(end+1) = "# GRIP 110"; return, end
            if u == "HOME",  obj.Replies(end+1) = "# HOME reached"; return, end
            v = str2double(split(msg, ","));
            if any(numel(v) == [5 6]) && all(isfinite(v))
                obj.Replies(end+1) = "# OK " + strjoin(string(v(1:5)'), ",");
            else
                obj.Replies(end+1) = "# ERR expected s1,s2,s3,s4,s5[,g] but got: " + msg;
            end
        end
        function line = readline(obj)
            line = obj.Replies(1);
            obj.Replies(1) = [];
        end
    end
end
