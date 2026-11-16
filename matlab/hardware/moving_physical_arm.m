clc;
clear;

%% Import Robot
robot = importrobot(fullfile(fileparts(mfilename("fullpath")),"..","..","urdf","urdf","robot.urdf"));
robot.DataFormat = "row";

%% Connect to ESP32
s = serialport("COM9",115200);

flush(s);

%% Create Figure
figure;

while true

    if s.NumBytesAvailable > 0

        line = readline(s);

        angles = str2double(split(line,","));

        if numel(angles) == 6

            config = homeConfiguration(robot);

            % Joint Corrections
            config(1) = deg2rad(angles(1) - 90);
            config(2) = deg2rad(angles(2) - 90);
            config(3) = deg2rad(angles(3) - 90);
            config(4) = deg2rad(angles(4) - 90);
            config(5) = deg2rad(angles(5) - 90);

            % Gripper joint is fixed in the URDF (5 movable joints), so there is no config(6)

            show(robot,config,'PreservePlot',false);

            xlim([-0.25 0.25]);
            ylim([-0.25 0.25]);
            zlim([0 0.50]);

            drawnow;

        end

    end

end