function arm_control_app()

robot = build_robot_model();

COM = "COM9";
BAUD = 115200;

 try
    esp = serialport(COM,BAUD);
    configureTerminator(esp,"LF");
    flush(esp);

    disp("ESP32 Connected")

 catch

    esp = [];

    warning("ESP32 Not Connected")

 end


%% ---------------- Chess Board ----------------

Squares = CreateChessBoard();

ServoInterp = CreateServoInterpolation();


%% ---------------- DLS IK Parameters ----------------

endEffector = "tool0";

lambda = 0.1;
alpha = 1.0;    % full damped-least-squares step: ~11 iterations per solve instead of ~125 with 0.1
tol = 1e-4;
maxIter = 300;

%% Current Target

config = homeConfiguration(robot);
configSol = config;

T = getTransform(robot,config,endEffector);
currentTarget = tform2trvec(T);
%% ---------------- Create GUI ----------------

fig = uifigure( ...
    'Name','Pick and Place Controller',...
    'Position',[100 100 1200 700]);

%% ---------------- Robot Axes ----------------

ax = uiaxes(fig);
ax.Position = [30 70 700 600];

config = homeConfiguration(robot);

show(robot,config,...
    'Parent',ax,...
    'Frames','on',...
    'PreservePlot',false);

view(ax,45,20);
axis(ax,'equal');

%% ---------------- Labels ----------------

uilabel(fig,... 
    'Text','X (m)',...
    'Position',[800 600 60 22]);

uilabel(fig,...
    'Text','Y (m)',...
    'Position',[800 550 60 22]);

uilabel(fig,...
    'Text','Z (m)',...
    'Position',[800 500 60 22]);
%% ---------------- Edit Boxes ---------------- 
xBox = uieditfield(fig,... 
   'numeric',... 
   'Position',[860 600 120 25],... 
   'Value',0.20); 

yBox = uieditfield(fig,... 
    'numeric',... 
    'Position',[860 550 120 25],... 
    'Value',0.00);

zBox = uieditfield(fig,... 
    'numeric',... 
    'Position',[860 500 120 25],... 
    'Value',0.20);

stepDrop = uidropdown(fig,...
    'Items',{'1 mm','5 mm','10 mm'},...
    'Value','5 mm',...
    'Position',[1020 500 100 25]);

squareDrop = uidropdown(fig,...
    'Items',fieldnames(Squares),...
    'Position',[820 450 120 25],...
    'Value','e2');
uibutton(fig,...
    'Text','Load Square',...
    'Position',[950 450 120 30],...
    'ButtonPushedFcn',@loadSquare);

%% ---------------- Status ----------------

statusLabel = uilabel(fig,...
    'Text','Waiting...',...
    'Position',[800 350 350 25],...
    'FontSize',14);

servo1Label = uilabel(fig,'Position',[800 300 200 22],'Text','S1 : 90');
servo2Label = uilabel(fig,'Position',[800 270 200 22],'Text','S2 : 90');
servo3Label = uilabel(fig,'Position',[800 240 200 22],'Text','S3 : 90');
servo4Label = uilabel(fig,'Position',[800 210 200 22],'Text','S4 : 90');
servo5Label = uilabel(fig,'Position',[800 180 200 22],'Text','S5 : 90');

%% ---------------- Solve Button ----------------

uibutton(fig,...
    'Text','Solve IK',...
    'Position',[820 400 160 40],...
    'ButtonPushedFcn',@solveIK);

%% ---------------- Jog Buttons ----------------

uibutton(fig,...
    'Text','+X',...
    'Position',[800 120 60 40],...
    'ButtonPushedFcn',@plusX);

uibutton(fig,...
    'Text','-X',...
    'Position',[660 120 60 40],...
    'ButtonPushedFcn',@minusX);

uibutton(fig,...
    'Text','+Y',...
    'Position',[730 70 60 40],...
    'ButtonPushedFcn',@plusY);

uibutton(fig,...
    'Text','-Y',...
    'Position',[730 170 60 40],...
    'ButtonPushedFcn',@minusY);

uibutton(fig,...
    'Text','+Z',...
    'Position',[870 120 60 40],...
    'ButtonPushedFcn',@plusZ);

uibutton(fig,...
    'Text','-Z',...
    'Position',[940 120 60 40],...
    'ButtonPushedFcn',@minusZ);

uibutton(fig,...
    'Text','Save Point',...
    'Position',[1010 120 120 40],...
    'ButtonPushedFcn',@savePoint);

%% ============================================================
%                   SOLVE IK
%% ============================================================


    function solveIK(~,~)

        x = xBox.Value;
        y = yBox.Value;
        z = zBox.Value;

        %% -------- Transform Target Into Base Frame --------
       
        target = [x y z];

       q = configSol;


        q(2) = deg2rad(-20);
        q(3) = deg2rad(30);
        q(4) = deg2rad(10);
        
        if norm(target-[0 0 0]) > 0.35
            uialert(fig,"Target outside robot workspace","IK");
            return;
        end

        [q, errNorm, k] = solve_ik_dls(robot, target, q, endEffector, ...
            Lambda=lambda, Alpha=alpha, Tol=tol, MaxIter=maxIter);

        configSol = q;
        currentTarget = target;

        statusLabel.Text = sprintf(...
            "Converged in %d iterations | Error %.2f mm",...
            k,...
            1000*errNorm);

        %% TCP verification

        Tactual = getTransform(robot,configSol,endEffector);

        actual = tform2trvec(Tactual);

        fprintf("\nDesired\n");
        disp(target)

        fprintf("Actual\n");
        disp(actual)

        fprintf("Error = %.3f mm\n",...
            1000*norm(target-actual));

        disp("====== IK Solution ======")
        disp(rad2deg(configSol))

        %% -------- Display Robot --------
        show(robot,...
            configSol,...
            'Parent',ax,...
            'Frames','on',...
            'PreservePlot',false);

        view(ax,45,20);
        axis(ax,'equal');

        %% -------- Convert To Servo Angles --------
        S1 = round(rad2deg(configSol(1)) + 90);
        S2 = round(rad2deg(configSol(2)) + 90);
        S3 = round(rad2deg(configSol(3)) + 90);
        S4 = round(rad2deg(configSol(4)) + 90);
        S5 = round(rad2deg(configSol(5)) + 90);

        %% -------- Joint Limits --------
        S1 = max(0,min(180,S1));
        S2 = max(0,min(180,S2));
        S3 = max(0,min(180,S3));
        S4 = max(0,min(180,S4));
        S5 = max(0,min(180,S5));

        %% -------- Send To ESP32 --------
        if ~isempty(esp)

            msg = sprintf("%d,%d,%d,%d,%d\n",...
                S1,S2,S3,S4,S5);

            writeline(esp,msg);

            fprintf("Sent : %s",msg);

        end

        %% -------- Update GUI --------
        servo1Label.Text = sprintf("S1 : %d°",S1);
        servo2Label.Text = sprintf("S2 : %d°",S2);
        servo3Label.Text = sprintf("S3 : %d°",S3);
        servo4Label.Text = sprintf("S4 : %d°",S4);
        servo5Label.Text = sprintf("S5 : %d°",S5);

        fprintf("\nServo Angles\n");
        fprintf("S1 = %d\n",S1);
        fprintf("S2 = %d\n",S2);
        fprintf("S3 = %d\n",S3);
        fprintf("S4 = %d\n",S4);
        fprintf("S5 = %d\n",S5);

    end

    function moveRobot(dx,dy,dz)

        switch stepDrop.Value
            case "1 mm"
                s = 0.001;
            case "5 mm"
                s = 0.005;
            case "10 mm"
                s = 0.010;
        end

        currentTarget = currentTarget + s*[dx dy dz];

        xBox.Value = currentTarget(1);
        yBox.Value = currentTarget(2);
        zBox.Value = currentTarget(3);

        solveIK();

    end

    % ===================================================
    % Jog Button Callbacks 
    % ===================================================

    function plusX(~,~)
        moveRobot(1,0,0);
    end

    function minusX(~,~)
        moveRobot(-1,0,0);
    end

    function plusY(~,~)
        moveRobot(0,1,0);
    end

    function minusY(~,~)
        moveRobot(0,-1,0);
    end

    function plusZ(~,~)
        moveRobot(0,0,1);
    end

    function minusZ(~,~)
        moveRobot(0,0,-1);
    end

    % ===================================================
    function savePoint(~,~)

        T = getTransform(robot,configSol,endEffector);

        p = tform2trvec(T);

        fprintf("\n========================\n");
        fprintf("Robot TCP Position\n");
        fprintf("X = %.4f\n",p(1));
        fprintf("Y = %.4f\n",p(2));
        fprintf("Z = %.4f\n",p(3));
        fprintf("========================\n");

       
    end

    function loadSquare(~,~)

        square = squareDrop.Value;

        % convert chess notation to numbers
        file = double(square(1)) - double('a') + 1;
        rank = str2double(square(2));

        % interpolate servo angles
        S1 = round(ServoInterp.S1(file,rank));
        S2 = round(ServoInterp.S2(file,rank));
        S3 = round(ServoInterp.S3(file,rank));
        S4 = round(ServoInterp.S4(file,rank));
        S5 = round(ServoInterp.S5(file,rank));

        % Only files a-d are measured; e-h are extrapolated and can fall outside
        % the servo range, so clamp before sending.
        S = max(0,min(180,[S1 S2 S3 S4 S5]));
        if file > 4 || any(S ~= [S1 S2 S3 S4 S5])
            warning("Square %s is outside the measured table (files a-d); angles are extrapolated.",square);
        end
        S1 = S(1); S2 = S(2); S3 = S(3); S4 = S(4); S5 = S(5);

        fprintf("\n%s\n",square);
        fprintf("S1=%d S2=%d S3=%d S4=%d S5=%d\n",...
            S1,S2,S3,S4,S5);

        % Update labels
        servo1Label.Text = sprintf("S1 : %d°",S1);
        servo2Label.Text = sprintf("S2 : %d°",S2);
        servo3Label.Text = sprintf("S3 : %d°",S3);
        servo4Label.Text = sprintf("S4 : %d°",S4);
        servo5Label.Text = sprintf("S5 : %d°",S5);

        % Send to ESP32
        if ~isempty(esp)

            msg = sprintf("%d,%d,%d,%d,%d\n",...
                S1,S2,S3,S4,S5);

            writeline(esp,msg);

            fprintf("Sent : %s",msg);

        end

    end


end