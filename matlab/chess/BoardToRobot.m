function robot = BoardToRobot(board)

% board is in meters

offset = 0.050;      % 50 mm

robot = zeros(3,1);

% Robot points along -X
robot(1) = -(board(2) + offset);

% Left-right
robot(2) = board(1);

% Height
robot(3) = board(3);

end