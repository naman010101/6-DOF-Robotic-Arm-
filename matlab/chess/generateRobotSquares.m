clc;
clear;

%% Chessboard Parameters

squareSize = 33;     % mm
robotOffset = 50;    % mm from robot base to d1/e1 line

files = 'abcdefgh';

% X coordinates (robot centered between d and e)
x = [-115.5 -82.5 -49.5 -16.5 ...
    16.5  49.5  82.5 115.5];

Squares = struct;

for r = 1:8

    % Center of each rank
    y = robotOffset + squareSize/2 + (r-1)*squareSize;

    for c = 1:8

        name = sprintf('%c%d',files(c),r);

        Squares.(name) = [ ...
            x(c), ...
            y, ...
            0 ...
            ];

    end

end

disp(Squares.e4)

save("robotSquares.mat","Squares")