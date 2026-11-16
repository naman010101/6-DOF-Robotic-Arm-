function Squares = CreateChessBoard()

%% Chessboard Parameters

square = 0.033;          % 33 mm

% Robot coordinates of board center
centerX = 0.1250;
centerY = 0.0000;
centerZ = 0.0151;

files = 'abcdefgh';

Squares = struct;

for rank = 1:8

    % Rank 1 = -3.5 squares
    y = -(rank - 4.5) * square;

    for file = 1:8

        % File a = -3.5 squares
        x = -(file - 4.5) * square;

        name = sprintf('%c%d',files(file),rank);

        Squares.(name) = [ ...
            centerX + x,...
            centerY + y,...
            centerZ];

    end
end

end