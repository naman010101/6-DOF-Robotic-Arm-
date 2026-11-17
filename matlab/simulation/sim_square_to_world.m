function p = sim_square_to_world(S, square)
%SIM_SQUARE_TO_WORLD Centre of a chess square on the board surface, in world coordinates.
square = char(square);
file = double(square(1)) - double('a') + 1;   % 1..8
rank = str2double(square(2:end));             % 1..8
local = [(rank - 4.5) * S.square, (file - 4.5) * S.square, 0, 1];
w = S.boardT * local';
p = w(1:3)';
end
