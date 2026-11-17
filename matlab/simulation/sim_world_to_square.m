function [square, file, rank] = sim_world_to_square(S, p)
%SIM_WORLD_TO_SQUARE Chess square under a world point ("" if it is off the board).
local = S.boardT \ [p(:); 1];
rank = floor(local(1) / S.square + 4) + 1;
file = floor(local(2) / S.square + 4) + 1;
if rank < 1 || rank > 8 || file < 1 || file > 8
    square = ""; return
end
square = string(char('a' + file - 1)) + rank;
end
