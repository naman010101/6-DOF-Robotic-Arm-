function env = sim_environment_collision(S)
%SIM_ENVIRONMENT_COLLISION Board and box walls as collision boxes (table = board plane outside the base).
b = S.square*8 + 2*S.frameWidth;
board = collisionBox(b, b, S.boardThickness);
board.Pose = S.boardT * trvec2tform([0 0 -S.boardThickness/2]);
env = {board};
t = 0.004; L = S.boxSize(1); W = S.boxSize(2); H = S.boxSize(3); c = S.boxCentre;
walls = {[L t H; 0  W/2 H/2], [L t H; 0 -W/2 H/2], [t W H;  L/2 0 H/2], [t W H; -L/2 0 H/2], [L W t; 0 0 t/2]};
for k = 1:numel(walls)
    s = walls{k};
    w = collisionBox(s(1,1), s(1,2), s(1,3));
    w.Pose = trvec2tform(c + s(2,:));
    env{end+1} = w; %#ok<AGROW>
end
end
