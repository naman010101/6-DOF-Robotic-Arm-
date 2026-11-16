function F = CreateServoInterpolation()

% file index
% a=1 b=2 ... h=8
% rank = 1..8

P = [

%file rank S1 S2 S3 S4 S5

1 1 31 88 25 110 150
1 2 40 93 45 130 140
1 3 45 96 46 120 130
1 4 47 100 58 128 130
1 5 53 109 73 140 130
1 6 55 109 84 140 120
1 7 56 113 95 140 110
% 1 8   TODO: a8 not measured yet - add "1 8 S1 S2 S3 S4 S5" here (a8 is extrapolated until then)

2 1 40 84 17 118 140
2 2 48 89 28 120 135
2 3 56 92 39 124 130
2 4 59 100 58 138 125
2 5 60 105 73 142 120
2 6 60 110 84 145 110
2 7 62 113 96 140 110
2 8 61 119 108 137 100

3 1 56 84 17 125 130
3 2 60 85 18 110 120
3 3 64 86 25 127 110
3 4 66 100 58 138 115
3 5 68 105 73 144 105
3 6 69 110 84 145 100
3 7 70 113 96 145 100
3 8 71 119 108 145 95


4 1 80 85 19 127 90
4 2 80 88 29 130 90
4 3 80 92 38 130 90
4 4 81 96 48 128 90
4 5 85 100 58 128 90
4 6 92 104 68 130 90
4 7 94 108 78 127 90
4 8 94 114 88 120 90

];

x = P(:,1);
y = P(:,2);

F.S1 = scatteredInterpolant(x,y,P(:,3),'natural');
F.S2 = scatteredInterpolant(x,y,P(:,4),'natural');
F.S3 = scatteredInterpolant(x,y,P(:,5),'natural');
F.S4 = scatteredInterpolant(x,y,P(:,6),'natural');
F.S5 = scatteredInterpolant(x,y,P(:,7),'natural');

end