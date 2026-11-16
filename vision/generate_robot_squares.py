import json

SQUARE = 33.0

files = "abcdefgh"

boardSquares = {}

for rank in range(8):

    # Rank 1 = -115.5 mm
    y = (rank - 3.5) * SQUARE

    for file in range(8):

        # File a = -115.5 mm
        x = (file - 3.5) * SQUARE

        square = f"{files[file]}{rank+1}"

        boardSquares[square] = {
            "board": [
                round(x,1),
                round(y,1),
                0.0
            ]
        }

with open("boardSquares.json","w") as f:
    json.dump(boardSquares,f,indent=4)

print("boardSquares.json created")