import random
# class PenColor():
#     def __init__(self, r, g, b, width, off) -> None:
#         self.r = r
#         self.g = g
#         self.b = b
#         self.width = width
#         self.off = off

# team_colors = {}

# team_colors[1] = PenColor(100, 100, 100, 10, False)
# team_colors[2] = PenColor(200, 200, 200, 5, False)

# print(team_colors)


# my_team = 1
# enemy_colors = [color for (team, color) in (team_colors.keys(), team_colors.values()) if team !=my_team]

# print(enemy_colors)

player_count = 32

chaser_count = round((player_count/4)/2)

chasers = []
chasers.append(random.choices([i for i in range(0,player_count) if i%2 == 0], k=chaser_count))
chasers.append(random.choices([i for i in range(0,player_count) if i%2 == 1], k=chaser_count))

print(chasers[0])
print(chasers[1])
 