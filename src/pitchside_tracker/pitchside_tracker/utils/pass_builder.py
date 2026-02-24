from pitchside_tracker.utils.geometry_utils import normalize

def build_pass_positions(player_positions, ball_height=0.3, offset=0.8):
    pass_positions = []

    for i in range(len(player_positions)):
        x1, y1, _ = player_positions[i]
        x2, y2, _ = player_positions[(i + 1) % len(player_positions)]

        dx = x2 - x1
        dy = y2 - y1

        ux, uy = normalize(dx, dy)

        target_x = x2 - ux * offset
        target_y = y2 - uy * offset

        pass_positions.append((target_x, target_y, ball_height))

    pass_positions.append(pass_positions[0])
    return pass_positions