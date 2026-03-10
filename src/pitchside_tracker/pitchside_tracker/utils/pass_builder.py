from pitchside_tracker.utils.geometry_utils import normalize

def build_pass_positions(player_positions, ball_height=0.3, offset=0.8):
    n = len(player_positions)
    pass_positions = []

    for i in range(n):
        x1, y1, _ = player_positions[i]
        x2, y2, _ = player_positions[(i + 1) % n]
        dx = x2 - x1
        dy = y2 - y1
        ux, uy = normalize(dx, dy)

        # Kick originates just in front of the SENDER (player i)
        kick_x = x1 + ux * offset
        kick_y = y1 + uy * offset
        pass_positions.append((kick_x, kick_y, ball_height))

    # Close the loop: return to the first kick origin
    pass_positions.append(pass_positions[0])
    return pass_positions