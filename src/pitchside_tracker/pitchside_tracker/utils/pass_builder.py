from pitchside_tracker.utils.geometry_utils import normalize

# Helper function that creates the positions the ball will reach
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
        # The offset is used to space out ball position from the player position
        kick_x = x1 + ux * offset 
        kick_y = y1 + uy * offset
        pass_positions.append((kick_x, kick_y, ball_height))

    return pass_positions