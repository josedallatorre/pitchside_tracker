import math

# Euclidean Distance
def distance(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )

def normalize(dx, dy):
    mag = math.sqrt(dx**2 + dy**2)
    if mag == 0:
        return 0.0, 0.0
    return dx / mag, dy / mag