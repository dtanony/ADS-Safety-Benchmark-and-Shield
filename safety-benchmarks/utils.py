import numpy as np

def edges_intersect(A, B, C, D):
    return (sign_line_eq(A, C, D) * sign_line_eq(B, C, D) <= 0 and
            sign_line_eq(C, A, B) * sign_line_eq(D, A, B) <= 0)

def is_collision(ego_vertices, npc_vertices):
    A, B, C, D = ego_vertices
    E, F, G, H = npc_vertices

    # Vertex containment
    for P in npc_vertices:
        if point_inside_rect(P, A,B,C,D):
            return True
    for Q in ego_vertices:
        if point_inside_rect(Q, E,F,G,H):
            return True

    # Edge intersection
    ego_edges = [(A,B), (B,C), (C,D), (D,A)]
    npc_edges = [(E,F), (F,G), (G,H), (H,E)]
    for e1 in ego_edges:
        for e2 in npc_edges:
            if edges_intersect(*e1, *e2):
                return True

    return False

def sign_line_eq(P, A, B):
    """
    Suppose AB has the line equation ax+by+c=0.
    This func returns a*px+b*py+c, where P(px,py
    """
    (ax, ay), (bx, by) = A, B
    px, py = P
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax)

def point_inside_rect(P, A,B,C,D):
    """
    Check if point P is inside or on the rectangle defined by points A, B, C, D.
    Points should be given as (x, y) tuples or numpy arrays.
    Assumes the points A, B, C, D are given in order (clockwise or counterclockwise).
    """
    signs = [
        sign_line_eq(P, A, B),
        sign_line_eq(P, B, C),
        sign_line_eq(P, C, D),
        sign_line_eq(P, D, A),
    ]
    all_pos = all(s >= 0 for s in signs)
    all_neg = all(s <= 0 for s in signs)
    return all_pos or all_neg

def rotate_point(point, pivot, angle):
    """
    Rotate `point` around `pivot` by `angle` radians (counter-clockwise).
    """
    # Translate point so pivot is origin
    translated_point = np.array(point) - np.array(pivot)

    # Rotation matrix
    rot = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])

    # Rotate and translate back
    rotated_point = rot @ translated_point
    return rotated_point + pivot

def heading_from_vector(vector):
    return np.arctan2(vector[1], vector[0])

def signed_angle_2d(a, b):
    """
    Returns signed angle from vector a to vector b.
    Positive if counter-clockwise, negative if clockwise.
    """
    a_norm = a / np.linalg.norm(a)
    b_norm = b / np.linalg.norm(b)
    angle = np.arctan2(b_norm[1], b_norm[0]) - np.arctan2(a_norm[1], a_norm[0])
    # Keep angle in [-180, 180]
    if angle > np.pi:
        angle -= 2*np.pi
    elif angle < -np.pi:
        angle += 2*np.pi
    return angle