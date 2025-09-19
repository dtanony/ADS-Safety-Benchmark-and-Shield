import numpy as np

def round_float(input):
    return round(float(input), 3)

def is_collision(ego_vertices, npc_vertices):
    A, B, C, D = ego_vertices
    E, F, G, H = npc_vertices
    for P in npc_vertices:
        if point_inside_rect(P, A,B,C,D):
            return True
    for Q in ego_vertices:
        if point_inside_rect(Q, E,F,G,H):
            return True

    return False

def sign_line_eq(P, A, B):
    """
    Suppose AB has the line equation ax+by+c=0.
    This func returns a*px+b*py+c, where P(px,py
    """
    ax, ay = A
    bx, by = B
    px,py = P
    m = float(ay - by)
    n = float(bx - ax)
    return m*(px - ax) + n*(py - ay)

def point_inside_rect(P, A,B,C,D):
    """
    Check if point P is inside or on the rectangle defined by points A, B, C, D.
    Points should be given as (x, y) tuples or numpy arrays.
    Assumes the points A, B, C, D are given in order (clockwise or counterclockwise).
    """
    signs = [
        sign_line_eq(P, A, B) >= 0,
        sign_line_eq(P, B, C) >= 0,
        sign_line_eq(P, C, D) >= 0,
        sign_line_eq(P, D, A) >= 0,
    ]
    all_left = not any(signs)
    all_right = all(signs)
    return all_left or all_right

def point_in_polygon(P, poly):
    """
    Ray casting algorithm for point-in-polygon test.
    P: (x, y) tuple
    poly: list of (x, y) tuples
    """
    x, y = P
    n = len(poly)
    inside = False
    p1x, p1y = poly[0]
    for i in range(n+1):
        p2x, p2y = poly[i % n]
        if min(p1y, p2y) < y <= max(p1y, p2y):
            if x <= max(p1x, p2x):
                if p1y != p2y:
                    xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                if p1x == p2x or x <= xinters:
                    inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def edges_intersect(A, B, C, D):
    """
    Check if line segments AB and CD intersect.
    """
    def ccw(X, Y, Z):
        return (Z[1]-X[1]) * (Y[0]-X[0]) > (Y[1]-X[1]) * (Z[0]-X[0])
    return (ccw(A, C, D) != ccw(B, C, D)) and (ccw(A, B, C) != ccw(A, B, D))