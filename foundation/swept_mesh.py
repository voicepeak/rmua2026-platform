"""Two-sided swept-sphere contact with triangle surfaces, in one coordinate frame.

This tests surfaces only; a sphere wholly inside a closed solid without touching
its boundary is not detected. It returns contact existence, not time of impact.
"""
import numpy as np
from foundation.obstacles import segment_distance_squared
from foundation.road_scene import segment_intersections


def point_triangle_distance_squared(p, triangle):
    a, b, c = triangle
    n = np.cross(b-a, c-a)
    n2 = n @ n
    best = min(segment_distance_squared(p, p, a, b),
               segment_distance_squared(p, p, b, c),
               segment_distance_squared(p, p, c, a))
    if n2 <= 1e-24:
        return best
    proj = p - n * ((p-a) @ n / n2)
    signs = [np.dot(np.cross(y-x, proj-x), n) for x, y in ((a, b), (b, c), (c, a))]
    if min(signs) >= -1e-10*n2:
        best = min(best, float(np.dot(p-proj, p-proj)))
    return best


def swept_sphere_triangle(start, end, radius, triangle):
    a, b = np.asarray(start, float), np.asarray(end, float)
    tri = np.asarray(triangle, float)
    if (a.shape != (3,) or b.shape != (3,) or tri.shape != (3, 3) or
            not np.isfinite(np.r_[a, b, tri.ravel(), radius]).all() or radius < 0):
        raise ValueError('invalid sweep')
    if np.linalg.norm(b-a) > 1e-12 and len(segment_intersections(a, b, tri, np.array([[0, 1, 2]]))):
        return True
    distance2 = min(point_triangle_distance_squared(a, tri), point_triangle_distance_squared(b, tri),
                    *(segment_distance_squared(a, b, tri[i], tri[(i+1) % 3]) for i in range(3)))
    return distance2 <= radius*radius + 1e-12
