import numpy as np
from scipy.interpolate import interp1d


def get_curve(P0, P1, V0, vel_bias, spacing_const):
    """
    Returns a numpy.ndarray

    Arguments:
    P0 - drone's current position (numpy.ndarray with 3 position entries)
    P1 - next gate postion (numpy.ndarray with 3 position entries)
    V1 - drone's current velocity (numpy.ndarray with 3 velocity entries)
    vel_bias - effect of inital velocity (float)
    spacing_const - increasing this number will increase how many points are in each path (float) // generally keep this number pretty low

    """

    # decrease effect of z velocity
    V0[2] *= 0.5

    P1a = P0 + V0 * vel_bias
    P1b = P1

    # Bezier equation
    def bezier(t):
        return (1 - t)**3 * P0 + 3 * (1 - t)**2 * t * P1a + 3 * (1 - t) * t**2 * P1b + t**3 * P1

    # get curve
    ts = np.linspace(0, 1, 25)
    curve = np.array([bezier(t) for t in ts])
    dists = np.linalg.norm(np.diff(curve, axis=0), axis=1)
    arc_lengths = np.concatenate([[0], np.cumsum(dists)])

    # determine number of points based on distance between
    num_points = spacing_const * int(np.linalg.norm(np.array(P0) - np.array(P1)))
    print(num_points)

    # make points on curve evenly spaced
    even_arc_lens = np.linspace(0, arc_lengths[-1], num_points)
    interp_func = interp1d(arc_lengths, curve, axis=0)
    evenly_spaced_curve = interp_func(even_arc_lens)

    return evenly_spaced_curve
