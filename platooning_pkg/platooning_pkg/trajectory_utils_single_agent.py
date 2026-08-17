import math
import numpy as np
import scipy.interpolate as spi

def get_figure8_target(t, speed_multiplier=0.2):
    """ Used by single_agent_clf.py """
    x = 0.8 * math.sin(t * speed_multiplier)
    y = 0.8 * math.sin(t * (speed_multiplier / 2.0))
    return np.array([[x], [y]])

def generate_custom_spline():
    """ 
    Generates a 2000-point custom track.
    Waypoints are now in the CORRECT order (Start near 0,0 -> Finish at 1.9, 0.1)
    """
    waypoints_x = [0.10, 0.15, 0.30, 0.90, 1.50, 1.90, 1.50, 0.50, 0.20, 0.30, 1.00, 1.90]
    waypoints_y = [0.00, 1.00, 1.70, 1.40, 1.80, 1.10, 0.80, 0.75, 0.60, 0.40, 0.30, 0.10]
    
    tck, _ = spi.splprep([waypoints_x, waypoints_y], s=0)
    u_new = np.linspace(0, 1, 2000)
    path_x, path_y = spi.splev(u_new, tck)
    return path_x, path_y