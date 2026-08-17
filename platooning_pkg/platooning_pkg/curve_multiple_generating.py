import numpy as np

def generate_constant_radius_curve(radius=1.5, angle_range=np.pi/2, num_points=1000):
    """
    Generates a scaled-down constant radius curve.
    Mimics Scenario S1 and S2 from the TROOP platooning paper.
    
    :param radius: Radius of the curve in meters.
    :param angle_range: How far the curve sweeps (default is 90 degrees).
    """
    theta = np.linspace(0, angle_range, num_points)
    
    # Starting at (0,0) and curving to the left
    x = radius * np.sin(theta)
    y = radius * (1 - np.cos(theta))
    
    return x, y

def generate_double_lane_change(length=4.0, lane_width=0.5, num_points=1000):
    """
    Generates a scaled-down Double Lane Change (DLC) trajectory.
    Mimics Scenario S3 from the TROOP platooning paper.
    
    :param length: Total longitudinal length of the track in meters.
    :param lane_width: Lateral offset of the lane change in meters.
    """
    x = np.linspace(0, length, num_points)
    y = np.zeros_like(x)
    
    # 1. First lane change (shift left to avoid obstacle)
    # Starts at x = 0.5m, finishes at x = 1.5m
    idx1 = (x > 0.5) & (x <= 1.5)
    y[idx1] = (lane_width / 2.0) * (1 - np.cos(np.pi * (x[idx1] - 0.5) / 1.0))
    
    # 2. Hold in the adjacent lane
    # Starts at x = 1.5m, finishes at x = 2.0m
    idx2 = (x > 1.5) & (x <= 2.0)
    y[idx2] = lane_width
    
    # 3. Second lane change (return to original lane)
    # Starts at x = 2.0m, finishes at x = 3.0m
    idx3 = (x > 2.0) & (x <= 3.0)
    y[idx3] = lane_width - (lane_width / 2.0) * (1 - np.cos(np.pi * (x[idx3] - 2.0) / 1.0))
    
    # 4. Beyond x = 3.0m, it remains at y = 0.0 (straight line)
    
    return x, y