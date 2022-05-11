import numpy as np

H3 = 1
h = 0.1
W = 3
l = 0.2
L = W - 2*l
N = 5
H = (H3 - 2 * h) / (N - 1)

default_height = 0.5
xyz_global = np.array([2, 2, 0.1])

waypoints = np.array([])

# Choose direction to start with
if xyz_global[1] > W / 2:
    # Start en bas à gauche : P(x0, l, 0.5)
    x0 = xyz_global[0]
    y0 = l
    
    # Initial point
    waypoints = np.append(waypoints, [x0, y0, default_height])
    
    # Second point
    #self.waypoints = np.append(self.waypoints[0:3] + np.array([self.H, 0, 0]))

    for i in range(N-1):
        waypoints = np.append(waypoints, waypoints[6*i:6*i+3] + np.array([H, 0, 0]))
        # Third point
        waypoints = np.append(waypoints, waypoints[6*i+3:6*i+6] + np.array([0, L * (-1)**i, 0]))
        # Fourth point
