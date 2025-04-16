import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

# Global variables to store the last plotted object marker and label
object_marker = None
object_label = None

def multilaterate(nodes, measurements):
    """
    Estimate the object location using least-squares multilateration.
    
    Parameters:
      nodes: dict mapping node names to their configuration dictionary (including "point")
      measurements: dict mapping node names to distance measurements.
    
    Returns:
      The estimated position as a numpy array.
    """
    positions = []
    distances = []
    for name, node_config in nodes.items():
        if name in measurements:
            # Extract the coordinate list from the node configuration.
            point = node_config.get("point")
            if point is None:
                raise Exception(f"Node '{name}' does not have a 'point' key.")
            positions.append(np.array(point, dtype=float))
            distances.append(measurements[name])
    positions = np.array(positions)
    distances = np.array(distances)
    
    if len(positions) == 0:
        raise Exception("No measurements available.")
    
    # Use the average of available node positions as an initial guess.
    x0 = np.mean(positions, axis=0)
    
    def residuals(X):
        return np.linalg.norm(positions - X, axis=1) - distances
    
    result = least_squares(residuals, x0)
    return result.x

def point_in_polygon(x, y, poly):
    """
    Determine if point (x, y) lies inside the polygon defined by a list of [x, y] points.
    
    Parameters:
      x, y: Coordinates of the point.
      poly: List of points [[x1,y1], [x2,y2], ...] defining the polygon.
      
    Returns:
      True if the point is inside the polygon, False otherwise.
    """
    inside = False
    n = len(poly)
    p1x, p1y = poly[0]
    for i in range(n + 1):
        p2x, p2y = poly[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def find_room_for_point(P, config):
    """
    Determine the room in which the point P = [x, y, z] lies.
    Iterates over each floor and room in the configuration.
    
    Parameters:
      P: The point [x, y, z] to check.
      config: The YAML configuration dictionary.
      
    Returns:
      The name of the room if found, or "Unknown" otherwise.
    """
    x, y, z = P
    for floor in config.get("floors", []):
        bounds = floor.get("bounds", [[0, 0, 0], [0, 0, 0]])
        zmin = min(bounds[0][2], bounds[1][2])
        zmax = max(bounds[0][2], bounds[1][2])
        if not (zmin <= z <= zmax):
            continue
        for room in floor.get("rooms", []):
            poly = room.get("points", [])
            if point_in_polygon(x, y, poly):
                return room.get("name", "Unknown")
    return "Unknown"

def plot_object(P, object_name, room_name, ax):
    """
    Plot the computed object position in the provided 3D axes as a black star.
    Removes any previously plotted marker and labels it with the object name and room.
    
    Parameters:
      P: The estimated position [x, y, z].
      object_name: Name of the object.
      room_name: The determined room name.
      ax: The matplotlib 3D axes to plot on.
    """
    global object_marker, object_label
    if object_marker is not None:
        try:
            object_marker.remove()
        except Exception:
            pass
    if object_label is not None:
        try:
            object_label.remove()
        except Exception:
            pass
    object_marker = ax.scatter(P[0], P[1], P[2], color='black', marker='*', s=200)
    object_label = ax.text(P[0], P[1], P[2] + 0.2, f"{object_name} ({room_name})",
                           color='black', fontsize=12)
    plt.draw()
    print(f"✅ [trilateration] Plotted object '{object_name}' in room '{room_name}' at {P}.")
