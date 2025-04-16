import yaml
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import queue

# Global variables for configuration, plot objects, node lookup, measurements, and update queue
config = None
fig = None
ax = None
node_spheres = {}       # maps node (lowercase) to its sphere surface
node_texts = {}         # maps node (lowercase) to its distance label text
nodes_dict = {}         # maps node (lowercase) to node config data
node_measurements = {}  # maps node (lowercase) to its last measured distance

# Global list to hold the floor (room) polygons for display
floor_collections = []

# Map specific node names to colors (all in lowercase)
node_color_map = {
    "kantoor": "red",
    "slaapkamer": "green",
    "woonkamer": "blue"
}

update_queue = queue.Queue()  # Thread-safe queue for update requests

def load_config():
    """Load the configuration from config.yaml and build a lookup for nodes."""
    global config, nodes_dict
    try:
        with open("config.yaml", "r") as f:
            config = yaml.safe_load(f)
        nodes_dict = {}
        for node in config.get("nodes", []):
            node_name = node.get("name", "").lower()
            nodes_dict[node_name] = node
        print("🔧 [3dhome] Configuration loaded successfully.")
    except Exception as e:
        print("❌ [3dhome] Error loading configuration:", e)
    return config

def extrude_polygon(points, zmin, zmax):
    """
    Create faces for an extruded 3D polygon given its 2D points.
    Returns a list of faces (each a list of 3D vertices).
    """
    faces = []
    bottom_face = [[x, y, zmin] for (x, y) in points]
    faces.append(bottom_face)
    top_face = [[x, y, zmax] for (x, y) in points]
    faces.append(top_face)
    n = len(points)
    for i in range(n - 1):
        j = i + 1
        side_face = [
            [points[i][0], points[i][1], zmin],
            [points[j][0], points[j][1], zmin],
            [points[j][0], points[j][1], zmax],
            [points[i][0], points[i][1], zmax]
        ]
        faces.append(side_face)
    return faces

def draw_sphere(center, radius, color='blue', alpha=0.3):
    """
    Draw a sphere on the current axes centered at 'center' with the given 'radius'.
    Returns the surface object.
    """
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    sphere = ax.plot_surface(x, y, z, color=color, alpha=alpha)
    return sphere

def init_plot():
    """
    Initialize the interactive 3D matplotlib figure by drawing floors, rooms, and node markers.
    Sets an equal aspect ratio so that spheres appear round.
    """
    global fig, ax, floor_collections
    plt.ion()  # Enable interactive mode
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    print("🔧 [3dhome] Plot figure and axes initialized.")
    
    if config is None:
        load_config()
    
    # Draw floors and rooms from configuration
    floor_collections = []
    map_config = config.get("map", {})
    wall_color = map_config.get("wall_color", "#ddd")
    wall_opacity = map_config.get("wall_opacity", 0.35)
    for floor in config.get("floors", []):
        bounds = floor.get("bounds", [[0, 0, 0], [0, 0, 0]])
        zmin = bounds[0][2]
        zmax = bounds[1][2]
        for room in floor.get("rooms", []):
            room_points = room.get("points", [])
            if len(room_points) < 3:
                continue
            faces = extrude_polygon(room_points, zmin, zmax)
            poly3d = Poly3DCollection(faces, facecolors=wall_color, edgecolors='k',
                                      linewidths=1, alpha=wall_opacity)
            ax.add_collection3d(poly3d)
            floor_collections.append(poly3d)
    print("🔧 [3dhome] Floors and rooms drawn.")
    
    # Plot node markers as red dots with labels.
    for node in config.get("nodes", []):
        node_name = node.get("name", "").lower()
        center = node.get("point", [0, 0, 0])
        ax.scatter(center[0], center[1], center[2], color='red', s=50)
        ax.text(center[0], center[1], center[2] + 0.1, node.get("name", ""), color='black')
    print("🔧 [3dhome] Node markers drawn.")
    
    # Set axis limits based on floor bounds and enforce equal aspect ratio.
    all_x, all_y, all_z = [], [], []
    for floor in config.get("floors", []):
        bounds = floor.get("bounds", [[0, 0, 0], [0, 0, 0]])
        all_x.extend([bounds[0][0], bounds[1][0]])
        all_y.extend([bounds[0][1], bounds[1][1]])
        all_z.extend([bounds[0][2], bounds[1][2]])
    ax.set_xlim(min(all_x) - 1, max(all_x) + 1)
    ax.set_ylim(min(all_y) - 1, max(all_y) + 1)
    ax.set_zlim(min(all_z) - 1, max(all_z) + 1)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("3D Model of Floors, Rooms, and Nodes")
    ax.set_box_aspect([1, 1, 1])
    plt.draw()
    print("🔧 [3dhome] Plot axis limits, labels, and aspect ratio set.")

def update_node_distance(node_name, distance):
    """
    Update the sphere for the given node (node_name should be lowercase) using the provided distance.
    Removes any previous sphere and draws a new one in a unique color.
    Also stores the measurement.
    """
    global node_spheres, node_texts, node_measurements
    node = nodes_dict.get(node_name)
    if not node:
        print(f"⚠️ [3dhome] Node '{node_name}' not found in configuration.")
        return
    center = node.get("point", [0, 0, 0])
    if node_name in node_spheres:
        try:
            node_spheres[node_name].remove()
            #print(f"🔄 [3dhome] Removed existing sphere for node '{node_name}'.")
        except Exception as e:
            print("❌ [3dhome] Error removing sphere:", e)
        if node_name in node_texts:
            try:
                node_texts[node_name].remove()
                #print(f"🔄 [3dhome] Removed existing text for node '{node_name}'.")
            except Exception as e:
                print("❌ [3dhome] Error removing text:", e)
    color = node_color_map.get(node_name, "blue")
    sphere = draw_sphere(center, distance, color=color, alpha=0.3)
    node_spheres[node_name] = sphere
    text = ax.text(center[0], center[1], center[2] + distance, f"{distance} m", color=color)
    node_texts[node_name] = text
    node_measurements[node_name] = distance
    plt.draw()
    print(f"✅ [3dhome] Updated node '{node_name}' with distance {distance} m (color: {color}).")

def queue_update(node_name, distance):
    """Enqueue an update for the given node."""
    update_queue.put((node_name, distance))
    #rint(f"🔄 [3dhome] Queued update for node '{node_name}' with distance {distance} m.")

def process_updates():
    """Process all queued updates (to be called from the main thread)."""
    while not update_queue.empty():
        node_name, distance = update_queue.get()
        #print(f"🔄 [3dhome] Processing queued update for node '{node_name}' with distance {distance} m.")
        update_node_distance(node_name, distance)

if __name__ == "__main__":
    load_config()
    init_plot()
    print("🚀 [3dhome] Interactive model running. Close the plot window to exit.")
    try:
        while plt.fignum_exists(fig.number):
            process_updates()
            plt.pause(0.1)
    except KeyboardInterrupt:
        print("🛑 [3dhome] Exiting interactive model.")
