import pysee
import numpy as np

# define the camera and algorithm parameters
see = pysee.init("../config.json")

# load the point cloud
points = np.loadtxt("../test.txt", dtype=np.float32)[:, 0:3].tolist()

# set the current view, position and orientation(pointing direction)
current_v = np.array([-1.25516047, 0.16405614, 0.90033961, 0.81845733, -0.11847473, -0.56222001], dtype=np.float32).tolist()

# search for the next view
nbv = see.search_nbv_once(points, current_v)

print(f"Next view: {nbv}")
