import numpy as np
import rospy
import math
import time


from pymoo.core.population import Population
from pymoo.algorithms.soo.nonconvex.pattern import PatternSearch
from pymoo.algorithms.soo.nonconvex.ga import GA
from pymoo.algorithms.soo.nonconvex.pso import PSO
from pymoo.optimize import minimize
from pymoo.termination.robust import RobustTermination
from pymoo.termination.ftol import SingleObjectiveSpaceTermination

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from sensor_msgs.point_cloud2 import create_cloud
from gazebo_msgs.srv import GetPhysicsProperties

import matplotlib.pyplot as plt
import matplotlib.path as mpath
import matplotlib.patches as mpatches
from matplotlib.patches import Ellipse
from matplotlib.path import Path
import mpl_toolkits.mplot3d.art3d as art3d

center = np.zeros(3)
out_p = 0.5*np.ones(3)
inn_p = 0.1*np.ones(3)

fig, axes = plt.subplots(1, 3, figsize=(10, 4), constrained_layout=True)

# XY Plane Projection
ell_out = Ellipse((center[0], center[1]), 2*out_p[0], 2*out_p[1])
ell_inn = Ellipse((center[0], center[1]), 2*inn_p[0], 2*inn_p[1])
out_trans = ell_out.get_transform()
out_path = ell_out.get_path().transformed(out_trans)
inn_trans = ell_inn.get_transform()
inn_path = ell_inn.get_path().transformed(inn_trans)

# Reverse the inner path manually
inn_vertices = inn_path.vertices[::-1]

# Define codes for the outer and inner paths
out_codes = [Path.MOVETO] + [Path.LINETO] * (len(out_path.vertices) - 1)
inn_codes = [Path.MOVETO] + [Path.LINETO] * (len(inn_vertices) - 1)

# Combine the outer and reversed inner paths
all_vertices = np.concatenate([out_path.vertices, inn_vertices])
all_codes = out_codes + inn_codes

# Create a new path from the combined vertices and codes
combined_path = Path(all_vertices, all_codes)

# Create the patch from the combined path
area = mpatches.PathPatch(combined_path, linewidth=1.5, edgecolor=(
    0, 0.5, 0), facecolor=(0, 0.5, 0, 0.5))
axes[0].add_patch(area)
plt.show()