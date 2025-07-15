from vpython import *
import numpy as np

# Link lengths
L1 = 4
L2 = 3
L3 = 2

# Joint angles (in radians)
theta1 = 0
theta2 = np.pi / 4
theta3 = np.pi / 2

# Forward kinematics to compute joint positions
p0 = vector(0, 0, 0)

# First link
dir1 = vector(0, 0, 1)
p1 = p0 + L1 * dir1

sphere(pos=p0, radius=0.2, color=color.white)
link1 = cylinder(pos=p0, axis = p1 - p0, radius = 0.1, color=color.red)

dir2 = vector()
scene.waitfor('click')