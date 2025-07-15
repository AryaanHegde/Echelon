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
dir1 = vector(np.cos(theta1), np.sin(theta1), 0)
p1 = p0 + L1 * dir1

# Second link
theta12 = theta1 + theta2
dir2 = vector(np.cos(theta12), np.sin(theta12), 0)
p2 = p1 + L2 * dir2

# Third link
theta123 = theta12 + theta3
dir3 = vector(np.cos(theta123), np.sin(theta123), 0)
p3 = p2 + L3 * dir3

# Render base
sphere(pos=p0, radius=0.2, color=color.white)

# Render links
link1 = cylinder(pos=p0, axis=p1 - p0, radius=0.1, color=color.red)
joint1 = sphere(pos=p1, radius=0.15, color=color.orange)

link2 = cylinder(pos=p1, axis=p2 - p1, radius=0.1, color=color.green)
joint2 = sphere(pos=p2, radius=0.15, color=color.orange)

link3 = cylinder(pos=p2, axis=p3 - p2, radius=0.1, color=color.blue)
end_effector = sphere(pos=p3, radius=0.15, color=color.cyan)

# Keep window open
scene.waitfor('click')
