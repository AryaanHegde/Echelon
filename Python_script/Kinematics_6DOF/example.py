from euler_angles import*
import random
import forward_kinematics
import inverse_kinematics
from angle_conversion import*

lengths = [0.3, 0.2, 0.5, 0.5, 0.2, 0.1]
theta = [random.uniform(-180, 180) for _ in range(6)]
x_1 = forward_kinematics.solve(lengths, theta)
print(x_1)
theta_prime = inverse_kinematics.solve(lengths, x_1)
x_2 = forward_kinematics.solve(lengths, theta_prime)
print(x_2)
