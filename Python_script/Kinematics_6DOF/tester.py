from euler_angles import*
import random
import forward_kinematics
import inverse_kinematics
from angle_conversion import*

lengths = [3, 2, 5, 5, 2, 1]
success = True
iterations = 1000

for test_iter in range(iterations):
    test_case = [random.uniform(-180, 180) for _ in range(6)]
    op_space = forward_kinematics.solve(lengths, test_case)
    c_space = inverse_kinematics.solve(lengths, op_space)
    calculated_op_space = forward_kinematics.solve(lengths, c_space)
    for i in range(6):
        if abs(op_space[i] - calculated_op_space[i]) > 0.001:
            print(f"Failure at test iteration {test_iter}")
            print(f"Original op space = {op_space}")
            print(f"Calculated op space = {calculated_op_space}")
            success = False
            break

if success:
    print("Kinematic calculations are successful!")
