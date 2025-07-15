from euler_angles import*
import random
import forward_kinematics
import inverse_kinematics
from angle_conversion import*


# Testing euler angle conversions
# for _ in range(10):
#     angles = [random.uniform(-360, 360) for _ in range(3)]
#     matrix = euler_to_matrix(angles)
#     solved_angles = matrix_to_euler(matrix)
#     calculated_matrix = euler_to_matrix(solved_angles)
#     if not np.allclose(matrix, calculated_matrix, atol=1e-6):
#         print(f"FAIL: {angles} != {solved_angles}")
#     else:
#         print(f"PASS")
    

# Testing forward kinematics
lengths = [3, 2, 5, 5, 2, 1]
for _ in range(5):
    test_case = [random.uniform(-180, 180) for _ in range(6)]
    op_space = forward_kinematics.solve(lengths, test_case)
    c_space = inverse_kinematics.solve(lengths, op_space)
    calculated_op_space = forward_kinematics.solve(lengths, c_space)
    print(op_space)
    print(calculated_op_space)
    print("\n")
