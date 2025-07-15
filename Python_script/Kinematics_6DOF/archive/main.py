# from transformation import*
# import random

# lengths = [3, 2, 5, 5, 2, 1]

# for j in range(100):
#     c_space = [random.uniform(-math.pi, math.pi) for _ in range(6)]

#     fk = ForwardKinematics(lengths, c_space)
#     op_space = fk.solve()

#     print(f"Op space: {op_space}")

#     ik = InverseKinematics(lengths, op_space)
#     try:
#         solved_c_space = ik.solve()
#         fk = ForwardKinematics(lengths, solved_c_space)
#         new_op_space = fk.solve()
#         print(f"Computed Op space: {new_op_space}")
#     except:
#         print("Failed")

from transformation import*
import random
import numpy as np

lengths = [3, 2, 5, 5, 2, 1]

c_space = [0, 1.48, 0, 0, 0, 0]

# print(f"Original c space = {c_space}")

fk = ForwardKinematics(lengths, c_space)
op_space = fk.solve()

print(f"Op space = {op_space}")

ik = InverseKinematics(lengths, op_space)
solved_c_space = ik.solve()

# print(f"Solved c_space = {solved_c_space}")

fk2 = ForwardKinematics(lengths, solved_c_space)
calculated_op_space = fk2.solve()

print(f"Calculated Op_space = {calculated_op_space}")