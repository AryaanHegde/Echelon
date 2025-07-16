from euler_angles import*
import random
import forward_kinematics
import inverse_kinematics
from angle_conversion import*

lengths = [3, 2, 5, 5, 2, 1]
success = True
iterations = 1000

for test_iter in range(iterations):
    # Generate random c-space
    theta = [random.uniform(-180, 180) for _ in range(6)]
    # Calculate the operation space of the input via forward kinematics
    x_1 = forward_kinematics.solve(lengths, theta)
    # Determine a potential solution for the previous operational space via inverse kinematics    
    theta_prime = inverse_kinematics.solve(lengths, x_1)
    # Calculate the operation space of the potential solution via forward kinematics
    x_2 = forward_kinematics.solve(lengths, theta_prime)

    # Verify if the potential solution is indeed a solution by comparing the two op-spaces
    for i in range(6):
        if abs(x_2[i] - x_1[i]) > 0.001:
            print(f"Failure at test iteration {test_iter}")
            print(f"X_1 = {x_1}")
            print(f"X_2 = {x_2}")
            print("\n")
            success = False
            break

if success:
    print("Kinematic calculations are successful!")
