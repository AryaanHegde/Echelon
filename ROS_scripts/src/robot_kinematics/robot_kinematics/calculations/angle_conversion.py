import numpy as np

# Function to convert a list of angles in radians to degrees
def rad_to_deg(angles):
    converted_angles = []
    for angle in angles:
        converted_angles.append(angle * (180/np.pi))
    
    return converted_angles

# Function to convert a list of angles in degrees to radians
def deg_to_rad(angles):
    converted_angles = []
    for angle in angles:
        converted_angles.append(angle * (np.pi/180))
    
    return converted_angles