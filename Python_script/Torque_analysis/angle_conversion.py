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

def rpm_to_rad_s(rpms):
    converted_vals = []
    for rpm in rpms:
        converted_vals.append(rpm * (np.pi)/30)
    
    return converted_vals