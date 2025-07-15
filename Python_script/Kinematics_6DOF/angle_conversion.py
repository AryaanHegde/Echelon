import numpy as np
def rad_to_deg(angles):
    converted_angles = []
    for angle in angles:
        converted_angles.append(angle * (180/np.pi))
    
    return converted_angles

def deg_to_rad(angles):
    converted_angles = []
    for angle in angles:
        converted_angles.append(angle * (np.pi/180))
    
    return converted_angles