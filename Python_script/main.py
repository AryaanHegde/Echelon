import pygame
from render import *
from kinematics import *

lengths = [100, 100, 80]

simulator = Interface([15, 30, 60], lengths)  # use realistic pixel lengths
simulator.start_up()
running = True

while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
    world_coordinates = simulator.get_position()
    base_coordinates = simulator.world_to_base(world_coordinates)
    ik = Inverse(lengths, base_coordinates)
    ik.get_joint_angles()
    ik.refactor_angles()
    fk = Forward(ik.angles, lengths)
    pos, orientation = fk.get_ee_state()
    simulator.update_angles(ik.angles)


    simulator.draw_robot()

pygame.quit()

