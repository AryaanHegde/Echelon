import pygame
import numpy as np
import math

class Interface:
    def __init__(self, angles, lengths):
        self.WIDTH = 900
        self.HEIGHT = 600
        self.angles = angles
        self.lengths = lengths
        self.deg_to_rad()
        self.origin = (self.WIDTH // 2, self.HEIGHT//2)
        self.win = None
        self.vec_wo = np.array([self.WIDTH // 2, -self.HEIGHT//2])
    
    def start_up(self):
        pygame.init()
        self.win = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Robot simulator")
    
    def world_to_base(self, vec_wp):
        x_wp, y_wp = vec_wp
        y_wp *= -1
        format_vec_wp = np.array([x_wp, y_wp])
        return format_vec_wp - self.vec_wo

    def draw_robot(self):
        self.win.fill((30, 30, 30))

        workspace_radius = sum(self.lengths)
        pygame.draw.circle(self.win, (100, 100, 100), self.origin, int(workspace_radius), 1)

        x0, y0 = self.origin
        positions = []
        x_prev = x0
        y_prev = y0

        for i in range(3):
            x_prev = x_prev + self.lengths[i] * math.cos(self.angles[i])
            y_prev = y_prev - self.lengths[i] * math.sin(self.angles[i])
            positions.append((x_prev, y_prev))

        x_prev = x0
        y_prev = y0
        for j in range(3):
            pygame.draw.circle(self.win, (255, 255, 255), (int(x_prev), int(y_prev)), 6)
            x_prev, y_prev = positions[j]
        
        x_prev = x0
        y_prev = y0
        
        for k in range(3):
            x_cur, y_cur = positions[k]
            pygame.draw.line(self.win, (0, 150, 255), (x_prev, y_prev), (x_cur, y_cur), 5)
            x_prev = x_cur
            y_prev = y_cur
        
        
        pygame.display.update()
    
    def update_angles(self, angles, step_deg=1):
        for joint_index in range(len(angles)):
            start_rad = self.angles[joint_index]
            end_rad = angles[joint_index]
            step_rad = step_deg
            self.angles[joint_index] = start_rad

            direction = 1 if start_rad < end_rad else -1

            while abs(self.angles[joint_index] - end_rad) > 0.01:
                pygame.time.Clock().tick(60)
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        quit()

                self.angles[joint_index] += direction * step_rad
                if direction == 1:
                    self.angles[joint_index] = min(self.angles[joint_index], end_rad)
                else:
                    self.angles[joint_index] = max(self.angles[joint_index], end_rad)

                self.draw_robot()

    def deg_to_rad(self):
        for idx in range(len(self.angles)):
            self.angles[idx] = self.angles[idx] * math.pi / 180


    def get_position(self):
        return pygame.mouse.get_pos()
