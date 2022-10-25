import pymunk
import pygame
from pymunk import Vec2d

def set_env_physics(self):
        self.space = pymunk.Space()
        self.space.gravity = (0.0, 0.0)
        #self.space.damping = 0.999 # to prevent it from blowing up

        self.display_flags = 0
        self.display_size = (600, 600)
        
        self.ground_y = 600
        self.ground_y = 600
        ground = pymunk.Segment(self.space.static_body, (0, 605), (605,605), 1.0)
        ground.friction = 0.2
        left = pymunk.Segment(self.space.static_body, (-5, 0), (-5, 600), 1)
        left.friction = 0.2
        right = pymunk.Segment(self.space.static_body, (603,0), (603,600), 1.0)
        right.friction = 0.2
        top = pymunk.Segment(self.space.static_body, (0,-5), (600,-5), 1.0)
        top.friction = 0.2
        ground.elasticity = 1.0
        left.elasticity = 1.0
        top.elasticity = 1.0
        right.elasticity = 1.0
        self.space.add(ground,left,right,top)
        self.reward = 0
        pygame.init()
        self.screen = pygame.display.set_mode(self.display_size)
        pygame.display.set_caption("CLAW_Arm")
        width, height = self.screen.get_size()
       ## bg_img = pygame.image.load('mod_code\water_bg.png').convert()
        #self.screen.blit(bg_img,[0,0])
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        # self.draw_options.flags = pymunk.SpaceDebugDrawOptions.DRAW_SHAPES | pymunk.SpaceDebugDrawOptions.DRAW_COLLISION_POINTS
        # Delete above line to show the purple colour joints, I chose to remove it to make opencv detection more simple
        self.clock = pygame.time.Clock()
        font = pygame.font.Font(None, 16)
        # Set the initial coordinates and the weights/inertias for all the bodies
        chassisXY = Vec2d(self.display_size[0]/2,300)
        chWd = 120; chHt = 5
        chassisMass = 40
        self.reward=0
        legWd_a = 100; legHt_a = 5
        legWd_b = 100; legHt_b = 5 
        legMass = 5
        relativeAnguVel = 0

        return chassisXY,chWd,chHt,chassisMass,legWd_a,legHt_a,legWd_b,legHt_b,legMass,relativeAnguVel
        
