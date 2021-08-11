from mod_code.cv2_code.main import window_det
import sys
import pygame
from pygame.locals import USEREVENT, QUIT, KEYDOWN, KEYUP,K_e,K_s, K_r, K_q,K_w,K_a,K_s,K_d,K_k,K_u,K_j,K_h,K_j,K_z,K_ESCAPE, K_UP, K_DOWN, K_RIGHT, K_LEFT,K_2,K_4,K_6,K_8
import numpy as np
import pymunk
from pymunk import Vec2d    
import pymunk.pygame_util
from pynput.keyboard import Key,Controller
keyboard = Controller()
delta = 1
rotationRate = 1


class Simulator(object):

    def __init__(self):
        ### NOTE:Initialising the space with all the motors, joints, bodies etc.
        self.space = pymunk.Space()
        self.space.gravity = (0.0, 0.0)
        #self.space.damping = 0.999 # to prevent it from blowing up

        self.display_flags = 0
        self.display_size = (600, 600)
        # Pymunk physics coordinates start from the lower right-hand corner of the screen
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
        self.draw_options.flags = pymunk.SpaceDebugDrawOptions.DRAW_SHAPES | pymunk.SpaceDebugDrawOptions.DRAW_COLLISION_POINTS
        # Delete above line to show the purple colour joints, I chose to remove it to make opencv detection more simple
        self.clock = pygame.time.Clock()
        font = pygame.font.Font(None, 16)
        # Create the spider
        chassisXY = Vec2d(self.display_size[0]/2,300)
        chWd = 100; chHt = 5
        chassisMass = 40

        legWd_a = 100; legHt_a = 5
        legWd_b = 100; legHt_b = 5 
        legMass = 5
        relativeAnguVel = 0
       ## Object definition         
        self.object_1 =pymunk.Body(2,pymunk.moment_for_circle(5,0,10))
        self.object_1.friction = 2
        self.object_1.position = Vec2d(210, 210)
        self.shape_box  = pymunk.Circle(self.object_1,10)
        self.shape_box.color = (0,0,128,0)            
        #---chassis
        self.chassis_b = pymunk.Body(chassisMass, pymunk.moment_for_box(chassisMass, (chWd, chHt)))
        self.chassis_b.position = chassisXY
        chassis_shape = pymunk.Poly.create_box(self.chassis_b, (chWd, chHt))
        chassis_shape.color = (200, 200, 200,0)
        self.chassis_b.start_position = chassisXY
        self.chassis_b.start_angle = 0
        print("chassis position");print(self.chassis_b.position)
        #---first left leg a
        self.leftLeg_1a_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_a, legHt_a)))
        self.leftLeg_1a_body.position = chassisXY - ((chWd/2)+(legWd_a/2), 0)
        self.leftLeg_1a_body.start_position = self.chassis_b.start_position - ((chWd/2)+(legWd_a/2), 0)
        self.leftLeg_1a_body.start_angle = 0 
        self.leftLeg_1a_shape = pymunk.Poly.create_box(self.leftLeg_1a_body, (legWd_a, legHt_a))        
        self.leftLeg_1a_shape.color = (255, 0, 0,0)

        #---first left leg b
        self.leftLeg_1b_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_b, legHt_b)))
        self.leftLeg_1b_body.position = self.leftLeg_1a_body.position - ((legWd_a/2)+(legWd_b/2), 0)
        self.leftLeg_1b_body.start_position = self.leftLeg_1a_body.start_position - ((legWd_a/2)+(legWd_b/2), 0)
        self.leftLeg_1b_body.start_angle = 0 
        self.leftLeg_1b_shape = pymunk.Poly.create_box(self.leftLeg_1b_body, (legWd_b, legHt_b))        
        self.leftLeg_1b_shape.color = (0, 255, 0,0)        

        #---first right leg a
        self.rightLeg_1a_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_a, legHt_a)))
        self.rightLeg_1a_body.position = chassisXY + ((chWd/2)+(legWd_a/2), 0)
        self.rightLeg_1a_body.start_position = self.chassis_b.start_position + ((chWd/2)+(legWd_a/2), 0)
        self.rightLeg_1a_body.start_angle = 0
        self.rightLeg_1a_shape = pymunk.Poly.create_box(self.rightLeg_1a_body, (legWd_a, legHt_a))        
        self.rightLeg_1a_shape.color = (255, 0, 0,0)        

        #---first right leg b
        self.rightLeg_1b_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_b, legHt_b)))
        self.rightLeg_1b_body.position = self.rightLeg_1a_body.position + ((legWd_a/2)+(legWd_b/2), 0)
        self.rightLeg_1b_body.start_position = self.rightLeg_1a_body.start_position + ((legWd_a/2)+(legWd_b/2), 0)
        self.rightLeg_1b_body.start_angle = 0
        self.rightLeg_1b_shape = pymunk.Poly.create_box(self.rightLeg_1b_body, (legWd_b, legHt_b))        
        self.rightLeg_1b_shape.color = (0, 255, 0,0)     
        
        self.object_1.start_position = Vec2d(210,210)
        self.object_1.start_angle = 0

        #---link left leg b with left leg a       
        pj_ba1left = pymunk.PinJoint(self.leftLeg_1b_body, self.leftLeg_1a_body, (legWd_b/2,0), (-legWd_a/2,0))#anchor point coordinates are wrt the body; not the space
        self.motor_ba1Left = pymunk.SimpleMotor(self.leftLeg_1b_body, self.leftLeg_1a_body, relativeAnguVel)
        self.motor_ba1Left.collide_bodies = False
        
        #---link left leg a with chassis
        pj_ac1left = pymunk.PinJoint(self.leftLeg_1a_body, self.chassis_b, (legWd_a/2,0), (-chWd/2, 0))
        self.motor_ac1Left = pymunk.SimpleMotor(self.leftLeg_1a_body, self.chassis_b, relativeAnguVel)
        self.motor_ac1Left.collide_bodies = False
        
        #---link right leg b with right leg a       
        pj_ba1Right = pymunk.PinJoint(self.rightLeg_1b_body, self.rightLeg_1a_body, (-legWd_b/2,0), (legWd_a/2,0))#anchor point coordinates are wrt the body; not the space
        self.motor_ba1Right = pymunk.SimpleMotor(self.rightLeg_1b_body, self.rightLeg_1a_body, relativeAnguVel)
        self.motor_ba1Right.collide_bodies = False
        #---link right leg a with chassis
        pj_ac1Right = pymunk.PinJoint(self.rightLeg_1a_body, self.chassis_b, (-legWd_a/2,0), (chWd/2, 0 ))
        self.motor_ac1Right = pymunk.SimpleMotor(self.rightLeg_1a_body, self.chassis_b, relativeAnguVel)      
        self.motor_ac1Right.collide_bodies = False

        self.space.add(self.chassis_b, chassis_shape) 
        self.space.add(self.leftLeg_1a_body, self.leftLeg_1a_shape, self.rightLeg_1a_body, self.rightLeg_1a_shape) 
        self.space.add(self.leftLeg_1b_body, self.leftLeg_1b_shape, self.rightLeg_1b_body, self.rightLeg_1b_shape) 
        self.space.add(pj_ba1left, self.motor_ba1Left, pj_ac1left, self.motor_ac1Left)  
        self.space.add(pj_ba1Right, self.motor_ba1Right, pj_ac1Right, self.motor_ac1Right)      
        self.space.add(self.object_1,self.shape_box)
#        ---prevent collisions with ShapeFilter
        self.leftLeg_1a_shape.filter = pymunk.ShapeFilter(group=1)
        self.rightLeg_1a_shape.filter = pymunk.ShapeFilter(group=2)
        self.leftLeg_1b_shape.filter = pymunk.ShapeFilter(group=1)
        self.rightLeg_1b_shape.filter = pymunk.ShapeFilter(group=2)  
        ## need to return the x,y coordinates of the object, and the destination angle
        ## need a method to change the angle
        self.motor_ac1Left.max_force = 1000000
        self.motor_ac1Right.max_force = 1000000
        self.motor_ba1Left.max_force = 1000000
        self.motor_ba1Right.max_force = 1000000
         
        handler = self.space.add_default_collision_handler()
        handler.begin = self.coll_begin
        handler.pre_solve = self.coll_pre
        handler.post_solve = self.coll_post
        handler.separate = self.coll_separate
        self.target = [120,60,-60,-120]    
        # class, with method :move joints 
        # one object for each joint , params: angle to achieve/ dircetion fro increment with resolution for
        # increment  
        
        self.passed = self.target ## initially same, then passed decreases to 0
        ## passed value should be decreasing, target should be constant, passed value should be target value minus current value
        self.simulate = True

## ***************************************-------------------------------------------------------------------------------------**********
## down , we have all the useful functions for this environment

    
    def render(self):
        self.screen.fill((255,255,255))
        self.space.debug_draw(self.draw_options)### Draw space        
        pygame.display.flip()
        fps = 50
        iterations = 25
        dt = 1.0/float(fps)/float(iterations)
        if self.simulate:
                for x in range(iterations): # 10 iterations to get a more stable simulation
                    self.space.step(dt)
            #########
        pygame.display.flip()
        self.clock.tick(fps)
    
    def return_angle_state(self):
        chassis_angle = self.chassis_b.angle*180/np.pi
        print(str(round(self.leftLeg_1b_body.angle*180/np.pi- chassis_angle))+" "+str(round(self.leftLeg_1a_body.angle*180/np.pi-chassis_angle))+" "+str(round(self.rightLeg_1a_body.angle*180/np.pi-chassis_angle))+" "+str(round(self.rightLeg_1b_body.angle*180/np.pi-chassis_angle)))
        ## state consists of angles, forward velocity of claw, angular velocity of claw, position of target object, caught object or not, net reward
        ## now gives constant angle
    
            
    def check_collide(self):
        tuple1 = self.object_1.position.int_tuple
        if(tuple1[0]>=585 or tuple1[0]<=15):
             self.object_1.velocity = Vec2d(-self.object_1.velocity.x,self.object_1.velocity.y)
        if(tuple1[1]>=585 or tuple1[1]<=15):
             self.object_1.velocity = Vec2d(self.object_1.velocity.x,-self.object_1.velocity.y)


    def reset(self):
        for body in self.space.bodies:
            body.position = body.start_position
            body.force = 0, 0
            body.torque = 0
            body.velocity = 0, 0
            body.angular_velocity = 0
            body.angle = body.start_angle
            
    def coll_begin(self,arbiter,space,data):
            print("collided")
            self.reward -= 1
            return True
    def coll_pre(self,arbiter,space,data):
        #print("presolve")
        return True
    def coll_post(self,arbiter,space,data):
        pass
        #print("post solve")
    def coll_separate(self,arbiter,space,data):
        # print("separate")
        pass



    def printinfo(self):
        #print("Angle:"+str(self.chassis_b.angle))
        #print("COG:"+str(self.chassis_b.center_of_gravity))
        #print("rotation vector:"+str(self.chassis_b.rotation_vector))
        print("local point velocity vector:"+str(self.chassis_b.velocity_at_local_point([0,0])))
    
    def change_angle(self,angles):

            chassis_angle = self.chassis_b.angle*180/np.pi
            cur_angles = [self.leftLeg_1b_body.angle*180/np.pi- chassis_angle,self.leftLeg_1a_body.angle*180/np.pi-chassis_angle,self.rightLeg_1a_body.angle*180/np.pi-chassis_angle,self.rightLeg_1b_body.angle*180/np.pi-chassis_angle]
            target_angles = self.target
            if (abs(target_angles[0]-cur_angles[0])<delta) and (abs(target_angles[1]-cur_angles[1])<delta) and (abs(target_angles[2]-cur_angles[2])<delta) and (abs(target_angles[3]-cur_angles[3])<delta):
                self.motor_ac1Left.rate = 0
                self.motor_ac1Right.rate = 0
                self.motor_ba1Left.rate = 0
                self.motor_ba1Right.rate  = 0

            if abs(target_angles[0]-cur_angles[0])<delta:
                   self.motor_ba1Left.rate = 0
            elif target_angles[0]-cur_angles[0]>delta:
                    self.motor_ba1Left.rate = 1
            elif target_angles[0]-cur_angles[0]<delta:
                    self.motor_ba1Left.rate = -1
        
            if abs(target_angles[1]-cur_angles[1])<delta:
                   self.motor_ac1Left.rate = 0
            elif target_angles[1]-cur_angles[1]>delta:
                    self.motor_ac1Left.rate = 1
            elif target_angles[1]-cur_angles[1]<delta:
                    self.motor_ac1Left.rate = -1
            
            if abs(target_angles[2]-cur_angles[2])<delta:
                   self.motor_ac1Right.rate = 0
            if target_angles[2]-cur_angles[2]>delta:
                    self.motor_ac1Right.rate = 1
            elif target_angles[2]-cur_angles[2]<delta:
                    self.motor_ac1Right.rate = -1
            
            if abs(target_angles[3]-cur_angles[3])<delta:
                   self.motor_ba1Right.rate = 0
            if target_angles[3]-cur_angles[3]>delta:
                    self.motor_ba1Right.rate = 1
            elif target_angles[3]-cur_angles[3]<delta:
                    self.motor_ba1Right.rate = -1

            self.passed = [round(target_angles[0]-cur_angles[0]),round(target_angles[1]-cur_angles[1]),round(target_angles[2]-cur_angles[2]),round(target_angles[3]-cur_angles[3])]
    
    def step_rl(self,action):
        ## actions: [x thruster (forward and backward upto -40 to 40), y thruster (forward and backward(-40 to 40)), 4 angle control values like (+1,-1,-1,+1)]
        ## this will also return the next state after action, reward for the action taken and done(boolean)
        # state will be : Current angle values, current thruster values, 
         for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    sys.exit(0)
                elif event.type == KEYDOWN and event.key == K_z:
                    # Start/stop simulation
                    simulate = not simulate
                elif event.type == KEYDOWN and event.key == K_r:
                    # Reset.
                    self.reset()
                elif event.type == KEYDOWN and event.key == K_h:
                    self.chassis_b.angular_velocity = -2
                elif event.type == KEYDOWN and event.key == K_u:
                    self.chassis_b.apply_force_at_local_point([0,-1500000])
                elif event.type == KEYDOWN and event.key == K_j:
                    self.chassis_b.apply_force_at_local_point([0,1500000])
                elif event.type == KEYDOWN and event.key == K_k:
                    self.chassis_b.angular_velocity = 2
                elif event.type == KEYDOWN and event.key == K_w:
                    self.motor_ba1Left.rate = -rotationRate
                elif event.type == KEYDOWN and event.key == K_s:
                    self.motor_ba1Left.rate = rotationRate
                elif event.type == KEYDOWN and event.key == K_a:
                    self.motor_ac1Left.rate = -rotationRate
                elif event.type == KEYDOWN and event.key == K_d:
                    self.motor_ac1Left.rate = rotationRate
                elif event.type == KEYDOWN and event.key ==K_UP:
                    self.motor_ba1Right.rate = rotationRate
                elif event.type == KEYDOWN and event.key ==K_DOWN:
                    self.motor_ba1Right.rate = -rotationRate
                elif event.type == KEYDOWN and event.key ==K_LEFT:
                    self.motor_ac1Right.rate = -rotationRate
                elif event.type == KEYDOWN and event.key ==K_RIGHT:
                    self.motor_ac1Right.rate = rotationRate
                elif event.type == KEYUP:
                    print("keyup")
                    self.motor_ba1Left.rate = 0
                    self.motor_ac1Left.rate = 0
                    self.motor_ba1Right.rate = 0 
                    self.motor_ac1Right.rate = 0
                    self.chassis_b.angular_velocity = 0

    def step_manual(self):
       ####This step function is for manual control (if this specific file is run then it will give keyboard control)
        for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    sys.exit(0)
                elif event.type == KEYDOWN and event.key == K_z:
                    # Start/stop simulation
                    simulate = not simulate
                elif event.type == KEYDOWN and event.key == K_r:
                    # Reset.
                    self.reset()
                elif event.type == KEYDOWN and event.key == K_h:
                    self.chassis_b.angular_velocity = -2
                elif event.type == KEYDOWN and event.key == K_u:
                    self.chassis_b.apply_force_at_local_point([0,-1500000])
                elif event.type == KEYDOWN and event.key == K_j:
                    self.chassis_b.apply_force_at_local_point([0,1500000])
                elif event.type == KEYDOWN and event.key == K_k:
                    self.chassis_b.angular_velocity = 2
                elif event.type == KEYDOWN and event.key == K_w:
                    self.motor_ba1Left.rate = -rotationRate
                elif event.type == KEYDOWN and event.key == K_s:
                    self.motor_ba1Left.rate = rotationRate
                elif event.type == KEYDOWN and event.key == K_a:
                    self.motor_ac1Left.rate = -rotationRate
                elif event.type == KEYDOWN and event.key == K_d:
                    self.motor_ac1Left.rate = rotationRate
                elif event.type == KEYDOWN and event.key ==K_UP:
                    self.motor_ba1Right.rate = rotationRate
                elif event.type == KEYDOWN and event.key ==K_DOWN:
                    self.motor_ba1Right.rate = -rotationRate
                elif event.type == KEYDOWN and event.key ==K_LEFT:
                    self.motor_ac1Right.rate = -rotationRate
                elif event.type == KEYDOWN and event.key ==K_RIGHT:
                    self.motor_ac1Right.rate = rotationRate
                elif event.type == KEYUP:
                    print("keyup")
                    self.motor_ba1Left.rate = 0
                    self.motor_ac1Left.rate = 0
                    self.motor_ba1Right.rate = 0 
                    self.motor_ac1Right.rate = 0
                    self.chassis_b.angular_velocity = 0

            

### Simulator class over***************************************End of file**********************************************************************************

if __name__ == '__main__':
    sim = Simulator()
    win = window_det()
    while(True):
        sim.step_manual() ## should be taking in an array of actions and returning current state, action, reward, and next s
        sim.render() ## thi
        win.ret_reward()


        ## 40 to -40 for thrusters and 100 to 200 for angles

