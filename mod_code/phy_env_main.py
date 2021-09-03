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
        # Set the initial coordinates and the weights/inertias for all the bodies
        chassisXY = Vec2d(self.display_size[0]/2,300)
        chWd = 120; chHt = 5
        chassisMass = 40
        self.reward=0
        legWd_a = 100; legHt_a = 5
        legWd_b = 100; legHt_b = 5 
        legMass = 5
        relativeAnguVel = 0
       ### DEFINITION of  arms of the claw
       ## Trash object definition(the blue circle which is the target)
        self.object_1 =pymunk.Body(2,pymunk.moment_for_circle(5,0,10))
        self.object_1.friction = 2
        self.object_1.position = Vec2d(210, 210)
        self.shape_box  = pymunk.Circle(self.object_1,10)
        self.shape_box.color = (0,0,128,0)            
        # Middle shoulder join definition
        self.chassis_b = pymunk.Body(chassisMass, pymunk.moment_for_box(chassisMass, (chWd, chHt)))
        self.chassis_b.position = chassisXY
        chassis_shape = pymunk.Poly.create_box(self.chassis_b, (chWd, chHt))
        chassis_shape.color = (200, 200, 200,0)
        self.chassis_b.start_position = chassisXY
        self.chassis_b.start_angle = 0
        print("chassis position");print(self.chassis_b.position)
        #-- Definition of left rarearm (adjacent connected  to the middle arm)
        self.leftLeg_1a_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_a, legHt_a)))
        self.leftLeg_1a_body.position = chassisXY - ((chWd/2)+(legWd_a/2), 0)
        self.leftLeg_1a_body.start_position = self.chassis_b.start_position - ((chWd/2)+(legWd_a/2), 0)
        self.leftLeg_1a_body.start_angle = 0 
        self.leftLeg_1a_shape = pymunk.Poly.create_box(self.leftLeg_1a_body, (legWd_a, legHt_a))        
        self.leftLeg_1a_shape.color = (255, 0, 0,0)

        #---Definition of the left forearm(adjacent connected to the left rarearm)
        self.leftLeg_1b_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_b, legHt_b)))
        self.leftLeg_1b_body.position = self.leftLeg_1a_body.position - ((legWd_a/2)+(legWd_b/2), 0)
        self.leftLeg_1b_body.start_position = self.leftLeg_1a_body.start_position - ((legWd_a/2)+(legWd_b/2), 0)
        self.leftLeg_1b_body.start_angle = 0 
        self.leftLeg_1b_shape = pymunk.Poly.create_box(self.leftLeg_1b_body, (legWd_b, legHt_b))        
        self.leftLeg_1b_shape.color = (0, 255, 0,0)        

        #---Definition of the right rarearm(adjacent connected to the right of the middle shoulder body)
        self.rightLeg_1a_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_a, legHt_a)))
        self.rightLeg_1a_body.position = chassisXY + ((chWd/2)+(legWd_a/2), 0)
        self.rightLeg_1a_body.start_position = self.chassis_b.start_position + ((chWd/2)+(legWd_a/2), 0)
        self.rightLeg_1a_body.start_angle = 0
        self.rightLeg_1a_shape = pymunk.Poly.create_box(self.rightLeg_1a_body, (legWd_a, legHt_a))        
        self.rightLeg_1a_shape.color = (255, 0, 0,0)        

        #---Definition of the right forearm (adjacent connected to the right of the right rarearm)
        self.rightLeg_1b_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_b, legHt_b)))
        self.rightLeg_1b_body.position = self.rightLeg_1a_body.position + ((legWd_a/2)+(legWd_b/2), 0)
        self.rightLeg_1b_body.start_position = self.rightLeg_1a_body.start_position + ((legWd_a/2)+(legWd_b/2), 0)
        self.rightLeg_1b_body.start_angle = 0
        self.rightLeg_1b_shape = pymunk.Poly.create_box(self.rightLeg_1b_body, (legWd_b, legHt_b))        
        self.rightLeg_1b_shape.color = (0, 255, 0,0)     
        
        self.object_1.start_position = Vec2d(210,210)
        self.object_1.start_angle = 0

        #---joint link between left rarearm and forearm    
        pj_ba1left = pymunk.PinJoint(self.leftLeg_1b_body, self.leftLeg_1a_body, (legWd_b/2,0), (-legWd_a/2,0))#anchor point coordinates are wrt the body; not the space
        self.motor_ba1Left = pymunk.SimpleMotor(self.leftLeg_1b_body, self.leftLeg_1a_body, relativeAnguVel)
        self.motor_ba1Left.collide_bodies = False
        
        # joint link betwwen left rarearm and middle chassis body
        pj_ac1left = pymunk.PinJoint(self.leftLeg_1a_body, self.chassis_b, (legWd_a/2,0), (-chWd/2, 0))
        self.motor_ac1Left = pymunk.SimpleMotor(self.leftLeg_1a_body, self.chassis_b, relativeAnguVel)
        self.motor_ac1Left.collide_bodies = False
        
        # joint link between the right forearm and right rarearm
        pj_ba1Right = pymunk.PinJoint(self.rightLeg_1b_body, self.rightLeg_1a_body, (-legWd_b/2,0), (legWd_a/2,0))#anchor point coordinates are wrt the body; not the space
        self.motor_ba1Right = pymunk.SimpleMotor(self.rightLeg_1b_body, self.rightLeg_1a_body, relativeAnguVel)
        self.motor_ba1Right.collide_bodies = False
        # joint link between the right rare arm and the middle chassis body
        pj_ac1Right = pymunk.PinJoint(self.rightLeg_1a_body, self.chassis_b, (-legWd_a/2,0), (chWd/2, 0 ))
        self.motor_ac1Right = pymunk.SimpleMotor(self.rightLeg_1a_body, self.chassis_b, relativeAnguVel)      
        self.motor_ac1Right.collide_bodies = False

        ## Adding all the bodies to the defined pymunk space, this will display it on the window

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

    
        ## adding a collision handler so that we can modify what happens when target object/ claw arms collide with
        # each other (if required)
        handler = self.space.add_default_collision_handler()
        handler.begin = self.coll_begin
        handler.pre_solve = self.coll_pre
        handler.post_solve = self.coll_post
        handler.separate = self.coll_separate
       
       ## variables to control the movement of the claw 
        self.target = [100,130,130,100]
        self.passed = self.target ## initially same, then passed decreases to 0
        ## passed value should be decreasing, target should be constant, passed value should be target value minus current value
        self.simulate = True
        self.x_thrust=0
        self.y_thrust=0

## ***************************************-------------------------------------------------------------------------------------**********
## down , we have all the useful functions for this environment


    def render(self):
        ## this function is called after every frame refresh,shows the changes in the position/ orientation of all objects
        ## refreshes the entire screen
        self.screen.fill((255,255,255))
        self.space.debug_draw(self.draw_options)### Draw space        
        pygame.display.flip()
        fps = 50
        iterations = 25
        dt = 1.0/float(fps)/float(iterations)
        if self.simulate:
                for x in range(iterations): # 10 iterations to get a more stable simulation
                    self.space.step(dt)
        self.return_angle_state()
        self.check_collide()
        self.change_angle(self.passed)
        pygame.display.flip()
        self.clock.tick(fps)
    
    def return_angle_state(self):
        ## this function is basically to note what angle values are currently there in the claw arms
        ## it doesnt have any utility, just for development purposes
            self.chassis_angle = self.chassis_b.angle*180/np.pi
            self.lr_angle = abs(round(self.leftLeg_1a_body.angle*180/np.pi-self.chassis_angle-180))
            self.rr_angle  = round(self.rightLeg_1a_body.angle*180/np.pi-self.chassis_angle+180)
            self.lf_angle = abs(round(self.leftLeg_1b_body.angle*180/np.pi+self.lr_angle-self.chassis_angle-360))
            self.rf_angle = round(self.rightLeg_1b_body.angle*180/np.pi-self.rr_angle-self.chassis_angle+360)
            print(str(self.lf_angle)+" "+str(self.lr_angle)+" "+str(self.rr_angle)+" "+str(self.rf_angle))
        ## state consists of angles, forward velocity of claw, angular velocity of claw, position of target object, caught object or not, net reward
        ## now gives constant angle
    
            
    def check_collide(self):
        ## this function is made so that the target object(trash) can bounce off the walls when it hits them
        tuple1 = self.object_1.position.int_tuple
        if(tuple1[0]>=585 or tuple1[0]<=15):
             self.object_1.velocity = Vec2d(-self.object_1.velocity.x,self.object_1.velocity.y)
        if(tuple1[1]>=585 or tuple1[1]<=15):
             self.object_1.velocity = Vec2d(self.object_1.velocity.x,-self.object_1.velocity.y)


    def reset(self):
        ## this is used to reset all the bodies and objects to the initial state
        for body in self.space.bodies:
            body.position = body.start_position
            body.force = 0, 0
            body.torque = 0
            body.velocity = 0, 0
            body.angular_velocity = 0
            body.angle = body.start_angle
        self.reward = 0
    ## the next four functions are for collision handlers, they dont do anything for now,but we can use them in the future
    # if required
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
        ## this function can be used if we want to know the velocity and direction information from the claw(in the python environment)
        #print("Angle:"+str(self.chassis_b.angle))
        #print("COG:"+str(self.chassis_b.center_of_gravity))
        #print("rotation vector:"+str(self.chassis_b.rotation_vector))
        print("local point velocity vector:"+str(self.chassis_b.velocity_at_local_point([0,0])))
    
    def change_angle(self,angles):
            ## this function is used to set the motor rate so that it can move to the desired target angle, (whether by increasing)
            # or decreasing
            self.chassis_angle = self.chassis_b.angle*180/np.pi
            self.lr_angle = abs(round(self.leftLeg_1a_body.angle*180/np.pi-self.chassis_angle-180))
            self.rr_angle  = round(self.rightLeg_1a_body.angle*180/np.pi-self.chassis_angle+180)
            self.lf_angle = abs(round(self.leftLeg_1b_body.angle*180/np.pi+self.lr_angle-self.chassis_angle-360))
            self.rf_angle = round(self.rightLeg_1b_body.angle*180/np.pi-self.rr_angle-self.chassis_angle+360)
            cur_angles = [self.lf_angle,self.lr_angle,self.rr_angle,self.rf_angle]
            target_angles = self.target
            if (abs(target_angles[0]-cur_angles[0])<delta) and (abs(target_angles[1]-cur_angles[1])<delta) and (abs(target_angles[2]-cur_angles[2])<delta) and (abs(target_angles[3]-cur_angles[3])<delta):
                self.motor_ac1Left.rate = 0
                self.motor_ac1Right.rate = 0
                self.motor_ba1Left.rate = 0
                self.motor_ba1Right.rate  = 0

            if abs(target_angles[0]-cur_angles[0])<delta:
                   self.motor_ba1Left.rate = 0
            elif target_angles[0]-cur_angles[0]>delta:
                    self.motor_ba1Left.rate = -1
            elif target_angles[0]-cur_angles[0]<delta:
                    self.motor_ba1Left.rate = 1
        
            if abs(target_angles[1]-cur_angles[1])<delta:
                   self.motor_ac1Left.rate = 0
            elif target_angles[1]-cur_angles[1]>delta:
                    self.motor_ac1Left.rate = -1
            elif target_angles[1]-cur_angles[1]<delta:
                    self.motor_ac1Left.rate = 1
            
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
        ## this function will be used for the deep learning algorithm, it will make the changes in the environment, get the reward, 
        # and return the next state to the RL model file
        ## actions: [x thruster (forward and backward upto -40 to 40), y thruster (forward and backward(-40 to 40)), 4 angle control values like (+1,-1,-1,+1)]
        ## thruster will also be controlled
        ## this will also return the next state after action, reward for the action taken and done(boolean)
        # state will be : Current angle values, current thruster values, 
        ## -40 to 40 for thruster(forward, backward, rotate let and rotate right) for small value it should rotate little and for large value it should rotate more
        #  and 100-200 for angles
        #action = [1,1,+1,-1,-1,+1] # first value will control the forward backward, second value will control the left and right, 3-6 values will control the angles
        if (action[0]==1) and (self.x_thrust<40):
            self.x_thrust +=1
        if(action[0]==-1) and (self.x_thrust>(-40)):
            self.x_thrust -=1
        if (action[1]==1) and (self.y_thrust<40):
            self.y_thrust +=1
        if(action[1]==-1) and (self.y_thrust>(-40)):
            self.y_thrust -=1
        if(action[2]==1) and (self.lf_angle<200):
            self.target[0] +=1
        if(action[2]==-1) and (self.lf_angle>100):
            self.target[0] -=1
        if(action[3]==1) and (self.lr_angle<200):
            self.target[1] +=1
        if(action[3]==-1) and (self.lr_angle>100):
            self.target[1] -=1
        if(action[4]==1) and (self.rr_angle<200):
            self.target[2] +=1
        if(action[4]==-1) and (self.rr_angle>100):
            self.target[2] -=1
        if(action[5]==1) and (self.rf_angle<200):
            self.target[3] +=1
        if(action[5]==-1) and (self.rf_angle>100):
            self.target[3] -=1
        

        ## applying the forward backward and rotation angles
        self.chassis_b.apply_force_at_local_point([0,-1500000/40*self.x_thrust])
        self.chassis_b.angular_velocity = 4/40*self.y_thrust
        # state will consist of current 4 angle values, position of object,position of claw,distance to object,velocity of claw, angular velocity of claw,current thruster value(x and y)
        new_state = [self.lf_angle,self.lr_angle,self.rr_angle,self.rf_angle,self.chassis_b.position,self.object_1.position,self.chassis_b.velocity,self.chassis_b.angular_velocity]
        done = False
        return new_state,self.reward,done

        

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
    ## here, we make an object of the simulator class and object detection class(from mod_code/cv2_code) 
    ## run it in an infinite loop, to exit we can use Q key or escape key, to reset we use r key.
    while(True):
        sim.step_manual() ## should be taking in an array of actions and returning current state, action, reward, and next s
        sim.render() 
        sim.reward += win.ret_reward()
        # print("reward is "+str(sim.reward))


        ## 40 to -40 for thrusters and 100 to 200 for angles

