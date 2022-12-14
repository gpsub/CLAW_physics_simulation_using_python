# from mod_code.cv2_code.main import window_det
import sys
import pygame
from pygame.locals import USEREVENT, QUIT, KEYDOWN, KEYUP,K_e,K_s, K_r, K_q,K_w,K_a,K_s,K_d,K_k,K_u,K_j,K_h,K_j,K_z,K_ESCAPE, K_UP, K_DOWN, K_RIGHT, K_LEFT,K_2,K_4,K_6,K_8
import numpy as np
import pymunk
from pymunk import Vec2d
import pymunk.pygame_util
from physics_params import set_env_physics
from newBody import create_body

delta = 1
rotationRate = 1


class Simulator(object):

    def __init__(self):
        ### NOTE:Initialising the space with all the motors, joints, bodies etc.
        chassisXY,chWd,chHt,chassisMass,legWd_a,legHt_a,legWd_b,legHt_b,legMass,relativeAnguVel = set_env_physics(self)

       ### DEFINITION of  arms of the claw
       ## Blue circle object definition
        self.object_1 =pymunk.Body(2,pymunk.moment_for_circle(5,0,10))
        self.object_1.friction = 2
        self.object_1.position = Vec2d(210, 210)
        self.shape_box  = pymunk.Circle(self.object_1,10)
        self.shape_box.color = (0,0,128,0)            

        # Middle shoulder joint definition                   #pymunk.Body.STATIC
        self.chassis_b = pymunk.Body(chassisMass, pymunk.moment_for_box(chassisMass, (chWd, chHt)))
        self.chassis_b.position = chassisXY
        chassis_shape = pymunk.Poly.create_box(self.chassis_b, (chWd, chHt))
        chassis_shape.color = (200, 200, 200,0)
        self.chassis_b.start_position = chassisXY
        self.chassis_b.start_angle = 0
        print("chassis position");print(self.chassis_b.position)


        #-- Definition of left rarearm (adjacent connected  to the middle arm)
        self.leftLeg_1a_body = create_body(legMass, legWd_a,legHt_a,chassisXY,chWd,0,self.chassis_b,"left")

        self.leftLeg_1a_shape = pymunk.Poly.create_box(self.leftLeg_1a_body, (legWd_a, legHt_a))        
        self.leftLeg_1a_shape.color = (255, 0, 0,0)

        #---Definition of the left forearm(adjacent connected to the left rarearm)
        self.leftLeg_1b_body = create_body(legMass, legWd_b,legHt_b,self.leftLeg_1a_body.position,legWd_a,0,self.leftLeg_1a_body,"left")

        self.leftLeg_1b_shape = pymunk.Poly.create_box(self.leftLeg_1b_body, (legWd_b, legHt_b))        
        self.leftLeg_1b_shape.color = (0, 255, 0,0)        

        #---Definition of the right rarearm(adjacent connected to the right of the middle shoulder body)
        
        self.rightLeg_1a_body = create_body(legMass, legWd_a,legHt_a,chassisXY,chWd,0,self.chassis_b,"right")

        self.rightLeg_1a_shape = pymunk.Poly.create_box(self.rightLeg_1a_body, (legWd_a, legHt_a))        
        self.rightLeg_1a_shape.color = (255, 0, 0,0)        

        #---Definition of the right forearm (adjacent connected to the right of the right rarearm)
        self.rightLeg_1b_body = create_body(legMass, legWd_b,legHt_b,self.rightLeg_1a_body.position,legWd_a,0,self.rightLeg_1a_body,"right")

        self.rightLeg_1b_shape = pymunk.Poly.create_box(self.rightLeg_1b_body, (legWd_b, legHt_b))
        self.rightLeg_1b_shape.color = (0, 255, 0,0)     
        
        self.object_1.start_position = Vec2d(210,210)
        self.object_1.start_angle = 0

        #---joint link between left rarearm and forearm    
        self.pj_ba1left = pymunk.PinJoint(self.leftLeg_1b_body, self.leftLeg_1a_body, (legWd_b/2,0), (-legWd_a/2,0))#anchor point coordinates are wrt the body; not the space
        self.motor_ba1Left = pymunk.SimpleMotor(self.leftLeg_1b_body, self.leftLeg_1a_body, relativeAnguVel)
        self.motor_ba1Left.collide_bodies = False
        
        # joint link betwwen left rarearm and middle chassis body
        self.pj_ac1left = pymunk.PinJoint(self.leftLeg_1a_body, self.chassis_b, (legWd_a/2,0), (-chWd/2, 0))
        self.motor_ac1Left = pymunk.SimpleMotor(self.leftLeg_1a_body, self.chassis_b, relativeAnguVel)
        self.motor_ac1Left.collide_bodies = False
        
        # joint link between the right forearm and right rarearm
        self.pj_ba1Right = pymunk.PinJoint(self.rightLeg_1b_body, self.rightLeg_1a_body, (-legWd_b/2,0), (legWd_a/2,0))#anchor point coordinates are wrt the body; not the space
        self.motor_ba1Right = pymunk.SimpleMotor(self.rightLeg_1b_body, self.rightLeg_1a_body, relativeAnguVel)
        self.motor_ba1Right.collide_bodies = False
        # joint link between the right rare arm and the middle chassis body
        self.pj_ac1Right = pymunk.PinJoint(self.rightLeg_1a_body, self.chassis_b, (-legWd_a/2,0), (chWd/2, 0 ))
        self.motor_ac1Right = pymunk.SimpleMotor(self.rightLeg_1a_body, self.chassis_b, relativeAnguVel)      
        self.motor_ac1Right.collide_bodies = False

        ## Adding all the bodies to the defined pymunk space, this will display it on the window

        self.space.add(self.chassis_b, chassis_shape) 
        self.space.add(self.leftLeg_1a_body, self.leftLeg_1a_shape, self.rightLeg_1a_body, self.rightLeg_1a_shape) 
        self.space.add(self.leftLeg_1b_body, self.leftLeg_1b_shape, self.rightLeg_1b_body, self.rightLeg_1b_shape) 
        self.space.add(self.pj_ba1left, self.motor_ba1Left, self.pj_ac1left, self.motor_ac1Left)  
        self.space.add(self.pj_ba1Right, self.motor_ba1Right, self.pj_ac1Right, self.motor_ac1Right)
        self.space.add(self.object_1,self.shape_box) ## Uncomment to add moving ball---prevent collisions with ShapeFilter
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

        self.collided = 0
        ## adding a collision handler so that we can modify what happens when target object/ claw arms collide with
        # each other (if required)
        self.handler = self.space.add_default_collision_handler()
        self.handler.begin = self.coll_begin
        self.handler.pre_solve = self.coll_pre
        self.handler.post_solve = self.coll_post
        self.handler.separate = self.coll_separate
       
       ## variables to control the movement of the claw 
        self.target = [100,180,180,100] # to set the target angle for self.change_angle()
        self.passed = self.target ## initially same, then passed decreases to 0
        ## passed value should be decreasing, target should be constant, passed value should be target value minus current value
        self.simulate = True
        self.x_thrust=0
        self.y_thrust=0

## ***************************************-------------------------------------------------------------------------------------**********
## down , we have all the useful functions for this environment                 -


    def render(self):
        ## this function is called after every frame refresh,shows the changes in the position/ orientation of all objects
        ## refreshes the entire screen
        self.screen.fill((255,255,255))
        self.space.debug_draw(self.draw_options)### Draw space        
        pygame.display.flip()
        fps = 50
        iterations = 25
        dt = (1.0/float(fps))/float(iterations)
        if self.simulate:
                for x in range(iterations): # 10 iterations to get a more stable simulation
                    self.space.step(dt)
        # self.return_angle_state()
        self.check_collide()
        # self.change_angle(self.passed)  
        self.getjointpositions()
        pygame.display.flip()
        self.clock.tick(fps)
    
    def return_angle_state(self):
        ## this function is basically to note what angle values are currently there in the claw arms
        ## it doesnt have any utility, just for development purposes
            convert = 180/np.pi
            self.chassis_angle = self.chassis_b.angle*convert
            self.lr_angle = abs(round(self.leftLeg_1a_body.angle*convert-self.chassis_angle-180))
            self.rr_angle  = round(self.rightLeg_1a_body.angle*convert-self.chassis_angle+180)
            self.lf_angle = abs(round(self.leftLeg_1b_body.angle*convert+self.lr_angle-self.chassis_angle-360))
            self.rf_angle = round(self.rightLeg_1b_body.angle*convert-self.rr_angle-self.chassis_angle+360)
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

    def coll_begin(self,arbiter,space,data):
           
            return True
    def coll_pre(self,arbiter,space,data):
        self.collided = 1
        return True
    def coll_post(self,arbiter,space,data):
        pass
        #print("post solve")
    def coll_separate(self,arbiter,space,data):
        self.collided=0
        pass
   
    def getjointpositions(self):
        body_center = Vec2d(0,0)
        body_left_joint = Vec2d(-60,-2)
        body_right_joint =Vec2d(60,-2)
        body_bc_leftjoint= Vec2d(-50,-2)
        body_ac_leftjoint = Vec2d(-50,-2)
        body_bc_rightjoint = Vec2d(50,-2)
        left1 = self.leftLeg_1a_body.local_to_world(body_bc_leftjoint.int_tuple).int_tuple
        left2 = self.leftLeg_1b_body.local_to_world(body_bc_leftjoint.int_tuple).int_tuple
        right1 = self.rightLeg_1a_body.local_to_world(body_bc_rightjoint.int_tuple).int_tuple
        right2 = self.rightLeg_1b_body.local_to_world(body_bc_rightjoint.int_tuple).int_tuple

        print("Pixel coordinates for joints: \nLeft2:"+str(left2)+" left1: "+str(left1)+" right1:"+str(right1)+" right2:"+str(right2)+" #Collided:"+str(self.collided))

    def printinfo(self):
        ## this function can be used if we want to know the velocity and direction information from the claw(in the python environment)
        #print("Angle:"+str(self.chassis_b.angle))
        #print("COG:"+str(self.chassis_b.center_of_gravity))
        #print("rotation vector:"+str(self.chassis_b.rotation_vector))
        print("local point velocity vector:"+str(self.chassis_b.velocity_at_local_point([0,0])))
    
    def change_angle(self,angles):
            ## this function is used to set the motor rate so that it can move to the desired target angle, (whether by increasing)
            # or decreasing
            convert = 180/np.pi
            self.chassis_angle = self.chassis_b.angle*convert # Converting since pygame angles are given in radians
            self.lr_angle = abs(round(self.leftLeg_1a_body.angle*convert-self.chassis_angle-180))
            self.rr_angle  = round(self.rightLeg_1a_body.angle*convert-self.chassis_angle+180)
            self.lf_angle = abs(round(self.leftLeg_1b_body.angle*convert+self.lr_angle-self.chassis_angle-360))
            self.rf_angle = round(self.rightLeg_1b_body.angle*convert-self.rr_angle-self.chassis_angle+360)
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
            elif target_angles[2]-cur_angles[2]>delta:
                    self.motor_ac1Right.rate = 1
            elif target_angles[2]-cur_angles[2]<delta:
                    self.motor_ac1Right.rate = -1
            
            if abs(target_angles[3]-cur_angles[3])<delta:
                   self.motor_ba1Right.rate = 0
            elif target_angles[3]-cur_angles[3]>delta:
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
    # here, we make an object of the simulator class and object detection class(from mod_code/cv2_code) 
    ## run it in an infinite loop, to exit we can use Q key or escape key, to reset we use r key.
    angle=[100,100,100,100]
    while(True):
        sim.step_manual() ## (to control the robot)should be taking in an array of actions and returning current state, action, reward, and next s
        sim.render() ## (to display the changes)