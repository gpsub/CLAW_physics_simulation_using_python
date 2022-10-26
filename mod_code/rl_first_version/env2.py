import numpy as np 
import pyglet
from pyglet import shapes
from pyglet.window import key
import pymunk
from pymunk.pyglet_util import DrawOptions

# Using Pyglet library for the GUI window
#  Using numpy for maths calculations
# This file is called in main.py for GUI functionality, this is the environment for the RL simulation
# the rewards and how the arm responds to the angles output by the model are calculated and returned to main.py in this file.

class ClawArms(object):

    def __init__(self):
      # arminfo['l'] is a 4-D np array, which states the length for each arm (constant)
      # arminfo['r'] is a 4-D np array which states the angles for each arm, this is controlled as 4 number input from rl.py file
      # example[0.3,-0.5,0.2,1] --> states how much angle should change w.r.t current angle
        self.arm_info= np.zeros(4,dtype=[('l',np.float32),('r',np.float32)])
        self.arm_info['l']=100
        self.arm_info['r']=[11*np.pi/18,np.pi/4,7*np.pi/18,3*np.pi/4]  ## if only this file is run
        self.on_goal=0
        self.viewer = None
        self.dt = .3 # refresh rate (how much difference change in the angle)
        self.action_bound = [-1, 1] 
        self.arm1_bound =[np.pi/3,np.pi]# limits for the left arm angle
        self.arm2_bound =[-np.pi/2,2*np.pi/3] # limits for the right arm angle
        self.goal = {'x': 180., 'y': 290.,'l':30. }
        self.state_dim = 5# we have 5d array for state
        self.action_dim = 4 # we have 4 d array for angle (actions)
        
    def step(self,action):
        ###Changing the angle
        done = False
        action = np.clip(action, *self.action_bound)
        self.arm_info['r'] += action * self.dt
        self.arm_info['r'] %= np.pi*2     # normalize
        self.arm_info['r'][2]=np.clip( self.arm_info['r'][2],*self.arm2_bound)
        self.arm_info['r'][0]=np.clip( self.arm_info['r'][0],*self.arm1_bound)
         ## calculating the postition of the arms for reward
        goals = [11*np.pi/18,np.pi/4,7*np.pi/18,3*np.pi/4]
        (a1l, a2l,a3l,a4l) = self.arm_info['l']  # radius, arm length
        (a1r, a2r,a3r,a4r) = self.arm_info['r']  # radian, angle
        dist1 = [a1r-goals[0]]
        dist2 = [a2r-goals[1]]
        dist3 = [a3r-goals[2]]
        dist4 = [a4r-goals[3]]
        # r is the reward
        r = -np.sqrt((a1r-goals[0])**2+(a2r-goals[1])**2+(a3r-goals[2])**2+(a4r-goals[3])**2)
        # setting penalty for distance from goal angle
        
          
        a1xy = np.array([230.,150.])    # a1 start (x0, y0)
        a3xy = np.array([130.,150.])    # a3 start (x0,y0)
        a1xy_ = np.array([np.cos(a1r), np.sin(a1r)]) * a1l + a1xy  # a1 end and a2 start (x1, y1)(left rare arm joint)
        a3xy_ = np.array([np.cos(a3r), np.sin(a3r)]) * a3l + a3xy  # a3 end and a4 start (x1, y1)(right rare arm joint)
        finger1 = np.array([np.cos(a2r), np.sin(a2r)]) * a2l + a1xy_  # left forearm tip
        finger2 = np.array([np.cos(a4r), np.sin(a4r)]) * a4l + a3xy_  # right forearm tip
        
        
       # dist1 = [(self.goal['x'] - finger1[0])/400 , (self.goal['y'] - finger1[1]) / 400]
        #dist2 = [(self.goal['x'] - finger2[0])/400 , (self.goal['y'] - finger2[1]) / 400]
        """ 
        r=0;
    
        if(a2r>np.pi/2):
            r -= (a2r-np.pi/2)/np.pi
        if(a4r<np.pi/2):
            r -= (np.pi/2-a4r)/np.pi
        r = r + (-np.sqrt(dist2[0]**2+dist2[1]**2)-np.sqrt(dist1[0]**2+dist1[1]**2))
        """ # normalize features
        
        # done and reward, checking if the endpoints of left and right arm are within the goal box
        if (self.goal['x'] - self.goal['l']/2 <= finger1[0] <= self.goal['x'] + self.goal['l']/2) and (self.goal['y'] - self.goal['l']/2 <= finger1[1] <= self.goal['y'] + self.goal['l']/2)and (self.goal['x'] - self.goal['l']/2 <= finger2[0] <= self.goal['x'] + self.goal['l']/2) and (self.goal['y'] - self.goal['l']/2 <= finger2[1] <= self.goal['y'] + self.goal['l']/2):
            r += 5.
            self.on_goal += 4
            if self.on_goal > 50:
                done = True
        else:
            self.on_goal = 0


        # returning the state after updation to rl.py for getting the next input
        s = np.concatenate((dist1,dist2,dist3,dist4,[1. if self.on_goal else 0.]))
        return s, r, done
          
    def render(self):### this function calls viewer class
        if self.viewer is None:
            self.viewer = Viewer(self.arm_info)
        self.viewer.render()   
        
    def reset(self):## resetting the angles for the arms to a random angle
        self.arm_info['r'] = 2 * np.pi * np.random.rand(4)
        self.on_goal = 0
        (a1l, a2l, a3l, a4l) = self.arm_info['l']  # radius, arm length
        (a1r, a2r ,a3r ,a4r) = self.arm_info['r']  # radian, angle
        goals = [11*np.pi/18,np.pi/4,7*np.pi/18,3*np.pi/4]
        dist1 = [a1r-goals[0]]
        dist2 = [a2r-goals[1]]
        dist3 = [a3r-goals[2]]
        dist4 = [a4r-goals[3]]

        """a1xy = np.array([230.,150.])    # a1 start (x0, y0)
        a3xy = np.array([130.,150.])    # a3 start (x0,y0)
        a1xy_ = np.array([np.cos(a1r), np.sin(a1r)]) * a1l + a1xy  # a1 end and a2 start (x1, y1)(left rare arm joint)
        a3xy_ = np.array([np.cos(a3r), np.sin(a3r)]) * a3l + a3xy  # a3 end and a4 start (x1, y1)(right rare arm joint)
        finger1 = np.array([np.cos(a2r), np.sin(a2r)]) * a2l + a1xy_  # left forearm tip
        finger2 = np.array([np.cos(a4r), np.sin(a4r)]) * a4l + a3xy_  # right forearm tip
        dist1 = [(self.goal['x'] - finger1[0]) / 400, (self.goal['y'] - finger1[1]) / 400]
        dist2 = [(self.goal['x'] - finger2[0]) / 400, (self.goal['y'] - finger2[1]) / 400]
        """
        # normalize features
        
        s = np.concatenate((dist1,dist2,dist3,dist4,[1. if self.on_goal else 0.]))
        return s

    def sample_action(self):### used only when this file is called separately
        return np.random.rand(4)-0.5 

class Viewer(pyglet.window.Window):
    bar_thc = 5
    
    def __init__(self, arm_info): ## getting input of arm angles from CLAW class
        # vsync=False to not use the monitor FPS, we can speed up training
        super(Viewer, self).__init__(width=400, height=400, resizable=False, caption='CLAW_Arm', vsync=False)
        pyglet.gl.glClearColor(1, 1, 1, 1)
        self.arm_info = arm_info
        self.center_coord = np.array([230,150])
        ## Displaying the rectangles, circles etc needed in gui

        self.batch = pyglet.graphics.Batch()    # display whole batch at once
        self.arm_main = self.batch.add(
            4, pyglet.gl.GL_QUADS, None,
            ('v2f', [130, 150,              # location
                     130, 160,
                     240, 160,
                     240, 150]), ('c3B', (0, 255, 0) * 4,))
        
        self.arm4 = self.batch.add(
            4, pyglet.gl.GL_QUADS, None,
            ('v2f', [250, 250,                # location
                     250, 300,
                     260, 300,
                     260, 250]),
            ('c3B', (249, 86, 86) * 4,))    # color
        self.c1 = shapes.Circle(233,155,7,color=(17, 205, 222),batch = self.batch)
        self.c2 = shapes.Circle(130,155,7,color=(17, 205, 222),batch = self.batch)
        self.c3 = shapes.Circle(200,160,7,color=(17, 205, 222),batch = self.batch)
        self.c4 = shapes.Circle(150,80,7,color=(17, 205, 222),batch = self.batch)
        self.arm3 = self.batch.add(
            4, pyglet.gl.GL_QUADS, None,
            ('v2f', [100, 150,              # location
                     100, 160,
                     200, 160,
                     200, 150]), ('c3B', (249, 86, 86) * 4,))
        
        self.arm1 =  self.batch.add(
            4, pyglet.gl.GL_QUADS, None,
            ('v2f', [140,150,              # location
                     130,150,
                     130,250 ,
                    140,250]), ('c3B', (249, 86, 86) * 4,))     
        self.arm2 =  self.batch.add(
            4, pyglet.gl.GL_QUADS, None,
            ('v2f', [130,250,              # location
                     140,250,
                     140,350,
                     130,350]), ('c3B', (249, 86, 86) * 4,))                     
        
        self.space = pymunk.Space()
        self.options = pymunk.pyglet_util.DrawOptions(batch = self.batch)


        ##self.arm_main = self.batch.add(4,pyglet.gl.GL_QUADS,None)
    def render(self):
        self._update_arm()  ##calculating the rectangle position from the angle value
        self.switch_to()
        self.dispatch_events()
        self.dispatch_event('on_show')
        self.flip()

    def on_show(self):##changed from on_draw
        self.clear()
        self.space.debug_draw(self.options)
        self.batch.draw()    

    def _update_arm(self):
        (a1l, a2l,a3l,a4l) = self.arm_info['l']     # radius, arm length
        (a1r, a2r,a3r,a4r) = self.arm_info['r']
         
        # radian, angle
        ##taking center as base of right arm(center coord)
        ## for right arms
        a3xy = self.center_coord            # a1 start (x0, y0)
        a3xy_ = np.array([np.cos(a3r), np.sin(a3r)]) * a3l + a3xy   # a1 end and a2 start (x1, y1)
        a4xy_ = np.array([np.cos(a4r), np.sin(a4r)]) * a4l + a3xy_  # a2 end (x2, y2)
        a3tr, a4tr = np.pi / 2 - self.arm_info['r'][2], np.pi / 2 + self.arm_info['r'][3]
       ## right arms  coordinates of the 4 corners of the rectangle
        xy01 = a3xy + np.array([-np.cos(a3tr), np.sin(a3tr)]) * self.bar_thc
        xy02 = a3xy + np.array([np.cos(a3tr), -np.sin(a3tr)]) * self.bar_thc
        xy11 = a3xy_ + np.array([np.cos(a3tr), -np.sin(a3tr)]) * self.bar_thc
        xy12 = a3xy_ + np.array([-np.cos(a3tr), np.sin(a3tr)]) * self.bar_thc

        xy11_ = a3xy_ + np.array([np.cos(a4tr), np.sin(a4tr)]) * self.bar_thc
        xy12_ = a3xy_ + np.array([-np.cos(a4tr), -np.sin(a4tr)]) * self.bar_thc
        xy21 = a4xy_ + np.array([-np.cos(a4tr), -np.sin(a4tr)]) * self.bar_thc
        xy22 = a4xy_ + np.array([np.cos(a4tr), np.sin(a4tr)]) * self.bar_thc
       ## left arms
       
        #a1r = np.pi-a1r
        a1xy = self.center_coord -np.array([100,0]);
        a1xy_ = np.array([np.cos(a1r),np.sin(a1r)])*a1l +a1xy;
        a2xy_= np.array([np.cos(a2r),np.sin(a2r)])*a2l  +a1xy_;
        a1tr,a2tr = np.pi/2+self.arm_info['r'][0],np.pi/2-self.arm_info['r'][1];
        ## coordinates of the four corners of rectangle for left arms
        _xy01 = a1xy + np.array([np.cos(a1tr), np.sin(a1tr)]) * self.bar_thc
        _xy02 = a1xy + np.array([-np.cos(a1tr), -np.sin(a1tr)]) * self.bar_thc
        _xy11 = a1xy_ + np.array([-np.cos(a1tr), -np.sin(a1tr)]) * self.bar_thc
        _xy12 = a1xy_ + np.array([np.cos(a1tr), np.sin(a1tr)]) * self.bar_thc

        _2xy11 = a1xy_ + np.array([-np.cos(a2tr), np.sin(a2tr)]) * self.bar_thc
        _2xy12 = a1xy_ + np.array([np.cos(a2tr), -np.sin(a2tr)]) * self.bar_thc
        _xy21 = a2xy_ + np.array([np.cos(a2tr), -np.sin(a2tr)]) * self.bar_thc
        _xy22 = a2xy_ + np.array([-np.cos(a2tr), np.sin(a2tr)]) * self.bar_thc
  
       
        ## updating the calculated coordinates and setting them as current coordinates
        self.arm4.vertices = np.concatenate((xy01, xy02, xy11, xy12))
        self.arm3.vertices = np.concatenate((xy11_, xy12_, xy21, xy22))
        self.arm2.vertices = np.concatenate((_2xy11,_2xy12, _xy21, _xy22))
        self.arm1.vertices = np.concatenate((_xy01,_xy02,_xy11,_xy12))
        self.c3.position = a3xy_
        self.c4.position = a1xy_

if __name__ == '__main__':
    env = ClawArms()
    while True:
        env.render()
        env.step(env.sample_action())

