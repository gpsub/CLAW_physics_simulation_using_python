import pymunk
def create_body(legMass, legWidth, LegHeight,chassis_pos,chassis_width,start_angle,main_chassis_object,side):

     body_object = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWidth, LegHeight)))
     if side=="left":
       body_object.position = chassis_pos - ((chassis_width/2)+(legWidth/2), 0)
     else:
       body_object.position = chassis_pos + ((chassis_width/2)+(legWidth/2), 0)   
     body_object.start_position = main_chassis_object.start_position + ((chassis_width/2)+(legWidth/2), 0)
     body_object.start_angle = start_angle
     
     return body_object