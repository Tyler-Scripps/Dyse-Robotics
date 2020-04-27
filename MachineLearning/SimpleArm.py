import math

EVALUE = - math.pi                  # error value


"""
    This function returns the endpoints of a line at angle theta
"""
def get_end_point(origin, theta, rotor, length):
    y = length * math.sin(theta)
    if theta == 0:
        return (length * math.sin(rotor) + origin[0],
                origin[1],
                length * math.cos(rotor) + origin[2])
    if theta == 90:
        return (origin[0], origin[1] + length, origin[2])
    xz_len = y / math.tan(theta)
    return (xz_len * math.sin(rotor) + origin[0],
            length * math.sin(theta) + origin[1],
            xz_len * math.cos(rotor) + origin[2])


"""
    This function retruns the distance between two points
"""
def distance(p1, p2):
        return math.sqrt(math.pow(p1[0] - p2[0], 2) + 
                            math.pow(p1[1] - p2[1], 2) + 
                            math.pow(p1[2] - p2[2], 2))


"""
    This class creates an arm obj that resides in a 3D plane.
    The arm consists of three links wich can be be adjusted so
        the head is positioned at a desired point within the range.
    
        - update_joints will recalculate the vertices based on the current true_angles
        - set_bow adjusts the angles of the joints to move the head a desired distance
            from the origin (the bow)
        - adjust_to will calculate the rotor angle and bow needed to reach a point in the plane
            Valid points are within a radius of 3 * link_length and cannot be in the negative Z
""" 
class SimpleArmController():
    def __init__(self, link_length, origin, offset):
        self.true_origin = origin
        self.link_length = link_length
        self.joints = [origin]                  # endpoints of each link
        self.angles = [45,45]                   # relative to X
        self.rotor = 0                          # relative to Z
        self.true_angles = []                   # relative to X            
        self.true_angles.append(self.angles[0] + offset)
        self.joints.append(0)
        self.true_angles.append(offset)
        self.joints.append(0)
        self.true_angles.append(- (180 - self.angles[1]) + offset)
        self.joints.append(0)
        self.update_joints()
        # self.claw_orientation = 0
        # self.claw = 0

    """ 
    def set_claw_pinch(self, width):
        #angle = math.acos((c^2 - a^2 - b^2) / (2 * a * b))
        self.claw = angle
    
    def set_claw_rot(self, angle):
        self.claw_orientation = angle
    """  
    def update_joints(self):
        self.joints[0] = self.true_origin
        for i in range(len(self.joints[1:])):
            self.joints[i + 1] = get_end_point(self.joints[i], 
                                                math.radians(self.true_angles[i]), 
                                                math.radians(self.rotor), 
                                                self.link_length)
        
    def set_bow(self, point):
        global EVALUE
        
        l = distance(self.true_origin, (point[0],0,point[2]))
        if l == 0:
            if point[1] >= len(self.true_angles) * self.link_length:
                Yoffset = 90
            elif point[1] == 0:
                Yoffset = 0
            else:
                print('Impossible Error(1): point cannot be below origin!')
                return EVALUE
        else:
            Yoffset = math.degrees(math.atan((point[1] - self.true_origin[1]) / l))
            
        dist = distance(self.true_origin, point)
        
        if dist > (self.link_length * len(self.true_angles)):
            print('OUT OF RANGE: adjusting to maximum')
            
        theta = math.degrees(math.acos((dist - self.link_length) / (2 * self.link_length)))
        
        self.angles[0] = theta
        self.angles[1] = 180 - self.angles[0]
        self.true_angles[0] = self.angles[0] + Yoffset
        self.true_angles[1] = Yoffset
        self.true_angles[2] = - self.angles[0] + Yoffset
        
        return 0
        
    def adjust_to_point(self, point):
        global EVALUE
        if point[2] - self.true_origin[2] == 0 and point[0]  -  self.true_origin[0] > 0:
            self.rotor = 90
        elif point[2] - self.true_origin[2] == 0 and point[0]  -  self.true_origin[0] < 0:
            self.rotor = -90
        elif point[2] - self.true_origin[2] > 0:
            self.rotor = math.degrees(math.atan(point[0] / point[2]))
        elif point[2] - self.true_origin[2] < 0:
            return EVALUE
        
        val = self.set_bow(point)
        if val == EVALUE:
            return val
            
        p_init = self.joints[-1]
        self.update_joints()
        
        return distance(p_init, self.joints[-1])
        
    def adjust_joints(self, angles):
        self.true_angles = angles
        p_init = self.joints[-1]
        self.update_joints()
        return distance(p_init, self.joints[-1])
        
    def monitor_step(self):
        if self.rotor < 89:
            self.rotor += 1
        else:
            self.rotor = - 90
        return self.adjust_joints([90,90,0])
            
    """ def grab_(self, grab_point, orientation):
        self.set_claw(self.MAX_CLAW_DIST)
        dist = self.adjust_to(grab_point)
        self.close_claw()
        
    def place_(self, drop_point):
        #is_clear()
        #   if not decide new drop_point
        p_init = self.joints[-1]
        self.adjust_to(drop_point)
        self.open_claw()"""
        
        
"""
    Draw is set to test the systems ability to adjust
"""
def draw(frame, arm, system, targets, head_loc, claw_state):
    """"
    global avg_loss, ctr 
    ctr += 1
    start = datetime.now()
    if len(targets) > 0:
        print('System adjusting')
        val = system.adjust_to_point(targets[0])
        if val == - math.pi:
            print('invalid point')
        print('joints:', system.joints)
        loss = distance(targets[0],system.joints[-1])
        avg_loss += loss
        print('target:', targets[0])
        print('actual:', system.joints[-1])
        print('Loss:', loss)
        targets = targets.remove(targets[0]) 
    else:
        #system.adjust_joints([90,90,0])
        print('Average-Loss:', avg_loss / ctr)
        #targets.append((0,0,5))
        exit(0)  
    """
    #print('Frame:',frame,'Time elapsed:', datetime.now() - start)
    #if frame % 25 == 0:
        
    step = system.monitor_step()
    x,y,z = zip(*system.joints)
    arm.set_data(x,z)                       # Z and Y are flipped for easier reading
    arm.set_3d_properties(y)
    head_loc.set_text('Dist: (%f)' %step)
    claw_state.set_text('Claw: %d' %system.claw)
    return arm, head_loc, claw_state
    
    
def main():
    targets = [(5,5,5)]
    #for i in range(50):
    #   targets.append((random.randrange(-10,10),0, random.randrange(0,10)))
    print(targets)
    time.sleep(3)
    system = SimpleArmController(5, (0,3,0), 45)
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    head_loc = ax.text(0, 0, 10, '')
    claw_state = ax.text(10, 0, 10, '')
    ax.set_xlim(-15, 15)
    ax.set_xlabel('X')
    ax.set_ylim(-15, 15)
    ax.set_ylabel('Z')
    ax.set_zlim(0,20)
    ax.set_ylabel('Y')
    ax.set_title('Hexagon Simulation')
    ax.view_init(90, 15)
    #x, y, z = zip(*system.joints)
    arm, = plt.plot([],[],[], lw=2)
    anim = animation.FuncAnimation(fig, draw, fargs=(arm, system, targets, head_loc, claw_state), frames=1000, interval=2000)
    plt.show()
    
    
    
    
if __name__ == "__main__":
    main()