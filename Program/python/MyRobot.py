import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import time
from pyfirmata import Arduino, util, SERVO
from time import sleep
import threading



#theta, d, a, alpha
#0 79 0 -90
#0 0 100 180
#0 0 130 0
# J1=[0, 79, 0, 90]
# J2=[0, 0, 100, 0]
# J3=[0, 0, 130, 90]

# dh_params = [J1, J2, J3]

# J1=[0, 79, 0, 90]
# J2=[0, 0, 100, 180]
# J3=[90, 0, -150, 0]

# dh_params = [J1, J2, J3]

class MyRobot():
    #DH parameters
    def __init__(self,dh_params):
        self.is_ready  = False
        self.theta1 = dh_params[0][0]
        self.theta2 = dh_params[1][0]
        self.theta3 = dh_params[2][0]
        self.d1     = dh_params[0][1]
        self.d2     = dh_params[1][1]
        self.d3     = dh_params[2][1]
        self.a1     = dh_params[0][2]
        self.a2     = dh_params[1][2]
        self.a3     = dh_params[2][2]
        self.alpha1 = dh_params[0][3]
        self.alpha2 = dh_params[1][3]
        self.alpha3 = dh_params[2][3]
        
        ## Servo control
        self.port= 'COM4'
        self.board = Arduino(self.port)
        self.servo1pin=4
        self.servo2pin=5
        self.servo3pin=6
        self.servo4pin=7
        self.board.digital[self.servo1pin].mode = SERVO
        self.board.digital[self.servo2pin].mode = SERVO
        self.board.digital[self.servo3pin].mode = SERVO
        self.board.digital[self.servo4pin].mode = SERVO
        self.board.digital[self.servo1pin].write(90)
        self.board.digital[self.servo2pin].write(90)
        self.board.digital[self.servo3pin].write(90)
        self.board.digital[self.servo4pin].write(30)
        time.sleep(1)
        self.is_ready  = True

        self.camera_tranform_matrix= []

    def cubic_poly_traj(self, q0, qf, v0, vf, t0, tf, t):
        a0 = q0
        a1 = v0
        a2 = (3*(qf-q0)/(tf-t0)**2) - ((vf+v0)/(tf-t0))
        a3 = (-2*(qf-q0)/(tf-t0)**3) + ((vf+v0)/(tf-t0)**2)
        qt = a0 + a1*(t-t0) + a2*(t-t0)**2 + a3*(t-t0)**3
        return qt
    
    def quintic_poly_traj(self, q0, qf, v0, vf, a0, af, t0, tf, t):
        a0 = q0
        a1 = v0
        a2 = a0/2
        a3 = (20*(qf-q0) - (8*vf+12*v0)*(tf-t0) - (3*af-af)*(tf-t0)**2)/(2*(tf-t0)**3)
        a4 = (30*(q0-qf) + (14*vf+16*v0)*(tf-t0) + (3*af-2*af)*(tf-t0)**2)/(2*(tf-t0)**4)
        a5 = (12*(qf-q0) - 6*(vf+v0)*(tf-t0) - (af-af)*(tf-t0)**2)/(2*(tf-t0)**5)
        qt = a0 + a1*(t-t0) + a2*(t-t0)**2 + a3*(t-t0)**3 + a4*(t-t0)**4 + a5*(t-t0)**5
        return qt
    
    def plan_trajectory_cubic(self, q0, qf, v0, vf, t0, tf, dt):
        t = np.arange(t0, tf, dt)
        plan=[]
        for i in range(len(t)):
            qt = self.cubic_poly_traj(q0, qf, v0, vf, t0, tf, t[i])
            plan.append(qt)
        return plan

    def plan_trajectory_quintic(self, q0, qf, v0, vf, a0, af, t0, tf, dt):
        t = np.arange(t0, tf, dt)
        plan=[]
        for i in range(len(t)):
            qt = self.quintic_poly_traj(q0, qf, v0, vf, a0, af, t0, tf, t[i])
            plan.append(qt)
        return plan
    
    def move_joint_cubic(self, pin, plan, dt):
        for i in range(len(plan)):
            self.board.digital[pin].write(plan[i])
            time.sleep(dt)
    
    def move_joint_quintic(self, pin, plan, dt):
        for i in range(len(plan)):
            self.board.digital[pin].write(plan[i])
            time.sleep(dt)

    def ready_position(self):
        self.initial_position()
        self.is_ready  = True

    def initial_position(self):
        self.board.digital[self.servo4pin].write(30)
        self.board.digital[self.servo2pin].write(90)
        time.sleep(0.25)
        self.board.digital[self.servo1pin].write(90)
        self.board.digital[self.servo3pin].write(90)
        time.sleep(0.75)
        

    def DH(self, theta, d, a, alpha):
        theta = math.radians(theta)
        alpha = math.radians(alpha)
        T = np.array([[math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
                      [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
                      [0, math.sin(alpha), math.cos(alpha), d],
                      [0, 0, 0, 1]])
        return T
    
    def FK(self, theta1, theta2, theta3):
        T1 = self.DH(theta1, self.d1, self.a1, self.alpha1)
        T2 = self.DH(theta2, self.d2, self.a2, self.alpha2)
        T3 = self.DH(theta3, self.d3, self.a3, self.alpha3)
        T = T1.dot(T2).dot(T3)
        return T
    
    def IK_left_elbow_up(self, x, y, z):
        r_2 = x**2+y**2
        w=math.sqrt(r_2-self.d2**2) 
        theta1 = math.atan2(y, x) - math.atan2(self.d2, w)
        theta3 = math.acos((w**2+(z-self.d1)**2-self.a2**2-self.a3**2)/(2*self.a2*self.a3))
        theta2 = math.atan2(z-self.d1, w) + math.atan2(self.a3*math.sin(theta3), self.a2+self.a3*math.cos(theta3))
        print("theta1,theta2,theta3")
        print(math.degrees(theta1),math.degrees(theta2),math.degrees(theta3))
        theta1=math.degrees(theta1)
        theta2 = 180 - math.degrees(theta2)
        theta3 = 180 - math.degrees(theta3)
        

        return round(max(0,min(180,theta1))),round(max(0,min(180,theta2))),round(max(0,min(180,theta3)))
    
    
    @staticmethod
    def goal_position(color):
        if color=="orange":
            x=150
            y=110
            z=20
        elif color=="green":
            x=150
            y=35
            z=20
        elif color=="blue":
            x=-150
            y=35
            z=20
        elif color=="pink":
            x=-150
            y=110
            z=20
        return x,y,z

    def start(self,mission):
        self.is_ready  = False
        target_x=mission[0]
        target_y=mission[1]
        target_z=17
        print("target_x","target_y","target_z")
        print(target_x,target_y,target_z)
        goal_x,goal_y,goal_z=self.goal_position(mission[2])
        theta1,theta2,theta3=self.IK_left_elbow_up(target_x,target_y,target_z)
        print("grab")
        print(theta1,theta2,theta3)

        self.board.digital[self.servo1pin].write(theta1)
        self.board.digital[self.servo3pin].write(theta3)
        time.sleep(1)
        self.board.digital[self.servo2pin].write(theta2)
        time.sleep(1)
        self.board.digital[self.servo4pin].write(70) #grab maybe 70
        time.sleep(1)

        self.board.digital[self.servo2pin].write(90)
        time.sleep(1)
        self.board.digital[self.servo1pin].write(90)
        self.board.digital[self.servo3pin].write(90)
        self.board.digital[self.servo4pin].write(70)
        time.sleep(1)
        
        print("goal_x","goal_y","goal_z")
        print(goal_x,goal_y,goal_z)
        
        theta1,theta2,theta3=self.IK_left_elbow_up(goal_x,goal_y,goal_z)
        
        self.board.digital[self.servo1pin].write(theta1)
        self.board.digital[self.servo2pin].write(theta2)
        self.board.digital[self.servo3pin].write(theta3)
        time.sleep(1)
        self.board.digital[self.servo4pin].write(30)
        time.sleep(1)
        self.ready_position()

        # servo1plan = self.plan_trajectory_cubic(90, theta1, 0, 0, 0, 0.4, 0.1)
        # servo3plan = self.plan_trajectory_cubic(90, theta3, 0, 0, 0, 0.5, 0.1)
        # servo2plan = self.plan_trajectory_cubic(90, theta2, 0, 0, 0, 1, 0.1)
        # servo4plan = self.plan_trajectory_cubic(30, 70, 0, 0, 0, 0.4, 0.1)
        # self.move_joint_cubic(self.servo1pin, servo1plan, 0.1)
        # self.move_joint_cubic(self.servo3pin, servo3plan, 0.1)
        # self.move_joint_cubic(self.servo2pin, servo2plan, 0.1)
        # threading.Thread(target=self.move_joint_cubic, args=(self.servo1pin, servo1plan, 0.001)).start()
        # threading.Thread(target=self.move_joint_cubic, args=(self.servo3pin, servo3plan, 0.001)).start()
        # time.sleep(0.7)
        # threading.Thread(target=self.move_joint_cubic, args=(self.servo2pin, servo2plan, 0.001)).start()
        # servo2plan = self.plan_trajectory_cubic(theta2, 90, 0, 0, 0, 1, 0.1)
        # servo3plan = self.plan_trajectory_cubic(theta3, 90, 0, 0, 0, 0.5, 0.1)
        # servo1plan = self.plan_trajectory_cubic(theta1, 90, 0, 0, 0, 0.4, 0.1)
        
        # self.move_joint_cubic(self.servo2pin, servo2plan, 0.1)
        # self.move_joint_cubic(self.servo3pin, servo3plan, 0.1)
        # self.move_joint_cubic(self.servo1pin, servo1plan, 0.1)

        # threading.Thread(target=self.move_joint_cubic, args=(self.servo1pin, servo1plan, 0.001)).start()
        # threading.Thread(target=self.move_joint_cubic, args=(self.servo2pin, servo2plan, 0.001)).start()
        # threading.Thread(target=self.move_joint_cubic, args=(self.servo3pin, servo3plan, 0.001)).start()
        # servo1plan = self.plan_trajectory_cubic(90, theta1, 0, 0, 0, 0.5, 0.1)
        # servo2plan = self.plan_trajectory_cubic(90, theta2, 0, 0, 0, 1, 0.1)
        # servo3plan = self.plan_trajectory_cubic(90, theta3, 0, 0, 0, 0.5, 0.1)
        # servo4plan = self.plan_trajectory_cubic(70, 30, 0, 0, 0, 0.4, 0.1)
        # self.move_joint_cubic(self.servo1pin, servo1plan, 0.1)
        # self.move_joint_cubic(self.servo3pin, servo3plan, 0.1)
        # self.move_joint_cubic(self.servo2pin, servo2plan, 0.1)
        # self.move_joint_cubic(self.servo4pin, servo4plan, 0.1)
        # threading.Thread(target=self.move_joint_cubic, args=(self.servo1pin, servo1plan, 0.001)).start()
        # threading.Thread(target=self.move_joint_cubic, args=(self.servo3pin, servo3plan, 0.001)).start()
        # time.sleep(0.7)
        # threading.Thread(target=self.move_joint_cubic, args=(self.servo2pin, servo2plan, 0.001)).start()
        # time.sleep(0.7)
        # threading.Thread(target=self.move_joint_cubic, args=(self.servo4pin, servo4plan, 0.001)).start()
        # time.sleep(0.5)
        # servo1plan = self.plan_trajectory_cubic(theta1, 90, 0, 0, 0, 1, 0.1)
        # servo2plan = self.plan_trajectory_cubic(theta2, 90, 0, 0, 0, 1, 0.1)
        # servo3plan = self.plan_trajectory_cubic(theta3, 90, 0, 0, 0, 1, 0.1)

        # self.move_joint_cubic(self.servo2pin, servo2plan, 0.1)
        # self.move_joint_cubic(self.servo3pin, servo3plan, 0.1)
        # self.move_joint_cubic(self.servo1pin, servo1plan, 0.1)

        # threading.Thread(target=self.move_joint_cubic, args=(self.servo1pin, servo1plan, 0.001)).start()
        # threading.Thread(target=self.move_joint_cubic, args=(self.servo2pin, servo2plan, 0.001)).start()
        # threading.Thread(target=self.move_joint_cubic, args=(self.servo3pin, servo3plan, 0.001)).start()



if __name__ == "__main__":
    # test IK FK DH
    J1=[0, 79, 0, 90]
    J2=[0, 0, 100, 0]
    J3=[0, 0, 150, 90]
    # J1=[180, 79, 0, 90]
    # J2=[0, 0, -100, 180]
    # J3=[0, 0, -150, 0]
    dh_params = [J1, J2, J3]
    d2=dh_params[1][1]
    print(d2)
    robot= MyRobot(dh_params)
    q1,q2,q3=robot.IK_left_elbow_up(0,121,20)
    print(q1,q2,q3)
    
