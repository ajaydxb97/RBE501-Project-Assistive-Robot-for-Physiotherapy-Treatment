import math
import time
import numpy as np
import rospy
from rbdl_script import *
from ambf_msgs.msg import ObjectState, ObjectCmd

class Simulate():

    def __init__(self):

        # # Recording data points:
        # self.myfile = open("trajectory_rightTriangle.txt", "a") #opening in append mode
        # self.fileCount = 0 #recording 2000 points.
        rospy.init_node('main_script')
        self.pub = rospy.Publisher('/ambf/env/base/Command', ObjectCmd, queue_size=1)
        self.sub = rospy.Subscriber('/ambf/env/base/State', ObjectState, self.run, queue_size=1)
        self.initialize()

    def initialize(self):
        
        cmd_msg = ObjectCmd()

        self.cmd_msg = cmd_msg
        self.t_start = rospy.get_time()
        self.t_old = 0

        self.q_old = np.zeros((6,1))
        self.pos_old = np.zeros((3,1))
        self.J_old = np.zeros((3,6))

        self.model,self.tip_body = model_generator()

        self.radius = 0.2
        self.omega = math.pi/36
        self.lin_vel = 0.1
        self.ylim = [-0.4,0.4]
        self.zlim = [0.1,0.5]
        self.dir = '+'
        self.velY = self.lin_vel
        self.velZ = self.lin_vel
        self.force = -25
        self.errcount = 0

        self.initial_pos = np.array(fwd_kinematics(self.model,np.array([0,math.pi/6,0,-math.pi/2,0,math.pi/3]).reshape(6,1),self.tip_body)).reshape(3,1)
        self.center = self.initial_pos - np.array([0,self.radius,0]).reshape(3,1)
        self.idx = 0

    def run(self,data):
        self.joint_pos = np.array(data.joint_positions[0:6]).reshape(6,1)
        self.task_pos = np.array(fwd_kinematics(self.model,self.joint_pos,self.tip_body)).reshape(3,1)
        self.Jac = Jacobian(self.model,self.joint_pos,self.tip_body)
        self.t = rospy.get_time() - self.t_start
        self.compute()

    def compute(self):

        self.dt = self.t - self.t_old
        print(self.dt)
        qcurr = self.joint_pos
        qcurr_dot = (qcurr - self.q_old)/self.dt
        pos = self.task_pos
        vel = (pos-self.pos_old)/self.dt
        J = self.Jac
        J_dot = (J - self.J_old)/self.dt
        inv_J = np.linalg.pinv(J)

        if self.errcount < 100:
            print('initialization')
            qdes_ddot = np.zeros((6,1))
            qdes_dot = np.zeros((6,1))
            qdes = np.array([0,math.pi/6,0,-math.pi/2,0,math.pi/3]).reshape(6,1)

            e = qcurr - qdes
            e_dot = qcurr_dot - qdes_dot

            Kp = np.diag([1,1,1,1,1,1])
            Kd = np.diag([0.1,0.1,0.1,0.1,0.1,0.1])

            aq = qdes_ddot - np.matmul(Kp,e) - np.matmul(Kd,e_dot)
            print(np.linalg.norm(e))
            if np.linalg.norm(e) < 0.45:
                self.errcount += 1

        else:
            print('tracking')

            # # Circular Path:
            # virtX = pos[0] - self.center[0]
            # virtY = pos[1] - self.center[1]
            #
            # theta = math.atan2(virtX,virtY)
            # net_vel = self.omega*self.radius
            # velX = net_vel*math.cos(theta)
            # velY = -net_vel*math.sin(theta)
            #
            # vel_ref = np.array([velX,velY,0]).reshape(3,1)
            # pos_ref = pos + vel_ref*self.dt
            # acc_ref = np.zeros((3,1))


            # Initial Config:
            # vel_ref = np.zeros((3,1))
            # pos_ref = self.initial_pos
            # acc_ref = np.zeros((3,1))

            # Linear Path Y:
            print(pos[1])
            if pos[1] > self.ylim[1]:
                self.velY = -self.lin_vel
                self.force = 25
            elif pos[1] < self.ylim[0]:
                self.velY = self.lin_vel
                self.force = -25

            # # Linear Path Z:
            # print(pos[2])
            # if pos[2] > self.zlim[1]:
            #     self.velZ = -self.lin_vel
            # elif pos[2] < self.zlim[0]:
            #     self.velZ = self.lin_vel

            # #Right Triangle Path
            # rad_angle = (np.pi/180) * 53.1
            #
            # if (pos[2] > self.zlim[1]):
            #     #Over vertical line and should follow inclined line down
            #     self.velX = (self.lin_vel) * (np.cos(rad_angle))
            #     self.velZ = -1 * (self.lin_vel) * (np.sin(rad_angle))
            #
            # elif ((pos[2] > self.zlim[1]) and (abs(pos[0] - self.xlim[0]) < 0.1)):
            #     #Over vertical line and should follow inclined line down
            #     self.velX = (self.lin_vel) * (np.cos(rad_angle))
            #     self.velZ = -1 * (self.lin_vel) * (np.sin(rad_angle))
            #
            # elif ((pos[2] < self.zlim[0]) and (pos[0] > self.xlim[1])):
            #     #Below inclined line and should follow horizontal line left
            #     self.velX = -(self.lin_vel)
            #     self.velZ = 0.0
            #
            # elif ((abs(pos[2] - self.zlim[0]) < 0.1) and (pos[0] < self.xlim[0])):
            #     #Left of horizontal line
            #     self.velX = 0.0
            #     self.velZ = self.lin_vel
            # else:
            #     self.velX = 0.0
            #     self.velZ = self.lin_vel

            # #Rectangle Path
            # #initialize velocities as zero
            # if (abs(pos[2] - self.zlim[0]) < 0.1) and (abs(pos[0] - self.xlim[0]) < 0.1):
            # 	self.velX = 0.0
            #     self.velZ = (self.lin_vel)
            #
            # elif ((pos[2] > self.zlim[1]) and (abs(pos[0] - self.xlim[0]) < 0.1)):
            #     #Over vertical line and should go right
            #     self.velX = (self.lin_vel)
            #     self.velZ = 0.0
            #
            # elif ((pos[2] < self.zlim[0]) and (abs(pos[0] - self.xlim[1]) < 0.1)):
            #     #Below vertical line and should go left
            #     self.velX = -(self.lin_vel)
            #     self.velZ = 0.0
            #
            # elif ((abs(pos[2] - self.zlim[0]) < 0.1) and (pos[0] < self.xlim[0])):
            #     #Left of horizontal line and should go up
            #     self.velX = 0.0
            #     self.velZ = self.lin_vel
            #
            # elif ((abs(pos[2] - self.zlim[1]) < 0.1) and (pos[0] > self.xlim[1])):
            #     #Right of horizontal line and should go down
            #     self.velX = 0.0
            #     self.velZ = -self.lin_vel

            vel_ref = np.array([0,self.velY,0]).reshape(3,1)
            pos_ref = pos + vel_ref * self.dt
            acc_ref = np.zeros((3,1))

            e = pos - pos_ref
            e_dot = vel - vel_ref

            Kp = np.diag([25,25,25])
            Kd = np.diag([3,3,3])

            ax = acc_ref - np.matmul(Kp,e) - np.matmul(Kd,e_dot)

            aq = np.matmul(inv_J,(ax - np.matmul(J_dot,qcurr_dot)))

        tau = np.zeros((6,1))
        torque = torque_computer(self.model,qcurr,qcurr_dot,aq,tau)
        self.cmd_msg.joint_cmds = [torque[0],torque[1],torque[2],torque[3],torque[4],torque[5]]
        self.cmd_msg.wrench.force.y = self.force
        self.pub.publish(self.cmd_msg)

        self.t_old = self.t
        self.q_old = qcurr
        self.pos_old = pos
        self.J_old = J
        self.idx += 1

        # # Recording:
        # if self.fileCount < 3000:
	    #     # self.myfile.write(str(pos[0][0]) + "," + str(pos[1][0]) + "," + str(pos[2][0]) + "\n")
        #     self.myfile.write(str(torque[0])+","+str(torque[1])+","+str(torque[2])+","+str(torque[3])+","+str(torque[4])+","+str(torque[5])+"\n")
        #     self.fileCount += 1
        # elif self.fileCount == 3000:
        #     self.myfile.close()

if __name__ == '__main__':
    try:
        Simulate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
