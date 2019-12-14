from ambf_client import Client
import time
import numpy as np
import math
import matplotlib.pyplot as plt

def fwd_kinematics(joint_variables):
    l1 = 0.34
    l2 = 0.31
    o1 = 0.25
    o2 = 0.25
    [q1,q2,q3,q4] = joint_variables

    DH_parameters = {'theta':[math.pi/2, -math.pi/2, q1, -math.pi/2 + q2, q3, math.pi/2 + q4],
                        'd':[0, o2, 0, 0, l1, 0],
                        'a':[o1, 0, 0, 0, 0, l2],
                    'alpha':[math.pi/2, 0, -math.pi/2, -math.pi/2, math.pi/2, 0]}

    T = dict()
    for i in range(0,len(DH_parameters['theta'])):
        name = 'A{}'.format(i)
        T[name] = np.array([math.cos(DH_parameters['theta'][i]),  -math.sin(DH_parameters['theta'][i])*math.cos(DH_parameters['alpha'][i]),  math.sin(DH_parameters['theta'][i])*math.sin(DH_parameters['alpha'][i]), DH_parameters['a'][i]*math.cos(DH_parameters['theta'][i]),
                math.sin(DH_parameters['theta'][i]),   math.cos(DH_parameters['theta'][i])*math.cos(DH_parameters['alpha'][i]), -math.cos(DH_parameters['theta'][i])*math.sin(DH_parameters['alpha'][i]), DH_parameters['a'][i]*math.sin(DH_parameters['theta'][i]),
                                0,                   math.sin(DH_parameters['alpha'][i]),                  math.cos(DH_parameters['alpha'][i]),                 DH_parameters['d'][i],
                                0,                                 0,                                0,                 1]).reshape((4,4))
    np.set_printoptions(suppress = True, precision=3)
    T_tip_base = np.eye(4)
    for i in range(0,6):
        key = 'A{}'.format(i)
        T_tip_base = np.matmul(T_tip_base,T[key])

    T_intermediate = dict()
    for i in range(2,6):
        name = 'T_int{}'.format(i)
        T_intermediate[name] = np.eye(4)
        key_list = list(T.keys())
        iter_list = key_list[0:i]
        for key in iter_list:
            T_intermediate[name] = np.matmul(T_intermediate[name],T[key])

    Jacobian = np.zeros((6,4))

    Jacobian[3:6,0] = np.array([0,0,1]).reshape((3))

    for i in range(1,4):
        key = list(T_intermediate.keys())[i]
        item = T_intermediate[key]
        Jacobian[3:6,i] = item[0:3,2]
        Jacobian[0:3,i] = np.cross(item[0:3,2],(T_tip_base[0:3,3]-item[0:3,3]))

    return T_tip_base,Jacobian

def inverse_kinematics(end_effector_pose):
    l1 = 0.34
    l2 = 0.31
    o1 = 0.25
    o2 = 0.25

    DH_parameters = {'theta':[math.pi/2, -math.pi/2],
                        'd':[0, o2],
                        'a':[o1, 0],
                    'alpha':[math.pi/2, 0]}

    T = dict()
    for i in range(0,len(DH_parameters['theta'])):
        name = 'A{}'.format(i)
        T[name] = np.array([math.cos(DH_parameters['theta'][i]),  -math.sin(DH_parameters['theta'][i])*math.cos(DH_parameters['alpha'][i]),  math.sin(DH_parameters['theta'][i])*math.sin(DH_parameters['alpha'][i]), DH_parameters['a'][i]*math.cos(DH_parameters['theta'][i]),
                math.sin(DH_parameters['theta'][i]),   math.cos(DH_parameters['theta'][i])*math.cos(DH_parameters['alpha'][i]), -math.cos(DH_parameters['theta'][i])*math.sin(DH_parameters['alpha'][i]), DH_parameters['a'][i]*math.sin(DH_parameters['theta'][i]),
                                0,                   math.sin(DH_parameters['alpha'][i]),                  math.cos(DH_parameters['alpha'][i]),                 DH_parameters['d'][i],
                                0,                                 0,                                0,                 1]).reshape((4,4))

    A1 = T['A0']
    A2 = T['A1']
    shoulder_pose = np.matmul(A1,A2)
    b = np.linalg.norm(end_effector_pose[0:3,3] - shoulder_pose[0:3,3])
    a = l2
    c = l1
    q4_res = round(-math.pi + math.acos((a**2 + c**2 - b**2)/(2*a*c)),3)
    A6 = np.array([math.cos(math.pi/2 + q4_res),  -math.sin(math.pi/2 + q4_res)*math.cos(0),  math.sin(math.pi/2 + q4_res)*math.sin(0), a*math.cos(math.pi/2 + q4_res),
                    math.sin(math.pi/2 + q4_res),   math.cos(math.pi/2 + q4_res)*math.cos(0), -math.cos(math.pi/2 + q4_res)*math.sin(0), a*math.sin(math.pi/2 + q4_res),
                                    0,                   math.sin(0),                  math.cos(0),                 0,
                                    0,                                 0,                                0,                 1]).reshape((4,4))
    T_2_5 = np.matmul(np.matmul(np.linalg.inv(shoulder_pose),end_effector_pose),np.linalg.inv(A6))
    q2_res = round(math.asin(-T_2_5[2,1]),3)
    q3_res = round(math.asin(T_2_5[2,2]/math.cos(q2_res)),3)
    q1_res = round(math.asin(T_2_5[1,1]/math.cos(q2_res)),3)

    return [q1_res,q2_res,q3_res,q4_res]

# Initialize client
c = Client()
# Connect client and simulation
c.connect()
time.sleep(2)
b = c.get_obj_handle('base')
elbow = c.get_obj_handle('elbow')
tip = c.get_obj_handle('wrist')
# Joints that can be actuated
joint_list = [0,1,2,5]
base_joint_names = []
for item in joint_list:
        base_joint_names.append(b.get_joint_names()[item])
print(base_joint_names)
# Set all joints to 0
for item in joint_list:
        b.set_joint_pos(item,0)
        time.sleep(0.01)
time.sleep(3)
# Print all joint positions
for item in joint_list:
        print(b.get_joint_pos(item))
# Degrees of freedom
n = 4
# Joint limits according to .yaml file
q_limits = {'q0':[-30,150],'q1':[-150,50],'q2':[-70,90],'q3':[-150,0]}
# Forward kinematics loop
while True:
        q = []
        print('Setting joint positions in degrees:')
        # Loop to get joint angle input
        for i in range(0,n):
                name = 'q{}'.format(i)
                print('Limits:\n min:{} max:{}:'.format(q_limits[name][0],q_limits[name][1]))
                q.append(float(input(name+':'))*math.pi/180)
                b.set_joint_pos(joint_list[i],q[i])
        print ("Creating Robotic Chain")
        print ("Forward kinematics")
        # Forward kinematics solution
        T_tip_base_init,Jacobian_init = fwd_kinematics(q)
        print('Fwd kinematics result:')
        np.set_printoptions(suppress = True, precision=3)
        print(T_tip_base_init)
        time.sleep(6)
        print("Tip pose real:")
        print(tip.get_pos())
        print(tip.get_rot())
        print("Joint angles real:")
        for item in joint_list:
                print(b.get_joint_pos(item)*180.0/math.pi)
        time.sleep(2)
        if raw_input("Again? (y/n): ") != 'y':
                break
while True:
        print("Inverse Kinematics")
        end_effector = T_tip_base_init
        q_result = inverse_kinematics(end_effector)
        print('q1 = {}'.format(q_result[0]*180.0/math.pi))
        print('q2 = {}'.format(q_result[1]*180.0/math.pi))
        print('q3 = {}'.format(q_result[2]*180.0/math.pi))
        print('q4 = {}'.format(q_result[3]*180.0/math.pi))
        if raw_input("Again? (y/n): ") != 'y':
                break

a = 0.31
theta = q_result[2]+math.pi/2
q_result_new = np.copy(q_result)
coef = 1
while theta<(150.0*math.pi/180):
        new_path_start = np.array([0,0,-1,0.25,1,0,0,0.56,0,-1,0,-0.34,0,0,0,1]).reshape((4,4))
        desiredPosition = np.copy(new_path_start)
        desiredPosition[0,3] = new_path_start[0,3] + (a*math.sin(theta))
        desiredPosition[1,3] = new_path_start[1,3] - (a*(1-math.cos(theta)))
        desiredPosition[2,3] = new_path_start[2,3]
        desiredPosition[0:3,0:3] = np.array([math.cos(theta-math.pi/2),0,math.sin(theta-math.pi/2),-math.sin(theta-math.pi/2),0,math.cos(theta-math.pi/2),0,-1,0]).reshape((3,3))
        plt.figure(1)
        plt.title('Reference Path')
        plt.scatter(desiredPosition[0,3],desiredPosition[1,3])
        plt.xlabel('x')
        plt.ylabel('y')
        plt.ylim(-0.2, 0.8)
        plt.xlim(-0.2, 0.8)
        theta = theta + math.pi/360
        q_result_new = inverse_kinematics(desiredPosition)
        fwd,jcb = fwd_kinematics(q_result_new)
        plt.figure(2)
        plt.scatter(fwd[0,3],fwd[1,3],color='green')
        print("Output angles in degrees: ")
        for i in range(0,len(q_result)):
                print(q_result_new[i]*180/math.pi)
                b.set_joint_pos(joint_list[i],q_result_new[i])
        time.sleep(0.01)
        plt.figure(1)
        plt.scatter(tip.get_pose()[0],tip.get_pose()[1],color='red')

plt.show()
