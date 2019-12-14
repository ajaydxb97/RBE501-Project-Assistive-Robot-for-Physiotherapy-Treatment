import rbdl
import numpy as np

# Generate dynamics model for rbdl computations:
def model_generator():

    mass_arr = [1,1,1,1,1,1,1,1,1,1]

    COM_matrix = np.array([[0.001,0.0,0.06],[0.0,-0.017,0.134],[0.0,-0.074,0.009]
                          ,[0.0,0.017,0.134],[-0.001,0.081,0.008],[0.0,-0.017,0.129]
                          ,[0.0,0.007,0.068],[0.006,0.0,0.015],[0.,0.,0.],[0.,0.,0.]])

    inertia_matrix = dict()
    inertia_matrix['base'] = np.diag([0.,0.,0.])
    inertia_matrix['link1'] = np.diag([0.00818,0.00737791,0.00331532])
    inertia_matrix['link2'] = np.diag([0.00820624,0.00337541,0.00741212])
    inertia_matrix['link3'] = np.diag([0.00812617,0.00739079,0.0033597])
    inertia_matrix['link4'] = np.diag([0.00819873,0.00333248,0.00737665])
    inertia_matrix['link5'] = np.diag([0.00775252,0.00696512,0.00333736])
    inertia_matrix['link6'] = np.diag([0.00300679,0.0032974,0.0031543])
    inertia_matrix['link7'] = np.diag([0.00065849,0.00065487,0.00113144])
    inertia_matrix['bar'] = np.diag([9.07819496e-04,9.07819496e-04,8.06572673e-05])
    inertia_matrix['tip'] = np.diag([0.00014625,0.00014625,0.00017098])

    joint_rot_z = rbdl.Joint.fromJointType("JointTypeRevoluteZ")
    joint_fixed = rbdl.Joint.fromJointType("JointTypeFixed")

    arm_model = rbdl.Model()
    arm_model.gravity = [0., 0., -9.81]

    trans0 = rbdl.SpatialTransform()
    trans0.E = np.eye(3)
    trans0.r = np.array([0.0,0.0,0.0])
    trans1 = rbdl.SpatialTransform()
    trans1.E = np.eye(3)
    trans1.r = np.array([0.0,0.0,0.103])
    trans2 = rbdl.SpatialTransform()
    trans2.E = np.transpose(np.array([1., 0., 0., 0., 0., 1., 0., -1., 0.]).reshape(3,3))
    trans2.r = np.array([0.,0.013,0.209])
    trans3 = rbdl.SpatialTransform()
    trans3.E = np.transpose(np.array([1., 0., 0., 0., 0., -1., 0., 1., 0.]).reshape(3,3))
    trans3.r = np.array([0.,-0.194,-0.009])
    trans4 = rbdl.SpatialTransform()
    trans4.E = np.transpose(np.array([1., 0., 0., 0., 0., -1., 0., 1., 0.]).reshape(3,3))
    trans4.r = np.array([0.,-0.013,0.202])
    trans5 = rbdl.SpatialTransform()
    trans5.E = np.transpose(np.array([1., 0., 0., 0., 0., 1., 0., -1., 0.]).reshape(3,3))
    trans5.r = np.array([-0.002,0.202,-0.008])
    trans6 = rbdl.SpatialTransform()
    trans6.E = np.transpose(np.array([1., 0., 0., 0., 0., 1., 0., -1., 0.]).reshape(3,3))
    trans6.r = np.array([0.002,-0.052,0.204])
    trans7 = rbdl.SpatialTransform()
    trans7.E = np.transpose(np.array([1., 0., 0., 0., 0., -1., 0., 1., 0.]).reshape(3,3))
    trans7.r = np.array([-0.003,-0.05,0.053])
    trans8 = rbdl.SpatialTransform()
    trans8.E = np.eye(3)
    trans8.r = np.array([0.005,-0.0,0.08])
    trans9 = rbdl.SpatialTransform()
    trans9.E = np.eye(3)
    trans9.r = np.array([0.001,-0.0,0.056])

    obj1 = rbdl.Body.fromMassComInertia(mass_arr[0],COM_matrix[0],inertia_matrix['base'])
    obj2 = rbdl.Body.fromMassComInertia(mass_arr[1],COM_matrix[1],inertia_matrix['link1'])
    obj3 = rbdl.Body.fromMassComInertia(mass_arr[2],COM_matrix[2],inertia_matrix['link2'])
    obj4 = rbdl.Body.fromMassComInertia(mass_arr[3],COM_matrix[3],inertia_matrix['link3'])
    obj5 = rbdl.Body.fromMassComInertia(mass_arr[4],COM_matrix[4],inertia_matrix['link4'])
    obj6 = rbdl.Body.fromMassComInertia(mass_arr[5],COM_matrix[5],inertia_matrix['link5'])
    obj7 = rbdl.Body.fromMassComInertia(mass_arr[6],COM_matrix[6],inertia_matrix['link6'])
    obj8 = rbdl.Body.fromMassComInertia(mass_arr[7],COM_matrix[7],inertia_matrix['link7'])
    obj9 = rbdl.Body.fromMassComInertia(mass_arr[8],COM_matrix[8],inertia_matrix['bar'])
    obj10 = rbdl.Body.fromMassComInertia(mass_arr[9],COM_matrix[9],inertia_matrix['tip'])

    body1 = arm_model.AppendBody(trans0,joint_fixed,obj1)
    body2 = arm_model.AppendBody(trans1,joint_rot_z,obj2)
    body3 = arm_model.AppendBody(trans2,joint_rot_z,obj3)
    body4 = arm_model.AppendBody(trans3,joint_rot_z,obj4)
    body5 = arm_model.AppendBody(trans4,joint_rot_z,obj5)
    body6 = arm_model.AppendBody(trans5,joint_rot_z,obj6)
    body7 = arm_model.AppendBody(trans6,joint_rot_z,obj7)
    body8 = arm_model.AppendBody(trans7,joint_fixed,obj8)
    body9 = arm_model.AppendBody(trans8,joint_fixed,obj9)
    body10 = arm_model.AppendBody(trans9,joint_fixed,obj10)

    return arm_model,body10

def torque_computer(model,q,qdot,qddot,tau):
    size = len(q)
    q = q.reshape(size,)
    qdot = qdot.reshape(size,)
    qddot = qddot.reshape(size,)
    tau = tau.reshape(size,)
    rbdl.InverseDynamics(model,q,qdot,qddot,tau)
    return tau

def fwd_kinematics(model,q,body):
    size = len(q)
    q = q.reshape(size,)
    point_local = np.array([0.0, 0.0, 0.0])
    end_pos = rbdl.CalcBodyToBaseCoordinates(model, q, body, point_local)
    return end_pos

def Jacobian(model,q,body):
    point_coords = np.array([0.,0.,0.])
    J = np.zeros((3,6))
    q = q.reshape(6,)
    rbdl.CalcPointJacobian(model,q,body,point_coords,J)
    return J
