import numpy as np
from lib.calculateFK import FK

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """
    fk = FK()
    joint_positions, T0e = fk.forward(q_in)
    z_axis = np.array([0,0,1])
    
    All_Ts = fk.get_T(q_in)
    All_Rs = [T[0:3,0:3] for T in All_Ts]
    
    p_end_eff = T0e[0:3,3]
    
    Jlinear = []
    Jangular = []
    
    Jlinear.append(np.cross(z_axis.T, (p_end_eff - joint_positions[0])))


    for ii in range(1,7):
        Jlinear.append(np.cross(All_Rs[ii-1][0:3,2], (p_end_eff - joint_positions[ii])))
        
        
    JlinearT = np.array(Jlinear).T
    #print(JlinearT)
    Jangular = [np.array([0,0,1])]
    Jangular.extend([np.matmul(R, np.array([0,0,1])) for R in All_Rs[0:6]])
    JangularT = np.array(Jangular).T
    #print(JangularT)
    Jtotal = np.vstack([JlinearT,JangularT])
    J = Jtotal
    return J


if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q),3))

if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q),3))
    
