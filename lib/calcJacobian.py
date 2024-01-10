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
    end_eff_position = T0e[:3,3]
    print(end_eff_position)
    end_eff_rotation_matrix = T0e [:3,:3]
    print(end_eff_rotation_matrix)
    All_T_matrices = fk.get_T(q_in)
    Rotation_matrices = [T0x[:3,:3] for T0x in All_T_matrices)

    axis_of_rotation = fk.get_axis_of_rotation(q_in)
    
    finaljp = joint_positions[-1, :]
    for ii in range(7)
        for i in range(7):
        r = np.array(finaljp - joint_positions[ii, :])
        J[:3, i] = np.cross(axis_of_rotations[:, i], r)
        J[3:, i] = axis_of_rotations[:, i]

    return J

if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q),3))
