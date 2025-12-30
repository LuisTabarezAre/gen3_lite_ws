import math
import numpy as np

def dk_gen3(q,dq,p):
    """
    Forward kinematics for Gen3 Lite

    q : ndarray (6,) Joints position
    dq : ndarray (6,) Joint velocities
    p : dict with keys a2, d2, d3, d4, d5, d6

    Returns
    -------
    dp   : ndarray (3,1) Linear velocities
    w    : ndarray (3,1) Angular velocities
    """

    q0, q1, q2, q3, q4, q5 = q
    a2 = p['a2']
    d2 = p['d2']
    d3 = p['d3']
    d4 = p['d4']
    d5 = p['d5']
    d6 = p['d6']
    # -------------------------------
    # Geometric Jacobian
    # -------------------------------
    ##### Row 1 #####
    J11 = d2*math.cos(q0) - d3*math.cos(q0) - d5*math.cos(q0)*math.sin(q3) + a2*math.sin(q0)*math.sin(q1) + d4*math.sin(q1 - q2)*math.sin(q0) + d6*math.cos(q0)*math.cos(q3)*math.sin(q4) + d6*math.sin(q1 - q2)*math.cos(q4)*math.sin(q0) - d5*math.cos(q1)*math.cos(q2)*math.cos(q3)*math.sin(q0) - d5*math.cos(q3)*math.sin(q0)*math.sin(q1)*math.sin(q2) - d6*math.cos(q1)*math.cos(q2)*math.sin(q0)*math.sin(q3)*math.sin(q4) - d6*math.sin(q0)*math.sin(q1)*math.sin(q2)*math.sin(q3)*math.sin(q4)
    J12 = -math.cos(q0)*(a2*math.cos(q1) + d4*math.cos(q1 - q2) - (d6*math.sin(q1 - q2)*math.cos(q3 + q4))/2 + d6*math.cos(q1 - q2)*math.cos(q4) + d5*math.sin(q1 - q2)*math.cos(q3) + (d6*math.cos(q3 - q4)*math.sin(q1 - q2))/2)
    J13 = math.cos(q0)*(d4*math.cos(q1 - q2) + d6*math.cos(q1 - q2)*math.cos(q4) + d5*math.sin(q1 - q2)*math.cos(q3) + d6*math.sin(q1 - q2)*math.sin(q3)*math.sin(q4))
    J14 = d6*math.cos(q0)*math.cos(q3)*math.sin(q1)*math.sin(q2)*math.sin(q4) - d6*math.sin(q0)*math.sin(q3)*math.sin(q4) - d5*math.cos(q0)*math.cos(q1)*math.cos(q2)*math.sin(q3) - d5*math.cos(q0)*math.sin(q1)*math.sin(q2)*math.sin(q3) - d5*math.cos(q3)*math.sin(q0) + d6*math.cos(q0)*math.cos(q1)*math.cos(q2)*math.cos(q3)*math.sin(q4)
    J15 = d6*math.cos(q3)*math.cos(q4)*math.sin(q0) - d6*math.cos(q0)*math.cos(q1)*math.sin(q2)*math.sin(q4) + d6*math.cos(q0)*math.cos(q2)*math.sin(q1)*math.sin(q4) + d6*math.cos(q0)*math.cos(q4)*math.sin(q1)*math.sin(q2)*math.sin(q3) + d6*math.cos(q0)*math.cos(q1)*math.cos(q2)*math.cos(q4)*math.sin(q3)
    J16 = 0
    ##### Row 2 #####
    J21 = d2*math.sin(q0) - d3*math.sin(q0) - a2*math.cos(q0)*math.sin(q1) - d5*math.sin(q0)*math.sin(q3) - d4*math.sin(q1 - q2)*math.cos(q0) + d6*math.cos(q3)*math.sin(q0)*math.sin(q4) - d6*math.sin(q1 - q2)*math.cos(q0)*math.cos(q4) + d5*math.cos(q0)*math.cos(q1)*math.cos(q2)*math.cos(q3) + d5*math.cos(q0)*math.cos(q3)*math.sin(q1)*math.sin(q2) + d6*math.cos(q0)*math.cos(q1)*math.cos(q2)*math.sin(q3)*math.sin(q4) + d6*math.cos(q0)*math.sin(q1)*math.sin(q2)*math.sin(q3)*math.sin(q4)
    J22 = -math.sin(q0)*(a2*math.cos(q1) + d4*math.cos(q1 - q2) - (d6*math.sin(q1 - q2)*math.cos(q3 + q4))/2 + d6*math.cos(q1 - q2)*math.cos(q4) + d5*math.sin(q1 - q2)*math.cos(q3) + (d6*math.cos(q3 - q4)*math.sin(q1 - q2))/2)
    J23 = math.sin(q0)*(d4*math.cos(q1 - q2) + d6*math.cos(q1 - q2)*math.cos(q4) + d5*math.sin(q1 - q2)*math.cos(q3) + d6*math.sin(q1 - q2)*math.sin(q3)*math.sin(q4))
    J24 = d5*math.cos(q0)*math.cos(q3) + d6*math.cos(q0)*math.sin(q3)*math.sin(q4) - d5*math.cos(q1)*math.cos(q2)*math.sin(q0)*math.sin(q3) - d5*math.sin(q0)*math.sin(q1)*math.sin(q2)*math.sin(q3) + d6*math.cos(q1)*math.cos(q2)*math.cos(q3)*math.sin(q0)*math.sin(q4) + d6*math.cos(q3)*math.sin(q0)*math.sin(q1)*math.sin(q2)*math.sin(q4)
    J25 = d6*math.cos(q2)*math.sin(q0)*math.sin(q1)*math.sin(q4) - d6*math.cos(q1)*math.sin(q0)*math.sin(q2)*math.sin(q4) - d6*math.cos(q0)*math.cos(q3)*math.cos(q4) + d6*math.cos(q1)*math.cos(q2)*math.cos(q4)*math.sin(q0)*math.sin(q3) + d6*math.cos(q4)*math.sin(q0)*math.sin(q1)*math.sin(q2)*math.sin(q3)
    J26 = 0
    ##### Row 3 #####
    J31 = 0
    J32 = d4*math.cos(q1)*math.sin(q2) - a2*math.sin(q1) - d4*math.cos(q2)*math.sin(q1) + d5*math.cos(q1)*math.cos(q2)*math.cos(q3) + d6*math.cos(q1)*math.cos(q4)*math.sin(q2) - d6*math.cos(q2)*math.cos(q4)*math.sin(q1) + d5*math.cos(q3)*math.sin(q1)*math.sin(q2) + d6*math.cos(q1)*math.cos(q2)*math.sin(q3)*math.sin(q4) + d6*math.sin(q1)*math.sin(q2)*math.sin(q3)*math.sin(q4)
    J33 = d4*math.cos(q2)*math.sin(q1) - d4*math.cos(q1)*math.sin(q2) - d5*math.cos(q1)*math.cos(q2)*math.cos(q3) - d6*math.cos(q1)*math.cos(q4)*math.sin(q2) + d6*math.cos(q2)*math.cos(q4)*math.sin(q1) - d5*math.cos(q3)*math.sin(q1)*math.sin(q2) - d6*math.cos(q1)*math.cos(q2)*math.sin(q3)*math.sin(q4) - d6*math.sin(q1)*math.sin(q2)*math.sin(q3)*math.sin(q4)
    J34 = -math.sin(q1 - q2)*(d5*math.sin(q3) - d6*math.cos(q3)*math.sin(q4))
    J35 = d6*math.cos(q2)*math.cos(q4)*math.sin(q1)*math.sin(q3) - d6*math.sin(q1)*math.sin(q2)*math.sin(q4) - d6*math.cos(q1)*math.cos(q4)*math.sin(q2)*math.sin(q3) - d6*math.cos(q1)*math.cos(q2)*math.sin(q4)
    J36 = 0
    ##### Row 4 #####
    J41 = 0
    J42 = math.sin(q0)
    J43 = -math.sin(q0)
    J44 = -math.sin(q1 - q2)*math.cos(q0)
    J45 = math.cos(q3)*(math.cos(q0)*math.cos(q1)*math.cos(q2) + math.cos(q0)*math.sin(q1)*math.sin(q2)) - math.sin(q0)*math.sin(q3)
    J46 = math.sin(q4)*(math.cos(q3)*math.sin(q0) + math.sin(q3)*(math.cos(q0)*math.cos(q1)*math.cos(q2) + math.cos(q0)*math.sin(q1)*math.sin(q2))) + math.cos(q4)*(math.cos(q0)*math.cos(q1)*math.sin(q2) - math.cos(q0)*math.cos(q2)*math.sin(q1))
    ##### Row 5 #####
    J51 = 0
    J52 = -math.cos(q0)
    J53 = math.cos(q0)
    J54 = -math.sin(q1 - q2)*math.sin(q0)
    J55 = math.cos(q0)*math.sin(q3) + math.cos(q3)*(math.sin(q0)*math.sin(q1)*math.sin(q2) + math.cos(q1)*math.cos(q2)*math.sin(q0))
    J56 = math.cos(q4)*(math.cos(q1)*math.sin(q0)*math.sin(q2) - math.cos(q2)*math.sin(q0)*math.sin(q1)) - math.sin(q4)*(math.cos(q0)*math.cos(q3) - math.sin(q3)*(math.sin(q0)*math.sin(q1)*math.sin(q2) + math.cos(q1)*math.cos(q2)*math.sin(q0)))
    ##### Row 6 #####
    J61 = 1
    J62 = 0
    J63 = 0
    J64 = math.cos(q1 - q2)
    J65 = math.sin(q1 - q2)*math.cos(q3)
    J66 = math.cos(q1 - q2)*math.cos(q4) + math.sin(q1 - q2)*math.sin(q3)*math.sin(q4)

    J = np.array([
        [J11, J12, J13, J14, J15, J16],
        [J21, J22, J23, J24, J25, J26],
        [J31, J32, J33, J34, J35, J36],
        [J41, J42, J43, J44, J45, J46],
        [J51, J52, J53, J54, J55, J56],
        [J61, J62, J63, J64, J65, J66]
    ])

    dx = J @ dq
    dp = dx[0:3]
    w  = dx[3:6]

    return dp, w
