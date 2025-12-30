import numpy as np
import math

def fk_gen3(q, p):
    """
    Forward kinematics for Gen3 Lite

    q : ndarray (6,)
    p : dict with keys a2, d2, d3, d4, d5, d6

    Returns
    -------
    pos : ndarray (3,)
    R   : ndarray (3,3)
    """

    q0, q1, q2, q3, q4, q5 = q
    a2 = p['a2']
    d2 = p['d2']
    d3 = p['d3']
    d4 = p['d4']
    d5 = p['d5']
    d6 = p['d6']

    # ---- POSITION ----
    px=d2*math.sin(q0) - d3*math.sin(q0) - a2*math.cos(q0)*math.sin(q1) - d5*math.sin(q0)*math.sin(q3) - d4*math.sin(q1 - q2)*math.cos(q0) + d6*math.cos(q3)*math.sin(q0)*math.sin(q4) - d6*math.sin(q1 - q2)*math.cos(q0)*math.cos(q4) + d5*math.cos(q0)*math.cos(q1)*math.cos(q2)*math.cos(q3) + d5*math.cos(q0)*math.cos(q3)*math.sin(q1)*math.sin(q2) + d6*math.cos(q0)*math.cos(q1)*math.cos(q2)*math.sin(q3)*math.sin(q4) + d6*math.cos(q0)*math.sin(q1)*math.sin(q2)*math.sin(q3)*math.sin(q4)
    py=d4*(math.cos(q1)*math.sin(q0)*math.sin(q2) - math.cos(q2)*math.sin(q0)*math.sin(q1)) - d2*math.cos(q0) + d3*math.cos(q0) + d5*(math.cos(q0)*math.sin(q3) + math.cos(q3)*(math.sin(q0)*math.sin(q1)*math.sin(q2) + math.cos(q1)*math.cos(q2)*math.sin(q0))) - d6*(math.sin(q4)*(math.cos(q0)*math.cos(q3) - math.sin(q3)*(math.sin(q0)*math.sin(q1)*math.sin(q2) + math.cos(q1)*math.cos(q2)*math.sin(q0))) - math.cos(q4)*(math.cos(q1)*math.sin(q0)*math.sin(q2) - math.cos(q2)*math.sin(q0)*math.sin(q1))) - a2*math.sin(q0)*math.sin(q1)
    pz=a2*math.cos(q1) + d4*math.cos(q1 - q2) - (d6*math.sin(q1 - q2)*math.cos(q3 + q4))/2 + d6*math.cos(q1 - q2)*math.cos(q4) + d5*math.sin(q1 - q2)*math.cos(q3) + (d6*math.cos(q3 - q4)*math.sin(q1 - q2))/2
        
    pos = np.array([px, py, pz])

    # ---- ROTATION MATRIX----
    R11=-math.cos(q5)*(math.sin(q0)*math.sin(q3) - math.cos(q3)*(math.cos(q0)*math.cos(q1)*math.cos(q2) + math.cos(q0)*math.sin(q1)*math.sin(q2))) - math.sin(q5)*(math.cos(q4)*(math.cos(q3)*math.sin(q0) + math.sin(q3)*(math.cos(q0)*math.cos(q1)*math.cos(q2) + math.cos(q0)*math.sin(q1)*math.sin(q2))) - math.sin(q4)*(math.cos(q0)*math.cos(q1)*math.sin(q2) - math.cos(q0)*math.cos(q2)*math.sin(q1)))
    R12=math.sin(q5)*(math.sin(q0)*math.sin(q3) - math.cos(q3)*(math.cos(q0)*math.cos(q1)*math.cos(q2) + math.cos(q0)*math.sin(q1)*math.sin(q2))) - math.cos(q5)*(math.cos(q4)*(math.cos(q3)*math.sin(q0) + math.sin(q3)*(math.cos(q0)*math.cos(q1)*math.cos(q2) + math.cos(q0)*math.sin(q1)*math.sin(q2))) - math.sin(q4)*(math.cos(q0)*math.cos(q1)*math.sin(q2) - math.cos(q0)*math.cos(q2)*math.sin(q1)))
    R13=math.sin(q4)*(math.cos(q3)*math.sin(q0) + math.sin(q3)*(math.cos(q0)*math.cos(q1)*math.cos(q2) + math.cos(q0)*math.sin(q1)*math.sin(q2))) + math.cos(q4)*(math.cos(q0)*math.cos(q1)*math.sin(q2) - math.cos(q0)*math.cos(q2)*math.sin(q1))
    R21=math.cos(q5)*(math.cos(q0)*math.sin(q3) + math.cos(q3)*(math.sin(q0)*math.sin(q1)*math.sin(q2) + math.cos(q1)*math.cos(q2)*math.sin(q0))) + math.sin(q5)*(math.cos(q4)*(math.cos(q0)*math.cos(q3) - math.sin(q3)*(math.sin(q0)*math.sin(q1)*math.sin(q2) + math.cos(q1)*math.cos(q2)*math.sin(q0))) + math.sin(q4)*(math.cos(q1)*math.sin(q0)*math.sin(q2) - math.cos(q2)*math.sin(q0)*math.sin(q1)))
    R22=math.cos(q5)*(math.cos(q4)*(math.cos(q0)*math.cos(q3) - math.sin(q3)*(math.sin(q0)*math.sin(q1)*math.sin(q2) + math.cos(q1)*math.cos(q2)*math.sin(q0))) + math.sin(q4)*(math.cos(q1)*math.sin(q0)*math.sin(q2) - math.cos(q2)*math.sin(q0)*math.sin(q1))) - math.sin(q5)*(math.cos(q0)*math.sin(q3) + math.cos(q3)*(math.sin(q0)*math.sin(q1)*math.sin(q2) + math.cos(q1)*math.cos(q2)*math.sin(q0)))
    R23=math.cos(q4)*(math.cos(q1)*math.sin(q0)*math.sin(q2) - math.cos(q2)*math.sin(q0)*math.sin(q1)) - math.sin(q4)*(math.cos(q0)*math.cos(q3) - math.sin(q3)*(math.sin(q0)*math.sin(q1)*math.sin(q2) + math.cos(q1)*math.cos(q2)*math.sin(q0)))
    R31=math.sin(q5)*(math.cos(q1 - q2)*math.sin(q4) - math.sin(q1 - q2)*math.cos(q4)*math.sin(q3)) + math.sin(q1 - q2)*math.cos(q3)*math.cos(q5)
    R32=math.cos(q5)*(math.cos(q1 - q2)*math.sin(q4) - math.sin(q1 - q2)*math.cos(q4)*math.sin(q3)) - math.sin(q1 - q2)*math.cos(q3)*math.sin(q5)
    R33=math.cos(q1 - q2)*math.cos(q4) + math.sin(q1 - q2)*math.sin(q3)*math.sin(q4)
        
    R = np.array([
        [R11, R12, R13],
        [R21, R22, R23],
        [R31, R32, R33]
    ])

    return pos, R
