import numpy as np
import math as m
import core as cr

def NextState(current_conf, speed, dt, max_speed=10000):
    current_conf = np.array(current_conf)
    speed = np.array(speed)
    
    l = 0.235
    w = 0.15
    r = 0.0475    
    
    old_arm_angles = current_conf[3:8]
    old_wheel_angles = current_conf[8:12]
    wheel_speed = speed[0:4]
    joint_speed = speed[4:9]
    
    arm_angles = old_arm_angles + joint_speed * dt 
    wheel_angles = old_wheel_angles + wheel_speed * dt
    
    phi = current_conf[0]   
    deltaTheta = wheel_speed * dt
    F = np.array([[-r/(4*(l+w)), r/(4*(l+w)), r/(4*(l+w)), -r/(4*(l+w))],
                  [r/4, r/4, r/4, r/4],
                  [-r/4, r/4, -r/4, r/4]])
    Vb = np.dot(F, deltaTheta.T)
    Vb6 = np.array([0, 0, Vb[0], Vb[1], Vb[2], 0]).T
    q_old = np.array([current_conf[0], current_conf[1], current_conf[2]]).T
    
    if Vb6[2] == 0:
        dqb = np.array([0, Vb6[3], Vb6[4]]).T
    else:
        dqb  = np.array([Vb6[2], 
                         (Vb6[3]*m.sin(Vb6[2]) + Vb6[4]*(m.cos(Vb6[2])-1))/Vb6[2], 
                         (Vb6[4]*m.sin(Vb6[2]) + Vb6[3]*(1-m.cos(Vb6[2])))/Vb6[2]]).T
                         
    delta_q = np.array([[1, 0, 0], [0, m.cos(phi), -m.sin(phi)], [0, m.sin(phi), m.cos(phi)]])
                        
    q = q_old + np.dot(delta_q, dqb)
    new_state = np.array([q[0], q[1], q[2], 
                          arm_angles[0], arm_angles[1], arm_angles[2], arm_angles[3], arm_angles[4], 
                          wheel_angles[0], wheel_angles[1], wheel_angles[2], wheel_angles[3]])    
    return new_state

def compute_Tse(conf):
    B = np.array([[0, 0, 1, 0, 0.033, 0], [0, -1, 0, -0.5076, 0, 0], [0, -1, 0, -0.3526, 0, 0], [0, -1, 0, -0.2176, 0, 0], [0, 0, 1, 0, 0, 0]]).T
    M0e = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])    

    v1 = np.matrix(cr.MatrixExp6(cr.VecTose3(np.dot(conf[3], B[:,0]))))
    v2 = np.matrix(cr.MatrixExp6(cr.VecTose3(np.dot(conf[4], B[:,1]))))
    v3 = np.matrix(cr.MatrixExp6(cr.VecTose3(np.dot(conf[5], B[:,2]))))
    v4 = np.matrix(cr.MatrixExp6(cr.VecTose3(np.dot(conf[6], B[:,3]))))
    v5 = np.matrix(cr.MatrixExp6(cr.VecTose3(np.dot(conf[7], B[:,4]))))

    Tb0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]])
    T0e = (M0e * v1 * v2 * v3 * v4 * v5).tolist()
    Tsb = np.matrix([[m.cos(conf[0]), -1 * m.sin(conf[0]), 0, conf[1]],
                     [m.sin(conf[0]), m.cos(conf[0]), 0, conf[2]],
                     [0, 0, 1, 0.0963],
                     [0, 0, 0, 1]])
    
    return (Tsb * Tb0 * T0e).tolist()