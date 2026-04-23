import numpy as np
import core as cr

def FeedbackControl(X, Xd, Xd_next, actual_se_config, Kp, Ki, dt, int_total_error):
    l = 0.235
    w = 0.15
    r = 0.0475
    F6 = np.array([[0, 0, 0, 0], [0, 0, 0, 0],
                   [(r/4)*(-1/(l+w)), (r/4)*(1/(l+w)), (r/4)*(1/(l+w)), (r/4)*(-1/(l+w))],
                   [(r/4), (r/4), (r/4), (r/4)],
                   [(-r/4), (r/4), (-r/4), (r/4)], [0, 0, 0, 0]])
                   
    Tb0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]])
    M0e = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])    
    Blist = np.array([[0, 0, 1, 0, 0.033, 0], [0, -1, 0, -0.5076, 0, 0], [0, -1, 0, -0.3526, 0, 0], [0, -1, 0, -0.2176, 0, 0], [0, 0, 1, 0, 0, 0]]).T
    thetalist = np.array([actual_se_config[3:8]]).T
    
    T0e = cr.FKinBody(M0e, Blist, thetalist) 
    Jbase = np.dot(cr.Adjoint(np.dot(np.linalg.inv(T0e), np.linalg.inv(Tb0))), F6)
    Jarm = cr.JacobianBody(Blist, thetalist) 
    Je = np.hstack((Jbase, Jarm))
    Je_pseudo = np.linalg.pinv(Je)
      
    Vd = cr.se3ToVec(cr.MatrixLog6(np.dot(np.linalg.inv(Xd), Xd_next)) * (1/dt))
    Xerr = cr.se3ToVec(cr.MatrixLog6(np.dot(np.linalg.inv(X), Xd)))
    adjoint = cr.Adjoint(np.dot(np.linalg.inv(X), Xd))
    total_error = np.add(Xerr, int_total_error)
    
    V = np.dot(adjoint, Vd) + np.dot(Kp, Xerr) + np.dot(Ki, total_error)
    u_theta = np.dot(Je_pseudo, V)
    
    return u_theta, Xerr, total_error