import numpy as np
import core as cr
from config import duration

def TrajectoryGenerator(Tse_in, Tsc_in, Tsc_goal, Tce_grasp, Tce_standoff, k):
    trajectories = []
    
    # ---------SEGMENT 1---------
    N1 = duration[0] * k / 0.01  
    Tse_standoff = np.dot(Tsc_in, Tce_standoff)
    Segment_1 = cr.CartesianTrajectory(Tse_in, Tse_standoff, duration[0], N1, 5)
    trajectories.extend(Segment_1)
     
    # ---------SEGMENT 2---------
    N2 = duration[1] * k / 0.01
    Tse_grasp = np.dot(Tsc_in, Tce_grasp)
    Segment_2 = cr.CartesianTrajectory(Tse_standoff, Tse_grasp, duration[1], N2, 5)
    trajectories.extend(Segment_2)
     
    # ---------SEGMENT 3---------
    N3 = duration[2] * k / 0.01  
    Segment_3 = cr.CartesianTrajectory(Tse_grasp, Tse_grasp, duration[2], N3, 5)
    trajectories.extend(Segment_3)
     
    # ---------SEGMENT 4---------
    N4 = duration[3] * k / 0.01
    Segment_4 = cr.CartesianTrajectory(Tse_grasp, Tse_standoff, duration[3], N4, 5)
    trajectories.extend(Segment_4)
     
    # ---------SEGMENT 5---------
    N5 = duration[4] * k / 0.01  
    Tse_standoff_2 = np.dot(Tsc_goal, Tce_standoff)
    Segment_5 = cr.CartesianTrajectory(Tse_standoff, Tse_standoff_2, duration[4], N5, 5)
    trajectories.extend(Segment_5)
     
    # ---------SEGMENT 6---------
    N6 = duration[5] * k / 0.01
    Tse_grasp_2 = np.dot(Tsc_goal, Tce_grasp)
    Segment_6 = cr.CartesianTrajectory(Tse_standoff_2, Tse_grasp_2, duration[5], N6, 5)
    trajectories.extend(Segment_6)
     
    # ---------SEGMENT 7---------
    N7 = duration[6] * k / 0.01  
    Segment_7 = cr.CartesianTrajectory(Tse_grasp_2, Tse_grasp_2, duration[6], N7, 5)
    trajectories.extend(Segment_7)
     
    # ---------SEGMENT 8---------
    N8 = duration[7] * k / 0.01  
    Segment_8 = cr.CartesianTrajectory(Tse_grasp_2, Tse_standoff_2, duration[7], N8, 5)
    trajectories.extend(Segment_8)
     
    return trajectories, Segment_1, Segment_2, Segment_3, Segment_4, Segment_5, Segment_6, Segment_7, Segment_8