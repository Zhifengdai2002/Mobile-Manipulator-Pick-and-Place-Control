import math as m
import numpy as np

pi = m.pi 
dt = 0.01
k = 1
duration = [1.5, 1, 0.5, 0.5, 3, 0.5, 0.5, 1] 

th = 3 * pi / 4 
Tce_grasp = np.array([[m.cos(th), 0, m.sin(th), 0.005], [0, 1, 0, 0], [-m.sin(th), 0, m.cos(th), 0], [0, 0, 0, 1]])
Tce_standoff = np.array([[m.cos(th), 0, m.sin(th), 0], [0, 1, 0, 0], [-m.sin(th), 0, m.cos(th), 0.3], [0, 0, 0, 1]])

def best():
    initial_conf = np.array([-pi/4.0, -0.6, 0.3, 0.0, 0.0, -pi/4, -0.3, 0.0, 0.0, 0, 0.0, 0.0, 0])
    Tsc_in = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]])
    Tsc_goal = np.array([[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]])
    Kp = np.dot(np.diag((1, 1, 1, 1, 1, 1)), 5)
    Ki = np.dot(np.diag((1, 1, 1, 1, 1, 1)), 0)
    return initial_conf, Tsc_in, Tsc_goal, Kp, Ki

def overshoot():
    initial_conf = np.array([-pi/4.0, -0.6, 0.3, 0.0, 0.0, -pi/4.0, 0.3, 0.0, 0.0, 0, 0.0, 0.0, 0])
    Tsc_in = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]])
    Tsc_goal = np.array([[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]])
    Kp = np.dot(np.diag((1, 1, 1, 1, 1, 1)), 5)
    Ki = np.dot(np.diag((1, 1, 1, 1, 1, 1)), 0.2)
    return initial_conf, Tsc_in, Tsc_goal, Kp, Ki

def new_task():
    initial_conf = np.array([-pi/4.0, -0.6, 0.3, 0.0, 0.0, -pi/4.0, -0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0])
    Tsc_in = np.array([[m.cos(pi/4), m.sin(pi/4), 0.0, 0.5], [m.sin(-pi/4), m.cos(pi/4), 0.0, 0.5], [0.0, 0.0, 1.0, 0.025], [0.0, 0.0, 0.0, 1.0]])
    Tsc_goal = np.array([[m.cos(pi), m.sin(pi), 0.0, 0.8], [m.sin(pi), m.cos(pi), 0.0, -0.6], [0.0, 0.0, 1.0, 0.025], [0.0, 0.0, 0.0, 1.0]])
    Kp = np.dot(np.diag((1, 1, 1, 1, 1, 1)), 5)
    Ki = np.dot(np.diag((1, 1, 1, 1, 1, 1)), 0)
    return initial_conf, Tsc_in, Tsc_goal, Kp, Ki