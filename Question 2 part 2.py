from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt     # libraries

class Car:

    def __init__(self,
                 length=2.3,             # Length = 2.3m
                 velocity=5.0,           # Velocity =5m/s
                 x_position=0,           # x(0) = 0
                 y_postion=0.3,          # y(0) = 30cm
                 pose=np.deg2rad(5)):    # 5 degrees theta initial

        self.__length = length           # self represents the current instance of the class
        self.__velocity = velocity
        self.__x_postion = x_position
        self.__y_postion = y_postion
        self.__pose = pose

    def move(self, steering_angle, offset, dt):
        # This method computes the position and orientation (pose)
        # of the car after time `dt` starting from its current
        # position and orientation

        # step 1: define the systems dynamics
        def system_dynamics(t, z):
            # This function defines the systems dynamics
            # given in equation 2.1a-c
            #           v*cos(theta)
            # g(t, z) = v*sin(theta)
            #           v*tan(theta + w)/L
            theta = z[2]                            # this shows that theta is the third portion of z (starting from 0)
            return [self.__velocity * np.cos(theta),
                    self.__velocity * np.sin(theta),
                    self.__velocity * np.tan(steering_angle+ offset) / self.__length]

        # step 2: define the initial conditions z(0) = [x(0), y(0), theta(0)]
        z_initial = [self.__x_postion,
                     self.__y_postion,
                     self.__pose]

        # Step 3: Call solve_ivp

        solution= solve_ivp(system_dynamics,
                             [0, dt],
                              z_initial)
        self.__x_postion = solution.y[0][-1]       # this defines the x position at [0,-1]
        self.__y_postion = solution.y[1][-1]       # this defines the y position at [1,-1]
        self.__pose = solution.y[2][-1]            # this defines the theta position at [2,-1]

    def x(self):
        return self.__x_postion     # defining the x position

    def y(self):
        return self.__y_postion     # defining the y position

    def theta(self):
        return self.__pose          # defining the theta position

class PDController:


    def __init__(self, kp, kd, ts):
        self.__kp = kp       # dicrete-time P parameter
        self.__kd = kd / ts  # discrete-time kd
        self.__ts = ts       # sampling time
        self.__error_previous = None
        self.__sum_errors = 0.0

    def control(self, y, set_point=0.):

        error = set_point - y   # Compute control error

        steering_action=self.__kp*error       # P controller

        if self.__error_previous is not None:

            # D component
            steering_action += self.__kd*(error - self.__error_previous)

        self.__error_previous = error  # save it for later

        return steering_action

MY_CAR = Car(y_postion=0.3, velocity=5)     # defines the car class as MY_CAR starting postion 30cm and velocity 5m/s

t_sampling =1/40        # sampling rate 40Hz

pd =PDController(kp=0.1, kd=0.1, ts=t_sampling)        # defines the PDController class as pd, with kp and kd as
                                                       # the discrete-time parameters and ts at t_sampling

n_sim_points =2000                     # this is the range 40Hz, * 50s Therefore 2000
y_cache = np.array([MY_CAR.y()])       # defines y_cache as an array of the y values of MY_CAR
x_cache = np.array([MY_CAR.x()])       # defines z_cache as an array of the x values of MY_CAR
for t in range(n_sim_points):

    steering_angle = pd.control(MY_CAR.y())                 # defines steering angle controlling the y value of the car
    MY_CAR.move(steering_angle, np.deg2rad(1), t_sampling)  # moves the car with steering angle (defined previously)
                                                            # offset of 1 degree, and t_sampling

    y_cache = np.vstack((y_cache, [MY_CAR.y()]))            # stack array of y for car
    x_cache = np.vstack((x_cache, [MY_CAR.x()]))            # stack array of x for car

t_span = t_sampling*np.arange(n_sim_points+1)
plt.plot(x_cache, y_cache)      # plots trajectory of (x,y)
plt.grid()                      # adds grid to graph
plt.xlabel('x(m)')              # gives x a label
plt.ylabel('y(m)')              # gives y a label
plt.show()                      # shows the graph
