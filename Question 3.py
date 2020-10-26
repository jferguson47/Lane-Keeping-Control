from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt         # libraries

class Car:

    def __init__(self,
                 length=10,               # Length will be changed for different values
                 velocity=100,             # Velocity will be changed for different values
                 x_position=0,             # x(0) = 0
                 y_postion=0,              # y(0) = 0
                 pose= 10):              # theta initial will be changed different

        self.__length = length             # self represents the current instance of the class
        self.__velocity = velocity
        self.__x_postion = x_position
        self.__y_postion = y_postion
        self.__pose = pose

    def move(self, steering_angle, dt):
        # This method computes the position and orientation (pose)
        # of the car after time `dt` starting from its current
        # position and orientation


        # step 1: define the systems dynamics
        def system_dynamics(t, z):
            # This function defines the systems dynamics
            # given in equation 2.1a-c
            #           v*cos(theta)
            # g(t, z) = v*sin(theta)
            #           v*tan(theta)/L
            theta = z[2]                             # this shows that theta is the third portion of z (starting from 0)
            return [self.__velocity * np.cos(theta),
                    self.__velocity * np.sin(theta),
                    self.__velocity * np.tan(steering_angle) / self.__length]

        # step 2: define the initial conditions z(0) = [x(0), y(0), theta(0)]

        z_initial = [self.__x_postion,
                     self.__y_postion,
                     self.__pose]

        t_final = 100         # Final time which was given in question
        num_points = 1000   # Resolution

        # Step 3: Call solve_ivp
        solution= solve_ivp(system_dynamics,
                             [0, dt],
                              z_initial,
                            t_eval=np.linspace(0, t_final, num_points)) # creates a even sequence of numbers starting
                                                                        # at 0 and finishing at t_final (2), with a
                                                                        # resultion of num_points (100)

        self.__x_postion = solution.y[0][-1]    # this defines the x position at [0,-1]
        self.__y_postion = solution.y[1][-1]    # this defines the y position at [1,-1]
        self.__pose = solution.y[2][-1]         # this defines the theta position at [2,-1]


        plt.xlabel('Time(s)')  # gives x a label
        plt.grid()  # adds grid to graph
        plt.plot(solution.t, solution.y.T)  # plots solution against time
        plt.legend( ['x', 'y', 'theta'], loc="upper center", bbox_to_anchor=(0.5, 1.1), ncol=3)
        plt.show()  # shows the graph

        print(solution.y.shape)

    def x(self):                    # defining the x position
        return self.__x_postion

    def y(self):                    # defining the y position
        return self.__y_postion

    def theta(self):                # defining the theta position
        return self.__pose


t_final = 100                             # Final time which was given in question
MY_CAR = Car()                          # gives the car class a name MY_CAR
MY_CAR.move(np.deg2rad(2), t_final)     # move the car at a constant steering angle 2 degrees (2pi/180) for 2 seconds
