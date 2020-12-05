import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from multiprocessing import Process, Queue
import time
import queue
import pandas as pd
import json


class Robot_Element:
    def update_dh_matrix(self, theta_i, sigma_i, lamda_i, alpha_i):
        dh_array = np.zeros((4, 4))

        dh_array[0][0] =  np.cos(theta_i)
        dh_array[0][1] = -np.sin(theta_i)*np.cos(alpha_i)
        dh_array[0][2] =  np.sin(theta_i)*np.sin(alpha_i)
        dh_array[0][3] =  lamda_i*np.cos(alpha_i)

        dh_array[1][0] =  np.sin(theta_i)
        dh_array[1][1] =  np.cos(theta_i)*np.cos(alpha_i)
        dh_array[1][2] = -np.cos(theta_i)*np.sin(alpha_i)
        dh_array[1][3] =  lamda_i*np.sin(alpha_i)

        dh_array[2][1] = np.sin(alpha_i)
        dh_array[2][2] = np.cos(alpha_i)
        dh_array[2][3] = sigma_i

        dh_array[3][3] = 1

        dh_matrix = np.matrix(dh_array)
        return dh_matrix


    def __init__(self, theta_i, sigma_i, lamda_i, alpha_i, dh_matrix_old=None):
        self.update(theta_i, sigma_i, lamda_i, alpha_i, dh_matrix_old)


    def update(self, theta_i, sigma_i, lamda_i, alpha_i, dh_matrix_old=None):
        self.dh_matrix = self.update_dh_matrix(theta_i, sigma_i, lamda_i, alpha_i)
        
        if dh_matrix_old is None:
            self.orientation_matrix = self.dh_matrix
        else:
            self.orientation_matrix = dh_matrix_old * self.dh_matrix

        self.point = [self.orientation_matrix[0, 3],  # x
                      self.orientation_matrix[1, 3],  # y
                      self.orientation_matrix[2, 3]]  # z


    def get_orientation_matrix(self):
        return self.orientation_matrix


    def get_point(self):
        return self.point


class RRP_Robot:
    def __init__(self, oop_robot_data):
        # constant attributes
        elements_lengths_data = oop_robot_data["elements_lengths"]
        self.l1 = elements_lengths_data["l1"]
        self.l2 = elements_lengths_data["l2"]

        # limitations
        limitations_data = oop_robot_data["limitations"]
        self.theta1_min = limitations_data["theta1_min"]
        self.theta1_max = limitations_data["theta1_max"]
        self.theta2_min = limitations_data["theta2_min"]
        self.theta2_max = limitations_data["theta2_max"]
        self.sigma_min  = limitations_data["sigma_min"]
        self.sigma_max  = limitations_data["sigma_max"]

        # working area
        working_area_data = oop_robot_data["working_area"]
        self.center = np.array(working_area_data["center"])
        self.radius = working_area_data["radius"]

        # variable attributes
        initial_position_data = oop_robot_data["initial_position"]
        self.theta1 = initial_position_data["theta1"]
        self.theta2 = initial_position_data["theta2"]
        self.sigma  = initial_position_data["sigma"]

        # Denavit - Hartenberg notation array for RRP robot
        self.dh_notation_array = self.__create_dh_notation_array(self.theta1, self.theta2, self.sigma)

        # Create segments of RRP robot arm
        self.elements = self.__create_robot_elements(self.dh_notation_array)


    def __create_dh_notation_array(self, theta1, theta2, sigma):
        # Denavit - Hartenberg notation array for RRP robot
        #                              theta     sigma          lamda  alpha
        dh_notation_array = np.array([[theta1,   self.l1,         0,  np.pi/2],
                                      [theta2,   0,               0, -np.pi/2],
                                      [-np.pi/2, 0,               0,  0      ],
                                      [0,        0,               0, -np.pi/2],
                                      [0,        self.l2 + sigma, 0,  0]])
        return dh_notation_array


    def __create_robot_elements(self, dh_notation_array):
        elements = []
        for i in range(len(dh_notation_array)):
            theta_i, sigma_i, lambda_i, alpha_i = dh_notation_array[i]
            if i == 0:
                orientation_matrix = None
            else:
                orientation_matrix = elements[-1].get_orientation_matrix()
            element = Robot_Element(theta_i, sigma_i, lambda_i, alpha_i, orientation_matrix)
            elements.append(element)
        return elements


    def update(self, new_theta1, new_theta2, new_sigma):
        # Denavit - Hartenberg notation array for RRP robot
        self.dh_notation_array = self.__create_dh_notation_array(new_theta1, new_theta2, new_sigma)

        # Create segments of RRP robot arm
        self.elements = self.__create_robot_elements(self.dh_notation_array)


    def get_joint_points(self):
        points_x = [0] 
        points_y = [0]
        points_z = [0]
        for element in self.elements:
            point = element.get_point()
            points_x.append(point[0])
            points_y.append(point[1])
            points_z.append(point[2])
        points_x_arr = np.array(points_x)
        points_y_arr = np.array(points_y)
        points_z_arr = np.array(points_z)
        return points_x_arr, points_y_arr, points_z_arr


    def __check_variables_borders(self):
        if self.theta1 < self.theta1_min:
            self.theta1 = self.theta1_min
        elif self.theta1 > self.theta1_max:
            self.theta1 = self.theta1_max

        if self.theta2 < self.theta2_min:
            self.theta2 = self.theta2_min
        elif self.theta2 > self.theta2_max:
            self.theta2 = self.theta2_max

        if self.sigma < self.sigma_min:
            self.sigma = self.sigma_min
        elif self.sigma > self.sigma_max:
            self.sigma = self.sigma_max


    def forward_kinematic(self, new_theta1, new_theta2, new_sigma):
        self.theta1 = new_theta1
        self.theta2 = new_theta2
        self.sigma  = new_sigma
        self.__check_variables_borders()
        self.update(self.theta1, self.theta2, self.sigma)


    def inverse_kinematic(self, new_x, new_y, new_z):
        self.theta1, self.theta2, self.sigma = self.get_inverse_kinematic_variables(new_x, new_y, new_z)
        self.update(self.theta1, self.theta2, self.sigma)


    def get_inverse_kinematic_variables(self, new_x, new_y, new_z):
        theta1 = np.arctan2(new_y, new_x)
        sigma = np.sqrt(new_x**2 + new_y**2 + (new_z-self.l1)**2) - self.l2
        theta2 = np.arcsin((new_z - self.l1)/(self.l2 + sigma))
        return theta1, theta2, sigma


    def get_current_variables(self):
        return self.theta1, self.theta2, self.sigma


    def get_grasper_coordinates(self):
        x_arr, y_arr, z_arr = self.get_joint_points()
        return x_arr[-1], y_arr[-1], z_arr[-1]


    def check_if_outside_working_area(self):
        grasper_point = np.array(self.get_grasper_coordinates())
        distance = np.sqrt(np.sum((grasper_point-self.center)**2, axis=0))
        if distance > self.radius:
            return True
        else:
            return False


    def __plot_working_area(self, center=[1.5,0,1], radius=1, plane=0):
            points_per_ring = 20
            points_x = []
            points_y = []
            points_z = []
            for j in range(points_per_ring):
                if plane == 0:
                    points_x.append(radius*np.cos(2*np.pi*j/points_per_ring)+center[0])
                    points_y.append(radius*np.sin(2*np.pi*j/points_per_ring)+center[1])
                    points_z.append(center[2])
                elif plane == 1:
                    points_y.append(radius*np.cos(2*np.pi*j/points_per_ring)+center[1])
                    points_z.append(radius*np.sin(2*np.pi*j/points_per_ring)+center[2])
                    points_x.append(center[0])
                else:
                    points_z.append(radius*np.cos(2*np.pi*j/points_per_ring)+center[2])
                    points_x.append(radius*np.sin(2*np.pi*j/points_per_ring)+center[0])
                    points_y.append(center[1])
            points_x.append(points_x[0])
            points_y.append(points_y[0])
            points_z.append(points_z[0])
            points_x_arr = np.array(points_x)
            points_y_arr = np.array(points_y)
            points_z_arr = np.array(points_z)
            return points_x_arr, points_y_arr, points_z_arr


    def plot_working_area(self, plane):
        return self.__plot_working_area(self.center, self.radius, plane)


class Visual_Controller:
    def __init__(self, rrp_robot, intermediate_thetas1, intermediate_thetas2, intermediate_sigmas):
        self.rrp_robot = rrp_robot
        self.update_ml_to_vis(intermediate_thetas1, intermediate_thetas2, intermediate_sigmas)


    def update_ml_to_vis(self, intermediate_thetas1, intermediate_thetas2, intermediate_sigmas):
        self.intermediate_thetas1 = intermediate_thetas1
        self.intermediate_thetas2 = intermediate_thetas2
        self.intermediate_sigmas  = intermediate_sigmas
        self.intermediate_steps   = len(intermediate_sigmas)
        self.block_visualization  = False
        self.current_step = 0


    def update_vis_to_ml(self):
        self.block_visualization = True


    def increment_step(self):
        self.current_step+=1


def visual(qsv, qvs):
    class Pack:
        def __init__(self, visual_controller):
            self.visual_controller = visual_controller

    visual_controller = qsv.get()
    pack = Pack(visual_controller)

    # Prepare figure
    fig = plt.figure(num=None, figsize=(8, 6))
    ax = fig.gca(projection='3d')

    # not working anymore?
    # Set equal grid
    # extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    # sz = extents[:,1] - extents[:,0]
    # centers = np.mean(extents, axis=1)
    # maxsize = max(abs(sz))
    # r = maxsize/2
    # for ctr, dim in zip(centers, 'xyz'):
    #     getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)

    # Line for robot
    line,  = ax.plot([], [], [], 'b-o')
    line2, = ax.plot([], [], [], 'r')
    line3, = ax.plot([], [], [], 'r')
    line4, = ax.plot([], [], [], 'r')

    # Set axes
    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    ax.set_zlim([ 0, 3])
    ax.set_xlabel('X[m]')
    ax.set_ylabel('Y[m]')
    ax.set_zlabel('Z[m]')

    def print_useful_data(pack):
        # coordinates of all joints
        # all variables
        # grasper is in working area
        print("O>xxx<[=]>xxx<[=]>xxx<[=]>xxx<[=]>xxx<O")
        print()
        print("Intermediate data from step: ", pack.visual_controller.current_step)
        print()
        print("Coordinates of joints:")
        x_arr, y_arr, z_arr = pack.visual_controller.rrp_robot.get_joint_points()
        df = pd.DataFrame({ "x[m]":x_arr,
                            "y[m]":y_arr,
                            "z[m]":z_arr})
        df.columns.name = "joint no."
        print(df)

        print()
        print("Variable values:")
        theta1, theta2, sigma = pack.visual_controller.rrp_robot.get_current_variables()
        df = pd.DataFrame({"theta1[deg]":[np.rad2deg(theta1)], "theta2[deg]":[np.rad2deg(theta2)], "sigma[m]":[sigma]})
        print(df.to_string(index=False))
        print()
        print("Grasper is in working area: ", not pack.visual_controller.rrp_robot.check_if_outside_working_area())
        print()
        print("O>xxx<[=]>xxx<[=]>xxx<[=]>xxx<[=]>xxx<O")
        

    def animate(i, pack):
        # Check if visual controller was changed in simulation and passed via queue to visual
        qsv_is_empty = False
        try:
            new_visual_controller = qsv.get(timeout=0)
        except queue.Empty:
            qsv_is_empty = True
        
        if qsv_is_empty == False:
            pack.visual_controller = new_visual_controller
            for i in range(5):
                print()

        if pack.visual_controller.block_visualization == False:
            # Unpack values
            intermediate_thetas1 = pack.visual_controller.intermediate_thetas1
            intermediate_thetas2 = pack.visual_controller.intermediate_thetas2
            intermediate_sigmas  = pack.visual_controller.intermediate_sigmas
            intermediate_steps   = pack.visual_controller.intermediate_steps
            current_step         = pack.visual_controller.current_step

            # Move robot
            intermediate_theta1 = intermediate_thetas1[min(current_step, intermediate_steps-1)]
            intermediate_theta2 = intermediate_thetas2[min(current_step, intermediate_steps-1)]
            intermediate_sigma  = intermediate_sigmas [min(current_step, intermediate_steps-1)]
            pack.visual_controller.rrp_robot.forward_kinematic(intermediate_theta1, 
                                                                intermediate_theta2, 
                                                                intermediate_sigma)

            # Plot robot
            x_arr, y_arr, z_arr = pack.visual_controller.rrp_robot.get_joint_points()
            line.set_data(x_arr, y_arr)
            line.set_3d_properties(z_arr)

            # Plot working area
            x_arr, y_arr, z_arr = pack.visual_controller.rrp_robot.plot_working_area(0)
            line2.set_data(x_arr, y_arr)
            line2.set_3d_properties(z_arr)
            x_arr, y_arr, z_arr = pack.visual_controller.rrp_robot.plot_working_area(1)
            line3.set_data(x_arr, y_arr)
            line3.set_3d_properties(z_arr)
            x_arr, y_arr, z_arr = pack.visual_controller.rrp_robot.plot_working_area(2)
            line4.set_data(x_arr, y_arr)
            line4.set_3d_properties(z_arr)

            # Print data
            print_useful_data(pack)

            # Check if current_step should be incremented, and if not, then enable main loop
            pack.visual_controller.increment_step()
            if pack.visual_controller.current_step >= pack.visual_controller.intermediate_steps:
                pack.visual_controller.update_vis_to_ml()
                qvs.put(pack.visual_controller)

        return line, line2

    # Run animation
    ani = animation.FuncAnimation(fig, animate, fargs=(pack,), interval=50, blit=False)
    plt.show()


def simulation(oop_robot_data):
    # Prepare objects for simulation and visualization
    rrp_robot = RRP_Robot(oop_robot_data)
    visual_controller = Visual_Controller(rrp_robot, [0], [0], [0])

    # Start plotting process
    qsv, qvs = Queue(), Queue()
    p = Process(target=visual, args=(qsv, qvs,))
    p.start()
    qsv.put(visual_controller)

    # Main loop of simulation    
    while True:
        visual_controller = qvs.get()

        # Get variables values:
        print("Provide values:")
        forward_or_inverse  = int(input())   # 1->forward, 0->inverse
        new_theta1_or_new_x = float(input())
        new_theta2_or_new_y = float(input())
        new_sigma_or_new_z  = float(input())
        intermediate_steps  = int(input())
        # forward_or_inverse  = 1   # 1->forward, 0->inverse
        # new_theta1_or_new_x = 90
        # new_theta2_or_new_y = 45
        # new_sigma_or_new_z  = 0.75
        # intermediate_steps  = 10

        # Get current variables values:
        old_theta1, old_theta2, old_sigma = visual_controller.rrp_robot.get_current_variables()

        # Prepare new variables values:
        if forward_or_inverse == 1:                         # Forward kinematic
            new_theta1 = np.deg2rad(new_theta1_or_new_x)
            new_theta2 = np.deg2rad(new_theta2_or_new_y)
            new_sigma  = new_sigma_or_new_z
        else:                                               # Inverse kinematic
            new_x, new_y, new_z = new_theta1_or_new_x, new_theta2_or_new_y,new_sigma_or_new_z
            new_theta1, new_theta2, new_sigma = visual_controller.rrp_robot.get_inverse_kinematic_variables(new_x, new_y, new_z)

        # Get intermediate variables values:
        intermediate_thetas1 = np.linspace(old_theta1, new_theta1, intermediate_steps)
        intermediate_thetas2 = np.linspace(old_theta2, new_theta2, intermediate_steps)
        intermediate_sigmas  = np.linspace(old_sigma,  new_sigma,  intermediate_steps)

        visual_controller.update_ml_to_vis(intermediate_thetas1, intermediate_thetas2, intermediate_sigmas)
        qsv.put(visual_controller)

    p.join()


if __name__ == "__main__":
    oop_robot_data = json.load(open("oop_robot_data.json", "r"))
    simulation(oop_robot_data)