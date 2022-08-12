#! /usr/bin/env python3


import rospy 
import numpy as np
import roboticstoolbox as rtb
import math

from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray
from ur_msgs.msg import IOStates


class controller:
    def __init__(self): # create instance variables I need
        self.robot = rtb.models.DH.UR10e()
        self.ts = 1/400 # sampling time
        self.M = [100, 100, 100, 100, 100, 100] # xyx, rpy
        self.D = [100, 100, 100, 100, 100, 100] # xyx, rpy

        # Task space variables initialisation
        self.v_ee_actual_current = np.matrix([0,0,0,0,0,0]) # np.ndarray type: [[0 0 0 0 0 0]]
        self.v_ee_actual_previous = np.matrix([0,0,0,0,0,0]) # np.ndarray type
        self.wrench = [0,0,0,0,0,0]
        self.v_ee_des = [0,0,0,0,0,0]

        # Joint space variables initialisation
        self.J0 = np.zeros((6,6)) # np.ndarray type
        self.pub_joint_vel = [0,0,0,0,0,0] # joint velocity command

        rospy.init_node('ur_admittance_control')

        rospy.Subscriber('/joint_states', JointState, self.callback1)

        rospy.Subscriber('/ft_sensor_topic', WrenchStamped, self.callback2)

        rospy.Subscriber('/ur_hardware_interface/io_states', IOStates, self.callback3)


        self.pub_joint_command = rospy.Publisher('/velocity_controller/command',Float64MultiArray, queue_size=1000)

        ## safety limits for joint velocity
        self.joint_vel_limit = (15/180)*math.pi 



    # Each method will obtain the value for an instance variable.
    def callback1(self, joint_data): #use this to return current ee velocity
        q = joint_data.position # 1x6 List []
        qd = joint_data.velocity

        self.J0 = self.robot.jacob0(q) # np.ndarray type

        qd_transpose = np.tranpose(np.matrix(qd))

        self.v_ee_actual_current = self.J0 * qd_transpose # 6x1 np.mdarray type, the estimation of current ee velocity


    def callback2(self, force_data):
        measured_wrench = force_data.wrench # 1x6 List
        for i in range(6):
            self.wrench[i] = 0.1 * measured_wrench[i] # scale down the measured force 


    def callback3(self, io_data): # deadman switch state (currently connected to the )
        io_state = io_data
        io_state.digital_in_states[0].state
   

    def vel_command(self): # calculates the desired ee velocity, return List type data 
        v_ee_actual_previous_list = self.v_ee_actual_previous.tolist() # convert to List type so that can access the value individually

        for i in range(6):
            self.v_ee_des[i] = (self.wrench[i] * self.ts + v_ee_actual_previous_list[0][i] * self.M[i]) / (self.M[i] + self.D[i]*self.ts)

        self.v_ee_actual_previous = self.v_ee_actual_current # update the v_ee_actual_previous so that it can be used at next iteration 
        
    
    ################ Converting task space velocity to joint space velocity ################
    def des_joint_vel(self):
        J_inverse = np.linalg.inv(self.J0) # calculate the inverse jacobian

        des_joint_vel = J_inverse * np.transpose(np.matrix(self.v_ee_des)) # 6x6 * 6x1 = 6x1

        joint_vel = np.transpose(des_joint_vel).tolist() # [[q1_dot],[q2_dot],[q3_dot],[q4_dot],[q5_dot],[q6_dot]]

        for i in range(6):
            self.pub_joint_vel[i] = joint_vel[i][0]

            abs_joint_vel =  [abs(ele) for ele in self.pub_joint_vel] # absolute value of joint velocity commands
            biggest_joint_vel = max(abs_joint_vel) # returns the biggest absolute joint velocity

            if biggest_joint_vel > self.joint_vel_limit: # if the maximum absolute joint velocity indeed exceeds the limit speed, scale down the all the joint velocities by the same ratio 
                rate = biggest_joint_vel / self.joint_vel_limit
                self.pub_joint_vel[i] = self.pub_joint_vel[i] / rate
            else: 
                pass 


    def myhook(self):
        msg = Float64MultiArray()
        msg.data = [0, 0, 0, 0, 0, 0]
        self.pub_joint_command.publish(msg)


    def send_joint_vel_command(self):
        msg = Float64MultiArray()
        for i in range(6):
            msg.data[i] = self.pub_joint_vel[i]

        rospy.on_shutdown(self.myhook)

        rate = rospy.Rate(100) # publish rate of joint velocity command

        while not rospy.is_shutdown():
            self.pub_joint_command.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    adm_control = controller() # ros node initialized

    adm_control.send_joint_vel_command() # do i need to call this method in order to send the vel command? 

    # where should rospy.spin() fit in for subscribers? 
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")



    

