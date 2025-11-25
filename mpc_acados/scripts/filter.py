#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, Int32

#############################################
# Low-pass filter (simple first-order)
#############################################
class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.last_output = 0.0

    def filter(self, input_val):
        output = self.alpha * input_val + (1 - self.alpha) * self.last_output
        self.last_output = output
        return output

#############################################
# Butterworth second-order low-pass filter
#############################################
class ButterworthLowPassFilter:
    def __init__(self, fs, fc):
        self.fs = fs
        self.fc = fc
        # Store the last two inputs and outputs (initially zeros)
        self.prevX = [0.0, 0.0]
        self.prevY = [0.0, 0.0]
        self.calculate_coefficients()

    def calculate_coefficients(self):
        ita = 1.0 / np.tan(np.pi * self.fc / self.fs)
        q = np.sqrt(2.0)
        a0_temp = 1.0 / (1.0 + q * ita + ita * ita)
        self.a0 = a0_temp
        self.a1 = 2 * a0_temp
        self.a2 = a0_temp
        self.b1 = 2.0 * a0_temp * (1 - ita * ita)
        self.b2 = a0_temp * (1 - q * ita + ita * ita)

    def filter(self, input_val):
        # Compute output using the difference equation:
        # y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] - b1*y[n-1] - b2*y[n-2]
        output = (self.a0 * input_val + 
                  self.a1 * self.prevX[0] + 
                  self.a2 * self.prevX[1] - 
                  self.b1 * self.prevY[0] - 
                  self.b2 * self.prevY[1])
        # Update history
        self.prevX[1] = self.prevX[0]
        self.prevX[0] = input_val
        self.prevY[1] = self.prevY[0]
        self.prevY[0] = output
        return output

#############################################
# Controller class (converted from C++ code)
#############################################
class MyController:
    def __init__(self):
        # Initialize control input (3-dimensional vector)
        self.u = np.zeros(3)

        # Real-time states and their derivatives
        self.x_real = 0.0
        self.x_real_last = 0.0
        self.y_real = 0.0
        self.y_real_last = 0.0
        self.z_real = 0.0
        self.z_real_last = 0.0

        self.x_real_derivative = 0.0
        self.y_real_derivative = 0.0
        self.z_real_derivative = 0.0

        self.x_filtered_deri = 0.0
        self.y_filtered_deri = 0.0
        self.z_filtered_deri = 0.0

        # Time record (in seconds)
        self.time_now = rospy.Time.now().to_sec()
        self.time_last = self.time_now
        self.time_pass = 0.0

        # State flags
        self.first_time_in_fun = True
        self.loss_target = True  # Matches initial state in C++ version
        self.loss_or_not_ = 1.0

        # Create low-pass filters (for image signal filtering)
        self.filter_for_imgx = LowPassFilter(0.95)
        self.filter_for_imgy = LowPassFilter(0.95)
        self.filter_for_imgz = LowPassFilter(0.95)

        # Create Butterworth low-pass filters (for derivative filtering), sampling rate 20Hz, cutoff 2.7Hz
        self.filter_4_for_derix = ButterworthLowPassFilter(20, 2.7)
        self.filter_4_for_deriy = ButterworthLowPassFilter(20, 2.7)
        self.filter_4_for_deriz = ButterworthLowPassFilter(20, 2.7)

        # PX4 state: initialize to 1
        self.px4_state = 1

        # ROS subscribers
        self.px4_state_sub = rospy.Subscriber("/px4_state_pub", Int32, self.state_callback, queue_size=1)
        self.sub = rospy.Subscriber("/point_with_fixed_delay", Float64MultiArray, self.callback, queue_size=1)

        # Additional subscribers can be added if needed (e.g., ground truth velocity, position)
        # Example:
        # self.ground_truth_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.ground_truth_callback, queue_size=10)
        # self.ground_truth_pose_sub = rospy.Subscriber("/vrpn_client_node/MCServer/5/pose", PoseStamped, self.ground_truth_pose_callback, queue_size=10)

        rospy.loginfo("MyController initialized.")

    def callback(self, msg):
        data = msg.data
        if len(data) < 5:
            rospy.logerr("lenth error{}".format(len(data)))
            return

        # Extract the first three values as position information
        xyz_1 = np.array([data[0], data[1], data[2]])
        loss_or_not_val = data[4]
        self.cal_single_axis_ctrl_input(xyz_1, loss_or_not_val)

    def cal_single_axis_ctrl_input(self, xyz_1, loss_or_not):
        """
        Compute single-axis control input (extended to three axes here):
          - Filter the measured position using the image filters
          - Calculate derivative from the difference with the previous timestamp
          - Filter the derivative using Butterworth filters
          - Call function() for further processing
        """
        self.loss_or_not_ = loss_or_not
        # Update filtered positions for each axis
        self.x_real = self.filter_for_imgx.filter(xyz_1[0])
        self.y_real = self.filter_for_imgy.filter(xyz_1[1])
        self.z_real = self.filter_for_imgz.filter(xyz_1[2])

        self.time_now = rospy.Time.now().to_sec()
        self.time_pass = self.time_now - self.time_last
        if self.time_pass <= 0:
            self.time_pass = 0.06  # Prevent division by zero

        # Compute derivatives
        self.x_real_derivative = (self.x_real - self.x_real_last) / self.time_pass
        self.y_real_derivative = (self.y_real - self.y_real_last) / self.time_pass
        self.z_real_derivative = (self.z_real - self.z_real_last) / self.time_pass

        # Filter derivatives using Butterworth filters
        self.x_filtered_deri = self.filter_4_for_derix.filter(self.x_real_derivative)
        self.y_filtered_deri = self.filter_4_for_deriy.filter(self.y_real_derivative)
        self.z_filtered_deri = self.filter_4_for_deriz.filter(self.z_real_derivative)

        # Call processing function (e.g., for control switching or state reset)
        self.function(self.loss_or_not_)

        # Update last states and timestamp
        self.x_real_last = self.x_real
        self.y_real_last = self.y_real
        self.z_real_last = self.z_real
        self.time_last = self.time_now

    def function(self, loss_or_not):
        if loss_or_not == 1 and not self.loss_target:
            self.loss_target = True
        if loss_or_not == 0 and self.loss_target:
            self.loss_target = False
            self.first_time_in_fun = True

        if self.first_time_in_fun:
            self.time_pass = 0.06
            self.x_filtered_deri = 0.0
            self.y_filtered_deri = 0.0
            self.z_filtered_deri = 0.0
            self.first_time_in_fun = False
        else:
            # No additional processing in else branch (matches C++ version)
            pass

    def state_callback(self, msg):
        """
        Handle the Int32 message from the /px4_state_pub topic:
        Update px4_state and reset control input to zero when not in command mode (value != 3)
        """
        self.px4_state = msg.data
        if msg.data != 3:
            self.u = np.zeros(3)

    def spin(self):
        rospy.spin()
