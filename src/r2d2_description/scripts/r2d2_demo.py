#!/usr/bin/env python
import time
import rospy
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
"""
Topics To Write on:
type: std_msgs/Float64
/r2d2/roll_joint_velocity_controller/command
/r2d2/wheel_arm1_joint_velocity_controller/command
/r2d2/wheel_arm2_joint_velocity_controller/command
/r2d2/incline_joint_position_controller/command
/r2d2/base_cylinder_joint_position_controller/command
/r2d2/base_head_joint_position_controller/command
/r2d2/shoulder1_joint_position_controller/command
/r2d2/coude1_joint_position_controller/command
/r2d2/shoulder2_joint_position_controller/command
/r2d2/coude2_joint_position_controller/command
"""

class r2d2JointMover(object):

    def __init__(self):
        rospy.init_node('jointmover_demo', anonymous=True)
        rospy.loginfo("R2D2 JointMover Initialising...")
        # Velocity Control 
        self.pub_r2d2_roll_joint_velocity = rospy.Publisher('/r2d2/roll_joint_velocity_controller/command', 
                                                            Float64, 
                                                            queue_size=10)
        self.pub_r2d2_wheel_arm1_joint_velocity = rospy.Publisher('/r2d2/wheel_arm1_joint_velocity_controller/command', 
                                                            Float64, 
                                                            queue_size=10)
        self.pub_r2d2_wheel_arm2_joint_velocity = rospy.Publisher('/r2d2/wheel_arm2_joint_velocity_controller/command', 
                                                            Float64, 
                                                            queue_size=10)
        #Position Control
        self.incline_joint_position = rospy.Publisher('/r2d2/incline_joint_position_controller/command', 
                                                            Float64, 
                                                            queue_size=10)
        self.base_cylinder_joint_position = rospy.Publisher('/r2d2/base_cylinder_joint_position_controller/command', 
                                                            Float64, 
                                                            queue_size=10)
        self.base_head_joint_position = rospy.Publisher('/r2d2/base_head_joint_position_controller/command', 
                                                            Float64, 
                                                            queue_size=10)
        self.shoulder1_joint_position = rospy.Publisher('/r2d2/shoulder1_joint_position_controller/command', 
                                                            Float64, 
                                                            queue_size=10)
        self.coude1_joint_position = rospy.Publisher('/r2d2/coude1_joint_position_controller/command', 
                                                            Float64, 
                                                            queue_size=10)
        self.shoulder2_joint_position = rospy.Publisher('/r2d2/shoulder2_joint_position_controller/command', 
                                                            Float64, 
                                                            queue_size=10)
        self.coude2_joint_position = rospy.Publisher('/r2d2/coude2_joint_position_controller/command', 
                                                            Float64, 
                                                            queue_size=10)
                                                                                                                                                                                                                                                                                                                 
        joint_states_topic_name = "/r2d2/joint_states"
        rospy.Subscriber(joint_states_topic_name, JointState, self.r2d2_joints_callback)
        r2d2_joints_data = None
        while r2d2_joints_data is None:
            try:
                r2d2_joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout=5)
            except:
                rospy.logwarn("Time out " + str(joint_states_topic_name))
                pass

        self.r2d2_joint_dictionary = dict(zip(r2d2_joints_data.name, r2d2_joints_data.position))


    def set_r2d2_velocity(self, velocity):
        """
        :param position:
        :return:
        """
        vel = Float64()
        vel.data = velocity

        self.pub_r2d2_roll_joint_velocity.publish(vel)
        self.pub_r2d2_wheel_arm1_joint_velocity.publish(vel)
        self.pub_r2d2_wheel_arm2_joint_velocity.publish(vel)

    def move_r2d2_head_joint(self, position):
        """
        limits radians : lower="-0.2" upper="0.2"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.base_head_joint_position.publish(angle)

    def move_r2d2_lift(self): #DOES NOT WORK (INERTIAS?)
        """
        limits radians : lower="-0.2" upper="0.2"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = 0.091
        self.coude1_joint_position.publish(angle)
        self.coude2_joint_position.publish(angle)

        angle2 = Float64()
        angle2.data = 0.1
        self.incline_joint_position.publish(angle2)

        angle4 = Float64()
        angle4.data = -0.095
        self.base_cylinder_joint_position.publish(angle4)

        angle3 = Float64()
        angle3.data = -0.2
        self.shoulder1_joint_position.publish(angle3)
        self.shoulder2_joint_position.publish(angle3)

        vel = Float64()
        vel.data = -1
        #self.pub_r2d2_roll_joint_velocity.publish(vel)

    def r2d2_joints_callback(self, msg):
        """
        sensor_msgs/JointState
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

        :param msg:
        :return:
        """
        self.r2d2_joint_dictionary = dict(zip(msg.name, msg.position))

    def r2d2_check_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name is near the value given
        :param value:
        :return:
        """
        similar = self.r2d2_joint_dictionary.get(joint_name) >= (value - error ) and self.r2d2_joint_dictionary.get(joint_name) <= (value + error )

        return similar

    def convert_angle_to_unitary(self, angle):
        """
        Removes complete revolutions from angle and converts to the positive equivalent
        if the angle is negative
        :param angle: Has to be in radians
        :return:
        """
        # Convert to angle between [0,360)
        complete_rev = 2 * pi
        mod_angle = int(angle / complete_rev)
        clean_angle = angle - mod_angle * complete_rev
        # Convert Negative angles to their corresponding positive values
        if clean_angle < 0:
            clean_angle += 2 * pi

        return clean_angle

    def assertAlmostEqualAngles(self, x, y,):
        c2 = (sin(x) - sin(y)) ** 2 + (cos(x) - cos(y)) ** 2
        angle_diff = acos((2.0 - c2) / 2.0)
        return angle_diff

    def r2d2_check_continuous_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name is near the value given
        We have to convert the joint values removing whole revolutions and converting negative versions
        of the same angle
        :param value:
        :return:
        """
        joint_reading = self.r2d2_joint_dictionary.get(joint_name)
        clean_joint_reading = self.convert_angle_to_unitary(angle=joint_reading)
        clean_value = self.convert_angle_to_unitary(angle=value)

        dif_angles = self.assertAlmostEqualAngles(clean_joint_reading, clean_value)
        similar = dif_angles <= error

        return similar


    def movement_5s(self):
        """
        Move forward
        :return:
        """
        rospy.loginfo("Start Moving r2d2...")
         # Set the motor velocity
        velocity_command = 2.0  # Adjust this value for desired velocity (rad/s)

        # Publish velocity commands for 5 seconds
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < 5:
            self.set_r2d2_velocity(velocity_command)
            rospy.sleep(0.1)  # Publish at 10 Hz

        # Stop the motor after 10 seconds
        self.set_r2d2_velocity(0.0)
        rospy.loginfo("Motor stopped.")


if __name__ == "__main__":
    r2d2_jointmover_object = r2d2JointMover()
    r2d2_jointmover_object.movement_5s()
    r2d2_jointmover_object.move_r2d2_head_joint(1.0)
    r2d2_jointmover_object.move_r2d2_lift()