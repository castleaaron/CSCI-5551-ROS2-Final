import openvr
import numpy as np
from scipy.spatial.transform import Rotation as R

import time

import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
# from bungee_msgs.msg import BoolVector3

from geometry_msgs.msg import Quaternion, Twist, Vector3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool

# publish to /offboard_velocity_cmd
class OpenVRPublisher(Node):

    def __init__(self):
        super().__init__('openvr_publisher')

        openvr.init(openvr.VRApplication_Scene)
        self.vr_system = openvr.VRSystem()

        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
        



        self.velocity_pub = self.create_publisher(Twist, '/offboard_velocity_cmd', qos_profile)

        self.arm_toggle = False
        self.arm_pub = self.create_publisher(Bool, '/arm_message', qos_profile)

        self.lock_toggle = False
        self.lockpoint_pub = self.create_publisher(Bool, '/new_lockpoint', qos_profile)


        self.follow_toggle = False
        self.followpoint_pub = self.create_publisher(Vector3, '/controller_pos', qos_profile)
        self.follow_mode_pub = self.create_publisher(Bool, '/follow_mode_toggle', qos_profile)

        self.timer = self.create_timer(1.0 / 60.0, self.timer_callback)
        self.poses = []
        self.get_logger().info("OpenVR publisher started.")

        self.calibration_offset = None
        self.last_vr_yaw = 0.0
        self.yaw = 0.0
        self.max_yaw_rate = 1.0
        self.max_velocity = 2.0

        self.arm_button_clicked = False

        self.last_r3_state = False
        self.last_a_state = False
        self.last_b_state = False
        self.last_trigger_state = False
    

    

    def get_right_controller_index(self):
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            role = self.vr_system.getControllerRoleForTrackedDeviceIndex(i)
            if role == openvr.TrackedControllerRole_RightHand:
                return i
        return None

    def matrix_to_quaternion(self, matrix):
        rot = np.array([
            [matrix[0][0], matrix[0][1], matrix[0][2]],
            [matrix[1][0], matrix[1][1], matrix[1][2]],
            [matrix[2][0], matrix[2][1], matrix[2][2]],
        ])
        r = R.from_matrix(rot)
        return r.as_quat()

    def matrix_to_position(self, matrix):
        pos = np.array([
            matrix[0][3], matrix[1][3], matrix[2][3]
        ])
        return pos

    def quat_to_euler(self, quat, joystick):
        twist = Twist()
        q = quat
        r = R.from_quat(q)

        if self.calibration_offset is None:
            self.calibration_offset = r
            self.last_vr_yaw = 0.0
            self.get_logger().info("VR controller calibrated to zero orientation")
            return

        # Apply calibration offset: subtract initial orientation
        r_calibrated = self.calibration_offset.inv() * r

        # Convert to Euler angles (roll, pitch, yaw)
        pitch, roll, _ = r_calibrated.as_euler('yxz', degrees=False)

        # --- Velocity control ---
        twist.linear.x = -np.clip(-pitch * self.max_velocity / (np.pi/4),
                                -self.max_velocity, self.max_velocity)
        twist.linear.y = -np.clip(roll * self.max_velocity / (np.pi/4),
                                -self.max_velocity, self.max_velocity)
        
        joy_x = 0.0
        if joystick[1] < -0.2 or joystick[1] > 0.2:
            joy_x = joystick[1]
            
        joy_y = 0.0
        if joystick[0] < -0.2 or joystick[0] > 0.2:
            joy_y = joystick[0]

        twist.linear.z = joy_x
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = joy_y

        self.velocity_pub.publish(twist)

        # Debug output
        # self.get_logger().info(f"Calibrated roll: {roll:.3f}, pitch: {pitch:.3f}, yaw: {joy_y:.3f}, throttle: {joy_x:.3f}")

    def arm(self):
        self.arm_toggle = not self.arm_toggle   
        arm_msg = Bool()
        arm_msg.data = self.arm_toggle
        self.arm_pub.publish(arm_msg)
        self.get_logger().info(f"Arm toggled: {self.arm_toggle}")


    def timer_callback(self):
        poses, _ = openvr.VRCompositor().waitGetPoses(self.poses, None)
        right_index = self.get_right_controller_index()

        if right_index is None or not poses[right_index].bPoseIsValid:
            return
        
        res, state = self.vr_system.getControllerState(right_index)



        # if there are buttons
        joystick = []
        if res:
            r3_pressed = bool(state.ulButtonPressed & (1 << 32))
            a_pressed = bool(state.ulButtonPressed & (1 << 7))
            b_pressed = bool(state.ulButtonPressed & (1 << 1))
            trigger = bool(state.ulButtonPressed & (1 << 33))

            if a_pressed and not self.last_a_state:
                if self.follow_toggle:
                    self.get_logger().warn("Folow mode is enabled, unable to enter lock mode.")
                    return
                
                self.lock_toggle = not self.lock_toggle
                lock_msg = Bool()
                lock_msg.data = self.lock_toggle
                self.lockpoint_pub.publish(lock_msg)
                if self.lock_toggle:
                    self.get_logger().info("Lock point set.")
                else:
                    self.get_logger().info("Lock point released.")

            if b_pressed and not self.last_b_state:
                if self.lock_toggle:
                    self.get_logger().warn("Lock mode is enabled, unable to enter follow mode.")
                    return
                
                self.follow_toggle = not self.follow_toggle
                follow_msg = Bool()
                follow_msg.data = self.follow_toggle
                self.follow_mode_pub.publish(follow_msg)
                if self.follow_toggle:
                    self.get_logger().info("Follow mode activated")
                else:
                    self.get_logger().info("Follow mode deactivated")

            
            # Detect rising edge (button just pressed)
            if r3_pressed and not self.last_r3_state:
                self.arm()

            if trigger and not self.last_trigger_state:
                self.calibration_offset = None
                self.get_logger().info("Trigger detected! Recalibrating...")

            self.last_r3_state = r3_pressed
            self.last_a_state = a_pressed
            self.last_b_state = b_pressed
            self.last_trigger_state = trigger

                        
            thumbstick_x = state.rAxis[0].x
            thumbstick_y = state.rAxis[0].y
            joystick = [thumbstick_x, thumbstick_y]
            

        matrix = poses[right_index].mDeviceToAbsoluteTracking
        quat = self.matrix_to_quaternion(matrix)

        if self.follow_toggle:
            follow_msg = Vector3()
            position = self.matrix_to_position(matrix)

            follow_msg.x = position[0] * 2
            follow_msg.y = position[2] * 2
            follow_msg.z = -5.0
            self.followpoint_pub.publish(follow_msg)

        msg = Quaternion()
        msg.x = quat[0]
        msg.y = quat[1]
        msg.z = quat[2]
        msg.w = quat[3]
        self.quat_to_euler(quat, joystick)


def main(args=None):
    rclpy.init(args=args)
    node = OpenVRPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()