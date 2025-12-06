import openvr
import numpy as np
from scipy.spatial.transform import Rotation as R

import time

import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from bungee_msgs.msg import BoolVector3

from geometry_msgs.msg import Quaternion, Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

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
        

        self.controller_marker_pub = self.create_publisher(Marker, '/vr_controller_marker', 10)


        self.velocity_pub = self.create_publisher(Twist, '/offboard_velocity_cmd', qos_profile)

        self.arm_toggle = False
        self.arm_pub = self.create_publisher(Bool, '/arm_message', qos_profile)

        self.lock_toggle = False
        self.lockpoint_pub = self.create_publisher(Bool, '/new_lockpoint', qos_profile)


        self.follow_toggle = False
        self.followpoint_pub = self.create_publisher(BoolVector3, '/controller_pos', qos_profile)

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
        
        # Double-click detection with delayed single-click action
        self.last_click_time = 0.0
        self.double_click_threshold = 0.3  # 300ms window for double-click
        self.pending_single_click = False
        self.pending_click_timer = None

    
    def publish_controller_marker(self, position):
        marker = Marker()
        marker.header.frame_id = 'map'  # or 'world' depending on your TF
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'vr_controller'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[2]
        marker.pose.position.y = position[0]
        marker.pose.position.z = position[1]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # 5cm diameter
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.controller_marker_pub.publish(marker)


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

    def execute_single_click(self):
        """Execute the single-click action (toggle arm) after delay"""
        self.pending_single_click = False
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
            b_pressed = bool(state.ulButtonPressed & (1 << 12))
            current_time = self.get_clock().now().nanoseconds / 1e9

            # if a_pressed and not self.last_a_state:
            #     # self.lock_toggle = not self.lock_toggle
            #     # lock_msg = Bool()
            #     # lock_msg.data = self.lock_toggle
            #     # self.lockpoint_pub.publish(lock_msg)
            #     # if self.lock_toggle:
            #     #     self.get_logger().info("Lock point set.")
            #     # else:
            #     #     self.get_logger().info("Lock point released.")

            if a_pressed and not self.last_a_state:
                self.follow_toggle = not self.follow_toggle
                if self.follow_toggle:
                    self.get_logger().info("Follow mode activated")
                else:
                    self.get_logger().info("Follow mode deactivated")

            
            # Detect rising edge (button just pressed)
            if r3_pressed and not self.last_r3_state:
                time_since_last_click = current_time - self.last_click_time
                
                if time_since_last_click < self.double_click_threshold:
                    # Double-click detected - recalibrate and cancel pending single click
                    if self.pending_click_timer is not None:
                        self.pending_click_timer.cancel()
                        self.pending_click_timer = None
                    self.pending_single_click = False
                    
                    self.calibration_offset = None
                    self.get_logger().info("Double-click detected! Recalibrating...")
                else:
                    # Potential single click - schedule delayed action
                    if self.pending_click_timer is not None:
                        self.pending_click_timer.cancel()
                    
                    self.pending_single_click = True
                    self.pending_click_timer = self.create_timer(
                        self.double_click_threshold,
                        self.execute_single_click
                    )
                    self.pending_click_timer._autostart = False
                    self.pending_click_timer.reset()
                
                self.last_click_time = current_time
            
            self.last_r3_state = r3_pressed
            self.last_a_state = a_pressed
            self.last_b_state = b_pressed

                        
            thumbstick_x = state.rAxis[0].x
            thumbstick_y = state.rAxis[0].y
            joystick = [thumbstick_x, thumbstick_y]
            

        matrix = poses[right_index].mDeviceToAbsoluteTracking
        quat = self.matrix_to_quaternion(matrix)

        if self.follow_toggle:
            follow_msg = BoolVector3()
            follow_msg.flag = self.follow_toggle
            position = self.matrix_to_position(matrix)

            follow_msg.vector.x = position[0]
            follow_msg.vector.y = position[1]
            follow_msg.vector.z = position[2]
            self.followpoint_pub.publish(follow_msg)

        msg = Quaternion()
        msg.x = quat[0]
        msg.y = quat[1]
        msg.z = quat[2]
        msg.w = quat[3]
        self.quat_to_euler(quat, joystick)

        position = self.matrix_to_position(matrix)
        self.publish_controller_marker(position)



def main(args=None):
    rclpy.init(args=args)
    node = OpenVRPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()