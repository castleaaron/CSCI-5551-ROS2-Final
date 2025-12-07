
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Point
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Create subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.offboard_velocity_sub = self.create_subscription(
            Twist,
            '/offboard_velocity_cmd',
            self.offboard_velocity_callback,
            qos_profile)
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)
        
        self.my_bool_sub = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,
            qos_profile)
        
        # these three are new subs
        self.lockpoint_sub = self.create_subscription(
            Bool,
            '/new_lockpoint',
            self.lock_pos_callback,
            qos_profile)
        
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.pos_update_callback,
            qos_profile)
        
        self.pos_sub = self.create_subscription(
            Vector3,
            '/controller_pos',
            self.follow_pos_callback,
            qos_profile)
        
        self.follow_mode_sub = self.create_subscription(
            Bool,
            '/follow_mode_toggle',
            self.follow_mode_callback,
            qos_profile
        )

        #Create publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_velocity = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.controller_marker_pub = self.create_publisher(Marker, '/vr_controller_marker', 10)

        
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        self.yaw = 0.0  #yaw value we send as command
        self.trueYaw = 0.0  #current yaw value of drone
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.lock_msg = False
        self.failsafe = False
        self.current_state = "IDLE"
        self.last_state = self.current_state
        self.lock = Vector3()
        self.x = 0
        self.y = 0
        self.z = 0
        self.follow_mode = False
        self.controller_pos = []
        self.first_time = False
        self.initial_pos = None

        self.orbit_angular_vel = 0.005
        self.orbit_radius = 5
        self.orbit_x_bar = 0.0
        self.orbit_y_bar = 0.0
        self.theta = 0.0

        self.counter = 0



    def publish_controller_marker(self, position):
        marker = Marker()
        marker.header.frame_id = 'map'  # or 'world' depending on your TF
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'vr_controller'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = -position[1]
        marker.pose.position.z = -position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # 5cm diameter
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.controller_marker_pub.publish(marker)




    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")
    
    def lock_pos_callback(self, msg):
        self.lock_msg = msg.data
        self.get_logger().info(f"Lockpoint Set: {self.lock_msg}")
        if self.lock_msg:
            self.lock.x = self.x
            self.lock.y = self.y
            self.lock.z = self.z

            self.publish_controller_marker([self.lock.x, self.lock.y, self.lock.z])

    def follow_mode_callback(self, msg):
        self.follow_mode = msg.data
        self.first_time = True
        

    def follow_pos_callback(self, msg):
        # what we want to do is update the home position in relation to the controller
        controller_pos = [msg.x, msg.y, msg.z]

        if self.first_time:
            self.initial_pos = controller_pos
            self.first_time = False

            self.orbit_x_bar = self.initial_pos[0] - self.x
            self.orbit_y_bar = self.initial_pos[1] - self.y

            self.theta = np.arctan(self.orbit_y_bar / self.orbit_x_bar)

        self.theta = self.theta + (self.orbit_angular_vel * self.yaw)
        
        orbit_x = controller_pos[0] + (self.orbit_radius * np.cos(self.theta))
        orbit_y = controller_pos[1] + (self.orbit_radius * np.sin(self.theta))


        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)            

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        trajectory_msg.velocity[0] = float('nan')
        trajectory_msg.velocity[1] = float('nan')
        trajectory_msg.velocity[2] = float('nan')
        trajectory_msg.position[0] = orbit_x
        trajectory_msg.position[1] = orbit_y
        trajectory_msg.position[2] = controller_pos[2]
        trajectory_msg.acceleration[0] = float('nan')
        trajectory_msg.acceleration[1] = float('nan')
        trajectory_msg.acceleration[2] = float('nan')
        trajectory_msg.yaw = self.theta + np.pi
        trajectory_msg.yawspeed = float('nan')


        self.publisher_trajectory.publish(trajectory_msg)

        self.publish_controller_marker([controller_pos[0], controller_pos[1], controller_pos[2]])



    
    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):
        match self.current_state:
            case "IDLE":
                if(self.flightCheck and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")
                self.arm() #send arm command

            case "TAKEOFF":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")
                self.arm() #send arm command
                self.take_off() #send takeoff command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                self.state_offboard()

        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.myCnt += 1

    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True   

    # Arms the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # Takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
        self.get_logger().info("Takeoff command send")

    #publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):

        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    def pos_update_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
    
    #receives Twist commands from Teleop and converts NED -> FLU
    def offboard_velocity_callback(self, msg):
        #implements NED -> FLU Transformation
        deadzone = 0.5
        if abs(msg.linear.y)>deadzone:
            # X (FLU) is -Y (NED)
            self.velocity.x = -msg.linear.y
        else:
            self.velocity.x = 0.0

        if abs(msg.linear.x)>deadzone:
            # Y (FLU) is X (NED)
            self.velocity.y = msg.linear.x
        else:
            self.velocity.y = 0.0

        joystick_dz = 0.3
        if abs(msg.linear.z) > joystick_dz:
            self.velocity.z = -msg.linear.z
        else:
            self.velocity.z = 0.0
        
        if abs(msg.angular.z) > joystick_dz:
            self.yaw = msg.angular.z
        else:
            self.yaw = 0.0
        

    #receives current trajectory values from drone and grabs the yaw value of the orientation
    def attitude_callback(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        
        

    #publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if(self.offboardMode == True and self.follow_mode == False):
            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            self.publisher_offboard_mode.publish(offboard_msg)            

            # Compute velocity in the world frame
            cos_yaw = np.cos(self.trueYaw)
            sin_yaw = np.sin(self.trueYaw)
            velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
            velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)



            # Create and publish TrajectorySetpoint message 
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)

            trajectory_msg.velocity[0] = velocity_world_x
            trajectory_msg.velocity[1] = velocity_world_y
            trajectory_msg.velocity[2] = self.velocity.z
            trajectory_msg.position[0] = float('nan')
            trajectory_msg.position[1] = float('nan')
            trajectory_msg.position[2] = float('nan')
            trajectory_msg.acceleration[0] = float('nan')
            trajectory_msg.acceleration[1] = float('nan')
            trajectory_msg.acceleration[2] = float('nan')
            trajectory_msg.yaw = float('nan')
            trajectory_msg.yawspeed = self.yaw
            
            radius = 5
            x_bar = self.x - self.lock.x
            y_bar = self.y - self.lock.y
            col_angle = np.arctan2(x_bar,y_bar)*(180/pi)
            vel_angle = np.arctan2(velocity_world_x,velocity_world_y)*(180/pi)
            # self.get_logger().info(f"X: {self.x}")
            # self.get_logger().info(f"Y: {self.y}")
            # self.get_logger().info(f"Col Angle: {col_angle}")
            # self.get_logger().info(f"vel Angle: {vel_angle}")
            if self.lock_msg:
                in_bounds = np.sqrt(pow(x_bar,2) + pow(y_bar,2)) < radius
                
                if not in_bounds:
                    trajectory_msg.velocity[0] = float('nan')
                    trajectory_msg.velocity[1] = float('nan')
                    trajectory_msg.velocity[2] = float('nan')
                    trajectory_msg.position[0] = self.x
                    trajectory_msg.position[1] = self.y
                    trajectory_msg.position[2] = self.lock.z
                    trajectory_msg.yaw = float('nan')
                    trajectory_msg.yawspeed = 0.0
                    if (vel_angle > col_angle + 90) or (vel_angle < col_angle - 90):
                        trajectory_msg.velocity[0] = velocity_world_x
                        trajectory_msg.velocity[1] = velocity_world_y
                        trajectory_msg.velocity[2] = 0.0
                        trajectory_msg.position[0] = float('nan')
                        trajectory_msg.position[1] = float('nan')
                        trajectory_msg.position[2] = float('nan')
                        trajectory_msg.yaw = float('nan')
                        trajectory_msg.yawspeed = self.yaw

                elif (abs(self.velocity.x)==0.0) and (abs(self.velocity.y)==0.0) and (abs(self.yaw)==0.0):
                    trajectory_msg.velocity[0] = float('nan')
                    trajectory_msg.velocity[1] = float('nan')
                    trajectory_msg.velocity[2] = float('nan')
                    trajectory_msg.position[0] = self.lock.x
                    trajectory_msg.position[1] = self.lock.y
                    trajectory_msg.position[2] = self.lock.z
                    trajectory_msg.yaw = float('nan')
                    trajectory_msg.yawspeed = 0.0
                else:
                    trajectory_msg.velocity[2] = 0.0

                if self.counter >= 10 and not (abs(self.velocity.x)==0.0) and (abs(self.velocity.y)==0.0) and (abs(self.yaw)==0.0):
                    trajectory_msg.velocity[0] = float('nan')
                    trajectory_msg.velocity[1] = float('nan')
                    trajectory_msg.velocity[2] = float('nan')
                    trajectory_msg.position[0] = self.x
                    trajectory_msg.position[1] = self.y
                    trajectory_msg.position[2] = self.lock.z
                    trajectory_msg.yaw = float('nan')
                    trajectory_msg.yawspeed = 0.0

                
                self.counter += 1
                self.counter %= 10
                

            self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
