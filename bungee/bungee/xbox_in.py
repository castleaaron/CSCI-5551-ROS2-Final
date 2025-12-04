import rclpy

import geometry_msgs.msg
import std_msgs.msg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from evdev import InputDevice, categorize, ecodes
import numpy as np

def main():
    rclpy.init()
    node = rclpy.create_node('xbox_publisher')
    device_path='/dev/input/event23'
    qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
    device = InputDevice(device_path)

    node.get_logger().info(f"Reading Xbox controller: {device_path}")

    pub = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)

    arm_toggle = False
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)
    
    lock_toggle = False
    lockpoint_pub = node.create_publisher(std_msgs.msg.Bool, '/new_lockpoint', qos_profile)
    # Timer loop

    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0

    try:
        while True:
            for event in device.read_loop():
                # Button events (A, B, X, Y, etc)
                if event.type == ecodes.EV_KEY:
                    e = categorize(event)
                    #msg = f"BUTTON {e.keycode} STATE {e.keystate}"
                    #print(msg)
                    if e.keycode[1] == 'BTN_Y' and e.keystate==1: 
                        arm_toggle = not arm_toggle  # Flip the value of arm_toggle
                        arm_msg = std_msgs.msg.Bool()
                        arm_msg.data = arm_toggle
                        arm_pub.publish(arm_msg)
                        print(f"Arm toggle is now: {arm_toggle}")
                
                    if e.keycode[0] == 'BTN_A' and e.keystate==1:
                        lock_toggle = not lock_toggle
                        lock_msg = std_msgs.msg.Bool()
                        lock_msg.data = lock_toggle
                        lockpoint_pub.publish(lock_msg)
                        if lock_toggle:
                            print(f"Lock Point Set")
                        else:
                            print(f"Lock Point Released")

                # Stick + trigger analog events
                if event.type == ecodes.EV_ABS:
                    if event.code == ecodes.ABS_X:
                        x = event.value/16384*-1
                    elif event.code == ecodes.ABS_Y:
                        y = event.value/16384*-1
                    elif event.code == ecodes.ABS_Z:
                        z = event.value*-1/1023
                    elif event.code == ecodes.ABS_RZ:
                        z = event.value/1023
                
                twist = geometry_msgs.msg.Twist()
                twist.linear.x = float(x)
                twist.linear.y = float(y)
                twist.linear.z = float(z)
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = th
                pub.publish(twist)
                print("X:",twist.linear.x, "   Y:",twist.linear.y, "   Z:",twist.linear.z, "   Yaw:",twist.angular.z)
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
