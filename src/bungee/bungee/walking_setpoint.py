import rclpy
from geometry_msgs.msg import Point
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

def main():
    rclpy.init()
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )
    node = rclpy.create_node("walking_setpoint")
    pub = node.create_publisher(Point, "/controller_pos", qos_profile)

    # Initial position
    x = 0.0
    y = 0.0
    z = 1

    # Velocity (units per second)
    vx = 0.1
    vy = 0.0
    vz = 0.0

    rate = 0.05   # 20 Hz (time.sleep seconds)

    node.get_logger().info("Publishing straight-line positions...")

    last_time = time.time()

    while rclpy.ok():
        now = time.time()
        dt = now - last_time
        last_time = now

        # Update position
        x += vx * dt
        y += vy * dt
        z += vz * dt

        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = z

        pub.publish(msg)

        time.sleep(rate)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
