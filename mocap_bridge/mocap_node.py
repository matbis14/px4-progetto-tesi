#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry, VehicleAttitude


class MocapNode(Node):
    def __init__(self):
        super().__init__('mocap_node')

        # QoS BEST_EFFORT (PX4 subscriber usa BEST_EFFORT)
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber
        self.create_subscription(Odometry, '/model/x500_0/odometry', self.listener_cb, qos_be)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_cb, qos_be)

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/x500/pose', 10)
        self.path_pub = self.create_publisher(Path, '/x500/path', 10)
        self.odom_pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            qos_be   # <--- QoS corretto per PX4
        )

        # Stato
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.path_cnt = 0
        self.att_msg = None

        self.get_logger().info('MOCAP node avviato (QoS BEST_EFFORT, timestamp da Gazebo, ENU→NED).')

    def listener_cb(self, msg: Odometry):
        # Pose per RViz (frame ENU "map")
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = 'map'
        ps.pose = msg.pose.pose
        self.pose_pub.publish(ps)

        if self.path_cnt == 0:
            self.path.header.stamp = ps.header.stamp
            self.path.poses.append(ps)
            self.path_pub.publish(self.path)
        self.path_cnt = (self.path_cnt + 1) % 10

        # VehicleOdometry per PX4
        odom = VehicleOdometry()

        # Timestamp dalla misura di Gazebo (in microsecondi)
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        now_us = sec * 1_000_000 + nsec // 1_000
        odom.timestamp = odom.timestamp_sample = now_us

        # Posizione: ENU -> NED
        p = msg.pose.pose.position  # ENU: x=E, y=N, z=U
        odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        odom.position = [float(p.y), float(p.x), float(-p.z)]

        # Orientazione: prendiamo quella di PX4 (non fondiamo yaw EV)
        if self.att_msg:
            odom.q = [float(q) for q in self.att_msg.q]  # FRD
        else:
            odom.q = [0.0, 0.0, 0.0, 1.0]

        # Velocità EV: ENU -> NED
        v = msg.twist.twist.linear
        odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        odom.velocity = [float(v.y), float(v.x), float(-v.z)]
        odom.angular_velocity = [float('nan')] * 3

        # Varianze finite (non NaN, altrimenti EKF le ignora)
        odom.position_variance    = [1e-5, 1e-5, 4e-5]
        odom.orientation_variance = [1e-4, 1e-4, 1e-4]
        odom.velocity_variance    = [1e-3, 1e-3, 2e-3]

        odom.quality = 1
        self.odom_pub.publish(odom)

    def attitude_cb(self, msg: VehicleAttitude):
        self.att_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = MocapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
