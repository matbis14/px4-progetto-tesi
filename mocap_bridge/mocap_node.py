#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    VehicleOdometry,
    VehicleAttitude,
    OffboardControlMode,
    TrajectorySetpoint
)


class MocapNode(Node):
    def __init__(self):
        super().__init__('mocap_node')

        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber
        self.create_subscription(Odometry, '/model/x500_0/odometry', self.listener_cb, 10)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_cb, qos_be)

        # Publisher (pose/path/debug)
        self.pose_pub = self.create_publisher(PoseStamped, '/x500/pose', 10)
        self.path_pub = self.create_publisher(Path, '/x500/path', 10)

        self.odom_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        self.mocap_odom_pub_ = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_mocap_odometry', 10)
        self.debug_odom_NED_pub_ = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_debug_odometry_NED', 10)

        # Publisher (Offboard mode)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # Timer for Offboard stream (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_offboard_setpoint)

        self.path = Path()
        self.path.header.frame_id = 'map'
        self.path_cnt = 0
        self.att_msg = None
        self.get_logger().info('MocapNode started')

    def listener_cb(self, msg: Odometry):
        self.get_logger().info(
            'Received message from Gazebo: pos=(%.2f, %.2f, %.2f)' %
            (msg.pose.pose.position.x,
             msg.pose.pose.position.y,
             msg.pose.pose.position.z),
            once=True
        )

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

        # Publish VehicleOdometry (visual odom)
        odom = VehicleOdometry()
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        now_us = sec * 1_000_000 + nsec // 1_000
        odom.timestamp = odom.timestamp_sample = now_us

        p = msg.pose.pose.position
        odom.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        # FLU->FRD
        odom.position = [float(p.x), -float(p.y), -float(p.z)]

        if self.att_msg:
            odom.q = [float(q) for q in self.att_msg.q]
        else:
            odom.q = [1.0, 0.0, 0.0, 0.0]

        odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        odom.velocity = odom.angular_velocity = [float('nan')] * 3
        odom.quality = 1

        self.odom_pub.publish(odom)

        # Publish debug odom in NED
        debug_odom_ned = VehicleOdometry()
        debug_odom_ned.timestamp = odom.timestamp
        debug_odom_ned.timestamp_sample = odom.timestamp_sample
        debug_odom_ned.position = [float(p.y), float(p.x), -float(p.z)]  # ENU->NED
        debug_odom_ned.pose_frame = VehicleOdometry.POSE_FRAME_NED
        debug_odom_ned.q = [float('nan')] * 4
        debug_odom_ned.velocity = [float('nan')] * 3
        debug_odom_ned.angular_velocity = [float('nan')] * 3
        self.debug_odom_NED_pub_.publish(debug_odom_ned)

    def attitude_cb(self, msg: VehicleAttitude):
        self.att_msg = msg

    def publish_offboard_setpoint(self):
        """Publish OffboardControlMode + TrajectorySetpoint at 20 Hz."""
        now_us = self.get_clock().now().nanoseconds // 1000

        # Offboard mode (tell PX4 what we control)
        mode = OffboardControlMode()
        mode.timestamp = now_us
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self.offboard_mode_pub.publish(mode)

        # Simple trajectory setpoint: hover at [0,0,-5] in NED
        sp = TrajectorySetpoint()
        sp.timestamp = now_us
        sp.position = [0.0, 0.0, -5.0]
        sp.velocity = [0.0, 0.0, 0.0]
        sp.acceleration = [0.0, 0.0, 0.0]
        sp.yaw = 0.0
        self.trajectory_pub.publish(sp)


def main(args=None):
    rclpy.init(args=args)
    node = MocapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
