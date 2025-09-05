import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations  # quaternion 변환에 사용 (pip install tf-transformations 필요, 하지만 ROS2에 기본 포함)
from math import sin, cos  # 회전 행렬 계산용 (Eigen 대신)

class OdomTransformer(Node):
    def __init__(self):
        super().__init__('odom_transformer')

        # 파라미터 선언 (offset: IMU가 base_link에서 x +0.7m 앞, 회전 offset 없음 가정)
        self.declare_parameter('offset_x', -0.7)  # IMU to base_link translation (뒤로 -0.7m)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)
        self.offset_x = self.get_parameter('offset_x').value
        self.offset_y = self.get_parameter('offset_y').value
        self.offset_z = self.get_parameter('offset_z').value

        # Subscriber: FAST-LIO의 원본 odom subscribe (기본 토픽 가정)
        self.subscription = self.create_subscription(
            Odometry,
            '/Odometry',  # FAST-LIO의 odom 토픽 (laserMapping.cpp에서 pubOdomAftMapped)
            self.odom_callback,
            10
        )

        # Publisher: 변환된 odom publish
        self.publisher = self.create_publisher(Odometry, '/odom_adjusted', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg: Odometry):
        # 원본 odom (IMU 기준, FAST-LIO 기본: child_frame_id = "camera_init")
        imu_pos_x = msg.pose.pose.position.x
        imu_pos_y = msg.pose.pose.position.y
        imu_pos_z = msg.pose.pose.position.z

        # Quaternion을 Euler로 변환 (회전 행렬 계산용)
        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        euler = tf_transformations.euler_from_quaternion(q)  # (roll, pitch, yaw)

        # IMU 회전 행렬 (FAST-LIO 기본: state_point.rot 기반)
        roll, pitch, yaw = euler
        rot_matrix = [
            [cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll)],
            [sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll)],
            [-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]
        ]

        # offset 벡터 (IMU/camera_init to base_link)
        offset_vec = [self.offset_x, self.offset_y, self.offset_z]

        # base_pos = imu_pos + imu_rot * offset_vec
        base_pos_x = imu_pos_x + (rot_matrix[0][0]*offset_vec[0] + rot_matrix[0][1]*offset_vec[1] + rot_matrix[0][2]*offset_vec[2])
        base_pos_y = imu_pos_y + (rot_matrix[1][0]*offset_vec[0] + rot_matrix[1][1]*offset_vec[1] + rot_matrix[1][2]*offset_vec[2])
        base_pos_z = imu_pos_z + (rot_matrix[2][0]*offset_vec[0] + rot_matrix[2][1]*offset_vec[1] + rot_matrix[2][2]*offset_vec[2])

        # 새로운 odom 메시지 생성
        new_odom = Odometry()
        new_odom.header = msg.header  # timestamp 등 유지
        new_odom.header.frame_id = 'odom'  # 그대로
        new_odom.child_frame_id = 'base_link'  # base_link로 설정 (FAST-LIO 기본 camera_init 대신)

        new_odom.pose.pose.position.x = base_pos_x
        new_odom.pose.pose.position.y = base_pos_y
        new_odom.pose.pose.position.z = base_pos_z
        new_odom.pose.pose.orientation = msg.pose.pose.orientation  # 회전은 IMU와 동일 (offset 없음 가정)

        # Velocity 등은 그대로 복사 (offset 영향 적음)
        new_odom.twist = msg.twist
        new_odom.pose.covariance = msg.pose.covariance
        new_odom.twist.covariance = msg.twist.covariance

        # Publish new odom
        self.publisher.publish(new_odom)

        # TF Broadcast (odom to base_link, FAST-LIO 기본 odom to camera_init 대신)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = base_pos_x
        t.transform.translation.y = base_pos_y
        t.transform.translation.z = base_pos_z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTransformer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()