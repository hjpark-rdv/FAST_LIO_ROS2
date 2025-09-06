import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
from math import sin, cos
import numpy as np
if not hasattr(np, 'float'):
        np.float = float

class OdomTransformer(Node):
    def __init__(self):
        super().__init__('odom_transformer')

        # 파라미터 선언 (imu_link와 base_link 입력받음, 기본값 설정)
        self.declare_parameter('imu_link', 'livox_frame')  # imu_link 이름 (예: body 또는 camera_init)
        self.declare_parameter('base_link', 'base_link')  # base_link 이름
        self.imu_link = self.get_parameter('imu_link').value
        self.base_link = self.get_parameter('base_link').value

        # TF Listener 설정 (offset 자동 계산용)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber: /Odometry subscribe (원본: frame_id=camera_init, child_frame_id=body)
        self.subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )

        # Publisher: 변환된 odom publish (/odom_adjusted)
        self.publisher = self.create_publisher(Odometry, '/odom_adjusted', 10)

        # TF Broadcaster (odom -> base_link)
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg: Odometry):
        # 원본 odom (camera_init/body 기준)
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

        # IMU 회전 행렬
        roll, pitch, yaw = euler
        rot_matrix = [
            [cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll)],
            [sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll)],
            [-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]
        ]

        # TF에서 offset 자동 계산 (source: imu_link, target: base_link)
        source_frame = self.imu_link
        target_frame = self.base_link
        try:
            # TF lookup (현재 timestamp 기준)
            trans = self.tf_buffer.lookup_transform(source_frame,target_frame, rclpy.time.Time())
            
            # offset 벡터 추출 (translation)
            offset_x = trans.transform.translation.x
            offset_y = trans.transform.translation.y
            offset_z = trans.transform.translation.z

            self.get_logger().info(f"Auto-calculated offset from TF: x={offset_x}, y={offset_y}, z={offset_z}")

        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}. Using default offset (0,0,0)")
            offset_x, offset_y, offset_z = 0.0, 0.0, 0.0  # 실패 시 default

        # offset 벡터
        offset_vec = [offset_x, offset_y, offset_z]

        # base_pos = imu_pos + imu_rot * offset_vec
        base_pos_x = imu_pos_x + (rot_matrix[0][0]*offset_vec[0] + rot_matrix[0][1]*offset_vec[1] + rot_matrix[0][2]*offset_vec[2])
        base_pos_y = imu_pos_y + (rot_matrix[1][0]*offset_vec[0] + rot_matrix[1][1]*offset_vec[1] + rot_matrix[1][2]*offset_vec[2])
        base_pos_z = imu_pos_z + (rot_matrix[2][0]*offset_vec[0] + rot_matrix[2][1]*offset_vec[1] + rot_matrix[2][2]*offset_vec[2])

        # 새로운 odom 메시지 생성
        new_odom = Odometry()
        new_odom.header = msg.header
        new_odom.header.frame_id = 'odom'  # odom으로 변경 (원본 camera_init 대신)
        new_odom.child_frame_id = self.base_link  # base_link

        new_odom.pose.pose.position.x = base_pos_x
        new_odom.pose.pose.position.y = base_pos_y
        new_odom.pose.pose.position.z = base_pos_z
        new_odom.pose.pose.orientation = msg.pose.pose.orientation  # 회전은 그대로 (offset 없음 가정)

        # Velocity 등 복사
        new_odom.twist = msg.twist
        new_odom.pose.covariance = msg.pose.covariance
        new_odom.twist.covariance = msg.twist.covariance

        # Publish new odom
        self.publisher.publish(new_odom)

        # TF Broadcast (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = self.base_link
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
