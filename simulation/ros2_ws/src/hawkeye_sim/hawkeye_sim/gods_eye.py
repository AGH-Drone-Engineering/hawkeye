import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose, PoseStamped

from scipy.spatial.transform import Rotation


def get_vec_in_frame(ref: Pose, target: Pose) -> Vector3:
    '''
    Returns a vector pointing from ref to target in the frame of ref.
    '''
    vec = [target.position.x - ref.position.x,
           target.position.y - ref.position.y,
           target.position.z - ref.position.z]
    rot = Rotation.from_quat([ref.orientation.x,
                              ref.orientation.y,
                              ref.orientation.z,
                              ref.orientation.w]).inv()
    x, y, z = rot.apply(vec)
    return Vector3(x=x, y=y, z=z)


class GodsEye(Node):
    def __init__(self):
        super().__init__('gods_eye')
        
        self.follower_pose = None

        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            'target/pose',
            self.target_pose_callback,
            1,
        )
        self.follower_pose_sub = self.create_subscription(
            PoseStamped,
            'follower/pose',
            self.follower_pose_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )
        
        self.target_vec_pub = self.create_publisher(
            Vector3,
            'follower/target/vec',
            1,
        )

    def follower_pose_callback(self, msg: PoseStamped):
        self.follower_pose = msg.pose

    def target_pose_callback(self, msg: PoseStamped):
        if self.follower_pose is None:
            return
        target_pose = msg.pose
        target_vec = get_vec_in_frame(self.follower_pose, target_pose)
        self.target_vec_pub.publish(target_vec)


def main(args=None):
    rclpy.init(args=args)
    gods_eye = GodsEye()
    rclpy.spin(gods_eye)
    gods_eye.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
