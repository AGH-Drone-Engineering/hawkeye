import threading
import subprocess
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from geometry_msgs.msg import Vector3


MODEL_REGEX = re.compile(r'^model {[\s\S]*}$', re.MULTILINE)
MODEL_NAME_REGEX = re.compile(r'^ {2}name: "(.*)"$', re.MULTILINE)
MODEL_POSITION_REGEX = re.compile(r'^ {4}position {\n {6}x: (.*)\n {6}y: (.*)\n {6}z: (.*)\n {4}}$', re.MULTILINE)


def extract_target_pose_from_output(output: str) -> Vector3:
    models = MODEL_REGEX.findall(output)
    for model in models:
        name = MODEL_NAME_REGEX.search(model).group(1)
        if name != 'iris_dummy':
            continue
        x, y, z = MODEL_POSITION_REGEX.search(model).group(1, 2, 3)
        pose = Vector3()
        pose.x = float(x)
        pose.y = float(y)
        pose.z = float(z)
        return pose


class LogicalCamera(Node):
    def __init__(self):
        super().__init__('logical_camera')

        self.target_vec_pub = self.create_publisher(
            Vector3,
            'target/vec',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )


def main(args=None):
    rclpy.init(args=args)
    logical_camera = LogicalCamera()

    spinner = threading.Thread(target=rclpy.spin, args=(logical_camera,), daemon=True)
    spinner.start()

    rate = logical_camera.create_rate(10)

    while rclpy.ok():
        output = subprocess.check_output(['bash', '-c', 'ign topic -n 1 -e -t /logical_camera'])
        target_pose = extract_target_pose_from_output(output.decode())
        if target_pose is not None:
            logical_camera.target_vec_pub.publish(target_pose)
        rate.sleep()

    rclpy.shutdown()
    spinner.join()


if __name__ == '__main__':
    main()
