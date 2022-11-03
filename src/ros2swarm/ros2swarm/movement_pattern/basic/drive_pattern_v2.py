#    Copyright 2020 Marian Begemann
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
from geometry_msgs.msg import Twist
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils import setup_node


# Import the ros messages
from nav_msgs.msg import Odometry
import numpy as np

class DrivePatternV2(MovementPattern):
    """
    A simple pattern for driving a constant direction vector.

    Which is configured in with the parameters of this pattern.
    How often the direction is published is configured in the timer period parameter.
    """

    def __init__(self):
        """Initialize the drive pattern."""
        super().__init__('drive_pattern')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('drive_timer_period', None),
                ('drive_linear', None),
                ('drive_angular', None),
                ('goal_x', None),
                ('goal_y', None),
            ])


        # Create a subscriber to odometry topic 
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        timer_period = float(
            self.get_parameter("drive_timer_period").get_parameter_value().double_value)
        #self.timer = self.create_timer(timer_period, self.swarm_command_controlled(self.timer_callback))
        self.timer = self.create_timer(timer_period, self.swarm_command_controlled_timer(self.timer_callback))
        self.i = 0
        self.param_x = float(self.get_parameter("drive_linear").get_parameter_value().double_value)
        self.param_z = float(self.get_parameter("drive_angular").get_parameter_value().double_value)
        self.param_goal_x = float(self.get_parameter("goal_x").get_parameter_value().double_value)
        self.param_goal_y = float(self.get_parameter("goal_y").get_parameter_value().double_value)

        self.pose_x = 0
        self.pose_y = 0
        self.pose_yaw = 0

        self.get_logger().warn('Logger is: ' + self.get_logger().get_effective_level().name)
        self.get_logger().info('Logger is: info ')
        self.get_logger().debug('Logger is: debug')

    def timer_callback(self):
        """Publish the configured twist message when called."""
        self.update_params()
        msg = Twist()
        # command to publish the message in the terminal by hand
        # ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{
        # linear: {x: 0.26, y: 0.0, z: 0.0},
        # angular: {x: 0.0, y: 0.0, z: 0.0}
        # }"
        msg.angular.z = 0.1*(self.pose_yaw - np.arctan((self.param_goal_y - self.pose_y)/(self.param_goal_x - self.pose_x)))

        if(np.abs(msg.angular.z) < 0.01):
            msg.linear.x = 0.1*((self.param_goal_x - self.pose_x)**2 + (self.param_goal_y - self.pose_y)**2)**0.5

        self.command_publisher.publish(msg)
        self.get_logger().debug('Publishing {}:"{}"'.format(self.i, msg))
        self.get_logger().info('Velocities {}:"{}"'.format(msg.linear.x, msg.angular.z))

    def update_params(self):
        self.param_x = float(self.get_parameter("drive_linear").get_parameter_value().double_value)
        self.param_z = float(self.get_parameter("drive_angular").get_parameter_value().double_value)
        self.param_goal_x = float(self.get_parameter("goal_x").get_parameter_value().double_value)
        self.param_goal_y = float(self.get_parameter("goal_y").get_parameter_value().double_value)

    # Define the odometry callback function
    def odom_callback(self, msg):

        # Get the position and orientation data
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Convert quaternion to euler
        (roll, pitch, yaw) = self.euler_from_quaternion (orientation)

        self.pose_x = position.x
        self.pose_y = position.y
        self.pose_yaw = yaw


        self.get_logger().info(f"Pose is {position.x}, {position.y}, {yaw}")

    # Convert the quaternion into euler angles
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    setup_node.init_and_spin(args, DrivePatternV2)


if __name__ == '__main__':
    main()
