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
from sensor_msgs.msg import LaserScan
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

        self.namespace = self.get_namespace()[1:]
        self.get_logger().info(f"Namespace {self.namespace}")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('drive_timer_period', None),
                ('drive_linear', None),
                ('drive_angular', None),
                (f'{self.namespace}_goal_x', None),
                (f'{self.namespace}_goal_y', None),
                *[(f"robot_namespace_{i}_goal_x", None) for i in range(5)],
                *[(f"robot_namespace_{i}_goal_y", None) for i in range(5)],
                ('goals', None),
            ])


        # Create a subscriber to odometry topic 
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        # Create a subscriber to scan topic 
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_policy)

        timer_period = float(
            self.get_parameter("drive_timer_period").get_parameter_value().double_value)
        #self.timer = self.create_timer(timer_period, self.swarm_command_controlled(self.timer_callback))
        self.timer = self.create_timer(timer_period, self.swarm_command_controlled_timer(self.timer_callback))
        self.i = 0

        self.distance_tolerance = 0.1
        self.angle_tolerance = 0.1
        self.K_a = 0.5
        self.K_l = 0.5
        self.go_to_waypoint = True
        self.velocity_cmd = Twist()

        self.param_x = float(self.get_parameter("drive_linear").get_parameter_value().double_value)
        self.param_z = float(self.get_parameter("drive_angular").get_parameter_value().double_value)
        self.param_goal_x = float(self.get_parameter(f"{self.namespace}_goal_x").get_parameter_value().double_value)
        self.param_goal_y = float(self.get_parameter(f"{self.namespace}_goal_y").get_parameter_value().double_value)
        self.goals_list = []
        self.goal = None

        for i in range(5):
            x = float(self.get_parameter(f"robot_namespace_{i}_goal_x").get_parameter_value().double_value)
            y = float(self.get_parameter(f"robot_namespace_{i}_goal_y").get_parameter_value().double_value)
            self.get_logger().info(f'Goals x {self.param_goal_x} and y {self.param_goal_y}')
            self.goals_list.append([x, y])

        self.pose_x = 0
        self.pose_y = 0
        self.pose_yaw = 0

        self.get_logger().warn('Logger is: ' + self.get_logger().get_effective_level().name)
        self.get_logger().info('Logger is: info ')
        self.get_logger().debug('Logger is: debug')

    def get_distance_to_waypoint(self, goal_x, goal_y):
        return np.sqrt((goal_x - self.pose_x)**2 + (goal_y - self.pose_y)**2)

    def get_heading_error(self, goal_x, goal_y):
        deltaX = goal_x - self.pose_x
        deltaY = goal_y - self.pose_y
        waypoint_heading = np.arctan2(deltaY, deltaX)
        heading_error = waypoint_heading - self.pose_yaw

        if(heading_error > np.pi):
            heading_error = heading_error - (2*np.pi)
        if(heading_error < -np.pi):
            heading_error = heading_error + (2*np.pi)

        return heading_error
        
    def set_velocity(self, goal_x, goal_y):
        distance_to_waypoint = self.get_distance_to_waypoint(goal_x, goal_y)
        heading_error = self.get_heading_error(goal_x, goal_y)

        if(self.go_to_waypoint == True and np.abs(distance_to_waypoint) > self.distance_tolerance):
            if(np.abs(heading_error) > self.angle_tolerance):
                self.velocity_cmd.linear.x = 0.0
                self.velocity_cmd.angular.z = self.K_a*heading_error
            else:
                self.velocity_cmd.linear.x = self.K_l*distance_to_waypoint
                self.velocity_cmd.angular.z = 0.0
        else:
            self.get_logger().info('Goal has been reached!')
            self.velocity_cmd.linear.x = 0.0
            self.velocity_cmd.angular.z = 0.0
            self.go_to_waypoint = False
        
    def timer_callback(self):
        """Publish the configured twist message when called."""
        self.update_params()
        goal = self.get_goal()
        self.set_velocity(goal[0], goal[1])

        self.command_publisher.publish(self.velocity_cmd)

    def update_params(self):
        self.param_x = float(self.get_parameter("drive_linear").get_parameter_value().double_value)
        self.param_z = float(self.get_parameter("drive_angular").get_parameter_value().double_value)
        self.param_goal_x = float(self.get_parameter(f"{self.namespace}_goal_x").get_parameter_value().double_value)
        self.param_goal_y = float(self.get_parameter(f"{self.namespace}_goal_y").get_parameter_value().double_value)

        self.get_logger().info(f"Param Goal X {self.param_goal_x}")
        self.get_logger().info(f"Param Goal Y {self.param_goal_y}")

    def get_goal(self):
        min_distance = 1000
        final_goal = None
        for goal in self.goals_list:
            if(self.get_distance_to_waypoint(goal[0], goal[1]) < min_distance):
                min_distance = self.get_distance_to_waypoint(goal[0], goal[1])
                final_goal = goal
        return final_goal

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


        # self.get_logger().info(f"Pose is {position.x}, {position.y}, {yaw}")

    def scan_callback(self, msg):
        print('Scan Data: ')
        points = []
        print("Range in front", np.mean(msg.ranges[0]))
        for deg, range in enumerate(msg.ranges):
            points.append([np.cos(np.deg2rad(deg))*range, np.sin(np.deg2rad(deg))*range, 1])

    def get_transformation_matrix(self):
        yaw = np.deg2rot(self.pose_yaw)

        transformation_matrix = [[np.cos(yaw), -np.sin(yaw), self.pose_x], [np.sin(yaw), np.cos(yaw), self.pose_y], [0, 0, 1]]
        return transformation_matrix

    def apply_transformation(self, points, transformation_matrix):


    def clustering(self, points):
        th = 0.01
        clusters = [[points[0]]]
        for point in points[1:]:
            for cluster in clusters:
                new_points_to_cluster = []
                for cluster_point in cluster:
                    if(self.get_distance_bw_2_points(point, cluster_point) <= th):
                        new_points_to_cluster.append(point)
                cluster.extend(new_points_to_cluster)
        print(clusters)


    def get_distance_bw_2_points(self, a, b):
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

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
