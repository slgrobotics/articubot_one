#
# See https://github.com/sushanthj/acorn_sim/blob/main/sim_ws/src/nav2_gps_waypoint_follower_demo/nav2_gps_waypoint_follower_demo/latlon_waypoint_follower.py
#

# Run this script:
# cd ~/robot_ws; colcon build; python3 install/articubot_one/lib/articubot_one/gps_waypoint_follower.py

import os
import time
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from robot_localization.srv import FromLL
from rclpy.node import Node
#from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        print("IP: reading file: " + wps_file_path)
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            print("Waypoint:   lat=" + str(latitude) + "  lon=" + str(longitude) + "  yaw=" + str(yaw))
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class GpsWpCommander(Node):
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self, wps_file_path):
        super().__init__('minimal_client_async')
        print("IP: initializing GpsWpCommander")
        self.navigator = BasicNavigator("basic_navigator") # Launch the ROS 2 Navigation Stack
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.localizer = self.create_client(FromLL,  '/fromLL')
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def start_wpf(self):
        """
        Function to start the waypoint following one by one.
        """
        print("IP: GpsWpCommander::start_wpf()")
        # Wait for navigation to fully activate. Use this line if autostart is set to true.
        # https://github.com/ros-navigation/navigation2/issues/2283
        #navigator.waitUntilNav2Active()  # must use AMCL, will wait for amcl/get_state service available
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')
        #self.navigator.waitUntilNav2Active(localizer='controller_server')
        wps = self.wp_parser.get_wps()

        for wp in wps:
            # Convert GPS coordinates to map frame using FromLL service
            self.req = FromLL.Request()
            self.req.ll_point.longitude = wp.position.longitude
            self.req.ll_point.latitude = wp.position.latitude
            self.req.ll_point.altitude = wp.position.altitude # -wp.position.altitude  - if you want alt=0

            log = 'long={:f}, lat={:f}, alt={:f}'.format(
                self.req.ll_point.longitude, self.req.ll_point.latitude, self.req.ll_point.altitude)
            self.get_logger().info(log)

            # Call the service and wait for the result
            self.future = self.localizer.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)

            if self.future.result() is None:
                self.get_logger().error("Failed to convert GPS to map coordinates")
                return

            # Create PoseStamped message for the current waypoint
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position = self.future.result().map_point
            pose.pose.orientation = wp.orientation

            log = 'x={:f}, y={:f}, z={:f}'.format(
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            self.get_logger().info(log)

            # Send the pose to the robot and wait until it reaches the waypoint
            self.navigator.goToPose(pose)

            while not self.navigator.isTaskComplete():
                time.sleep(0.1)

            result = self.navigator.getResult()
            # Do something depending on the return code
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

        self.get_logger().info("All waypoints completed successfully")


# ====================================================
#
# See https://github.com/sushanthj/acorn_sim/blob/main/sim_ws/src/nav2_gps_waypoint_follower_demo/nav2_gps_waypoint_follower_demo/utils/gps_utils.py
#

import math
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Quaternion


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def euler_from_quaternion(q: Quaternion):
    """
    Convert a quaternion into euler angles
    taken from: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def latLonYaw2Geopose(latitude: float, longitude: float, yaw: float = 0.0) -> GeoPose:
    """
    Creates a geographic_msgs/msg/GeoPose object from latitude, longitude and yaw
    """
    geopose = GeoPose()
    geopose.position.latitude = latitude
    geopose.position.longitude = longitude
    geopose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    return geopose

# ====================================================

def main(wpt_file_name):

    # Start the ROS 2 Python Client Library
    rclpy.init()

    default_yaml_file_path = os.path.join(get_package_share_directory(
        "articubot_one"), "config", wpt_file_name + ".yaml")
    yaml_file_path = default_yaml_file_path

    print("Waypoints file: " + yaml_file_path)

    gps_wpf = GpsWpCommander(yaml_file_path)
    gps_wpf.start_wpf()

    #navigator.lifecycleShutdown()

    exit(0)

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='GPS Waypoints Follower')
    parser.add_argument('--file',
                        metavar='.yaml',
                        default=('sim_waypoints'),
                        required=False,
                        help='file name containing waypoints (e.g "sim_waypoints")')
    args = parser.parse_args()
    main(wpt_file_name=args.file)

