#!/usr/bin/env python3
import rclpy
import time
import openrouteservice
from rclpy.node import Node
from openrouteservice import convert
from sensor_msgs.msg import NavSatFix
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose

class GpsWaypointCommander(Node):
    def __init__(self):
        super().__init__('gps_waypoint_commander')
        self.navigator = BasicNavigator("basic_navigator")
        self.gps_subscriber = self.create_subscription(
            NavSatFix, 'gps_coordinates', self.gps_callback, 10)

        self.waypoints = None
        self.API_KEY = 'YOUR_API_KEY'

    def gps_callback(self, msg):
        self.get_waypoints_from_coordinates(msg.latitude, msg.longitude)
        self.wp_callback()

    def get_waypoints_from_coordinates(self, latitude, longitude):
        try:
            coords = ((longitude, latitude))
            client = openrouteservice.Client(key=self.API_KEY)
            geometry = client.directions(coords)['routes'][0]['geometry']
            decoded = convert.decode_polyline(geometry)

            waypoints = []
            for waypoint in decoded['coordinates']:
                lat, lon = waypoint
                waypoints.append(
                    {"latitude": lat, "longitude": lon, "yaw": 0.0})
            self.waypoints = waypoints  # Store waypoints
        except Exception as e:
            self.get_logger().error(f"Error while getting waypoints: {str(e)}")
            # Consider handling errors here

    def wp_callback(self):
        if self.waypoints is not None:
            self.navigator.waitUntilNav2Active(localizer='robot_localization')

            wps = []
            for waypoint in self.waypoints:
                lat = waypoint['latitude']
                lon = waypoint['longitude']
                wps.append(latLonYaw2Geopose(lat, lon, 0.0))

            self.navigator.followGpsWaypoints(wps)
            while (not self.navigator.isTaskComplete()):
                time.sleep(0.1)
            self.get_logger().info("Waypoints completed successfully")
            self.waypoints = None  # Clear waypoints after completion


def main():
    rclpy.init()
    gps_wp_commander = GpsWaypointCommander()
    rclpy.spin(gps_wp_commander)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
