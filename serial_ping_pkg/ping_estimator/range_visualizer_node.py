#!/usr/bin/python3

import json
import math

import rclpy
from rclpy.node import Node

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from smarc_utilities.georef_utils import convert_latlon_to_utm
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


class ModemRangeVisualizerNode(Node):
    def __init__(self):
        super().__init__('modem_range_visualizer_node')

        self.declare_parameter(
            'range_observation_topic',
            '/modem_estimates/range_observation_json',
        )
        self.declare_parameter(
            'marker_topic',
            '/modem_estimates/rviz',
        )
        self.declare_parameter(
            'map_frame',
            'map',
        )
        self.declare_parameter(
            'draw_horizontal_circles',
            True,
        )
        self.declare_parameter(
            'draw_slant_spheres',
            False,
        )
        self.declare_parameter(
            'draw_ping_centers',
            True,
        )
        self.declare_parameter(
            'circle_segments',
            120,
        )

        self.range_observation_topic = self.get_parameter(
            'range_observation_topic'
        ).value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.map_frame = self.get_parameter('map_frame').value

        self.draw_horizontal_circles = bool(
            self.get_parameter('draw_horizontal_circles').value
        )
        self.draw_slant_spheres = bool(
            self.get_parameter('draw_slant_spheres').value
        )
        self.draw_ping_centers = bool(
            self.get_parameter('draw_ping_centers').value
        )
        self.circle_segments = int(self.get_parameter('circle_segments').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10,
        )

        self.create_subscription(
            String,
            self.range_observation_topic,
            self._range_cb,
            10,
        )

        self.marker_count = 0

        self.ref_lat = None
        self.ref_lon = None

        self.get_logger().info(
            f"range visualizer ready: input={self.range_observation_topic}, "
            f"markers={self.marker_topic}, map_frame={self.map_frame}"
        )

    def _next_id(self) -> int:
        self.marker_count += 1
        return self.marker_count

    def _range_cb(self, msg: String):
        try:
            obs = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"bad range observation JSON: {e}")
            return

        center = self._latlon_alt_to_map_point(
            obs["center_latitude"],
            obs["center_longitude"],
            obs["center_altitude_m"],
        )

        if center is None:
            return

        markers = []

        if self.draw_ping_centers:
            markers.append(self._make_ping_center_marker(obs, center))

        if self.draw_horizontal_circles:
            circle = self._make_horizontal_circle_marker(obs, center)
            if circle is not None:
                markers.append(circle)

        if self.draw_slant_spheres:
            markers.append(self._make_slant_sphere_marker(obs, center))

        if markers:
            self.marker_pub.publish(MarkerArray(markers=markers))

    def _latlon_alt_to_map_point(self, lat: float, lon: float, alt: float) -> Point | None:
        if self.ref_lat is None or self.ref_lon is None:
            self.ref_lat = float(lat)
            self.ref_lon = float(lon)
            self.get_logger().info(
                f"fake local origin set: lat={self.ref_lat:.8f}, lon={self.ref_lon:.8f}"
            )

        lat = float(lat)
        lon = float(lon)

        meters_per_deg_lat = 111_320.0
        meters_per_deg_lon = 111_320.0 * math.cos(math.radians(self.ref_lat))

        p = Point()
        p.x = (lon - self.ref_lon) * meters_per_deg_lon
        p.y = (lat - self.ref_lat) * meters_per_deg_lat
        p.z = float(alt)

        return p

    def _make_ping_center_marker(self, obs: dict, center: Point) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = f"modem_{obs['modem_id']}_ping_centers"
        marker.id = self._next_id()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = center
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.9

        return marker

    def _make_slant_sphere_marker(self, obs: dict, center: Point) -> Marker:
        r = float(obs["slant_range_m"])

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = f"modem_{obs['modem_id']}_slant_spheres"
        marker.id = self._next_id()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = center
        marker.pose.orientation.w = 1.0

        marker.scale.x = 2.0 * r
        marker.scale.y = 2.0 * r
        marker.scale.z = 2.0 * r

        marker.color.r = 0.5
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.06

        return marker

    def _make_horizontal_circle_marker(self, obs: dict, center: Point) -> Marker | None:
        if not obs.get("horizontal_radius_valid", False):
            return None

        if obs.get("remote_depth_m") is None:
            return None

        rho = float(obs["horizontal_radius_m"])

        # Because center.z is own modem z in map frame, approximate target z by
        # shifting with the known depth difference.
        own_depth_m = float(obs["own_depth_m"])
        remote_depth_m = float(obs["remote_depth_m"])
        dz_map = -(remote_depth_m - own_depth_m)
        z_circle = center.z + dz_map

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = f"modem_{obs['modem_id']}_range_circles"
        marker.id = self._next_id()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25

        marker.color.r = 1.0
        marker.color.g = 0.8
        marker.color.b = 0.0
        marker.color.a = 0.85

        n = max(12, self.circle_segments)

        for k in range(n + 1):
            a = 2.0 * math.pi * float(k) / float(n)

            p = Point()
            p.x = center.x + rho * math.cos(a)
            p.y = center.y + rho * math.sin(a)
            p.z = z_circle

            marker.points.append(p)

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = ModemRangeVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()