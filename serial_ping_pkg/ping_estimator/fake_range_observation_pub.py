#!/usr/bin/python3

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FakeRangeObservationPublisher(Node):
    def __init__(self):
        super().__init__('fake_range_observation_publisher')

        self.declare_parameter(
            'range_observation_topic',
            '/modem_estimates/range_observation_json',
        )

        self.topic = self.get_parameter('range_observation_topic').value

        self.pub = self.create_publisher(String, self.topic, 10)

        # Use coordinates near your test area.
        # These are fake ping-center positions around roughly the same place.
        self.fake_centers = [
            (58.82348711944454, 17.634170710292903, 12.0),
            (58.82353711944454, 17.634170710292903, 10.5),
            (58.82348711944454, 17.634250710292903, 11.2),
            (58.82343711944454, 17.634170710292903, 13.0),
            (58.82348711944454, 17.634090710292903, 10.8),
        ]

        self.modem_id = '007'
        self.own_depth_m = 1.0
        self.remote_depth_m = 2.0
        self.sound_velocity_mps = 1500.0
        self.range_sigma_m = 1.4

        self.i = 0
        self.timer = self.create_timer(1.0, self._timer_cb)

        self.get_logger().info(f'Publishing fake range observations on {self.topic}')

    def _timer_cb(self):
        lat, lon, slant_range_m = self.fake_centers[self.i]

        dz = self.remote_depth_m - self.own_depth_m
        horizontal_sq = slant_range_m * slant_range_m - dz * dz

        horizontal_valid = horizontal_sq >= 0.0
        horizontal_radius_m = math.sqrt(horizontal_sq) if horizontal_valid else 0.0

        payload = {
            'stamp_sec': self.get_clock().now().nanoseconds * 1e-9,
            'modem_id': self.modem_id,

            # Center of the 3D acoustic range sphere.
            'center_latitude': lat,
            'center_longitude': lon,
            'center_altitude_m': -self.own_depth_m,

            # Depth convention: positive down.
            'own_depth_m': self.own_depth_m,
            'remote_depth_m': self.remote_depth_m,
            'remote_depth_known': True,

            # Actual sphere radius.
            'slant_range_m': slant_range_m,

            # Horizontal slice radius at remote modem depth.
            'horizontal_radius_m': horizontal_radius_m,
            'horizontal_radius_valid': horizontal_valid,

            'sound_velocity_mps': self.sound_velocity_mps,
            'range_sigma_m': self.range_sigma_m,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)

        self.get_logger().info(
            f'fake obs {self.i}: lat={lat:.8f}, lon={lon:.8f}, '
            f'slant={slant_range_m:.2f}m, horizontal={horizontal_radius_m:.2f}m'
        )

        self.i = (self.i + 1) % len(self.fake_centers)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRangeObservationPublisher()

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