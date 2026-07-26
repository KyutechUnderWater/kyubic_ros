#!/usr/bin/env python3
"""Estimate pinger yaw and pitch from six hydrophones.

This is the direction-estimation portion of sbl-ros-action_2.py only.  It
does not start a ROS Action, command the vehicle, or publish WrenchPlan.
"""

import time

import numpy as np
import pywt
import rclpy
import scipy.optimize as optimize
import scipy.signal as signal_tools
import serial
from rclpy.node import Node
from scipy.optimize import Bounds

from planner_msgs.msg import PingerDirection


class SblDirectionEstimator(Node):
    """Read hydrophones over serial and publish yaw/pitch estimates."""

    def __init__(self) -> None:
        super().__init__('sbl_direction_estimator')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pinger_fq', 21164.0),
                ('pinger_interval', 0.25),
                ('sampling_rate', 200000),
                ('sampling_channel', 6),
                ('sound_velocity', 1500.0),
                ('threshold_rising', 20.0),
                ('x_range', [-10.0, 10.0]),
                ('y_range', [-10.0, 10.0]),
                ('z_range', [0.0, 10.0]),
                ('num_trials', 1),
                ('serial_port', '/dev/ttyACM0'),
                ('baudrate', 480000000),
                ('hydrophone_pos', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('channel_weights', [0.0, 1.0, 1.0, 1.0, 1.0, 0.0]),
                ('topic_name', '/pinger_direction'),
            ],
        )

        self.pinger_fq = float(self.get_parameter('pinger_fq').value)
        self.interval = float(self.get_parameter('pinger_interval').value)
        self.sampling_rate = int(self.get_parameter('sampling_rate').value)
        self.channel_count = int(self.get_parameter('sampling_channel').value)
        self.sound_velocity = float(self.get_parameter('sound_velocity').value)
        self.threshold_rising = float(self.get_parameter('threshold_rising').value)
        self.num_trials = int(self.get_parameter('num_trials').value)
        self.channel_weights = np.asarray(
            self.get_parameter('channel_weights').value, dtype=float
        )
        self.x_range = self.get_parameter('x_range').value
        self.y_range = self.get_parameter('y_range').value
        self.z_range = self.get_parameter('z_range').value
        self.sample_count = int(2 * self.interval * self.sampling_rate)

        flat_positions = np.asarray(self.get_parameter('hydrophone_pos').value, dtype=float)
        expected_position_values = self.channel_count * 3
        if flat_positions.size != expected_position_values:
            raise ValueError(
                f'hydrophone_pos needs {expected_position_values} values '
                f'({self.channel_count} channels x 3 axes), got {flat_positions.size}'
            )
        if self.channel_weights.size != self.channel_count:
            raise ValueError('channel_weights must have one value per channel')
        self.hydrophone_pos = flat_positions.reshape(self.channel_count, 3)
        self.freqs = np.array([self.pinger_fq - 5000, self.pinger_fq, self.pinger_fq + 5000])

        self.publisher = self.create_publisher(
            PingerDirection, str(self.get_parameter('topic_name').value), 10
        )
        try:
            self.serial_port = serial.Serial(
                str(self.get_parameter('serial_port').value),
                baudrate=int(self.get_parameter('baudrate').value),
                timeout=5.0,
            )
        except serial.SerialException as error:
            self.get_logger().error(f'Could not open serial port: {error}')
            raise

        self.timer = self.create_timer(self.interval, self.estimate_and_publish)
        self.get_logger().info('SBL yaw/pitch estimator is ready.')

    def estimate_and_publish(self) -> None:
        started = time.perf_counter()
        expected_bytes = 2 * self.sample_count * self.channel_count
        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(self.sample_count.to_bytes(4, byteorder='little'))
            raw_bytes = self.serial_port.read(expected_bytes)
        except serial.SerialException as error:
            self.get_logger().error(f'Serial communication error: {error}')
            return

        if len(raw_bytes) != expected_bytes:
            self.get_logger().warning(
                f'Incomplete sample: {len(raw_bytes)}/{expected_bytes} bytes received.'
            )
            return

        interleaved = np.frombuffer(raw_bytes, dtype=np.int16)
        channels = [interleaved[index::self.channel_count] for index in range(self.channel_count)]

        try:
            yaw, pitch, score = self.estimate_direction(channels)
        except (ValueError, FloatingPointError) as error:
            self.get_logger().warning(f'Direction estimation failed: {error}')
            return

        msg = PingerDirection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'hydrophone_array_link'
        msg.yaw = float(yaw)
        msg.pitch = float(pitch)
        msg.score = float(score)
        self.publisher.publish(msg)
        elapsed = time.perf_counter() - started
        self.get_logger().info(
            f'Yaw: {yaw:.1f} deg, Pitch: {pitch:.1f} deg, '
            f'Score: {score:.3f}, Time: {elapsed:.3f}s'
        )

    def estimate_direction(self, channels):
        centered = [channel - np.mean(channel) for channel in channels]
        filtered = self.wavelet_transform(centered)
        envelopes = np.asarray([np.abs(signal_tools.hilbert(item)) ** 2 for item in filtered])
        rising_indices = self.rising_indices(envelopes)
        pair_delays = [
            (rising_indices[j] - rising_indices[i]) / self.sampling_rate
            for i in range(self.channel_count - 1)
            for j in range(i + 1, self.channel_count)
        ]
        position, score = self.estimate_position(pair_delays)

        # yaw is the horizontal bearing.  pitch is elevation from the horizontal
        # plane, so it approaches +/-90 degrees when the pinger is directly
        # above or below the array.
        yaw = np.degrees(np.arctan2(position[1], position[0]))
        pitch = np.degrees(np.arctan2(position[2], np.hypot(position[0], position[1])))
        return yaw, pitch, score

    def wavelet_transform(self, channels):
        wavelet = 'cmor2-1.0'
        scales = 1 / (self.freqs / self.sampling_rate)
        transformed = []
        for channel in channels:
            cwt, _ = pywt.cwt(channel, scales=scales, wavelet=wavelet, method='fft')
            transformed.append(cwt[1])  # Select the pinger's centre frequency.
        return transformed

    def rising_indices(self, envelopes):
        indices = []
        for envelope in envelopes:
            threshold = float(np.max(envelope)) * self.threshold_rising / 100.0
            candidates = np.flatnonzero(envelope >= threshold)
            if candidates.size == 0:
                raise ValueError('Pinger signal was not detected')
            detection_index = int(candidates[0])

            # action_2 performs its rising-time detection on the envelope.
            # An envelope is non-negative, so its first threshold crossing is
            # the effective arrival index used for the pairwise delays.
            indices.append(detection_index)
        return indices

    def estimate_position(self, observed_delays):
        bounds = Bounds(
            [self.x_range[0], self.y_range[0], self.z_range[0]],
            [self.x_range[1], self.y_range[1], self.z_range[1]],
        )
        best_score = float('inf')
        best_position = None
        initial_positions = np.random.uniform(
            low=bounds.lb, high=bounds.ub, size=(self.num_trials, 3)
        )
        for initial_position in initial_positions:
            result = optimize.minimize(
                self.direction_error,
                x0=initial_position,
                args=(observed_delays,),
                method='Powell',
                bounds=bounds,
                options={'maxiter': 50},
            )
            if result.fun < best_score:
                best_score = float(result.fun)
                best_position = result.x
        if best_position is None:
            raise ValueError('No position estimate was produced')
        return best_position, best_score

    def direction_error(self, source_position, observed_delays):
        expected_angles = []
        observed_angles = []
        weights = []
        pair_index = 0
        for i in range(self.channel_count - 1):
            for j in range(i + 1, self.channel_count):
                baseline = np.linalg.norm(self.hydrophone_pos[j] - self.hydrophone_pos[i])
                range_i = np.linalg.norm(source_position - self.hydrophone_pos[i])
                range_j = np.linalg.norm(source_position - self.hydrophone_pos[j])
                expected_delay = (range_j - range_i) / self.sound_velocity
                expected_angles.append(np.arcsin(np.clip(self.sound_velocity * expected_delay / baseline, -1, 1)))
                observed_angles.append(np.arcsin(np.clip(
                    self.sound_velocity * observed_delays[pair_index] / baseline, -1, 1
                )))
                weights.append(self.channel_weights[i] * self.channel_weights[j])
                pair_index += 1
        return float(np.sum(np.abs(np.asarray(expected_angles) - np.asarray(observed_angles)) * weights))

    def destroy_node(self) -> bool:
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SblDirectionEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
