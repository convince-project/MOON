#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import random


random.seed(0)


class ObjectPickSimulator(Node):
    def __init__(self):
        super().__init__('object_pick_simulator')

        self.action_pub = self.create_publisher(String, 'action_status', 10)
        self.grip_pub = self.create_publisher(Float32, 'gripper_strength', 10)

        self.action_timer = self.create_timer(1.0, self.update_action)
        self.gripper_timer = self.create_timer(0.02, self.update_gripper)

        self.t_pick = 1.0
        self.t_move = 3.0
        self.t_place = 8.0
        self.t_end = 10.0

        self.pick_change_offset = 0.8
        self.pick_change_duration = 0.4
        self.move_drop_offset = 2.0
        self.move_drop_duration = 0.2

        self.noise_min = -1.0
        self.noise_max = 1.0

        self.start_time = self.get_clock().now()
        self.current_action = "IDLE"
        self.last_action_change_time = self.start_time
        self.gripper_strength = 0.0
        self.sequence_finished = False

        self.get_logger().info("Object pick simulator started.")


    def update_action(self):
        if self.sequence_finished:
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        prev_action = self.current_action

        if elapsed < self.t_pick:
            self.current_action = "IDLE"
        elif elapsed < self.t_move:
            self.current_action = "PICK"
        elif elapsed < self.t_place:
            self.current_action = "MOVE"
        elif elapsed < self.t_end:
            self.current_action = "PLACE"
        else:
            self.current_action = "IDLE"
            self.sequence_finished = True

        if self.current_action != prev_action:
            self.last_action_change_time = now
            self.publish_action()

        if self.sequence_finished:
            self.publish_action()
            self.get_logger().info("Sequence completed. Shutting down.")
            rclpy.shutdown()


    def publish_action(self):
        msg = String()
        msg.data = self.current_action
        self.action_pub.publish(msg)
        self.get_logger().info(f"[ACTION] {self.current_action}")


    def update_gripper(self):
        if self.sequence_finished:
            return

        now = self.get_clock().now()
        time_in_action = (now - self.last_action_change_time).nanoseconds / 1e9


        if self.current_action == "PICK":
            if time_in_action < self.pick_change_offset:
                strength = 0.0
            elif time_in_action < self.pick_change_offset + self.pick_change_duration:
                phase = (time_in_action - self.pick_change_offset) / self.pick_change_duration
                strength = 100.0 * min(max(phase, 0.0), 1.0)
            else:
                strength = 100.0
        elif self.current_action == "MOVE":
            if time_in_action < self.move_drop_offset:
                strength = 100.0
            elif time_in_action < self.move_drop_offset + self.move_drop_duration:
                phase = (time_in_action - self.move_drop_offset) / self.move_drop_duration
                strength = 100.0 * (1 - min(max(phase, 0.0), 1.0))
            else:
                strength = 0.0
        elif self.current_action == "PLACE":
            strength = 0.0
        else:
            strength = 0.0


        noise = random.uniform(self.noise_min, self.noise_max)
        noisy_strength = strength + noise
        noisy_strength = max(0.0, min(100.0, noisy_strength))

        self.gripper_strength = noisy_strength
        msg = Float32()
        msg.data = float(self.gripper_strength)
        self.grip_pub.publish(msg)

        self.get_logger().info(f"[STRENGTH] {self.gripper_strength:.1f}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPickSimulator()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
