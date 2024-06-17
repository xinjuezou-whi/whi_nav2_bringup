#!/usr/bin/env python3
# Copyright 2026 WheelHub Intelligent
#
# Interactive Mock Elevator Node for Multi-Floor AGV Testing

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool


class MockElevatorNode(Node):
    def __init__(self):
        super().__init__('mock_elevator_node')

        # Internal State Variables
        self.current_floor = "Floor_1"
        self.target_floor = "Floor_1"
        self.door_state = "CLOSED"  # OPEN, OPENING, CLOSING, CLOSED
        self.lift_state = "IDLE"    # IDLE, MOVING, ARRIVED
        self.auto_transit = False   # If True, moves automatically via timer; if False, manual control

        # Publishers
        self.status_pub = self.create_publisher(String, 'elevator/status', 10)
        self.door_pub = self.create_publisher(String, 'elevator/door_state', 10)

        # Subscribers for automatic or manual requests
        self.call_sub = self.create_subscription(
            String, 'elevator/call_floor', self.handle_call_floor, 10
        )
        self.manual_floor_sub = self.create_subscription(
            String, 'elevator/set_floor', self.handle_set_floor, 10
        )
        self.manual_lift_state_sub = self.create_subscription(
            String, 'elevator/set_lift_state', self.handle_set_lift_state, 10
        )
        self.manual_door_state_sub = self.create_subscription(
            String, 'elevator/set_door_state', self.handle_set_door_state, 10
        )

        # Services
        self.open_door_srv = self.create_service(
            Trigger, 'elevator/open_door', self.handle_open_door
        )
        self.close_door_srv = self.create_service(
            Trigger, 'elevator/close_door', self.handle_close_door
        )
        self.toggle_auto_srv = self.create_service(
            SetBool, 'elevator/set_auto_mode', self.handle_set_auto_mode
        )

        # Non-blocking transit timer
        self.transit_timer = None

        # State Publishing Timer (2 Hz)
        self.timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info(
            "Mock Elevator Node Initialized on Floor_1. [Manual Mode Active]"
        )

    def publish_status(self):
        msg_status = String()
        msg_status.data = f"FLOOR:{self.current_floor}|LIFT:{self.lift_state}|DOOR:{self.door_state}"
        self.status_pub.publish(msg_status)

        msg_door = String()
        msg_door.data = self.door_state
        self.door_pub.publish(msg_door)

    def handle_call_floor(self, msg):
        """Called when BT or AGV requests a floor transit."""
        requested_floor = msg.data
        self.get_logger().info(f"Received request to transition to: {requested_floor}")

        if requested_floor == self.current_floor and self.lift_state == "ARRIVED":
            self.get_logger().info(f"Elevator is already at {requested_floor}")
            return

        self.target_floor = requested_floor
        self.lift_state = "MOVING"
        self.door_state = "CLOSED"

        if self.auto_transit:
            # Non-blocking async timer for simulation mode
            if self.transit_timer:
                self.transit_timer.cancel()
            self.transit_timer = self.create_timer(4.0, self._on_arrival)
            self.get_logger().info("Auto mode enabled. Simulating 4s transit delay...")
        else:
            self.get_logger().info(
                f"Manual mode active. Elevator set to MOVING towards {requested_floor}. "
                f"Publish to 'elevator/set_floor' or 'elevator/set_lift_state' to simulate arrival."
            )

    def _on_arrival(self):
        """Internal callback for automatic mode completion."""
        self.current_floor = self.target_floor
        self.lift_state = "ARRIVED"
        self.door_state = "OPEN"
        self.get_logger().info(f"Elevator arrived at {self.current_floor}. Doors OPEN.")
        if self.transit_timer:
            self.transit_timer.cancel()
            self.transit_timer = None

    # --- MANUAL CONTROL HANDLERS ---

    def handle_set_floor(self, msg):
        """Manually force current floor."""
        self.current_floor = msg.data
        self.get_logger().info(f"Manual override: Floor set to '{self.current_floor}'")

    def handle_set_lift_state(self, msg):
        """Manually force lift status (e.g., IDLE, MOVING, ARRIVED)."""
        self.lift_state = msg.data.upper()
        self.get_logger().info(f"Manual override: Lift state set to '{self.lift_state}'")

    def handle_set_door_state(self, msg):
        """Manually force door status (e.g., OPEN, CLOSED)."""
        self.door_state = msg.data.upper()
        self.get_logger().info(f"Manual override: Door state set to '{self.door_state}'")

    def handle_set_auto_mode(self, request, response):
        """Toggle between pure manual control and non-blocking auto mode."""
        self.auto_transit = request.data
        mode_str = "AUTO" if self.auto_transit else "MANUAL"
        self.get_logger().info(f"Elevator mode changed to: {mode_str}")
        response.success = True
        response.message = f"Elevator mode set to {mode_str}"
        return response

    def handle_open_door(self, request, response):
        self.get_logger().info("Opening elevator doors...")
        self.door_state = "OPEN"
        response.success = True
        response.message = "Doors are OPEN"
        return response

    def handle_close_door(self, request, response):
        self.get_logger().info("Closing elevator doors...")
        self.door_state = "CLOSED"
        response.success = True
        response.message = "Doors are CLOSED"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockElevatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()