#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node

from dae_relay_controller_ros.srv import (
    GetAllRelays,
    SetAllRelays,
    SetRelay,
)
import dae_RelayBoard
from dae_RelayBoard.dae_RelayBoard_Common import Denkovi_Exception


class RelayNode(Node):

    def __init__(self):
        super().__init__("relay_node")

        self.declare_parameter("device", "/dev/relay")
        self.declare_parameter("module_type", "type16")

        device = self.get_parameter("device").get_parameter_value().string_value
        module_type = self.get_parameter("module_type").get_parameter_value().string_value

        self.get_logger().info(f"Device type: {module_type}")
        self.get_logger().info(f"Device path/id: {device}")

        self.dr = dae_RelayBoard.DAE_RelayBoard(module_type)
        self.dr.initialise(device)
        self.dr.setAllStatesOff()

        self.lock = threading.Lock()

        self.set_relay_srv = self.create_service(
            SetRelay, "~/set_relay", self.set_callback
        )
        self.get_all_relays_srv = self.create_service(
            GetAllRelays, "~/get_all_relays", self.get_callback
        )
        self.set_all_relays_srv = self.create_service(
            SetAllRelays, "~/set_all_relays", self.set_all_callback
        )

        self.get_logger().info("Relay node started!")

    def set_callback(self, request, response):
        with self.lock:
            try:
                self.dr.setState(request.relay_number, request.state)
            except Denkovi_Exception as e:
                self.get_logger().error(f"Failed to set relay state: {e}")
                response.success = False
                return response

            response.success = True
            return response

    def get_callback(self, request, response):
        with self.lock:
            state = self.dr.getStates()
            state_arr = [False] * len(state)

            for tmp in state:
                state_arr[tmp - 1] = state[tmp]

            response.success = True
            response.states = state_arr
            return response

    def set_all_callback(self, request, response):
        with self.lock:
            is_ok = True
            self.dr.setAllStatesOn() if request.state else self.dr.setAllStatesOff()

            resp = self.dr.getStates()

            for tmp in resp:
                if resp[tmp] != request.state:
                    is_ok = False

            response.success = is_ok
            return response


def main(args=None):
    rclpy.init(args=args)
    relay_node = RelayNode()
    try:
        rclpy.spin(relay_node)
    except KeyboardInterrupt:
        pass
    finally:
        relay_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
