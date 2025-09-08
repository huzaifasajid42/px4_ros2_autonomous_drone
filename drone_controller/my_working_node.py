# I have taken the liberty of commenting out important aspects our misison. Please let me know if you require any further clarifications.

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
from enum import auto, Enum
import math
import time

class DroneState(Enum):
    DISARMED = auto()
    SETPOINT_STREAMING = auto()
    ARMING = auto()
    ARMING_WAIT = auto()
    ARMED = auto()
    TAKEOFF = auto()
    MISSION = auto()
    LANDING = auto()
    LANDED = auto()
    AUTO_LAND_SWITCH = auto()

class PositionMissionDrone(Node):
    def __init__(self):
        super().__init__('position_mission_drone')

        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)

        # Publisher
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Mission variables
        self.current_state = DroneState.DISARMED
        self.state_start_time = time.time()
        self.waypoint_start_time = time.time()
        self.current_waypoint_index = 0
        self.current_pose = None
        self.current_position = [0.0, 0.0, 0.0]
        self.pose_received = False
        self.waypoint_arrival_time = None

        # Waypoints
        self.waypoints = [
            (0.0, 0.0, 5.0),   # Takeoff
            (5.0, 0.0, 5.0),   # Move East
            (5.0, 5.0, 5.0),   # Move North
            (0.0, 5.0, 5.0),   # Move West
            (0.0, 0.0, 5.0)    # Return to home
        ]

        # Control parameters
        self.tolerance = 1.0
        self.takeoff_timeout = 10.0
        self.waypoint_timeout = 12.0
        self.waypoint_hold_time = 3.0  # seconds to hold at each waypoint

        # Starting streaming setpoints as it is mandatory for our offboard mode.
        self.setpoint_timer = self.create_timer(0.02, self.publish_setpoints)
        self.get_logger().info("Position control drone initialized")

    def pose_callback(self, msg):
        self.current_pose = msg
        self.current_position = [
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z)
        ]
        self.pose_received = True

    def state_callback(self, msg):
        current_time = time.time()
        self.mavros_state = msg

        if self.current_state == DroneState.DISARMED and msg.connected:
            self.get_logger().info("Connected to MAVROS")
            self.current_state = DroneState.SETPOINT_STREAMING
            self.state_start_time = current_time

        elif self.current_state == DroneState.SETPOINT_STREAMING:
            if current_time - self.state_start_time > 3.0:
                self.arm_drone()
                self.current_state = DroneState.ARMING
                self.state_start_time = current_time

        elif self.current_state == DroneState.ARMING:
            if msg.armed:
                self.set_offboard_mode()
                self.current_state = DroneState.ARMING_WAIT
                self.state_start_time = current_time
            elif current_time - self.state_start_time > 5.0:
                self.arm_drone()
                self.state_start_time = current_time

        elif self.current_state == DroneState.ARMING_WAIT:
            if msg.mode == "OFFBOARD":
                self.current_state = DroneState.ARMED
                self.state_start_time = current_time
            elif current_time - self.state_start_time > 5.0:
                self.set_offboard_mode()
                self.state_start_time = current_time

        elif self.current_state == DroneState.ARMED:
            if current_time - self.state_start_time > 1.0:
                self.current_state = DroneState.TAKEOFF
                self.state_start_time = current_time
                self.waypoint_start_time = current_time

    def publish_setpoints(self):
        current_time = time.time()
        pos_msg = PoseStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = "map"
        pos_msg.pose.orientation.w = 1.0

        # Hold at origin before takeoff
        if self.current_state in [DroneState.SETPOINT_STREAMING, DroneState.ARMING, DroneState.ARMING_WAIT, DroneState.ARMED]:
            pos_msg.pose.position.x = 0.0
            pos_msg.pose.position.y = 0.0
            pos_msg.pose.position.z = 0.0

        # Takeoff logic
        elif self.current_state == DroneState.TAKEOFF:
            target = self.waypoints[0]
            pos_msg.pose.position.x, pos_msg.pose.position.y, pos_msg.pose.position.z = target

            if self.pose_received and abs(self.current_position[2] - target[2]) < 0.6:
                self.current_state = DroneState.MISSION
                self.waypoint_start_time = current_time
                self.waypoint_arrival_time = None
                self.get_logger().info("Takeoff complete! Starting mission")

            elif current_time - self.waypoint_start_time > self.takeoff_timeout:
                self.current_state = DroneState.MISSION
                self.get_logger().warn("Takeoff timeout - moving to mission")

        # Mission with hold + hysteresis
        elif self.current_state == DroneState.MISSION:
            if self.current_waypoint_index < len(self.waypoints):
                target = self.waypoints[self.current_waypoint_index]
                pos_msg.pose.position.x, pos_msg.pose.position.y, pos_msg.pose.position.z = target

                arrival_tolerance = 0.5
                reset_tolerance = 0.75
                dist = self.distance_to(target)

                if self.pose_received and dist <= arrival_tolerance:
                    if self.waypoint_arrival_time is None:
                        self.waypoint_arrival_time = current_time
                        self.get_logger().info(f"Arrived WP {self.current_waypoint_index} (dist={dist:.2f}), holding...")
                    else:
                        elapsed = current_time - self.waypoint_arrival_time
                        if elapsed >= self.waypoint_hold_time:
                            self.get_logger().info(f"WP {self.current_waypoint_index} held {self.waypoint_hold_time}s, moving on")
                            self.current_waypoint_index += 1
                            self.waypoint_start_time = current_time
                            self.waypoint_arrival_time = None
                else:
                    if dist > reset_tolerance:
                        if self.waypoint_arrival_time is not None:
                            self.get_logger().info(f"Drifted from WP {self.current_waypoint_index} (dist={dist:.2f}), reset hold")
                        self.waypoint_arrival_time = None

                # Safety timeout
                if current_time - self.waypoint_start_time > self.waypoint_timeout:
                    self.get_logger().warn(f"WP {self.current_waypoint_index} timeout, moving to next")
                    self.current_waypoint_index += 1
                    self.waypoint_start_time = current_time
                    self.waypoint_arrival_time = None

                if self.current_waypoint_index >= len(self.waypoints):
                    self.current_state = DroneState.LANDING
                    self.state_start_time = current_time
                    self.get_logger().info("Mission complete! Starting landing")

        # Landing handled directly by PX4 AUTO.LAND
        elif self.current_state == DroneState.LANDING:
            self.set_auto_land_mode()
            self.get_logger().info("Landing initiated. PX4 handling landing via AUTO.LAND")
            self.current_state = DroneState.LANDED

        elif self.current_state == DroneState.LANDED and self.mavros_state and not self.mavros_state.armed:
            self.get_logger().info("Drone has landed and disarmed. Shutting off our node")
            self.setpoint_timer.cancel()
            self.get_clock().call_later(0.1, rclpy.shutdown) # Our node willl shutdown once the drone is landed and disarmed.
        return 

        self.pos_pub.publish(pos_msg)

    def distance_to(self, target):
        if not self.current_pose or not self.pose_received:
            return float('inf')
        dx = self.current_pose.pose.position.x - target[0]
        dy = self.current_pose.pose.position.y - target[1]
        dz = self.current_pose.pose.position.z - target[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def is_at_position(self, target, tolerance):
        return self.distance_to(target) < tolerance

    def arm_drone(self, value=True):
        if not self.arming_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = value
        self.arming_client.call_async(req)

    def set_offboard_mode(self):
        if not self.set_mode_client.service_is_ready():
            return
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        self.set_mode_client.call_async(req)

    def set_auto_land_mode(self):
        if not self.set_mode_client.service_is_ready():
            return
        req = SetMode.Request()
        req.custom_mode = "AUTO.LAND"
        self.set_mode_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = PositionMissionDrone()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
