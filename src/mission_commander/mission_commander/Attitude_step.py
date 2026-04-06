#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from std_msgs.msg import Bool
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude
from px4_msgs.msg import VehicleAttitudeSetpoint

import time

from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    VehicleAttitudeSetpoint,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleCommandAck
)

class MinimalStepInput(Node):
    def __init__(self):
        super().__init__('minimal_step_input')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Publishers ---
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        self.attitude_pub = self.create_publisher(VehicleAttitudeSetpoint,'/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        self.command_pub = self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command', qos_profile)

        # --- Subscribers ---
        self.local_pos = VehicleLocalPosition()
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_profile)
        

        self.thrustattitude_sub = VehicleAttitudeSetpoint()
        self.create_subscription(VehicleAttitudeSetpoint, '/fmu/out/vehicle_attitude_setpoint', self.attitude_sub_callback, qos_profile)

        self.vehicle_status = VehicleStatus()
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)

        self.create_subscription(VehicleCommandAck,
                        '/fmu/out/vehicle_command_ack',
                        lambda msg: print("ACK:", msg.command, msg.result),
                        qos_profile)



        # --- State ---
        self.stage = 0
        self.start_time = time.time()
        self.hover_record = []

        # Timer @ 33 Hz
        self.timer = self.create_timer(0.03, self.loop)


    def position_callback(self, msg):
        self.local_pos = msg

    def attitude_sub_callback(self, msg):
        self.attitude_sub = msg

    def status_callback(self, msg):
        self.vehicle_status = msg

    # ------------------------------------------------------------
    # Helper: send attitude + thrust setpoint
    # ------------------------------------------------------------
    def send_attitude_setpoint(self, roll, pitch, yaw_rate, thrust):
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Euler angle commands
        msg.roll_body = roll
        msg.pitch_body = pitch
        msg.yaw_body = float('nan')
        msg.yaw_sp_move_rate = yaw_rate

        # Thrust in body NED frame (down is positive)
        msg.thrust_body = [0.0, 0.0, -float(thrust)]

        self.attitude_pub.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_publisher.publish(msg)

    # ------------------------------------------------------------
    # Helper: offboard heartbeat
    # ------------------------------------------------------------
    def send_heartbeat(self, position: bool = False, attitude:bool = True):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = position
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = attitude
        msg.body_rate = False
        msg.thrust_and_torque = False
        self.offboard_pub.publish(msg)

    # ------------------------------------------------------------
    # Helper: vehicle command
    # ------------------------------------------------------------
    def command(self, cmd, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = cmd
        msg.param1 = p1
        msg.param2 = p2
        msg.param3 = p3
        msg.param4 = p4
        msg.param5 = p5
        msg.param6 = p6
        msg.param7 = p7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    # ------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------
    def loop(self):
        if self.stage < 2:
            self.send_heartbeat(True, False)
        if self.stage >= 2:
            self.send_heartbeat(False, True)

        if self.stage == 0:
            self.publish_position_setpoint(0.0, 0.0, -2.0, 0.0)

            if time.time() - self.start_time > 0.5:
                # Switch to OFFBOARD
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

                # Arm
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

                if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                    print("Armed in OFFBOARD")
                    self.start_time = time.time()
                    self.stage = 1

        elif self.stage == 1:

            self.publish_position_setpoint(0.0, 0.0, -2.0, 0.0)

            if time.time() - self.start_time > 10.0:
                self.hover_record.append(self.attitude_sub.thrust_body[2])
                print(self.attitude_sub.thrust_body)
                if time.time() - self.start_time > 20.0:
                    self.start_time = time.time()
                    self.stage = 1.5


        elif self.stage == 1.5:
            self.publish_position_setpoint(0.0, 0.0, -2.0, 0.0)
            self.send_attitude_setpoint(0.0, 0.0, 0.0, 0.0)
            if time.time() - self.start_time > 10.0:
                self.start_time = time.time()
                self.hover_thrust = abs(sum(self.hover_record)/len(self.hover_record))
                self.stage = 2
                print("hover_thrust:", self.hover_thrust)

        # --- Stage 2: apply step input ---
        elif self.stage == 2:
            roll_step = np.deg2rad(10)
            self.send_attitude_setpoint(roll_step, 0.0, 0.0, self.hover_thrust)

            if time.time() - self.start_time > 3.0:
                self.stage = 3

        # --- Stage 3: land ---
        elif self.stage == 3:
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0)  # AUTO
            self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.stage = 4

        # --- Stage 4: done ---
        elif self.stage == 4:
            pass



def main(args=None):
    rclpy.init(args=args)
    node = MinimalStepInput()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
