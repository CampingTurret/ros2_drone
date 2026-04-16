#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from std_msgs.msg import Bool
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude
from px4_msgs.msg import VehicleAttitudeSetpoint
import csv 
import datetime
import os
import glob

import time

from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    VehicleAttitudeSetpoint,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleCommandAck,
    ActuatorMotors
)

class MinimalStepInput(Node):
    def __init__(self, run_name = "Default", step_amplitude=10.0, step_axis='roll', hover_thrust=0.8):
        super().__init__('minimal_step_input')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.run_name = run_name
        self.step_amplitude = step_amplitude
        self.step_axis = step_axis
        self.hover_thrust = hover_thrust
        # --- Publishers ---
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        self.attitude_pub = self.create_publisher(VehicleAttitudeSetpoint,'/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        self.command_pub = self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command', qos_profile)

        # --- Subscribers ---
        self.local_pos = VehicleLocalPosition()
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_profile)
        

        self.motors_sub = ActuatorMotors()
        self.create_subscription(ActuatorMotors, '/fmu/out/actuator_motors', self.motors_callback, qos_profile)

        self.vehicle_status = VehicleStatus()
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)

        self.create_subscription(VehicleCommandAck,
                        '/fmu/out/vehicle_command_ack',
                        lambda msg: print("ACK:", msg.command, msg.result),
                        qos_profile)
        

        # --- Ground truth holders ---


        self.att_gt = VehicleAttitude()
        self.pos_gt = VehicleLocalPosition()

        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.att_gt_callback,
            qos_profile
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.pos_gt_callback,
            qos_profile
        )

        # --- Logger ---
        rint = np.random.randint(0, 100)

        log_dir = "/root/logs"

        # Remove only CSV files
        for f in glob.glob(os.path.join(log_dir, "*.csv")):
            os.remove(f)

        # Now create your new log file
        self.logfile = open(f"{log_dir}/{run_name}.csv", "w", newline="")
        self.logger = csv.writer(self.logfile)

        self.logger.writerow([
            "t",
            "roll_cmd", "pitch_cmd", "yaw_rate_cmd", "thrust_cmd",
            "roll_gt", "pitch_gt", "yaw_gt",
            "x_gt", "y_gt", "z_gt",
            "vx_gt", "vy_gt", "vz_gt",
            "ax_gt", "ay_gt", "az_gt",
            "stage"
        ])


        # --- State ---
        self.stage = 0
        self.start_time = time.time()
        self.hover_record = []

        # Timer @ 33 Hz
        self.timer = self.create_timer(0.03, self.loop)


        self.last_roll_cmd = np.nan
        self.last_pitch_cmd = np.nan
        self.last_yaw_rate_cmd = np.nan
        self.last_thrust_cmd = np.nan



    def position_callback(self, msg):
        self.local_pos = msg

    def motors_callback(self, msg):
        self.motors_sub = msg

    def status_callback(self, msg):
        self.vehicle_status = msg

    def att_gt_callback(self, msg):
        self.att_gt = msg

    def pos_gt_callback(self, msg):
        self.pos_gt = msg

    def quat_to_euler(self, q):
        w, x, y, z = q
        sinr = 2.0 * (w*x + y*z)
        cosr = 1.0 - 2.0 * (x*x + y*y)
        roll = np.arctan2(sinr, cosr)

        sinp = 2.0 * (w*y - z*x)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

        siny = 2.0 * (w*z + x*y)
        cosy = 1.0 - 2.0 * (y*y + z*z)
        yaw = np.arctan2(siny, cosy)

        return roll, pitch, yaw

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

        self.last_roll_cmd = roll
        self.last_pitch_cmd = pitch
        self.last_yaw_rate_cmd = yaw_rate
        self.last_thrust_cmd = thrust

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
            self.publish_position_setpoint(0.0, 0.0, -5.0, 0.0)

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
            self.publish_position_setpoint(0.0, 0.0, -5.0, 0.0)
            if time.time() - self.start_time > 20.0:
                self.start_time = time.time()
                self.stage = 1.5


        elif self.stage == 1.5:
            self.publish_position_setpoint(0.0, 0.0, -5.0, 0.0)
            self.send_attitude_setpoint(0.0, 0.0, 0.0, self.hover_thrust)
            if time.time() - self.start_time > 10.0:
                self.start_time = time.time()
                self.stage = 2

        # --- Stage 2: apply step input ---
        elif self.stage == 2:
            roll_step = np.deg2rad(10)
            print(self.motors_sub.control)
            if self.step_axis == "roll":
                self.send_attitude_setpoint(np.deg2rad(self.step_amplitude), 0.0, 0.0, self.hover_thrust)
            elif self.step_axis == "pitch":
                self.send_attitude_setpoint(0.0, np.deg2rad(self.step_amplitude), 0.0, self.hover_thrust)
            elif self.step_axis == "yaw_rate":
                self.send_attitude_setpoint(0.0, 0.0, np.deg2rad(self.step_amplitude), self.hover_thrust)

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


        t = time.time() - self.start_time

        # Commands
        roll_cmd = self.last_roll_cmd
        pitch_cmd = self.last_pitch_cmd
        yaw_rate_cmd = self.last_yaw_rate_cmd
        thrust_cmd = self.last_thrust_cmd

        # Ground truth attitude
        roll_gt, pitch_gt, yaw_gt = self.quat_to_euler(self.att_gt.q)

        # Ground truth position/velocity/acc
        x_gt = self.pos_gt.x
        y_gt = self.pos_gt.y
        z_gt = self.pos_gt.z

        vx_gt = self.pos_gt.vx
        vy_gt = self.pos_gt.vy
        vz_gt = self.pos_gt.vz

        ax_gt = self.pos_gt.ax
        ay_gt = self.pos_gt.ay
        az_gt = self.pos_gt.az

        self.logger.writerow([
            t,
            roll_cmd, pitch_cmd, yaw_rate_cmd, thrust_cmd,
            roll_gt, pitch_gt, yaw_gt,
            x_gt, y_gt, z_gt,
            vx_gt, vy_gt, vz_gt,
            ax_gt, ay_gt, az_gt,
            self.stage
        ])




def main(args=None):
    import sys

    # Default values
    step_amplitude = 0.0
    step_axis = "roll"
    hover_thrust = 0.8
    filename = "default_run"

    # Parse arguments
    if len(sys.argv) > 1:
        step_amplitude = float(sys.argv[1])
    if len(sys.argv) > 2:
        step_axis = sys.argv[2]
    if len(sys.argv) > 3:
        hover_thrust = float(sys.argv[3])
    if len(sys.argv) > 4:
        filename = sys.argv[4]

    print("=== Experiment Parameters ===")
    print("Step amplitude:", step_amplitude)
    print("Step axis:", step_axis)
    print("Hover thrust:", hover_thrust)
    print("Filename:", filename)

    rclpy.init(args=args)
    node = MinimalStepInput(run_name=filename, step_amplitude=step_amplitude, step_axis=step_axis, hover_thrust=hover_thrust)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
