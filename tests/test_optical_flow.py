import math
import sys
import struct
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from flight_controller.config import AppConfig, OpticalFlowConfig, validate_config, ConfigError
from flight_controller.models import AttitudeData
from flight_controller.optical_flow import decode_flow, flow_velocity, FlowWatchdog
from flight_controller.msp import MSPError
from flight_controller.models import GPSData
import tempfile
import csv
from flight_controller.mission import MissionController, MissionState
from flight_controller.simulator import SimulationVehicle, SimulationClock
from flight_controller.modes import SafetyStop
from flight_controller.inav import INAVClient, MSP2_INAV_FULL_LOCAL_POSE


class FlowTests(unittest.TestCase):
    def test_relative_uses_synchronized_local_pose_not_legacy_attitude_altitude(self):
        class LocalOnly(SimulationVehicle):
            def attitude(self):
                raise AssertionError("relative mode must use local-pose attitude")
            def altitude(self):
                raise AssertionError("relative mode must use local-pose Z/velocity")
        config = replace(AppConfig(), optical_flow=OpticalFlowConfig(mode='relative', calibrated=True))
        vehicle = LocalOnly(config)
        with tempfile.TemporaryDirectory() as root:
            controller = MissionController(vehicle, config, simulation=True,
                clock=SimulationClock(vehicle), log_root=root, output=lambda _: None)
            controller.run()
            self.assertEqual(controller.state, MissionState.COMPLETE)

    def test_relative_position_comes_directly_from_local_pose(self):
        class FixedPose(SimulationVehicle):
            def local_pose(self):
                pose = super().local_pose()
                return replace(pose, north_m=5.0, east_m=-2.0,
                               velocity_north_mps=0.4, velocity_east_mps=-0.3)
        config = replace(AppConfig(), optical_flow=OpticalFlowConfig(mode='relative', calibrated=True))
        vehicle = FixedPose(config)
        with tempfile.TemporaryDirectory() as root:
            controller = MissionController(vehicle, config, simulation=True,
                clock=SimulationClock(vehicle), log_root=root, output=lambda _: None)
            path = controller.run()
            with path.open(newline='', encoding='utf-8') as handle:
                rows = list(csv.DictReader(handle))
            relative = [r for r in rows if r['position_source'] == 'FLOW_RELATIVE']
            self.assertTrue(relative)
            self.assertTrue(all(float(r['relative_north_m']) == 0 for r in relative))
            self.assertTrue(all(float(r['relative_east_m']) == 0 for r in relative))

    def test_local_pose_decoder(self):
        class Transport:
            def request_v2(self, command):
                self.command = command
                return struct.pack('<hhhihihih', 15, -20, 321, 120, 30, -250, -40, 75, 5)
        transport = Transport()
        pose = INAVClient(transport).local_pose()
        self.assertEqual(transport.command, MSP2_INAV_FULL_LOCAL_POSE)
        self.assertEqual((pose.roll_deg, pose.pitch_deg, pose.yaw_deg), (1.5, -2, 32.1))
        self.assertEqual((pose.north_m, pose.velocity_north_mps), (1.2, .3))
        self.assertEqual((pose.east_m, pose.velocity_east_mps), (-2.5, -.4))

    def test_watchdog_debounces_then_loses(self):
        watchdog = FlowWatchdog(.4)
        self.assertEqual(watchdog.update(False, 1), 'grace')
        self.assertEqual(watchdog.update(False, 1.39), 'grace')
        self.assertEqual(watchdog.update(False, 1.41), 'lost')
        self.assertEqual(watchdog.update(True, 1.5), 'ok')

    def test_relative_mission_without_gps_fix(self):
        class NoFix(SimulationVehicle):
            def gps(self):
                return GPSData(0, 0, 0, 0, 99)
            def sensor_status(self):
                return replace(super().sensor_status(), overall_healthy=False,
                               compass=3, gps=3)
        config = replace(AppConfig(), optical_flow=OpticalFlowConfig(mode='relative', calibrated=True))
        vehicle = NoFix(config)
        with tempfile.TemporaryDirectory() as root:
            controller = MissionController(vehicle, config, simulation=True,
                clock=SimulationClock(vehicle), log_root=root, output=lambda _: None)
            controller.run()
            self.assertEqual(controller.state, MissionState.COMPLETE)

    def test_flow_loss_aborts_without_disarm(self):
        class LostFlow(SimulationVehicle):
            def optical_flow(self):
                return (0, 0, 0, 0, 0) if self.height_m > .5 else super().optical_flow()
        config = replace(AppConfig(), optical_flow=OpticalFlowConfig(mode='relative', calibrated=True))
        vehicle = LostFlow(config)
        with tempfile.TemporaryDirectory() as root:
            controller = MissionController(vehicle, config, simulation=True,
                clock=SimulationClock(vehicle), log_root=root, output=lambda _: None)
            with self.assertRaisesRegex(SafetyStop, 'lost optical flow'):
                controller.run()
            self.assertEqual(controller.state, MissionState.ABORT)
            self.assertTrue(vehicle._armed)

    def test_relative_complete_mission(self):
        config = replace(AppConfig(), optical_flow=OpticalFlowConfig(mode='relative', calibrated=True))
        vehicle = SimulationVehicle(config)
        with tempfile.TemporaryDirectory() as root:
            controller = MissionController(vehicle, config, simulation=True,
                clock=SimulationClock(vehicle), log_root=root, output=lambda _: None)
            path = controller.run()
            self.assertEqual(controller.state, MissionState.COMPLETE)
            self.assertIn('FLOW_RELATIVE', path.read_text())

    def test_decode(self):
        self.assertEqual(decode_flow(struct.pack('<Bhhhh', 100, -9, 12, -2, 3)), (100, -9, 12, -2, 3))
        with self.assertRaises(MSPError):
            decode_flow(b'')

    def test_translation_and_yaw(self):
        c = OpticalFlowConfig()
        n, e, row = flow_velocity((100, 0, -9, 0, 0), 1, 1, AttitudeData(0, 0, 0), c)
        self.assertAlmostEqual(n, math.radians(9))
        self.assertEqual(e, 0)
        n, e, row = flow_velocity((100, 0, -9, 0, 0), 1, 1, AttitudeData(0, 0, 90), c)
        self.assertAlmostEqual(e, math.radians(9))
        self.assertAlmostEqual(n, 0)

    def test_rotation_cancels(self):
        n, e, row = flow_velocity((100, 12, -9, 12, -9), 1, 1, AttitudeData(0, 0, 0), OpticalFlowConfig())
        self.assertEqual((n, e), (0, 0))

    def test_gates(self):
        for quality, status, distance, tilt in ((10, 1, 1, 0), (100, 3, 1, 0), (100, 1, .1, 0), (100, 1, 1, 20)):
            _, _, row = flow_velocity((quality, 0, 0, 0, 0), status, distance, AttitudeData(tilt, 0, 0), OpticalFlowConfig())
            self.assertEqual(row['flow_valid'], 0)

    def test_relative_health_does_not_depend_on_pi_raw_scale(self):
        _, _, row = flow_velocity(
            (100, 30000, 30000, 0, 0), 1, 1, AttitudeData(0, 0, 0),
            OpticalFlowConfig(), check_speed=False,
        )
        self.assertEqual(row['flow_valid'], 1)

    def test_assist_requires_calibration(self):
        with self.assertRaises(ConfigError):
            validate_config(replace(AppConfig(), optical_flow=OpticalFlowConfig(mode='assist')))


if __name__ == '__main__':
    unittest.main()
