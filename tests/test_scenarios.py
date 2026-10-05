import copy
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import carla

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import scenarios
from scenarios import RandomSpeedProfile


class SpeedProfileTests(unittest.TestCase):
    def test_bounds_over_full_run_including_zero(self):
        for nominal in (0.0, 0.1, 40.0, 70.0):
            for seed in range(10):
                profile = RandomSpeedProfile(nominal, 0.10, 1.0, 10.0, seed)
                for frame in range(601):
                    speed = profile.at(frame / 60.0)
                    self.assertGreaterEqual(speed + 1e-12, nominal * 0.9)
                    self.assertLessEqual(speed - 1e-12, nominal * 1.1)

    def test_reproducible_and_independent(self):
        def sample(seed):
            p = RandomSpeedProfile(50, 0.1, 1, 10, seed)
            return [p.at(i / 60) for i in range(600)]
        self.assertEqual(sample(42), sample(42))
        self.assertNotEqual(sample(42), sample(43))
        self.assertGreater(len(set(sample(42))), 100)

    def test_smooth_updates_and_disabled_variation(self):
        p = RandomSpeedProfile(50, 0.1, 1, 10, 42)
        self.assertLess(abs(p.at(1 - 1e-6) - p.at(1 + 1e-6)), 0.001)
        p = RandomSpeedProfile(50, 0, 1, 10, 42)
        self.assertTrue(all(p.at(i / 60) == 50 for i in range(600)))

    def test_invalid_input(self):
        for nominal, ratio, interval in ((-1, .1, 1), (math.nan, .1, 1),
                                          (50, .11, 1), (50, .1, 0)):
            with self.assertRaises(ValueError):
                RandomSpeedProfile(nominal, ratio, interval, 10, 42)


class Vehicle:
    def __init__(self, world, transform):
        self.world = world
        self.transform = transform
        self.is_alive = True
        self.enabled = False
        self.velocity = carla.Vector3D()
        self.started = []
        self.disabled = []

    def set_autopilot(self, enabled):
        pass

    def apply_control(self, control):
        pass

    def enable_constant_velocity(self, velocity):
        if not self.enabled:
            self.started.append(self.world.frame)
        self.enabled = True
        self.velocity = velocity
        if velocity.y != 0 or velocity.z != 0:
            raise AssertionError('World vector passed as local velocity')

    def disable_constant_velocity(self):
        self.enabled = False
        self.disabled.append(self.world.frame)

    def get_velocity(self):
        return self.velocity


class Extractor:
    def __init__(self, world):
        self.world = world
        self.start = None
        self.cleaned = False
        world.extractor = self

    def spawn_ego_vehicle(self, bp, transform):
        vehicle = Vehicle(self.world, transform)
        self.world.vehicles.append(vehicle)
        return vehicle

    spawn_adv_vehicle = spawn_ego_vehicle

    def setup_sensors(self, config):
        pass

    def wait_for_frame(self, frame):
        pass

    def start_recording(self, frame):
        self.start = frame

    @property
    def collision_detected(self):
        return self.world.collision_at is not None and self.start is not None and (
            self.world.frame - self.start >= self.world.collision_at)

    def save_accident_clip_and_get_data(self, *args, **kwargs):
        return kwargs

    def cleanup(self):
        self.cleaned = True


class World:
    def __init__(self, collision_at=None):
        self.settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)
        self.frame = 0
        self.vehicles = []
        self.observations = []
        self.collision_at = collision_at

    def get_settings(self):
        return copy.copy(self.settings)

    def apply_settings(self, settings):
        self.settings = settings

    def get_actors(self):
        return SimpleNamespace(filter=lambda _: [])

    def get_spectator(self):
        return SimpleNamespace(set_transform=lambda _: None)

    def tick(self):
        self.frame += 1
        if self.vehicles:
            self.observations.append(tuple(v.enabled for v in self.vehicles))
        return self.frame


class ScenarioLoopTests(unittest.TestCase):
    def run_pair(self, world, completion_check=None):
        with patch.object(scenarios, 'AccidentDataExtractor', Extractor), \
             patch.object(scenarios, 'SCENARIO_DURATION_SECONDS', 0.5):
            return scenarios._run_vehicle_pair(
                world, 'ego', 'adv', 40, 60, 1,
                carla.Transform(rotation=carla.Rotation(yaw=90)), carla.Transform(),
                carla.Transform(), 'test', .1, 1, 42, completion_check=completion_check,
            )

    def test_simultaneous_start_and_settings_restored(self):
        world = World()
        result = self.run_pair(world)
        self.assertIsNotNone(result)
        self.assertEqual(world.vehicles[0].started, world.vehicles[1].started)
        self.assertEqual(len(world.vehicles[0].started), 1)
        self.assertNotIn((True, False), world.observations)
        self.assertNotIn((False, True), world.observations)
        self.assertFalse(world.settings.synchronous_mode)
        self.assertIsNone(world.settings.fixed_delta_seconds)
        self.assertTrue(world.extractor.cleaned)
        self.assertEqual(len(result['speed_trace']), 30)

    def test_collision_releases_both_without_reenabling(self):
        world = World(collision_at=5)
        result = self.run_pair(world)
        for vehicle in world.vehicles:
            self.assertEqual(len(vehicle.started), 1)
            self.assertEqual(vehicle.disabled[0], world.extractor.start + 5)
        trace = result['speed_trace']
        self.assertEqual(result['metadata']['collision_world_frame'], world.extractor.start + 5)
        self.assertEqual(result['metadata']['last_world_frame'], world.frame)
        self.assertTrue(all(row['speed_control_active'] for row in trace[:5]))
        self.assertTrue(all(row['ego_target_kmh'] is None for row in trace[5:]))

    def test_keyboard_interrupt_propagates_and_restores(self):
        world = World()
        with patch.object(Extractor, 'wait_for_frame', side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                self.run_pair(world)
        self.assertTrue(world.extractor.cleaned)
        self.assertFalse(world.settings.synchronous_mode)

    def test_noncollision_completion_and_collision_recording(self):
        world = World()
        result = self.run_pair(world, completion_check=lambda vehicles: True)
        self.assertEqual(len(result['speed_trace']), 1)
        self.assertEqual(result['scenario_metadata']['시뮬레이션 시간(초)'], 1 / 60)
        world = World(collision_at=1)
        result = self.run_pair(world, completion_check=lambda vehicles: True)
        self.assertEqual(len(result['speed_trace']), 30)

    def test_main_has_three_callable_runners(self):
        import main
        available = (main.run_rear_end_scenario, main.run_intersection_scenario,
                     main.run_lane_change_scenario)
        self.assertTrue(all(callable(runner) for runner in available))
        self.assertTrue(main.SCENARIOS)
        self.assertTrue(all(runner in available for runner in main.SCENARIOS))


class RearEndBrakingTests(unittest.TestCase):
    def test_lead_gradually_reduces_target_and_does_not_brake_to_zero(self):
        world = World()
        commands = []

        def record_control(vehicle, control):
            if vehicle is world.vehicles[0]:
                commands.append((world.frame, control))

        with patch.object(scenarios, 'AccidentDataExtractor', Extractor), \
             patch.object(Vehicle, 'apply_control', record_control):
            result = scenarios._run_vehicle_pair(
                world, 'ego', 'adv', 60, 70, 1,
                carla.Transform(), carla.Transform(carla.Location(x=-25)),
                {}, 'RearEnd', 0, 1, 42, rear_end_braking=True,
            )
        targets = [row['ego_target_kmh'] for row in result['speed_trace']]
        self.assertEqual(targets[0], 60)
        self.assertAlmostEqual(targets[-1], 42)
        self.assertTrue(all(a >= b for a, b in zip(targets, targets[1:])))
        self.assertGreater(len(set(targets)), 100)
        self.assertTrue(all(target >= 42 for target in targets))
        self.assertTrue(any(control.brake > 0 for _, control in commands))
        self.assertLessEqual(max(control.brake for _, control in commands), 0.150001)
        self.assertGreaterEqual(world.extractor.ego_brake_time, 3.0)
        self.assertLessEqual(world.extractor.ego_brake_time, 4.0)

        lead = world.vehicles[0]
        lead.velocity = carla.Vector3D(x=42 / 3.6)
        self.assertAlmostEqual(scenarios._rear_end_lead_control(lead, 42).brake, 0, places=5)
        lead.velocity = carla.Vector3D(x=40 / 3.6)
        control = scenarios._rear_end_lead_control(lead, 42)
        self.assertEqual(control.brake, 0)
        self.assertGreater(control.throttle, 0)

    def test_collision_stops_lead_speed_controller(self):
        world = World(collision_at=400)
        commands = []

        def record_control(vehicle, control):
            if vehicle is world.vehicles[0]:
                commands.append((world.frame, control))

        with patch.object(scenarios, 'AccidentDataExtractor', Extractor), \
             patch.object(Vehicle, 'apply_control', record_control):
            result = scenarios._run_vehicle_pair(
                world, 'ego', 'adv', 60, 70, 1,
                carla.Transform(), carla.Transform(carla.Location(x=-25)),
                {}, 'RearEnd', 0, 1, 42, rear_end_braking=True,
            )
        collision_frame = result['metadata']['collision_world_frame']
        self.assertTrue(any(control.brake > 0 for frame, control in commands if frame < collision_frame))
        after = [control for frame, control in commands if frame >= collision_frame]
        self.assertTrue(after)
        self.assertTrue(all(control.brake == 0 and control.throttle == 0 for control in after))

    def test_rear_end_spawn_gap_and_metadata_match(self):
        spawn = carla.Transform(carla.Location(10, 20, 0.6), carla.Rotation(yaw=90))
        with patch.object(scenarios, '_spawn_point', return_value=spawn), \
             patch.object(scenarios, '_clear_background_near'), \
             patch.object(scenarios, '_run_vehicle_pair') as run_pair:
            scenarios.run_rear_end_scenario(World(), 'ego', 'adv', 60, 70, 1)
        ego, adv = run_pair.call_args.args[6:8]
        self.assertAlmostEqual(ego.location.distance(adv.location), 25)
        metadata = run_pair.call_args.kwargs['scenario_metadata']
        self.assertEqual(metadata['초기 차량 간격(m)'], 25)
        self.assertEqual(metadata['선행차 감속 후 목표 속력(km/h)'], 42)


if __name__ == '__main__':
    unittest.main()
