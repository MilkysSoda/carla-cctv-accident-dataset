from contextlib import ExitStack, redirect_stdout
from io import StringIO
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import main
import videodownload_module_system as video


class MainSpeedTests(unittest.TestCase):
    def run_main(self, enabled):
        world = MagicMock()
        world.get_actors.return_value.filter.return_value = []
        extractor = video.AccidentDataExtractor(world)
        extractor.ego_model_name = 'ego'
        extractor.adv_model_name = 'adv'
        calls = []

        def run_scenario(**kwargs):
            calls.append(kwargs)
            extractor.frame_buffer.clear()
            extractor.frame_ids.clear()
            for frame in (501, 502, 503):
                extractor._on_camera_capture(SimpleNamespace(
                    frame=frame, width=1, height=1, raw_data=bytes(4),
                ))
            extractor._on_collision(SimpleNamespace(frame=502))
            return {'metadata': extractor.save_accident_clip_and_get_data(
                'test', kwargs['ego_spd_kmh'], kwargs['adv_spd_kmh'], True,
            )}

        runners = [MagicMock(side_effect=run_scenario) for _ in range(3)]
        for runner, name in zip(runners, (
                'run_rear_end_scenario', 'run_lane_change_scenario',
                'run_intersection_scenario')):
            runner.__name__ = name

        with TemporaryDirectory() as directory, ExitStack() as stack:
            values = {
                'SCENARIOS': runners,
                'VEHICLE_POOL': ['vehicle.test'],
                'EGO_SPEED_LIST': [40],
                'ADV_SPEED_LIST': [70],
                'WEATHER_RANDOMIZATION_ENABLED': 0,
                'SPEED_RANDOMIZATION_ENABLED': enabled,
                'SPEED_RANDOM_SEED': 42,
                'OUTPUT_DIR': directory + '/',
            }
            for name, value in values.items():
                stack.enter_context(patch.object(main, name, value))
            stack.enter_context(patch.object(main, 'connect_simulator',
                                            return_value=(MagicMock(), world)))
            stack.enter_context(patch.object(main, 'BackgroundTrafficManager'))
            stack.enter_context(patch.object(main, 'ScenarioViewController'))
            stack.enter_context(patch.object(video.cv2, 'VideoWriter'))
            stack.enter_context(redirect_stdout(StringIO()))
            main.main()
            excel = next(Path(directory).glob('*_master_accident_dataset.xlsx'))
            rows = main.pd.read_excel(excel).to_dict('records')
            for row in rows:
                self.assertNotIn('선행차 브레이크 시점(초)', row)
                self.assertNotIn('비디오 파일명', row)
                self.assertNotIn('선행차 브레이크 여부', row)
                self.assertNotIn('충돌 시점(초)', row)
        return calls, rows

    def test_enabled_speeds_reach_scenarios_and_excel(self):
        calls, rows = self.run_main(1)
        self.assertEqual(len(calls), 3)
        for call, row in zip(calls, rows):
            self.assertEqual(row['충돌 발생 프레임'], 2)
            self.assertEqual(row['영상 총 프레임'], 3)
            self.assertEqual(row['충돌 프레임 기준'], '충돌 센서')
            for actor, column, nominal in (('ego', 'A', 40), ('adv', 'B', 70)):
                speed = call[f'{actor}_spd_kmh']
                self.assertGreaterEqual(speed, nominal * 0.9)
                self.assertLessEqual(speed, nominal * 1.1)
                self.assertNotEqual(speed, nominal)
                self.assertAlmostEqual(row[f'{column} 속력(km/h)'], speed)
                self.assertEqual(row[f'{column} 기준 속력(km/h)'], nominal)
            self.assertEqual(call['speed_variation_ratio'], 0.0)
            self.assertEqual(row['속도 랜덤화 적용'], 1)
            self.assertEqual(row['속도 랜덤화 범위(%)'], 10)
            self.assertEqual(row['속도 랜덤 시드'], 42)
            self.assertAlmostEqual(row['상대 속력(km/h)'], round(
                abs(call['adv_spd_kmh'] - call['ego_spd_kmh']), 2))

    def test_disabled_keeps_nominal_speeds_in_scenarios_and_excel(self):
        calls, rows = self.run_main(0)
        for call, row in zip(calls, rows):
            self.assertEqual(call['ego_spd_kmh'], 40)
            self.assertEqual(call['adv_spd_kmh'], 70)
            self.assertEqual(call['speed_variation_ratio'], 0.0)
            self.assertEqual(row['A 속력(km/h)'], 40)
            self.assertEqual(row['B 속력(km/h)'], 70)
            self.assertEqual(row['상대 속력(km/h)'], 30)
            self.assertEqual(row['속도 랜덤화 적용'], 0)
            self.assertEqual(row['속도 랜덤화 범위(%)'], 0)


if __name__ == '__main__':
    unittest.main()
