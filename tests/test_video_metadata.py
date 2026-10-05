from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import videodownload_module_system as video


class CollisionFrameTests(unittest.TestCase):
    def setUp(self):
        self.extractor = video.AccidentDataExtractor(MagicMock())

    def capture(self, frame_ids):
        for frame_id in frame_ids:
            self.extractor._on_camera_capture(SimpleNamespace(
                frame=frame_id, width=1, height=1,
                raw_data=bytes([frame_id % 256, 0, 0, 255]),
            ))

    def save(self, collision=True, **kwargs):
        with patch.object(video.cv2, 'VideoWriter') as writer:
            metadata = self.extractor.save_accident_clip_and_get_data(
                'test', 40, 70, collision, **kwargs,
            )
            written = [int(call.args[0][0, 0, 0])
                       for call in writer.return_value.write.call_args_list]
        self.assertNotIn('선행차 브레이크 시점(초)', metadata)
        self.assertNotIn('비디오 파일명', metadata)
        self.assertNotIn('선행차 브레이크 여부', metadata)
        self.assertNotIn('충돌 시점(초)', metadata)
        return metadata, written

    def test_sensor_frame_uses_saved_order_not_world_or_loop_number(self):
        self.capture([1000, 1004, 1002, 1005])
        for frame in (1004, 1002, 1002, 1005):
            self.extractor._on_collision(SimpleNamespace(frame=frame))
        metadata, written = self.save(collision_world_frame=1004, last_world_frame=1004)
        self.assertEqual(written, [1000 % 256, 1002 % 256, 1004 % 256])
        self.assertEqual(metadata['충돌 발생 프레임'], 2)
        self.assertEqual(metadata['영상 총 프레임'], len(written))
        self.assertEqual(metadata['충돌 프레임 기준'], '충돌 센서')

    def test_distance_only_detection_is_explicitly_estimated(self):
        self.capture([401, 402, 403])
        metadata, _ = self.save(collision_world_frame=401)
        self.assertEqual(metadata['충돌 발생 프레임'], 1)
        self.assertEqual(metadata['충돌 프레임 기준'], '차량 간 거리(추정)')

    def test_missing_collision_image_uses_next_saved_image_with_label(self):
        self.capture([100, 103])
        self.extractor._on_collision(SimpleNamespace(frame=102))
        metadata, _ = self.save()
        self.assertEqual(metadata['충돌 발생 프레임'], 2)
        self.assertIn('다음 수신 프레임', metadata['충돌 프레임 기준'])

    def test_no_collision_or_event_outside_clip_has_no_frame_number(self):
        self.capture([100, 101])
        metadata, _ = self.save(collision=False)
        self.assertIsNone(metadata['충돌 발생 프레임'])
        self.assertEqual(metadata['충돌 프레임 기준'], '비충돌')
        self.extractor._on_collision(SimpleNamespace(frame=102))
        metadata, _ = self.save()
        self.assertIsNone(metadata['충돌 발생 프레임'])
        self.assertEqual(metadata['충돌 프레임 기준'], '충돌 시점이 저장 영상 범위 밖')


if __name__ == '__main__':
    unittest.main()
