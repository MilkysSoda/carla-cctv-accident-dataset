import carla
import time
import os
from bisect import bisect_left
from threading import Condition
import numpy as np
import cv2
from datetime import datetime
from typing import cast, List, Optional


IMG_WIDTH = 1280
IMG_HEIGHT = 720

current_cycle_time = datetime.now().strftime("%Y%m%d_%H%M%S")
OUTPUT_DIR = f'./accident_{current_cycle_time}/'

if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

class AccidentDataExtractor:
    def __init__(self, world: carla.World):
        self.collision_detected = False
        self.world = world
        self.blueprint_library = world.get_blueprint_library()
        self.actor_list: List[carla.Actor] = []
        self.frame_buffer = []
        self.frame_ids = []
        self.first_collision_world_frame = None
        self._capture_condition = Condition()
        
        self.ego_brake_time: Optional[float] = None
        
        self.ego_vehicle: Optional[carla.Vehicle] = None
        self.adversary_vehicle: Optional[carla.Vehicle] = None
        self.camera: Optional[carla.Sensor] = None
        
        self.ego_model_name = ""
        self.adv_model_name = ""

    # =========================================================================
    # 🎯 [모듈화 완료] 위치 계산은 시나리오가 하고, 여긴 스폰만 담당합니다.
    # =========================================================================
    def spawn_ego_vehicle(self, bp_name: str, transform: carla.Transform) -> carla.Vehicle:
        self.ego_model_name = bp_name.replace('vehicle.', '').replace('.', '_')
        v_bp = self.blueprint_library.find(bp_name)
        self.ego_vehicle = cast(carla.Vehicle, self.world.spawn_actor(v_bp, transform))
        self.actor_list.append(self.ego_vehicle)
        return self.ego_vehicle

    def spawn_adv_vehicle(self, bp_name: str, transform: carla.Transform) -> carla.Vehicle:
        self.adv_model_name = bp_name.replace('vehicle.', '').replace('.', '_')
        v_bp = self.blueprint_library.find(bp_name)
        self.adversary_vehicle = cast(carla.Vehicle, self.world.spawn_actor(v_bp, transform))
        self.actor_list.append(self.adversary_vehicle)
        return self.adversary_vehicle

    def _on_collision(self, event):
        with self._capture_condition:
            if (self.first_collision_world_frame is None
                    or event.frame < self.first_collision_world_frame):
                self.first_collision_world_frame = event.frame
            self.collision_detected = True

    def setup_sensors(self, sensor_config: dict):
        assert self.ego_vehicle is not None
        assert self.adversary_vehicle is not None
        
        cam_bp = self.blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(sensor_config.get('width', IMG_WIDTH)))
        cam_bp.set_attribute('image_size_y', str(sensor_config.get('height', IMG_HEIGHT)))
        cam_bp.set_attribute('fov', str(sensor_config.get('fov', 90)))
        cam_bp.set_attribute('bloom_intensity', '3.0')
        
        cam_transform = cast(carla.Transform, sensor_config.get('transform'))
        
        self.camera = cast(carla.Sensor, self.world.spawn_actor(cam_bp, cam_transform))
        self.actor_list.append(self.camera)
        self.camera.listen(lambda image: self._on_camera_capture(image))

        collision_bp = self.blueprint_library.find('sensor.other.collision')
        for vehicle in (self.ego_vehicle, self.adversary_vehicle):
            collision_sensor = cast(
                carla.Sensor,
                self.world.spawn_actor(
                    collision_bp,
                    carla.Transform(),
                    attach_to=vehicle,
                ),
            )
            self.actor_list.append(collision_sensor)
            collision_sensor.listen(lambda event: self._on_collision(event))

    def _on_camera_capture(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        bgr_img = array[:, :, :3].copy() 
        with self._capture_condition:
            self.frame_buffer.append(bgr_img)
            self.frame_ids.append(image.frame)
            self._capture_condition.notify_all()

    def save_accident_clip_and_get_data(
        self, scenario_name: str, ego_spd_kmh: float, adv_spd_kmh: float,
        is_collision: bool, *, collision_world_frame=None, last_world_frame=None,
    ) -> dict:
        with self._capture_condition:
            # GPU 카메라의 마지막 콜백까지 받은 뒤, 저장할 영상과 프레임 표를 함께 고정한다.
            if last_world_frame is not None:
                received = self._capture_condition.wait_for(
                    lambda: bool(self.frame_ids) and max(self.frame_ids) >= last_world_frame,
                    timeout=5.0,
                )
                if not received:
                    raise RuntimeError(f"영상 마지막 프레임 {last_world_frame} 수신 시간 초과")
            captured = sorted(
                ((frame_id, image) for frame_id, image in zip(self.frame_ids, self.frame_buffer)
                 if last_world_frame is None or frame_id <= last_world_frame),
                key=lambda item: item[0],
            )
            sensor_collision_frame = self.first_collision_world_frame

        # 센서 이벤트의 실제 프레임을 우선한다. 거리 조건만 충족한 경우는 추정으로 구분한다.
        collision_source = "충돌 센서" if sensor_collision_frame is not None else "차량 간 거리(추정)"
        event_frame = sensor_collision_frame if sensor_collision_frame is not None else collision_world_frame
        collision_video_frame = None
        frame_status = "비충돌" if not is_collision else "충돌 프레임 미확인"
        if is_collision and event_frame is not None:
            ids = [frame_id for frame_id, _ in captured]
            index = bisect_left(ids, event_frame)
            if index < len(ids):
                collision_video_frame = index + 1  # 영상 첫 프레임은 1
                frame_status = collision_source
                if ids[index] != event_frame:
                    frame_status += " / 해당 프레임 누락으로 다음 수신 프레임"
            else:
                frame_status = "충돌 시점이 저장 영상 범위 밖"

        # 영상 파일명 맨 앞에 시나리오명을 추가하여 구분하기 쉽게 변경
        base_filename = f"{scenario_name}_{self.ego_model_name}_{int(ego_spd_kmh)}_{self.adv_model_name}_{int(adv_spd_kmh)}"
        video_filename = os.path.join(OUTPUT_DIR, f"{base_filename}.mp4")
        
        fourcc = cv2.VideoWriter_fourcc(*'mp4v') # type: ignore
        out = cv2.VideoWriter(video_filename, fourcc, 60.0, (IMG_WIDTH, IMG_HEIGHT))
        
        print(f"[-] 비디오 렌더링 중 ({len(captured)} 프레임) : {video_filename}")
        for _, frame in captured:
            out.write(frame)
        out.release()

        relative_speed = abs(adv_spd_kmh - ego_spd_kmh)
        
        # =====================================================================
        # 🎯 [추가됨] 시나리오명과 충돌 여부가 엑셀 메타데이터에 포함됩니다.
        # =====================================================================
        metadata = {
            "시나리오명": scenario_name,
            "차량A (앞차)": self.ego_model_name,
            "차량B (뒷차)": self.adv_model_name,
            "A 속력(km/h)": ego_spd_kmh,
            "B 속력(km/h)": adv_spd_kmh,
            "상대 속력(km/h)": round(relative_speed, 2),
            "선행차 브레이크 시점(초)": round(self.ego_brake_time, 2) if self.ego_brake_time is not None else "미작동",
            "충돌 여부": "O" if is_collision else "X",
            "충돌 발생 프레임": collision_video_frame,
            "충돌 시점(초)": ((collision_video_frame - 1) / 60.0
                            if collision_video_frame is not None else None),
            "충돌 프레임 기준": frame_status,
            "영상 총 프레임": len(captured),
            "영상 FPS": 60,
            "비디오 파일명": f"{base_filename}.mp4"
        }
        return metadata

    def cleanup(self):
        for actor in self.actor_list:
            if actor and actor.is_alive:
                actor.destroy()
        self.actor_list.clear()
        with self._capture_condition:
            self.frame_buffer.clear()
            self.frame_ids.clear()
        self.world.tick()
