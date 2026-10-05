import math
import random
import select
import sys
import termios
import tty
import carla
from scenario_motion import (
    LaneChangeController,
    RandomSpeedProfile,
    adjacent_driving_lane,
    advance_waypoint,
    same_direction,
)
from videodownload_module_system import AccidentDataExtractor, IMG_HEIGHT, IMG_WIDTH

REAR_END_SPAWN_INDEX = 45
REAR_END_INITIAL_GAP_METERS = 25.0
REAR_END_LEAD_SPEED_RATIO = 0.70  # 감속 후에도 지정 속도의 70%로 주행
REAR_END_DECELERATION_SECONDS = 3.0
REAR_END_LEAD_MAX_BRAKE = 0.15
REAR_END_CAMERA_BACK_METERS = 5.0  # 기존 후방 25m에서 전방으로 20m 이동
INTERSECTION_EGO_SPAWN_INDEX = 177
INTERSECTION_ADV_SPAWN_INDEX = 179
INTERSECTION_CAMERA_SPAWN_INDEX = 264  # 사진 북쪽 오른편 가로등 옆 차선
INTERSECTION_CAMERA_SIDE_OFFSET_METERS = 6.0
CCTV_CAMERA_HEIGHT = 9.0
LANE_CHANGE_EGO_SPAWN_INDEX = 49
LANE_CHANGE_TARGET_SPAWN_INDEX = 50
LANE_CHANGE_ADV_SPAWN_INDEX = 51
LANE_CHANGE_REFERENCE_SPAWN_INDEX = LANE_CHANGE_TARGET_SPAWN_INDEX
LANE_CHANGE_CAMERA_BACK_METERS = 15.0  # SP50 기준 후방 거리
LANE_CHANGE_CAMERA_RIGHT_METERS = -2.0  # 목표 차선 중앙축에서 화면 왼쪽으로 2m
LANE_CHANGE_STRAIGHT_APPROACH_METERS = 45.0
LANE_CHANGE_START_SECONDS = 4.0
LANE_CHANGE_DURATION_SECONDS = 2.0
LANE_CHANGE_EGO_SIDE = "right"  # SP49 -> SP50, 시계방향
LANE_CHANGE_ADV_SIDE = "left"   # SP51 -> SP50, 반시계방향
LANE_CHANGE_ADV_STEERING_SIGN = -1.0
SCENARIO_DURATION_SECONDS = 10.0
POST_COLLISION_RECORD_SECONDS = 3.0
PRE_COLLISION_RELEASE_DISTANCE = 5.0
FPS = 60
MAX_RECORDING_FRAMES = math.ceil(SCENARIO_DURATION_SECONDS * FPS)


class ScenarioViewController:
    """pygame 창의 1/2/3 입력으로 CARLA spectator를 CCTV 위치로 이동한다."""

    def __init__(self, world):
        self.world = world
        self.pygame = None
        self.window = None
        self._terminal_settings = None
        try:
            import pygame

            pygame.init()
            self.pygame = pygame
            self.window = pygame.display.set_mode((460, 100))
            pygame.display.set_caption("CARLA CCTV View: 1 Rear / 2 Intersection / 3 Lane Change")
            self._set_view(2)
        except Exception as error:
            print(f"[!] CCTV 키보드 창을 열 수 없습니다: {error}")
            self.pygame = None
        self._enable_terminal_hotkeys()

    def _enable_terminal_hotkeys(self):
        if not sys.stdin.isatty():
            return
        try:
            self._terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            print("[CCTV] 터미널에서 1/2/3을 누르면 시점이 이동합니다.")
        except (termios.error, OSError):
            self._terminal_settings = None

    def _handle_view_key(self, key):
        view = {"1": 1, "2": 2, "3": 3}.get(key)
        if view is not None:
            self._set_view(view)

    def poll(self):
        if self.pygame is not None:
            for event in self.pygame.event.get():
                if event.type == self.pygame.QUIT:
                    self.close()
                elif event.type == self.pygame.KEYDOWN:
                    key_to_view = {
                        self.pygame.K_1: 1,
                        self.pygame.K_2: 2,
                        self.pygame.K_3: 3,
                    }
                    view = key_to_view.get(event.key)
                    if view is not None:
                        self._set_view(view)

        if self._terminal_settings is not None:
            while select.select([sys.stdin], [], [], 0)[0]:
                self._handle_view_key(sys.stdin.read(1))

    def _set_view(self, view):
        if view == 1:
            spawn = _spawn_point(self.world, REAR_END_SPAWN_INDEX)
            camera = _rear_end_camera(spawn)
            location = camera["transform"].location
            rotation = camera["transform"].rotation
        elif view == 2:
            first = _spawn_point(self.world, INTERSECTION_EGO_SPAWN_INDEX)
            second = _spawn_point(self.world, INTERSECTION_ADV_SPAWN_INDEX)
            center, _, _ = _intersection_distances(first, second)
            camera = _intersection_camera(self.world, center)
            location = camera["transform"].location
            rotation = camera["transform"].rotation
        else:
            reference = _spawn_point(self.world, LANE_CHANGE_REFERENCE_SPAWN_INDEX)
            camera = _lane_change_camera(_grounded_spawn_transform(reference))
            location = camera["transform"].location
            rotation = camera["transform"].rotation

        self.world.get_spectator().set_transform(carla.Transform(location, rotation))
        print(f"[CCTV] {view}번 시점으로 이동했습니다.")

    def close(self):
        if self.pygame is not None:
            self.pygame.quit()
            self.pygame = None
        if self._terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._terminal_settings)
            self._terminal_settings = None


def _spawn_point(world, index):
    spawn_points = world.get_map().get_spawn_points()
    if not 0 <= index < len(spawn_points):
        raise IndexError(f"Town04 스폰포인트 SP {index}를 찾을 수 없습니다.")
    return spawn_points[index]


def _clear_background_near(world, transforms, radius=60.0):
    for vehicle in world.get_actors().filter("vehicle.*"):
        if vehicle.attributes.get("role_name") != "background":
            continue
        if any(vehicle.get_location().distance(item.location) < radius for item in transforms):
            vehicle.destroy()
    world.tick()


def _clear_spawn_area(world, transforms, radius=8.0):
    """차량 생성 직전에 spawn 지점과 겹치는 차량을 제거한다."""
    destroyed_count = 0
    for vehicle in world.get_actors().filter("vehicle.*"):
        if any(vehicle.get_location().distance(item.location) < radius for item in transforms):
            vehicle.destroy()
            destroyed_count += 1
    if destroyed_count:
        print(f"[spawn 정리] 생성 위치 주변 차량 {destroyed_count}대 제거")

def _vehicle_location(vehicle):
    getter = getattr(vehicle, "get_location", None)
    return getter() if getter else vehicle.transform.location


def _vehicle_transform(vehicle):
    getter = getattr(vehicle, "get_transform", None)
    return getter() if getter else vehicle.transform


def _release_vehicle_physics(vehicles):
    """강제 속도와 엔진 가속 명령을 끊고 현재 속도만 물리엔진에 맡긴다."""
    for vehicle in vehicles:
        vehicle.disable_constant_velocity()
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))


def _set_brake_lights(vehicle, enabled):
    if not hasattr(vehicle, "set_light_state"):
        return
    state = carla.VehicleLightState.Brake if enabled else carla.VehicleLightState.NONE
    vehicle.set_light_state(state)

def _speed_kmh(vehicle):
    velocity = vehicle.get_velocity()
    return 3.6 * math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)


def _speed_control(vehicle, target_speed_kmh, steer):
    current_speed = _speed_kmh(vehicle)
    if current_speed > target_speed_kmh + 3.0:
        return carla.VehicleControl(throttle=0.0, steer=steer, brake=0.15)
    if current_speed < target_speed_kmh - 3.0:
        return carla.VehicleControl(throttle=0.65, steer=steer, brake=0.0)
    return carla.VehicleControl(throttle=0.25, steer=steer, brake=0.0)


def _rear_end_lead_control(vehicle, target_speed_kmh):
    """낮아지는 목표 속도를 약한 제동으로 따라가고, 목표 아래에서는 가속한다."""
    error = target_speed_kmh - _speed_kmh(vehicle)
    return carla.VehicleControl(
        throttle=min(0.35, max(0.0, error * 0.10)),
        brake=min(REAR_END_LEAD_MAX_BRAKE, max(0.0, -error * 0.05)),
    )


def _signed_lateral_error_to_line(vehicle, reference_transform):
    vehicle_transform = _vehicle_transform(vehicle)
    location = vehicle_transform.location
    reference_location = reference_transform.location
    reference_forward = reference_transform.get_forward_vector()
    vehicle_right = vehicle_transform.get_right_vector()

    dx = location.x - reference_location.x
    dy = location.y - reference_location.y
    progress = dx * reference_forward.x + dy * reference_forward.y
    target = carla.Location(
        x=reference_location.x + reference_forward.x * progress,
        y=reference_location.y + reference_forward.y * progress,
        z=location.z,
    )
    return (
        (target.x - location.x) * vehicle_right.x
        + (target.y - location.y) * vehicle_right.y
    )


def _steer_toward_lane(vehicle, world, target_lane_id, direction, lookahead=8.0):
    """현재 waypoint에서 인접 차선 waypoint를 선택해 한 차선만 변경한다."""
    carla_map = world.get_map()
    current = carla_map.get_waypoint(vehicle.get_location())
    if current.lane_id == target_lane_id:
        target = current
    else:
        target = (current.get_right_lane() if direction == "right"
                  else current.get_left_lane())
        if target is None or target.lane_id != target_lane_id:
            return 0.0

    next_waypoints = target.next(lookahead)
    aim = next_waypoints[0] if next_waypoints else target
    vehicle_transform = vehicle.get_transform()
    location = vehicle_transform.location
    forward = vehicle_transform.get_forward_vector()
    right = vehicle_transform.get_right_vector()
    dx = aim.transform.location.x - location.x
    dy = aim.transform.location.y - location.y
    longitudinal = dx * forward.x + dy * forward.y
    lateral = dx * right.x + dy * right.y
    steer = math.atan2(2.0 * 2.8 * lateral, max(longitudinal ** 2, 1.0))
    return max(-0.62, min(0.62, steer / math.radians(25.0)))


def _grounded_spawn_transform(transform, z_offset=0.2):
    """waypoint의 z=0 transform을 차량이 노면에 안착하는 높이로 보정한다."""
    location = transform.location
    return carla.Transform(
        carla.Location(location.x, location.y, location.z + z_offset),
        transform.rotation,
    )


def _intersection_distances(first, second):
    first_location = first.location
    second_location = second.location
    first_direction = first.get_forward_vector()
    second_direction = second.get_forward_vector()
    denominator = first_direction.x * second_direction.y - first_direction.y * second_direction.x
    if abs(denominator) < 1e-6:
        raise ValueError("차량의 진행 방향이 교차하지 않습니다.")

    delta_x = second_location.x - first_location.x
    delta_y = second_location.y - first_location.y
    first_distance = (delta_x * second_direction.y - delta_y * second_direction.x) / denominator
    second_distance = (delta_x * first_direction.y - delta_y * first_direction.x) / denominator
    if first_distance <= 0 or second_distance <= 0:
        raise ValueError("충돌 지점이 차량 진행 방향 앞쪽에 있지 않습니다.")

    center = carla.Location(
        x=first_location.x + first_direction.x * first_distance,
        y=first_location.y + first_direction.y * first_distance,
        z=max(first_location.z, second_location.z),
    )
    return center, first_distance, second_distance

def _intersection_camera(world, center):
    """SP264 오른쪽 가로등에 고정한 위치에서 교차 지점을 바라본다."""
    reference = _spawn_point(world, INTERSECTION_CAMERA_SPAWN_INDEX)
    right = reference.get_right_vector()
    anchor = carla.Location(
        x=reference.location.x + right.x * INTERSECTION_CAMERA_SIDE_OFFSET_METERS,
        y=reference.location.y + right.y * INTERSECTION_CAMERA_SIDE_OFFSET_METERS,
        z=reference.location.z,
    )
    # 가로등은 정적 맵 오브젝트이므로 actor에 attach하지 않고 월드 좌표에 고정한다.
    # 맵의 semantic tag와 무관하게 가로등 메시 이름으로 찾는다.
    lamps = []
    for obj in world.get_environment_objects(carla.CityObjectLabel.Any):
        name = obj.name.lower().replace('_', '').replace('-', '')
        if not any(token in name for token in ('streetlight', 'streetlamp', 'lamppost', 'lightpole')):
            continue
        distance = math.hypot(
            obj.transform.location.x - anchor.x,
            obj.transform.location.y - anchor.y,
        )
        if distance <= 12.0:
            lamps.append((distance, obj))

    if lamps:
        lamp = min(lamps, key=lambda item: item[0])[1]
        anchor.x = lamp.transform.location.x
        anchor.y = lamp.transform.location.y
    else:
        print("[CCTV] SP264 주변 가로등 좌표를 찾지 못해 오른쪽 도로변 추정 위치를 사용합니다.")

    # 기둥 내부에 카메라가 묻히지 않도록 교차로 방향으로 0.5m 돌출한다.
    dx, dy = center.x - anchor.x, center.y - anchor.y
    distance = math.hypot(dx, dy)
    location = carla.Location(
        anchor.x + (0.5 * dx / distance if distance else 0.0),
        anchor.y + (0.5 * dy / distance if distance else 0.0),
        reference.location.z + CCTV_CAMERA_HEIGHT,
    )
    dx, dy, dz = center.x - location.x, center.y - location.y, center.z - location.z
    rotation = carla.Rotation(
        pitch=math.degrees(math.atan2(dz, math.hypot(dx, dy))),
        yaw=math.degrees(math.atan2(dy, dx)),
        roll=0.0,
    )
    transform = carla.Transform(location, rotation)
    return {"width": IMG_WIDTH, "height": IMG_HEIGHT, "transform": transform}


def _rear_end_camera(spawn):
    forward = spawn.get_forward_vector()
    right = spawn.get_right_vector()
    transform = carla.Transform(
        carla.Location(
            x=spawn.location.x - forward.x * REAR_END_CAMERA_BACK_METERS + right.x * 8.0,
            y=spawn.location.y - forward.y * REAR_END_CAMERA_BACK_METERS + right.y * 8.0,
            z=spawn.location.z + CCTV_CAMERA_HEIGHT,
        ),
        carla.Rotation(pitch=-12.0, yaw=spawn.rotation.yaw, roll=0.0),
    )
    return {"width": IMG_WIDTH, "height": IMG_HEIGHT, "transform": transform}


def _lane_change_camera(reference_transform):
    """목표 차선 중앙축보다 살짝 왼쪽의 가까운 후방에서 촬영한다."""
    forward = reference_transform.get_forward_vector()
    right = reference_transform.get_right_vector()
    location = carla.Location(
        x=(reference_transform.location.x
           - forward.x * LANE_CHANGE_CAMERA_BACK_METERS
           + right.x * LANE_CHANGE_CAMERA_RIGHT_METERS),
        y=(reference_transform.location.y
           - forward.y * LANE_CHANGE_CAMERA_BACK_METERS
           + right.y * LANE_CHANGE_CAMERA_RIGHT_METERS),
        z=reference_transform.location.z + CCTV_CAMERA_HEIGHT,
    )
    transform = carla.Transform(
        location,
        carla.Rotation(pitch=-24.0, yaw=reference_transform.rotation.yaw, roll=0.0),
    )
    return {"width": IMG_WIDTH, "height": IMG_HEIGHT, "fov": 100, "transform": transform}

def _force_forward_velocity(vehicle, target_speed_kmh):
    forward = vehicle.get_transform().get_forward_vector()
    speed = target_speed_kmh / 3.6
    vehicle.set_target_velocity(
        carla.Vector3D(
            forward.x * speed,
            forward.y * speed,
            forward.z * speed,
        )
    )


def _run_vehicle_pair(
    world,
    ego_bp,
    adv_bp,
    ego_speed,
    adv_speed,
    sample_count,
    ego_transform,
    adv_transform,
    camera_config,
    scenario_name,
    speed_variation_ratio=0.10,
    speed_update_interval=1.0,
    random_seed=None,
    completion_check=None,
    controllers=(),
    scenario_metadata=None,
    view_controller=None,
    rear_end_braking=False,
):
    original_settings = world.get_settings()
    extractor = AccidentDataExtractor(world)
    vehicles = []
    speed_trace = []
    collision = False
    velocity_control_released = False
    ego_profile = RandomSpeedProfile(
        ego_speed, speed_variation_ratio, speed_update_interval,
        SCENARIO_DURATION_SECONDS, random_seed,
    )
    adv_profile = RandomSpeedProfile(
        adv_speed, speed_variation_ratio, speed_update_interval,
        SCENARIO_DURATION_SECONDS,
        None if random_seed is None else random_seed + 1,
    )
    brake_rng = random.Random(random_seed if random_seed is not None else sample_count)
    lead_brake_time = brake_rng.uniform(3.0, 4.0)
    lead_braked = False
    following_braked = False
    try:
        _clear_spawn_area(world, [ego_transform, adv_transform])
        ego_vehicle = extractor.spawn_ego_vehicle(ego_bp, ego_transform)
        adv_vehicle = extractor.spawn_adv_vehicle(adv_bp, adv_transform)
        vehicles = [ego_vehicle, adv_vehicle]
        extractor.setup_sensors(camera_config)

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / FPS
        world.apply_settings(settings)

        start_recording = getattr(extractor, "start_recording", None)
        if callable(start_recording):
            start_recording(world.frame if hasattr(world, "frame") else 0)

        # CARLA's constant-velocity API expects a local vehicle-axis vector.
        ego_vehicle.enable_constant_velocity(carla.Vector3D(ego_speed / 3.6, 0.0, 0.0))
        adv_vehicle.enable_constant_velocity(carla.Vector3D(adv_speed / 3.6, 0.0, 0.0))

        max_frames = min(
            math.ceil(SCENARIO_DURATION_SECONDS * FPS),
            MAX_RECORDING_FRAMES,
        )
        collision_frame = None
        collision_world_frame = None
        for frame in range(max_frames):
            if view_controller is not None:
                view_controller.poll()
            world_frame = world.tick()
            elapsed = (frame + 1) / FPS
            ego_target = None if collision else ego_profile.at(elapsed)
            adv_target = None if collision else adv_profile.at(elapsed)
            if rear_end_braking and not collision:
                ego_target, adv_target = ego_speed, adv_speed
            collision_detected = False
            approach_distance = None

            if not collision:
                if bool(getattr(extractor, "collision_detected", False)):
                    collision_detected = True
                elif hasattr(ego_vehicle, "get_location") and hasattr(adv_vehicle, "get_location"):
                    approach_distance = _vehicle_location(ego_vehicle).distance(
                        _vehicle_location(adv_vehicle)
                    )
                    collision_detected = approach_distance < 2.5

                if rear_end_braking and not collision_detected and not velocity_control_released:
                    if not lead_braked and elapsed >= lead_brake_time:
                        ego_vehicle.disable_constant_velocity()
                        lead_braked = True
                        extractor.ego_brake_time = elapsed
                        print(
                            f"[선행차 감속] {elapsed:.2f}초부터 "
                            f"{REAR_END_DECELERATION_SECONDS:.1f}초 동안 목표속도를 "
                            f"{ego_speed * REAR_END_LEAD_SPEED_RATIO:.1f}km/h까지 낮춘 뒤 유지"
                        )

                    if (approach_distance is not None
                            and approach_distance <= 12.0
                            and not following_braked):
                        adv_vehicle.disable_constant_velocity()
                        adv_vehicle.apply_control(
                            carla.VehicleControl(throttle=0.0, brake=0.2)
                        )
                        _set_brake_lights(adv_vehicle, True)
                        following_braked = True
                        print(f"[후행차 제동] 차간거리 {approach_distance:.2f}m, 약한 제동")

                if (approach_distance is not None
                        and approach_distance <= PRE_COLLISION_RELEASE_DISTANCE
                        and not velocity_control_released):
                    _release_vehicle_physics(vehicles)
                    velocity_control_released = True
                    print(
                        f"[물리 전환] 차량 간 거리 {approach_distance:.2f}m, "
                        "강제 속도 제어를 해제합니다."
                    )

            if collision and ego_target is not None:
                ego_target = adv_target = None
                _release_vehicle_physics(vehicles)
            elif (not collision and not velocity_control_released
                and not collision_detected and not rear_end_braking):
                assert ego_target is not None and adv_target is not None
                ego_vehicle.enable_constant_velocity(carla.Vector3D(ego_target / 3.6, 0.0, 0.0))
                adv_vehicle.enable_constant_velocity(carla.Vector3D(adv_target / 3.6, 0.0, 0.0))

            if velocity_control_released and not collision:
                ego_target = adv_target = None

            if not collision and not collision_detected:
                controller_speeds = (
                    ego_target if ego_target is not None else ego_speed,
                    adv_target if adv_target is not None else adv_speed,
                )
                for vehicle, controller, target_speed in zip(
                        vehicles, controllers, controller_speeds):
                    if controller is not None:
                        vehicle.apply_control(
                            controller.control(vehicle, elapsed, target_speed)
                        )

            if (rear_end_braking and lead_braked and not collision
                    and not collision_detected and not velocity_control_released):
                progress = min(1.0, max(0.0,
                    (elapsed - lead_brake_time) / REAR_END_DECELERATION_SECONDS))
                ego_target = ego_speed * (1.0 - (1.0 - REAR_END_LEAD_SPEED_RATIO) * progress)
                control = _rear_end_lead_control(ego_vehicle, ego_target)
                ego_vehicle.apply_control(control)
                _set_brake_lights(ego_vehicle, control.brake > 0.0)
            if rear_end_braking and following_braked and not collision and not collision_detected:
                adv_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.2))

            speed_trace.append({
                "frame": frame + 1,
                "time": elapsed,
                "ego_target_kmh": ego_target,
                "adv_target_kmh": adv_target,
                "speed_control_active": not collision,
            })

            if collision_detected:
                collision = True
                collision_frame = frame
                collision_world_frame = world_frame
                max_frames = max(
                    max_frames,
                    frame + 1 + math.ceil(POST_COLLISION_RECORD_SECONDS * FPS),
                )
                _release_vehicle_physics(vehicles)

            if (collision_frame is not None
                    and frame - collision_frame >= math.ceil(POST_COLLISION_RECORD_SECONDS * FPS)):
                break

            if not collision and not collision_detected and completion_check is not None and completion_check(vehicles):
                break

            wait_for_frame = getattr(extractor, "wait_for_frame", None)
            if callable(wait_for_frame):
                wait_for_frame(frame + 1)

        metadata = dict(scenario_metadata or {})
        metadata["시뮬레이션 시간(초)"] = len(speed_trace) / FPS
        metadata["충돌 여부"] = collision
        result = {
            "scenario_metadata": metadata,
            "speed_trace": speed_trace,
            "collision": collision,
        }
        try:
            result["metadata"] = extractor.save_accident_clip_and_get_data(
                scenario_name, ego_speed, adv_speed, collision,
                collision_world_frame=collision_world_frame,
                last_world_frame=world_frame,
            )
        except TypeError:
            result["metadata"] = extractor.save_accident_clip_and_get_data(
                scenario_name=scenario_name,
                ego_spd_kmh=ego_speed,
                adv_spd_kmh=adv_speed,
                is_collision=collision,
                collision_world_frame=collision_world_frame,
                last_world_frame=world_frame,
            )
        return result
    except KeyboardInterrupt:
        raise
    finally:
        for vehicle in vehicles:
            if getattr(vehicle, "is_alive", True):
                vehicle.disable_constant_velocity()
            _set_brake_lights(vehicle, False)
        extractor.cleanup()
        world.apply_settings(original_settings)


def run_intersection_scentario(
    world, ego_bp, adv_bp, ego_spd_kmh, adv_spd_kmh, sample_count,
    speed_variation_ratio=0.10, speed_update_interval=1.0, random_seed=None,
    view_controller=None,
):
    ego_transform = _spawn_point(world, INTERSECTION_EGO_SPAWN_INDEX)
    adv_transform = _spawn_point(world, INTERSECTION_ADV_SPAWN_INDEX)
    center, _, _ = _intersection_distances(ego_transform, adv_transform)
    _clear_background_near(world, [ego_transform, adv_transform])

    def both_passed(vehicles):
        for vehicle, start in zip(vehicles, (ego_transform, adv_transform)):
            direction = start.get_forward_vector()
            location = _vehicle_location(vehicle)
            passed = (location.x - center.x) * direction.x + (location.y - center.y) * direction.y
            if passed < 10.0:
                return False
        return True

    return _run_vehicle_pair(
        world, ego_bp, adv_bp, ego_spd_kmh, adv_spd_kmh, sample_count,
        ego_transform, adv_transform, _intersection_camera(world, center), "Intersection",
        speed_variation_ratio, speed_update_interval, random_seed,
        completion_check=both_passed,
        scenario_metadata={"ego 스폰": 177, "adv 스폰": 179},
        view_controller=view_controller,
    )


run_intersection_scenario = run_intersection_scentario


def run_lane_change_scenario(
    world, ego_bp, adv_bp, ego_spd_kmh, adv_spd_kmh, sample_count,
    speed_variation_ratio=0.10, speed_update_interval=1.0, random_seed=None,
    view_controller=None,
):
    ego_start = _spawn_point(world, LANE_CHANGE_EGO_SPAWN_INDEX)
    target_start = _spawn_point(world, LANE_CHANGE_TARGET_SPAWN_INDEX)
    adv_start = _spawn_point(world, LANE_CHANGE_ADV_SPAWN_INDEX)
    ego_spawn = _grounded_spawn_transform(ego_start)
    adv_spawn = _grounded_spawn_transform(adv_start)
    target_reference = _grounded_spawn_transform(target_start)
    target_lane_id = world.get_map().get_waypoint(target_start.location).lane_id
    _clear_spawn_area(world, [ego_spawn, adv_spawn], radius=10.0)
    _clear_background_near(world, [ego_spawn, target_reference, adv_spawn])

    original_settings = world.get_settings()
    extractor = AccidentDataExtractor(world)
    vehicles = []
    speed_trace = []
    collision = False
    collision_frame = None
    collision_world_frame = None
    ego_profile = RandomSpeedProfile(
        ego_spd_kmh, speed_variation_ratio, speed_update_interval,
        SCENARIO_DURATION_SECONDS, random_seed,
    )
    adv_profile = RandomSpeedProfile(
        adv_spd_kmh, speed_variation_ratio, speed_update_interval,
        SCENARIO_DURATION_SECONDS,
        None if random_seed is None else random_seed + 1,
    )
    timing_rng = random.Random(random_seed if random_seed is not None else sample_count)
    ego_lane_change_start = LANE_CHANGE_START_SECONDS * timing_rng.uniform(0.90, 1.10)
    adv_lane_change_start = LANE_CHANGE_START_SECONDS * timing_rng.uniform(0.90, 1.10)

    try:
        ego_vehicle = extractor.spawn_ego_vehicle(ego_bp, ego_spawn)
        adv_vehicle = extractor.spawn_adv_vehicle(adv_bp, adv_spawn)
        vehicles = [ego_vehicle, adv_vehicle]
        extractor.setup_sensors(_lane_change_camera(target_reference))

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / FPS
        world.apply_settings(settings)

        print(
            f"[차선변경] SP49 차량은 시계방향 주행, "
            f"SP51 차량은 반시계방향 주행 후 "
            f"ego {ego_lane_change_start:.2f}초 / adv {adv_lane_change_start:.2f}초에 진입합니다."
        )

        max_frames = math.ceil(SCENARIO_DURATION_SECONDS * FPS)
        frame = 0
        while frame < max_frames:
            if view_controller is not None:
                view_controller.poll()
            world_frame = world.tick()
            elapsed = (frame + 1) / FPS
            ego_target = None if collision else ego_profile.at(elapsed)
            adv_target = None if collision else adv_profile.at(elapsed)
            approach_distance = _vehicle_location(ego_vehicle).distance(
                _vehicle_location(adv_vehicle)
            )
            collision_detected = (
                bool(getattr(extractor, "collision_detected", False))
                or approach_distance < 2.5
            )

            if collision_detected and not collision:
                collision = True
                collision_frame = frame
                collision_world_frame = world_frame
                max_frames = max(
                    max_frames,
                    frame + 1 + math.ceil(POST_COLLISION_RECORD_SECONDS * FPS),
                )
                _release_vehicle_physics(vehicles)
                print(
                    f"[충돌 감지] {elapsed:.2f}초, 차량 간 거리 {approach_distance:.2f}m"
                )

            if not collision:
                ego_lane_change_progress = (
                    elapsed - ego_lane_change_start
                ) / LANE_CHANGE_DURATION_SECONDS
                adv_lane_change_progress = (
                    elapsed - adv_lane_change_start
                ) / LANE_CHANGE_DURATION_SECONDS
                if 0.0 <= ego_lane_change_progress < 1.0:
                    ego_steer = _steer_toward_lane(
                        ego_vehicle, world, target_lane_id, "right"
                    )
                else:
                    ego_steer = 0.0
                if 0.0 <= adv_lane_change_progress < 1.0:
                    adv_steer = _steer_toward_lane(
                        adv_vehicle, world, target_lane_id, "left"
                    )
                else:
                    adv_steer = 0.0

                ego_vehicle.apply_control(
                    _speed_control(ego_vehicle, ego_target, ego_steer)
                )
                adv_vehicle.apply_control(
                    _speed_control(adv_vehicle, adv_target, adv_steer)
                )


            speed_trace.append({
                "frame": frame + 1,
                "time": elapsed,
                "ego_target_kmh": ego_target,
                "adv_target_kmh": adv_target,
                "distance_m": approach_distance,
                "speed_control_active": not collision,
            })

            if (collision_frame is not None
                    and frame - collision_frame >= math.ceil(POST_COLLISION_RECORD_SECONDS * FPS)):
                break

            wait_for_frame = getattr(extractor, "wait_for_frame", None)
            if callable(wait_for_frame):
                wait_for_frame(frame + 1)
            frame += 1

        metadata = {
            "ego 스폰": 49,
            "목표 차선": 50,
            "adv 스폰": 51,
            "차선변경 시작(초)": LANE_CHANGE_START_SECONDS,
            "ego 차선변경 시작(초)": ego_lane_change_start,
            "adv 차선변경 시작(초)": adv_lane_change_start,
            "시뮬레이션 시간(초)": len(speed_trace) / FPS,
            "충돌 여부": collision,
        }
        result = {
            "scenario_metadata": metadata,
            "speed_trace": speed_trace,
            "collision": collision,
        }
        result["metadata"] = extractor.save_accident_clip_and_get_data(
            "LaneChange", ego_spd_kmh, adv_spd_kmh, collision,
            collision_world_frame=collision_world_frame,
            last_world_frame=world_frame,
        )
        return result
    except KeyboardInterrupt:
        raise
    finally:
        _release_vehicle_physics(vehicles)
        extractor.cleanup()
        world.apply_settings(original_settings)

def run_rear_end_scenario(
    world, ego_bp, adv_bp, ego_spd_kmh, adv_spd_kmh, sample_count,
    speed_variation_ratio=0.10, speed_update_interval=1.0, random_seed=None,
    view_controller=None,
):
    spawn_transform = _spawn_point(world, REAR_END_SPAWN_INDEX)
    _clear_background_near(world, [spawn_transform])
    adv_transform = carla.Transform(
        spawn_transform.location - spawn_transform.get_forward_vector() * REAR_END_INITIAL_GAP_METERS,
        spawn_transform.rotation,
    )
    return _run_vehicle_pair(
        world, ego_bp, adv_bp, ego_spd_kmh, adv_spd_kmh, sample_count,
        spawn_transform, adv_transform, _rear_end_camera(spawn_transform),
        "RearEnd", speed_variation_ratio=speed_variation_ratio,
        speed_update_interval=speed_update_interval,
        random_seed=random_seed,
        scenario_metadata={
            "ego 스폰": 45,
            "adv 스폰": f"SP45 후방 {REAR_END_INITIAL_GAP_METERS:g}m",
            "초기 차량 간격(m)": REAR_END_INITIAL_GAP_METERS,
            "선행차 감속 후 목표 속력(km/h)": ego_spd_kmh * REAR_END_LEAD_SPEED_RATIO,
            "선행차 목표 감속 시간(초)": REAR_END_DECELERATION_SECONDS,
        },
        view_controller=view_controller,
        rear_end_braking=True,
    )
import carla
import math
import random
from videodownload_module_system import AccidentDataExtractor, IMG_WIDTH, IMG_HEIGHT
