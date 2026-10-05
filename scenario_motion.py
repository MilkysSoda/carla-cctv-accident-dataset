"""두 시나리오의 속도 변동과 물리 조향 제어. 속도 단위는 km/h."""

import math
import random

import carla


class RandomSpeedProfile:
    """독립 난수 목표를 선형 보간해 입력 속도의 ±ratio 범위를 유지한다."""

    def __init__(self, nominal_kmh, ratio, interval, duration, seed):
        if not math.isfinite(nominal_kmh) or nominal_kmh < 0:
            raise ValueError("입력 속도는 유한한 0 이상의 값이어야 합니다.")
        if not math.isfinite(ratio) or not 0 <= ratio <= 0.10:
            raise ValueError("속도 변동 비율은 0~0.10이어야 합니다.")
        if not math.isfinite(interval) or interval <= 0:
            raise ValueError("속도 갱신 간격은 양수여야 합니다.")
        self.interval = interval
        rng = random.Random(seed)
        self.values = [
            nominal_kmh * (1.0 + rng.uniform(-ratio, ratio))
            for _ in range(math.ceil(duration / interval) + 2)
        ]

    def at(self, elapsed):
        position = max(0.0, elapsed) / self.interval
        index = min(int(position), len(self.values) - 2)
        fraction = min(1.0, position - index)
        return self.values[index] + (self.values[index + 1] - self.values[index]) * fraction


def same_direction(first, second):
    a = first.transform.get_forward_vector()
    b = second.transform.get_forward_vector()
    return a.x * b.x + a.y * b.y > 0.9


def adjacent_driving_lane(waypoint, side):
    lane = (waypoint.get_left_lane() if side == "left"
            else waypoint.get_right_lane())
    if (lane is None or lane.lane_type != carla.LaneType.Driving
            or not same_direction(waypoint, lane)):
        raise ValueError("차선변경 경로에 같은 진행 방향의 인접 주행 차선이 없습니다.")
    return lane


def advance_waypoint(waypoint, distance):
    """앞/뒤로 이동하되 분기에서는 현재 방향과 가장 가까운 경로를 택한다."""
    if abs(distance) < 1e-6:
        return waypoint
    candidates = (waypoint.next(distance) if distance > 0
                  else waypoint.previous(-distance))
    if not candidates:
        raise ValueError("요청한 거리만큼 이어지는 도로가 없습니다.")
    forward = waypoint.transform.get_forward_vector()
    return max(candidates, key=lambda wp: (
        forward.x * wp.transform.get_forward_vector().x
        + forward.y * wp.transform.get_forward_vector().y
    ))


class LaneChangeController:
    """CARLA waypoint 경로를 따라 지정 시각에 한 번 차선을 변경한다."""

    def __init__(self, waypoint, side, route_length, start_time, duration, steering_sign=1.0):
        if start_time < 0 or duration <= 0:
            raise ValueError("차선변경 시작 시각은 0 이상, 소요 시간은 양수여야 합니다.")
        self.start_time = start_time
        self.duration = duration
        self.steering_sign = steering_sign
        self.route = []
        target = adjacent_driving_lane(waypoint, side)
        step = 2.0

        if route_length <= 0:
            raise ValueError("차선변경 경로 길이는 양수여야 합니다.")

        # 각 진행 위치에 원래 차선과 목표 차선 waypoint를 함께 저장한다.
        current = waypoint
        distance = 0.0
        while distance <= route_length:
            target = adjacent_driving_lane(current, side)
            self.route.append((current.transform.location, target.transform.location))
            candidates = current.next(step)
            if not candidates:
                break
            current = candidates[0]
            distance += step

        # 차선변경 후에는 목표 차선 waypoint를 계속 앞에 두어 마지막 점을 지나도
        # 뒤쪽 점으로 되돌아 조향하지 않게 한다.
        target_current = target
        distance = 0.0
        while distance < 45.0:
            self.route.append((target_current.transform.location, target_current.transform.location))
            candidates = target_current.next(step)
            if not candidates:
                break
            target_current = candidates[0]
            distance += step

        if len(self.route) < 2:
            raise ValueError("차선변경 waypoint 경로를 만들 수 없습니다.")

    def control(self, vehicle, elapsed, speed_kmh):
        location = vehicle.get_location()
        elapsed_since_start = max(0.0, elapsed - self.start_time)
        distance_since_start = elapsed_since_start * max(speed_kmh, 1.0) / 3.6
        target_index = min(
            len(self.route) - 1,
            math.floor(distance_since_start / 2.0),
        )
        source, destination = self.route[target_index]
        progress = max(0.0, min(1.0, (elapsed - self.start_time) / self.duration))
        if progress >= 1.0:
            return carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0)

        target = carla.Location(
            x=source.x + (destination.x - source.x) * progress,
            y=source.y + (destination.y - source.y) * progress,
            z=source.z,
        )
        dx = target.x - location.x
        dy = target.y - location.y
        forward = vehicle.get_transform().get_forward_vector()
        right = vehicle.get_transform().get_right_vector()
        longitudinal = dx * forward.x + dy * forward.y
        lateral = dx * right.x + dy * right.y
        lookahead_distance = max(longitudinal, 5.0)
        steering_angle = math.atan2(2.0 * 2.8 * lateral, lookahead_distance ** 2)
        steering_limit = math.radians(25.0 if progress < 1.0 else 12.0)
        # CARLA의 right 벡터 기준 lateral 부호를 VehicleControl.steer에 그대로 사용한다.
        steering = self.steering_sign * steering_angle / steering_limit
        steering = max(-1.0, min(1.0, steering))
        return carla.VehicleControl(throttle=0.0, steer=steering, brake=0.0)
