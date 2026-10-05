"""Town04의 모든 차량 스폰포인트를 CARLA 화면에 표시한다.

노란색 번호는 get_spawn_points()의 인덱스이고, 초록색 화살표는 차량의
초기 진행방향이다. 이 인덱스를 시나리오의 기준 스폰 위치로 기록한다.
"""

import argparse
import time
from typing import List

import carla

from carla_connection import connect_simulator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Town04 spawn point viewer")
    parser.add_argument("--host", default="172.31.160.1", help="CARLA 서버 IP")
    parser.add_argument("--port", type=int, default=2000, help="CARLA 서버 포트")
    parser.add_argument(
        "--duration",
        type=float,
        default=180.0,
        help="표시 유지 시간(초). 0이면 Ctrl+C를 누를 때까지 유지",
    )
    parser.add_argument(
        "--focus",
        type=int,
        default=45,
        help="시작 시 관전자 화면을 맞출 스폰포인트 인덱스",
    )
    return parser.parse_args()


def set_spectator_near_spawn(world: carla.World, spawn: carla.Transform) -> None:
    """선택한 스폰포인트가 확실히 보이도록 관전자를 수직 상공으로 이동한다."""
    location = spawn.location
    spectator_transform = carla.Transform(
        carla.Location(x=location.x, y=location.y, z=55.0),
        carla.Rotation(pitch=-90.0, yaw=0.0),
    )
    world.get_spectator().set_transform(spectator_transform)


def draw_spawn_points(world: carla.World, life_time: float) -> List[carla.Transform]:
    """모든 스폰포인트의 번호와 초기 진행방향을 디버그 레이어에 그린다."""
    spawn_points = world.get_map().get_spawn_points()

    for index, transform in enumerate(spawn_points):
        label_location = transform.location + carla.Location(z=0.8)
        forward = transform.get_forward_vector()
        arrow_end = transform.location + carla.Location(
            x=forward.x * 5.0,
            y=forward.y * 5.0,
            z=0.5,
        )

        world.debug.draw_string(
            label_location,
            f"SP {index}",
            draw_shadow=True,
            color=carla.Color(255, 230, 0),
            life_time=life_time,
        )
        world.debug.draw_arrow(
            transform.location + carla.Location(z=0.25),
            arrow_end,
            thickness=0.15,
            arrow_size=0.45,
            color=carla.Color(0, 255, 80),
            life_time=life_time,
        )

    return spawn_points


def main() -> None:
    args = parse_args()
    # main.py와 같은 연결 함수를 사용한다. 이 함수는 Town04 로드 및
    # 동기화 모드 설정까지 처리하므로, 본 뷰어도 실험 코드와 동일한 월드를 본다.
    _, world = connect_simulator(
        host=args.host,
        port=args.port,
        map_name="Town04",
    )
    print(f"[INFO] 연결 완료: {world.get_map().name}")

    # duration=0이면 주기적으로 다시 그려서 Ctrl+C까지 계속 보이게 한다.
    draw_life_time = args.duration if args.duration > 0 else 10.0
    spawn_points = draw_spawn_points(world, draw_life_time)

    if not 0 <= args.focus < len(spawn_points):
        raise ValueError(f"focus는 0~{len(spawn_points) - 1} 범위여야 합니다.")

    focus = spawn_points[args.focus]
    waypoint = world.get_map().get_waypoint(focus.location)
    set_spectator_near_spawn(world, focus)

    print(f"[INFO] Town04 스폰포인트 수: {len(spawn_points)}")
    print(f"[INFO] focus=SP {args.focus}")
    print(f"       location={focus.location}")
    print(f"       rotation={focus.rotation}")
    print(f"       road_id={waypoint.road_id}, lane_id={waypoint.lane_id}")
    print("[INFO] 노란색: 스폰포인트 번호 / 초록색: 초기 차량 진행 방향")

    start_time = time.monotonic()
    try:
        while args.duration == 0 or time.monotonic() - start_time < args.duration:
            if world.get_settings().synchronous_mode:
                world.tick()
                time.sleep(0.05)  # 화면 확인용: 불필요하게 빠른 시뮬레이션 진행을 방지
            else:
                world.wait_for_tick(1.0)

            if args.duration == 0 and int(time.monotonic() - start_time) % 8 == 0:
                draw_spawn_points(world, draw_life_time)
                time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[INFO] 사용자가 종료했습니다.")


if __name__ == "__main__":
    main()
