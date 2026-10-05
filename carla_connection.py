# testcode/carla_connection.py
import carla
import sys
from typing import Tuple

def connect_simulator(host: str = 'localhost', port: int = 2000, map_name: str = 'Town04') -> Tuple[carla.Client, carla.World]:
    """
    CARLA 시뮬레이터 서버에 연결. 
    동일한 맵이 이미 켜져 있다면 재로드하지 않고 기존 월드를 즉시 재사용.
    """
    try:
        print(f"[-] CARLA 서버 커넥션 수립 중... ({host}:{port})")
        client = carla.Client(host, port)
        client.set_timeout(30.0)
        
        current_world = client.get_world()
        current_map_name = current_world.get_map().name
        
        if map_name in current_map_name:
            print(f"[+] [최적화] 시뮬레이터에 이미 {map_name}가 로드되어 있습니다. 맵 로딩을 건너뛰고 월드를 재사용합니다.")
            world = current_world
        else:
            print(f"[-] 새로운 맵 로드 중 (시간이 다소 소요될 수 있음): {map_name}")
            world = client.load_world(map_name)
        
        # 동기화 모드 및 고정 FPS 설정
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.0333  # 30 FPS
        world.apply_settings(settings)
        
        return client, world

    except RuntimeError as e:
        print(f"[!] 시뮬레이터 서버 연결 실패: {e}")
        sys.exit(1)