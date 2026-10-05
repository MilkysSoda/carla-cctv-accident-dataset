# background_traffic.py
import carla
import random
from typing import Iterable, Optional

class BackgroundTrafficManager:
    def __init__(self, client: carla.Client, world: carla.World):
        self.client = client
        self.world = world
        
        # ==========================================================
        # [스마트 포트 할당 로직] 8000번이 막혀있으면 알아서 다음 포트를 찾습니다!
        # ==========================================================
        tm_port = 8000
        while True:
            try:
                self.traffic_manager = client.get_trafficmanager(tm_port)
                self.traffic_manager.set_synchronous_mode(True)
                print(f"[+] Traffic Manager가 {tm_port}번 포트로 성공적으로 바인딩되었습니다.")
                break  # 성공하면 루프 탈출
            except RuntimeError as e:
                # 에러 메시지에 bind error가 포함되어 있다면 다음 포트로 시도
                if "bind error" in str(e).lower():
                    print(f"[-] {tm_port}번 포트가 사용 중입니다. {tm_port + 1}번 포트를 시도합니다...")
                    tm_port += 1
                else:
                    # 포트 문제가 아닌 진짜 에러라면 강제 종료
                    raise e
        # ==========================================================
        
        self.bg_vehicles = []

    def spawn_background_traffic(
        self,
        number_of_vehicles: int = 15,
        safe_transform: Optional[carla.Transform] = None,
        safe_transforms: Optional[Iterable[carla.Transform]] = None,
        safe_distance: float = 200.0,
    ):
        """
        맵 전역에 스스로 주행하는 주변 배경 차량을 생성합니다.
        """
        blueprints = self.world.get_blueprint_library().filter('vehicle.*')
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)
        protected_transforms = list(safe_transforms or [])
        if safe_transform is not None:
            protected_transforms.append(safe_transform)
        
        count = 0
        for spawn_point in spawn_points:
            if count >= number_of_vehicles:
                break
                
            # =========================================================
            # [핵심 수정] 같은 도로, "같은 차선(lane_id)"인지 확인하여 옆 차선은 허용!
            is_protected = False
            for protected_transform in protected_transforms:
                target_wp = self.world.get_map().get_waypoint(protected_transform.location)
                spawn_wp = self.world.get_map().get_waypoint(spawn_point.location)
                
                # 도로 ID와 차선 ID가 완벽히 일치할 때만(즉, 앞뒤로 일직선 상에 있을 때만)
                if target_wp.road_id == spawn_wp.road_id and target_wp.lane_id == spawn_wp.lane_id:
                    # 사고 발생 지점 기준 반경 200m 이내면 스폰 배제
                    if spawn_point.location.distance(protected_transform.location) < safe_distance:
                        is_protected = True
                        break

            if is_protected:
                continue
            # =========================================================
            
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('number_of_wheels'):
                if int(blueprint.get_attribute('number_of_wheels')) < 4:
                    continue
            
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            
            blueprint.set_attribute('role_name', 'background')
            
            vehicle = self.world.try_spawn_actor(blueprint, spawn_point)
            if vehicle is not None:
                vehicle.set_autopilot(True, self.traffic_manager.get_port())  # type: ignore
                self.traffic_manager.distance_to_leading_vehicle(vehicle, 5.0)
                self.traffic_manager.ignore_lights_percentage(vehicle, 0)
                self.traffic_manager.ignore_signs_percentage(vehicle, 0)
                
                self.bg_vehicles.append(vehicle)
                count += 1

        for _ in range(20):
            self.world.tick()
                
        print(f"[+] 배경 차량 {len(self.bg_vehicles)}대가 도로 위에 스폰되어 주행을 시작했습니다.")

    def clear_background_traffic(self):
        """
        생성했던 주변 배경 차량들을 모두 안전하게 제거합니다.
        """
        destroyed_count = 0
        for vehicle in self.bg_vehicles:
            if vehicle and vehicle.is_alive:
                vehicle.destroy()
                destroyed_count += 1
        self.bg_vehicles.clear()
        self.world.tick()
        print(f"[-] 배경 차량 {destroyed_count}대 정리 완료.")
