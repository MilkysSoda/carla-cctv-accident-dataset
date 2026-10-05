import carla

def clean_and_recover():
    # 1. 시뮬레이터 연결
    client = carla.Client('172.22.48.1', 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    # 2. 삭제할 액터 유형 모으기 (차량, 센서, 보행자, AI 컨트롤러)
    target_patterns = ['vehicle.*', 'sensor.*', 'walker.*', 'controller.ai.walker']
    
    total_destroyed = 0
    for pattern in target_patterns:
        actors = world.get_actors().filter(pattern)
        for actor in actors:
            if actor.is_alive:
                actor.destroy()
                total_destroyed += 1

    # 3. 삭제 명령이 서버에 반영되도록 강제로 1프레임 굴림
    world.tick()

    # 4. 동기화 모드가 켜진 채로 스크립트가 끊겨서 서버가 멈춘(Freeze) 경우를 대비해 일반 비동기 모드로 복구
    settings = world.get_settings()
    if settings.synchronous_mode:
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print("[!] 멈춰있던 동기화 모드를 비동기 모드로 강제 해제했습니다.")

    print(f"[+] 총 {total_destroyed}개의 액터를 싹 지우고 시뮬레이터 렉을 해소했습니다!")

if __name__ == '__main__':
    clean_and_recover()