import carla
import random
import sys
import pandas as pd
from datetime import datetime

from carla_connection import connect_simulator
from traffic_add import BackgroundTrafficManager
from videodownload_module_system import OUTPUT_DIR
from scenarios import (
    ScenarioViewController,
    run_rear_end_scenario,
    run_intersection_scenario,
    run_lane_change_scenario,
)

# ==============================================================================
# [설정 항목] 환경 및 실험 파라미터
# ==============================================================================
CARLA_HOST = '172.28.208.1'  # 현재 연결 IP
CARLA_PORT = 2000
MAP_NAME = 'Town04'
WEATHER_RANDOMIZATION_ENABLED = 1  # 1: 시나리오마다 날씨 랜덤, 0: 기본 날씨 유지
WEATHER_RANDOM_SEED = None
SPEED_RANDOMIZATION_ENABLED = 1  # 1: 시나리오별 지정 속도에 ±10% 적용, 0: 지정 속도 유지
SPEED_RANDOM_SEED = None

VEHICLE_POOL = [
    'vehicle.tesla.model3',        
    'vehicle.audi.etron',          
    'vehicle.bmw.grandtourer',     
]

# 속도 설정 (km/h)
EGO_SPEED_LIST = [40, 60]   
ADV_SPEED_LIST = [40, 70]   

# ==============================================================================
# run_intersection_scenario
# run_lane_change_scenario
# run_rear_end_scenario
# ==============================================================================
SCENARIOS = [
    run_rear_end_scenario,
    #run_intersection_scenario,
    run_lane_change_scenario,
]


def scenario_label(scenario_runner):
    labels = {
        "run_rear_end_scenario": "전방·후방 차량 충돌",
        "run_intersection_scenario": "십자교차로 사고",
        "run_lane_change_scenario": "동시 차선변경 사고",
    }
    return labels.get(scenario_runner.__name__, scenario_runner.__name__)


def apply_scenario_weather(world, rng):
    weather_options = (
        ("맑음", carla.WeatherParameters(
            cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0,
            wind_intensity=5.0, sun_altitude_angle=70.0,
        )),
        ("흐림", carla.WeatherParameters(
            cloudiness=80.0, precipitation=0.0, precipitation_deposits=0.0,
            wind_intensity=10.0, sun_altitude_angle=35.0,
        )),
        ("비", carla.WeatherParameters(
            cloudiness=90.0, precipitation=75.0, precipitation_deposits=65.0,
            wind_intensity=25.0, sun_altitude_angle=25.0,
        )),
    )
    weather_name, weather = rng.choice(weather_options)
    world.set_weather(weather)
    print(f"[날씨] {weather_name}")
    return weather_name


def main():
    client, world = connect_simulator(host=CARLA_HOST, port=CARLA_PORT, map_name=MAP_NAME)
    
    # 1. 잔해 청소
    print("\n[-] 이전 시뮬레이션의 잔해를 정리하는 중입니다...")
    actor_list = world.get_actors()
    for vehicle in actor_list.filter('vehicle.*'):
        vehicle.destroy()
    for sensor in actor_list.filter('sensor.*'):
        sensor.destroy()
    world.tick()
    print("[+] 청소 완료!")
    
    # ========================================================================
    # 배경 차량 매니저 선언
    # ========================================================================
    bg_manager = BackgroundTrafficManager(client, world)
    bg_manager.spawn_background_traffic(
        number_of_vehicles=100
    )
    view_controller = ScenarioViewController(world)

    total_scenarios = (
        len(SCENARIOS)
        * len(VEHICLE_POOL)
        * len(VEHICLE_POOL)
        * len(EGO_SPEED_LIST)
        * len(ADV_SPEED_LIST)
    )
    print(f"[실행 예정] 총 {total_scenarios}개 시나리오")
    
    sample_counter = 0
    failed_scenarios = 0
    master_dataset = []
    weather_rng = random.Random(WEATHER_RANDOM_SEED)
    speed_rng = random.Random(SPEED_RANDOM_SEED)
    
    try:
        # 조합 루프 가동
        for scenario_runner in SCENARIOS:
            current_label = scenario_label(scenario_runner)
            for ego_vehicle_bp in VEHICLE_POOL:
                for adv_vehicle_bp in VEHICLE_POOL:
                    for target_ego_speed in EGO_SPEED_LIST:
                        for target_adv_speed in ADV_SPEED_LIST:
                            sample_counter += 1
                            ego_speed = target_ego_speed
                            adv_speed = target_adv_speed
                            if SPEED_RANDOMIZATION_ENABLED:
                                ego_speed *= 1.0 + speed_rng.uniform(-0.10, 0.10)
                                adv_speed *= 1.0 + speed_rng.uniform(-0.10, 0.10)
                            print(
                                f"\n[시나리오 {sample_counter}/{total_scenarios}] "
                                f"{current_label} | "
                                f"{ego_vehicle_bp} {ego_speed:.2f}km/h -> "
                                f"{adv_vehicle_bp} {adv_speed:.2f}km/h"
                            )

                            try:
                                weather_name = None
                                if WEATHER_RANDOMIZATION_ENABLED:
                                    weather_name = apply_scenario_weather(world, weather_rng)
                                result_dict = scenario_runner(
                                    world=world,
                                    ego_bp=ego_vehicle_bp,
                                    adv_bp=adv_vehicle_bp,
                                    ego_spd_kmh=ego_speed,
                                    adv_spd_kmh=adv_speed,
                                    # 메인에서 정한 속도에 주행 중 랜덤 변동을 중복 적용하지 않는다.
                                    speed_variation_ratio=0.0,
                                    sample_count=sample_counter,
                                    view_controller=view_controller,
                                    random_seed=sample_counter,
                                )
                                if result_dict:
                                    scenario_metadata = result_dict.setdefault("scenario_metadata", {})
                                    scenario_metadata.update({
                                        "A 기준 속력(km/h)": target_ego_speed,
                                        "B 기준 속력(km/h)": target_adv_speed,
                                        "속도 랜덤화 적용": int(bool(SPEED_RANDOMIZATION_ENABLED)),
                                        "속도 랜덤화 범위(%)": 10 if SPEED_RANDOMIZATION_ENABLED else 0,
                                        "속도 랜덤 시드": SPEED_RANDOM_SEED,
                                    })
                                    if weather_name is not None:
                                        scenario_metadata["날씨"] = weather_name
                            except RuntimeError as error:
                                failed_scenarios += 1
                                result_dict = None
                                print(
                                    f"[경고] 시나리오 {sample_counter} 생성 실패, "
                                    f"다음 시나리오로 진행합니다: {error}"
                                )
                            if result_dict:
                                summary = dict(result_dict.get("metadata", {}))
                                summary.update(result_dict.get("scenario_metadata", {}))
                                if "collision" in result_dict:
                                    summary["충돌 여부"] = result_dict["collision"]
                                master_dataset.append(summary or result_dict)
                            
    except KeyboardInterrupt:
        print("\n[!] 사용자에 의해 시뮬레이션이 강제 중단되었습니다.")
        
    finally:
        view_controller.close()
        # 종료 시 배경 차량 정리
        bg_manager.clear_background_traffic()
        
        # 데이터 저장부
        if master_dataset:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            excel_path = f"{OUTPUT_DIR}{timestamp}_master_accident_dataset.xlsx"
            df = pd.DataFrame(master_dataset)
            df.to_excel(excel_path, index=False)
            print(f"\n[엑셀 출력 완료]: {excel_path}")

        print(
            f"[실행 종료] 시도 {sample_counter}/{total_scenarios}개, "
            f"성공 {sample_counter - failed_scenarios}개, "
            f"실패 {failed_scenarios}개"
        )
            
        if world is not None:
            settings = world.get_settings()
            settings.synchronous_mode = False
            world.apply_settings(settings)
            print("[+] 동기화 모드 해제 완료.")

if __name__ == '__main__':
    main()
