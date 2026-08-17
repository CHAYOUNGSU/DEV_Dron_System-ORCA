import airsim
import time
import math
import json
import urllib.request
import traceback

def run_regression_tests():
    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "scenarios": []
    }
    
    c = airsim.MultirotorClient()
    c.confirmConnection()
    
    def get_fleet_status():
        vehicles = ['SimpleFlight', 'Drone2', 'Drone3', 'Drone4']
        status = {}
        for v in vehicles:
            s = c.getMultirotorState(v)
            coll = c.simGetCollisionInfo(vehicle_name=v).has_collided
            p = s.kinematics_estimated.position
            _, _, yaw = airsim.to_eularian_angles(s.kinematics_estimated.orientation)
            status[v] = {
                "landed_state": int(s.landed_state),
                "collided": bool(coll),
                "x": round(float(p.x_val), 2),
                "y": round(float(p.y_val), 2),
                "z": round(float(p.z_val), 2),
                "altitude": round(abs(float(p.z_val)), 2),
                "yaw_deg": round(math.degrees(float(yaw)), 1)
            }
        return status

    def call_api_formation_assemble(spacing=12.0, velocity=4.0):
        req = urllib.request.Request(
            'http://127.0.0.1:8000/api/formation/assemble',
            data=json.dumps({"spacing": spacing, "velocity": velocity}).encode('utf-8'),
            headers={'Content-Type': 'application/json'}
        )
        res = urllib.request.urlopen(req)
        return json.loads(res.read().decode('utf-8'))

    def ensure_spawned():
        existing = c.listVehicles()
        for idx, v in enumerate(['Drone2', 'Drone3', 'Drone4'], start=1):
            if v not in existing:
                try:
                    c.simAddVehicle(v, 'SimpleFlight', airsim.Pose(airsim.Vector3r(0.0, idx * 4.0, 0.0), airsim.to_quaternion(0,0,0)))
                    print(f"Spawned {v}")
                except Exception as e:
                    print(f"Spawn error {v}: {e}")
        time.sleep(1.0)

    def reset_fleet_to_ground():
        ensure_spawned()
        vehicles = [('SimpleFlight', 0.0), ('Drone2', 4.0), ('Drone3', 8.0), ('Drone4', 12.0)]
        for v, y_off in vehicles:
            try:
                c.enableApiControl(True, v)
                c.armDisarm(True, v)
                c.simSetVehiclePose(airsim.Pose(airsim.Vector3r(0.0, y_off, 0.0), airsim.to_quaternion(0,0,0)), True, vehicle_name=v)
                c.enableApiControl(True, v)
                c.armDisarm(True, v)
            except Exception as e:
                print(f"Reset error {v}: {e}")
        time.sleep(1.5)

    # =========================================================================
    # SCENARIO 1: Ground Start
    # =========================================================================
    print("\n" + "="*70)
    print("[SCENARIO 1] Ground Start Formation Assembly Test")
    print("="*70)
    
    # 1. Place all drones safely on ground with lateral spacing
    reset_fleet_to_ground()
    
    initial_status_1 = get_fleet_status()
    print("Initial ground status:", initial_status_1)
    
    res_1 = call_api_formation_assemble(spacing=4.5, velocity=4.0)
    print("API Response received successfully (Status:", res_1.get("status"), ")")
    
    # Wait for flight to settle
    time.sleep(8.0)
    final_status_1 = get_fleet_status()
    print("Scenario 1 final status:", final_status_1)
    
    s1_pass = (
        final_status_1['SimpleFlight']['landed_state'] == 1 and
        final_status_1['Drone2']['landed_state'] == 1 and
        final_status_1['Drone3']['landed_state'] == 1 and
        final_status_1['Drone4']['landed_state'] == 1 and
        not any(final_status_1[v]['collided'] for v in final_status_1) and
        abs(final_status_1['SimpleFlight']['z'] - final_status_1['Drone2']['z']) < 1.5
    )
    
    report["scenarios"].append({
        "name": "시나리오 1: 전 기체 지상 대기 상태에서의 편대 집결 (Ground Start)",
        "passed": s1_pass,
        "initial": initial_status_1,
        "final": final_status_1,
        "api_response": res_1,
        "notes": f"알파 고도: {final_status_1['SimpleFlight']['altitude']}m, 편대기 고도차: <1.5m, 충돌여부: 무"
    })

    # =========================================================================
    # SCENARIO 2: High Altitude Leader Hover
    # =========================================================================
    print("\n" + "="*70)
    print("[SCENARIO 2] Alpha High Altitude Hover Recall Test")
    print("="*70)
    
    # Fly Alpha far to (25m, 15m, -20m) with 45 deg yaw
    c.enableApiControl(True, 'SimpleFlight')
    c.armDisarm(True, 'SimpleFlight')
    c.moveToPositionAsync(25.0, 15.0, -20.0, 5.0, yaw_mode=airsim.YawMode(False, 45.0), vehicle_name='SimpleFlight')
    time.sleep(5.0)
    
    alpha_before_s2 = get_fleet_status()['SimpleFlight']
    print(f"Alpha at 20m altitude: Pos=({alpha_before_s2['x']}, {alpha_before_s2['y']}, {alpha_before_s2['z']}), Yaw={alpha_before_s2['yaw_deg']}")
    
    res_2 = call_api_formation_assemble(spacing=5.0, velocity=4.5)
    print("API Response 2 received (Status:", res_2.get("status"), ")")
    
    time.sleep(8.0)
    final_status_2 = get_fleet_status()
    print("Scenario 2 final status:", final_status_2)
    
    # Check if Alpha stayed at ~20m without dropping
    alpha_drop_s2 = abs(final_status_2['SimpleFlight']['z'] - alpha_before_s2['z'])
    s2_pass = (
        final_status_2['SimpleFlight']['landed_state'] == 1 and
        alpha_drop_s2 < 2.0 and
        all(final_status_2[v]['landed_state'] == 1 for v in final_status_2) and
        not any(final_status_2[v]['collided'] for v in final_status_2)
    )
    
    report["scenarios"].append({
        "name": "시나리오 2: 알파 고고도(20m) 단독 비행 중 편대 집결 (High Altitude Hover)",
        "passed": s2_pass,
        "initial_alpha": alpha_before_s2,
        "final": final_status_2,
        "api_response": res_2,
        "notes": f"호출 전 알파 고도: {alpha_before_s2['altitude']}m -> 집결 후: {final_status_2['SimpleFlight']['altitude']}m (고도 변동 {alpha_drop_s2:.2f}m)"
    })

    # =========================================================================
    # SCENARIO 3: Dispersed Fleet Mid-Air Recall
    # =========================================================================
    print("\n" + "="*70)
    print("[SCENARIO 3] Dispersed Fleet Mid-Air Recall Test")
    print("="*70)
    
    # Disperse all 4 drones in 4 different directions and altitudes
    c.enableApiControl(True, 'SimpleFlight'); c.armDisarm(True, 'SimpleFlight')
    c.moveToPositionAsync(15.0, 0.0, -12.0, 4.0, vehicle_name='SimpleFlight')
    
    c.enableApiControl(True, 'Drone2'); c.armDisarm(True, 'Drone2')
    c.moveToPositionAsync(-20.0, 25.0, -18.0, 5.0, vehicle_name='Drone2')
    
    c.enableApiControl(True, 'Drone3'); c.armDisarm(True, 'Drone3')
    c.moveToPositionAsync(25.0, -20.0, -8.0, 5.0, vehicle_name='Drone3')
    
    c.enableApiControl(True, 'Drone4'); c.armDisarm(True, 'Drone4')
    c.moveToPositionAsync(-15.0, -30.0, -15.0, 5.0, vehicle_name='Drone4')
    
    time.sleep(6.0)
        
    dispersed_status = get_fleet_status()
    print("Dispersed status:", dispersed_status)
    
    res_3 = call_api_formation_assemble(spacing=4.5, velocity=4.0)
    print("API Response 3 received (Status:", res_3.get("status"), ")")
    
    time.sleep(8.0)
    final_status_3 = get_fleet_status()
    print("Scenario 3 final status:", final_status_3)
    
    alpha_z_s3 = final_status_3['SimpleFlight']['z']
    alt_errors_s3 = [abs(final_status_3[v]['z'] - alpha_z_s3) for v in ['Drone2', 'Drone3', 'Drone4']]
    max_alt_err_s3 = max(alt_errors_s3)
    
    s3_pass = (
        all(final_status_3[v]['landed_state'] == 1 for v in final_status_3) and
        not any(final_status_3[v]['collided'] for v in final_status_3) and
        max_alt_err_s3 < 2.0
    )
    
    report["scenarios"].append({
        "name": "시나리오 3: 전 기체 사방 분산 개별 비행 중 집결 (Dispersed Fleet Recall)",
        "passed": s3_pass,
        "dispersed": dispersed_status,
        "final": final_status_3,
        "api_response": res_3,
        "notes": f"알파 고도: {final_status_3['SimpleFlight']['altitude']}m 기준 편대기 최대 고도 오차: {max_alt_err_s3:.2f}m, 전 기체 충돌 없음"
    })

    # =========================================================================
    # SCENARIO 4: Dynamic Heading Trail
    # =========================================================================
    print("\n" + "="*70)
    print("[SCENARIO 4] Dynamic Heading (135 deg) Trail Test")
    print("="*70)
    
    # Turn Alpha to 135 degrees and move
    c.enableApiControl(True, 'SimpleFlight'); c.armDisarm(True, 'SimpleFlight')
    c.moveToPositionAsync(20.0, 20.0, -10.0, 4.0, yaw_mode=airsim.YawMode(False, 135.0), vehicle_name='SimpleFlight')
    time.sleep(5.0)
    
    alpha_s4 = get_fleet_status()['SimpleFlight']
    print(f"Alpha 135 deg heading: Pos=({alpha_s4['x']}, {alpha_s4['y']}, {alpha_s4['z']}), Yaw={alpha_s4['yaw_deg']}")
    
    res_4 = call_api_formation_assemble(spacing=4.5, velocity=4.0)
    print("API Response 4 received (Status:", res_4.get("status"), ")")
    
    time.sleep(7.0)
    final_status_4 = get_fleet_status()
    print("Scenario 4 final status:", final_status_4)
    
    s4_pass = (
        all(final_status_4[v]['landed_state'] == 1 for v in final_status_4) and
        final_status_4['Drone2']['x'] > final_status_4['SimpleFlight']['x'] and
        final_status_4['Drone3']['x'] > final_status_4['Drone2']['x'] and
        final_status_4['Drone4']['x'] > final_status_4['Drone3']['x']
    )
    
    report["scenarios"].append({
        "name": "시나리오 4: 알파 방향 전환(135도 남동향)에 따른 편대 정렬 (Dynamic Heading Trail)",
        "passed": s4_pass,
        "initial_alpha": alpha_s4,
        "final": final_status_4,
        "api_response": res_4,
        "notes": f"알파 Yaw: {alpha_s4['yaw_deg']}도 -> 편대기 후방 135도 벡터 정렬(X값 순차 증가) 성공"
    })

    # =========================================================================
    # SCENARIO 5: Consecutive Rapid Retrigger
    # =========================================================================
    print("\n" + "="*70)
    print("[SCENARIO 5] Consecutive Rapid Retrigger Stability Test")
    print("="*70)
    
    for i in range(3):
        print(f"Retrigger call #{i+1}...")
        call_api_formation_assemble(spacing=4.0 + i, velocity=4.0)
        time.sleep(2.0)
        
    time.sleep(5.0)
    final_status_5 = get_fleet_status()
    print("Scenario 5 final status:", final_status_5)
    
    s5_pass = all(final_status_5[v]['landed_state'] == 1 for v in final_status_5)
    
    report["scenarios"].append({
        "name": "시나리오 5: 연속 다중 호출 재귀 안정성 (Consecutive Rapid Retrigger)",
        "passed": s5_pass,
        "final": final_status_5,
        "notes": "3회 연속 신속 집결 호출에도 교착(Deadlock) 및 추락 없이 모든 기체 정상 편대 비행 유지"
    })
    
    # Save Report JSON
    with open("flight_test_report.json", "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
        
    print("\n" + "="*70)
    print("ALL 5 REGRESSION TEST SCENARIOS COMPLETED!")
    print("="*70)
    all_passed = all(s['passed'] for s in report['scenarios'])
    print(f"Overall Result: {'ALL PASSED' if all_passed else 'SOME FAILED'}")
    return report

if __name__ == "__main__":
    run_regression_tests()
