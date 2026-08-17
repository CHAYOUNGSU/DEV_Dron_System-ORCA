# 작업계획서 #14: RTH 동시성 수정 및 공유 control_lock 일원화 (리뷰 반영 개정판)

- 작성자: Antigravity (구현 담당)
- 대상 작업지시서: `docs/13_work_order_rth_concurrency_fix.md` (작성: Claude)
- 검수 예정: Codex (독립검수 / Review 역할)
- 대상 레포: `DEV_Dron_System-ORCA`

---

## 1. 개요 및 목적

리뷰 피드백을 반영하여 `_do_rth()`의 락 구조를 **틱(Tick)당 단일 원자적 `control_lock` 블록("상태 조회 → ORCA 계산 → 속도 명령 전송")**으로 완전히 통일하고, 검증 시나리오를 **"RTH 수행 중인 기체 자신(특히 Alpha)"에 대한 보호되지 않은 명령 개입 검증**으로 수정합니다.

이를 통해:
1. **틱 레벨 완전 원자성 보장**: 읽기와 쓰기 사이에 다른 제어 명령이 개입하여 기체 상태가 변한 뒤 낡은 속도 명령이 덮어씌워지는 시간차 경쟁 상태(Time-of-Check to Time-of-Use)를 완벽히 차단합니다.
2. **다중 기체 동시 비행 보장**: 틱 연산(수 밀리초) 동안만 락을 유지하고 `time.sleep(tick_dt)`(약 100ms)는 락 외부에서 대기하므로, 다중 기체의 RTH 스레드가 번갈아 락을 획득하며 완벽한 동시 병렬 비행을 유지합니다.
3. **취약 엔드포인트 직렬화 및 안전성 실증**: Alpha RTH 중 개별 조종 명령, RTH 중인 기체에 대한 `/api/land` 등의 호출이 `control_lock`을 통해 안전하게 직렬화됨을 실측으로 입증합니다.

---

## 2. 상세 구현 계획

### 2.1 `server.py`의 `_do_rth()` 단일 락 원자화 구현

```python
def _do_rth(target_drone_id: str):
    # 1. 원자적 RTH 중복 진입 검사 및 등록 (rth_lock)
    with rth_lock:
        if target_drone_id in rth_in_progress:
            print(f"[RTH] ⚠️ {target_drone_id}는 이미 RTH 진행 중입니다.", flush=True)
            return False
        rth_in_progress.add(target_drone_id)

    try:
        w_off = DRONES_CONFIG[target_drone_id].get("spawn_offset", (0.0, 0.0, 0.0))
        home_wx, home_wy = w_off[0], w_off[1]

        # 초기 기체 상태 조회 (공유 클라이언트 + 락)
        with control_lock:
            ctrl = get_control_client()
            v_name = get_real_vehicle_name(ctrl, target_drone_id)
            ensure_api_control(ctrl, v_name)
            s = ctrl.getMultirotorState(vehicle_name=v_name)
            p = s.kinematics_estimated.position

        # 틱 단위 ORCA 속도 제어 헬퍼 (틱당 1회 단일 락 원자적 처리)
        def run_rth_orca_leg(target_wpos, max_speed, max_vz, tol_3d, max_sec, leg_name):
            t_start = time.time()
            tick_dt = FOLLOW_TICK_INTERVAL_SEC
            while time.time() - t_start < max_sec:
                # [단일 락 원자적 블록 시작]: 읽기 -> ORCA 계산 -> 쓰기 전체를 1회 락으로 보호
                with control_lock:
                    ctrl = get_control_client()
                    s_curr = ctrl.getMultirotorState(vehicle_name=v_name)
                    p_curr = s_curr.kinematics_estimated.position
                    v_curr = s_curr.kinematics_estimated.linear_velocity

                    cur_wpos = (p_curr.x_val + w_off[0], p_curr.y_val + w_off[1], p_curr.z_val + w_off[2])
                    cur_wvel = (v_curr.x_val, v_curr.y_val, v_curr.z_val)

                    tx, ty, tz = target_wpos
                    dx, dy, dz = tx - cur_wpos[0], ty - cur_wpos[1], tz - cur_wpos[2]
                    dist_2d = math.sqrt(dx**2 + dy**2)
                    dist_3d = math.sqrt(dist_2d**2 + dz**2)

                    # 목표 도달 판정
                    if dist_3d <= tol_3d and (time.time() - t_start > 0.5):
                        print(f"[RTH] ✅ [{target_drone_id}] {leg_name} 완료 (오차={dist_3d:.2f}m, 소요={time.time()-t_start:.1f}초)", flush=True)
                        break

                    # 선호 속도 계산
                    desired_speed = min(max_speed, dist_2d / 0.8) if dist_2d > 0.05 else 0.0
                    pref_vx = (dx / dist_2d) * desired_speed if dist_2d > 0.05 else 0.0
                    pref_vy = (dy / dist_2d) * desired_speed if dist_2d > 0.05 else 0.0
                    pref_vz = dz / 0.5

                    # 이웃 기체 텔레메트리 수집
                    neighbors = []
                    for other_id in DRONES_CONFIG.keys():
                        if other_id == target_drone_id:
                            continue
                        other_t = latest_telemetries.get(other_id)
                        if not other_t or not other_t.get("connected", False):
                            continue
                        neighbors.append({
                            "pos": (other_t["x"], other_t["y"], other_t["z"]),
                            "vel": (other_t["vx"], other_t["vy"], other_t["vz"]),
                            "radius": ORCA_AGENT_RADIUS_M,
                            "weight": 0.5
                        })

                    # ORCA 2D + Z 안전 속도 계산
                    safe_vx, safe_vy, safe_vz = orca.compute_safe_velocity(
                        agent_pos=cur_wpos,
                        agent_vel=cur_wvel,
                        preferred_vel=(pref_vx, pref_vy, pref_vz),
                        neighbors=neighbors,
                        agent_radius=1.7,
                        time_horizon=ORCA_TIME_HORIZON_SEC,
                        max_speed=max_speed,
                        max_vz=max_vz,
                        time_step=tick_dt
                    )

                    # 속도 명령 전달 (원자적 쓰기)
                    ensure_api_control(ctrl, v_name)
                    ctrl.moveByVelocityAsync(safe_vx, safe_vy, safe_vz, tick_dt * 1.5, vehicle_name=v_name)
                # [단일 락 블록 종료]

                # 락 외부에서 주기 대기 (타 RTH 스레드 및 관제 API 락 획득 기회 제공)
                time.sleep(tick_dt)

        # Leg 1: 안전 고도 상승 (15m 상승, 최소 -15m)
        safe_climb_z = min(p.z_val - 15.0, -15.0)
        run_rth_orca_leg((p.x_val + w_off[0], p.y_val + w_off[1], safe_climb_z), max_speed=2.0, max_vz=ORCA_MAX_VZ_MPS, tol_3d=0.8, max_sec=12.0, leg_name="Leg 1 상승")

        # Leg 2: 홈 원점 수평 복귀
        run_rth_orca_leg((home_wx, home_wy, safe_climb_z), max_speed=4.0, max_vz=1.0, tol_3d=0.8, max_sec=25.0, leg_name="Leg 2 수평 복귀")

        # Leg 3: 홈 상공 하강
        run_rth_orca_leg((home_wx, home_wy, -3.0), max_speed=1.5, max_vz=1.5, tol_3d=0.8, max_sec=12.0, leg_name="Leg 3 감속 하강")

        # 4. 정밀 착륙 (공유 클라이언트 + 락)
        with control_lock:
            ctrl = get_control_client()
            ctrl.landAsync(vehicle_name=v_name).join()
            ctrl.armDisarm(False, vehicle_name=v_name)
            ctrl.enableApiControl(False, vehicle_name=v_name)
        return True

    finally:
        with rth_lock:
            rth_in_progress.discard(target_drone_id)
```

---

### 2.2 테스트 스크립트 보강 (`test_orca_rth.py`)

1. **다중 기체 동시 RTH ORCA 실측**:
   - Bravo(Drone2)와 Charlie(Drone3) 동시 RTH 비행 중첩 시간 $overlap\_sec \ge 5.0s$ 실측.
   - 20Hz 독립 샘플러: 유효 샘플($\ge 40$), 무충돌(0회), 최소 이격 거리($\ge 3.0m$), 착륙 정합성($\le 1.5m$).
   - 원자적 동일 기체 중복 RTH 요청 방어 (`status: error` 또는 `ignored`).
2. **신규 핵심 취약 지점 검증 (리뷰 반영)**:
   - **시나리오 A (Alpha RTH 중 Alpha 조종 명령 개입)**:
     - 편대장 Alpha(Drone1)를 전방으로 이동시킨 후 RTH를 실행.
     - Alpha가 RTH 비행 중인 도중에 Alpha를 대상으로 `/api/rotate` (스캔 회전) 또는 조이스틱 명령을 유입시킴.
     - `control_lock`을 통해 RPC 충돌(소켓 깨짐/메모리 충돌) 없이 안전하게 직렬화 처리되는지 검증.
   - **시나리오 B (RTH 진행 중인 기체에 `/api/land` 비보호 엔드포인트 개입)**:
     - RTH 진행 중인 기체(Bravo)에 `/api/land` 명령을 직접 호출하여, `control_lock`을 통해 경쟁 상태 없이 순차 처리되고 서버 및 기체 제어가 크래시 없이 정상 완료되는지 실측 검증.

---

## 3. 검증 계획

1. **Python 구문 검사**:
   ```bash
   python -m py_compile server.py test_orca_rth.py
   ```
2. **ORCA 단위 테스트**:
   ```bash
   python test_orca_unit.py
   ```
3. **공유 클라이언트 기반 동시 RTH 실측 및 취약 엔드포인트 개입 실측**:
   ```bash
   python test_orca_rth.py
   ```
4. **전체 회귀 테스트**:
   - `python test_orca_formation_assemble.py`
   - `python test_orca_collision_avoidance.py`
   - `python test_ui_playwright.py`

---

## 4. 산출물

- `server.py`
- `test_orca_rth.py`
- `orca_rth_report.json`
- `docs/15_completion_report_rth_concurrency_fix.md`
- `docs/00_INDEX.md`
