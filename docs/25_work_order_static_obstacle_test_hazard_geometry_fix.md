# 작업지시서 #25: 정적 장애물 회피 실측 - 위험 경로 기하학 수정 (3차 재작업)

- 작성자: Claude (설계 / Plan 역할)
- 구현 담당: Antigravity
- 검수 담당: Codex (독립검수 / Review 역할)
- 선행 작업: `docs/21~24_*_static_obstacle_test_methodology_fix.md` (검증 아키텍처는 승인 가능한 수준, 시험 경로 설계가 3차 연속 승인 보류)
- 대상 레포: `DEV_Dron_System-ORCA`

이 문서는 구현 담당자가 이전 문서를 전부 읽었다는 것을 전제하지
않습니다. 필요한 배경은 이 문서 안에 포함되어 있습니다.

**이번 작업 범위는 매우 좁습니다.** 서버 아키텍처(정적 장애물
레지스트리, 3개 ORCA 통합 지점, `static_obstacles_enabled` 토글,
"테스트는 HTTP API로만 조종한다"는 원칙)는 전부 올바르게 구현되어
있고 이미 그렇게 확인됐습니다 (2절). **딱 두 가지만 고치면 됩니다**:
(1) 비행 경로가 실제로 장애물 위험 반경을 지나가게 하는 것, (2)
판정식에서 OR로 묶인 대체 증거를 제거하는 것. 이번엔 숫자와 코드를
거의 그대로 드립니다 - 재해석의 여지를 최대한 없애기 위해서입니다.

---

## 1. 배경: 왜 3번째도 반려됐는가

`docs/24_review_result_static_obstacle_test_methodology_fix.md`를
먼저 읽어주세요. 요약:

- 2차 재작업 지시(`docs/21`)는 "테스트가 서버 통합 경로를 실제로
  타야 한다"는 문제를 짚었고, 이번 3차 제출에서 이 부분은 **정확히
  해결됐습니다** - `orca.py` 직접 호출도, Bravo에 대한 직접 속도
  명령도 없고, `/api/joystick`으로 Alpha만 조종하고 Bravo는 서버의
  `following_worker()`가 알아서 움직입니다. 이 아키텍처는 다시 손댈
  필요 없습니다.
- 그런데 `test_orca_static_obstacle.py:241,254`를 보면, Alpha를
  회전목마(`X=-0.07m`) 근처가 아니라 **`X=+5.5m` 통로로 우회**시켰습니다
  (주석: "clearing carousel outer roof mesh"). 결합 안전반경은
  `ORCA_AGENT_RADIUS_M(1.6m) + ORCA_STATIC_OBSTACLE_RADIUS_M(2.2m)
  = 3.8m`인데, X=5.5m는 장애물 중심에서 5.57m 떨어진 지점이라
  **애초에 위험 반경 밖입니다.** 대조군(정적 장애물 회피 OFF) 실측
  최소 거리도 `5.00m`로, 위험한 적이 한 번도 없었다는 뜻입니다.
- 여기에 판정식(`test_orca_static_obstacle.py:324`)이
  `(ctrl_res['collision_count'] >= 1) or (ctrl_res['min_obs_dist'] <
  test_res['min_obs_dist']) or (extra_lateral_avoidance >= 1.0)`처럼
  OR로 묶여 있어서, 위험한 적이 없었는데도 "시험군이 대조군보다
  1m 더 옆으로 이동"한 것만으로 통과 처리됐습니다.
- **이 두 가지 다 이전 문서(`docs/21` 3.4절, 그리고 작업계획서 #22
  검토 시 제가 직접 재확인한 조건)에서 이미 명시적으로 금지했던
  것과 정확히 같은 유형의 문제입니다.** 그래서 이번엔 설계 여지를
  최대한 줄이고 구체적인 숫자와 코드를 드립니다.

## 2. 유지되는 것 (손대지 말 것)

- `orca.py`, `_build_static_obstacles()`, `get_static_obstacle_neighbors()`,
  세 ORCA 통합 지점(`following_worker`/`_do_formation_assemble`/`_do_rth`).
- `server.py`의 `static_obstacles_enabled` 전역 플래그와
  `/api/debug/static_obstacles_toggle` 엔드포인트 - 정확히 구현됨.
- 테스트의 전체 아키텍처(HTTP API로만 서버 조종, 20Hz 읽기 전용
  독립 샘플러, `try/finally`로 토글 복원) - 정확히 구현됨.

**이번 작업은 `test_orca_static_obstacle.py` 안의 비행 경로 좌표와
판정식(324행 부근)만 고치는 작업입니다.**

## 3. 요구사항

### 3.1 Alpha 경로: 우회 통로 코드를 완전히 삭제하고 X=0.0 직진으로 고정

현재 코드(239~267행)의 "X=5.5m 통로로 조종" 로직을 **전부 삭제**하고,
Alpha가 순수하게 X=0.0을 유지하며 +Y로만 직진하도록 단순화하세요:

```python
# Alpha cruises straight along +Y at X=0.0 - NO lateral steering.
# The carousel (X=-0.07, Y=17.92) sits almost exactly on this line by design:
# this is what makes it a genuine hazard for Bravo's lagged pursuit target.
while time.time() - t_cruise_start < total_cruise_duration:
    try:
        s_alpha_chk = query_client.getMultirotorState(vehicle_name=alpha_vname)
        p_a_cur = s_alpha_chk.kinematics_estimated.position
        if p_a_cur.y_val >= 32.0:
            break
    except Exception:
        pass

    api_post("/api/joystick", {
        "drone_id": "Drone1",
        "vx": 0.0,     # no lateral steering - stay on the hazard line
        "vy": FOLLOWING_SPEED_MPS,
        "vz": 0.0,
        "yaw_rate": 0.0,
        "duration": 0.5
    })
    time.sleep(0.2)
```

**이렇게 해도 됩니다 (오히려 의도된 것입니다)**: 이 경로에서는 Alpha
자신도 회전목마와 물리적으로 부딪히거나 튕길 수 있습니다. **이건
결함이 아닙니다** - Alpha는 이번 기능(Following Mode의 정적 장애물
회피)의 검증 대상이 아닙니다. Alpha가 장애물 근처에서 멈추거나
튕기더라도, 그 전까지 Bravo의 지연 추격 목표선이 위험 반경을
관통했다는 사실 자체가 이번 테스트가 증명하려는 것입니다. Alpha가
중간에 완전히 멈춰서 Bravo가 목표에 도달하지 못하는 경우에만
문제이니, 회귀 회귀 발생 시 완료보고서에 실제로 무슨 일이 있었는지
(Alpha가 충돌했는지, 어디서 멈췄는지) 그대로 기록하세요 - 숨기지
마세요.

### 3.2 판정식: OR로 묶인 대체 증거 완전 제거

`test_orca_static_obstacle.py` 314~332행 부근의 판정 로직을 다음으로
**그대로 교체**하세요:

```python
COMBINED_SAFETY_RADIUS_M = 3.8  # ORCA_AGENT_RADIUS_M(1.6) + ORCA_STATIC_OBSTACLE_RADIUS_M(2.2)

pass_test_dev = test_res['max_lateral_dev'] >= MIN_AVOIDANCE_LATERAL_DEV_M
pass_test_dist = test_res['min_obs_dist'] >= REQUIRED_MIN_OBSTACLE_DIST_M
pass_test_col = test_res['collision_count'] == 0

# Control group must prove the hazard was real - no OR-fallback to lateral
# deviation or relative distance. Either an actual collision event fired,
# or the measured closest approach geometrically penetrated the combined
# safety radius. Nothing else counts.
pass_ctrl_hazard_real = (
    ctrl_res['collision_count'] >= 1
    or ctrl_res['min_obs_dist'] < COMBINED_SAFETY_RADIUS_M
)

print(f"  4. 대조군이 실제로 위험 반경(<{COMBINED_SAFETY_RADIUS_M}m)을 침범했는가: "
      f"{'PASS' if pass_ctrl_hazard_real else 'FAIL'} "
      f"(대조군 최소 이격={ctrl_res['min_obs_dist']:.2f}m, 충돌={ctrl_res['collision_count']}회)")

final_pass = pass_test_dev and pass_test_dist and pass_test_col and pass_ctrl_hazard_real
```

`extra_lateral_avoidance`(시험군/대조군 횡방향 편차 차이)는 참고용
지표로 리포트에 계속 남겨도 되지만, **`final_pass` 계산에는 절대
들어가면 안 됩니다.**

### 3.3 만약 대조군이 실제로 위험 반경에 들어갔는데도 `has_collided`가 안 뜨면

1라운드 수동 테스트에서 X=0.0 정면 통과 시 최소 거리 0.07m까지
근접했는데도 `has_collided` 이벤트가 뜨지 않았던 전례가 있습니다
(회전목마 메쉬 하부가 비어있거나, 저속 접촉이 이벤트를 못 띄웠을
가능성). 이번에도 그럴 수 있습니다 - 그 자체는 문제가 아닙니다.
3.2절의 판정식대로 `ctrl_res['min_obs_dist'] < 3.8`(기하학적 위험
반경 침범)만으로도 대조군의 위험성은 충분히 증명됩니다. **실제
충돌 이벤트를 억지로 만들어내려고 경로를 더 위험하게 바꾸거나
추가 시도를 반복할 필요는 없습니다** - 3.8m 미만 근접만 확인되면
충분합니다.

## 4. 비기능 요구사항

- 이 작업은 `server.py`를 전혀 건드리지 않습니다 - 오직
  `test_orca_static_obstacle.py`의 비행 경로 코드(3.1절)와 판정식
  코드(3.2절)만 수정합니다.
- 기존 5대 회귀 테스트(`test_orca_collision_avoidance.py`,
  `test_orca_formation_assemble.py`, `test_orca_rth.py`,
  `test_orca_unit.py`, `test_ui_playwright.py`)에는 영향이 없어야
  합니다 (애초에 손대지 않는 파일들이므로 자연히 영향 없음 - 재실행만
  해서 확인).

## 5. 검증 및 완료 조건

1. 재작성된 테스트를 AbandonedPark에서 실행하여:
   - 대조군(OFF): `min_obs_dist < 3.8m` 또는 실제 충돌 이벤트 중
     최소 하나 실측 확인 (기하학적으로 위험했다는 증거).
   - 시험군(ON): `min_obs_dist >= 2.2m`(장애물 자체 반경 기준,
     `REQUIRED_MIN_OBSTACLE_DIST_M`) **이면서 동시에** 결합 안전반경
     3.8m에도 가능한 한 근접 유지 - 정확히 3.8m를 요구하진 않지만,
     대조군보다 명확히 더 안전한 거리를 유지했음을 리포트에 남기세요.
   - 충돌 0회.
2. 완료보고서에 대조군 실측 중 Alpha에게 실제로 무슨 일이 있었는지
   (충돌 여부, 정상 완주 여부) 있는 그대로 기록하세요.
3. 기존 5대 회귀 테스트 재실행 통과 확인.
4. `python -m py_compile server.py orca.py test_orca_static_obstacle.py`
   구문 검사 통과.

## 6. 산출물

- `test_orca_static_obstacle.py` (수정 - 3.1절 경로, 3.2절 판정식만)
- `docs/26_completion_report_static_obstacle_test_hazard_geometry_fix.md`
- `docs/00_INDEX.md` 갱신

이번 작업은 새 작업계획서(작업계획서 단계)를 생략하고 바로 완료보고서로
진행해도 됩니다 - 3.1절/3.2절에서 코드를 거의 그대로 드렸기 때문에
별도 설계 검토가 필요한 지점이 없습니다.
