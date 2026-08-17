# 작업지시서 #09: RTH(자동 복귀) 좌표계 수정 + ORCA 충돌 회피 적용

- 작성자: Claude (설계 / Plan 역할)
- 구현 담당: Antigravity
- 검수 담당: Codex (독립검수 / Review 역할)
- 선행 작업: `docs/01~04_*_following_mode_orca.md`, `docs/05~08_*_formation_assemble_orca.md` (둘 다 승인 완료)
- 대상 레포: `DEV_Dron_System-ORCA`

이 문서는 구현 담당자가 이 대화의 맥락이나 이전 문서들의 세부 내용을
전부 읽었다는 것을 전제하지 않습니다. 필요한 배경은 이 문서 안에
포함되어 있습니다. 다만 `orca.py`와 `following_worker()`,
`_do_formation_assemble()`은 이미 검수를 통과해 동작 중인 **참고
구현체**이므로, 이번 작업도 그 패턴을 최대한 재사용합니다.

---

## 1. 배경 및 목적

Following Mode(#01~#04)와 편대 집결(#05~#08)에 이어, 이번에는 **RTH
(Return To Home, 자동 복귀)** 기능을 다룹니다. RTH는 선택된 드론 1대를
15m 상승 -> 홈 좌표로 수평 이동 -> 3m 고도로 하강 -> 착륙시키는 개별
기체 명령입니다.

이번 작업은 두 가지를 함께 다룹니다 - **이 둘은 서로 독립적이지 않고
연결되어 있습니다** (2.2절 참고):

1. **좌표계 버그 수정**: `_do_rth()`가 편대 집결에서 발견했던 것과
   똑같은 "원시 로컬 좌표를 월드 좌표인 것처럼 사용" 버그를 그대로
   가지고 있습니다. 아래 2.2절에서 정확한 위치와 이유를 설명합니다.
2. **ORCA 충돌 회피 적용**: RTH 비행 경로(상승/수평이동/하강)는 지금
   다른 기체의 존재를 전혀 고려하지 않습니다. 사용자가 여러 드론을
   짧은 시간 안에 연달아 RTH시키거나(각 RTH는 총 비행시간이 길어서
   앞선 RTH가 끝나기 전에 다음 드론을 RTH시킬 수 있음), 한 드론이
   RTH로 복귀하는 동안 다른 드론이 Following Mode로 자동비행 중이거나
   그 근처에 정지해 있으면 충돌할 수 있습니다.

## 2. 현재 시스템 구조 및 확인된 문제

### 2.1 재사용할 참고 구현체

- **`orca.py`**: 수정하지 않고 그대로 재사용. 핵심 함수
  `compute_safe_velocity(agent_pos, agent_vel, preferred_vel, neighbors,
  agent_radius, time_horizon, max_speed, max_vz, time_step)`.
- **`following_worker()`** 및 **`_do_formation_assemble()`**: ORCA를
  실제 비행 제어에 통합한 두 개의 검증된 사례입니다. 공통 패턴:
  1. 목표까지의 거리 비례 감속을 적용한 선호 속도 계산
  2. `latest_telemetries`에서 자신을 제외한 나머지 기체의 실시간
     위치/속도로 `neighbors` 구성 (다른 기체는 weight 0.5, 리더 성격의
     기체는 weight 1.0 - 이번 RTH의 경우 "리더"라는 개념이 없으므로
     전부 weight 0.5로 상호 회피 취급하는 것을 권장)
  3. `orca.compute_safe_velocity(...)` 호출
  4. `ensure_api_control(ctrl, vehicle_name)` 확인 후
     `moveByVelocityAsync(vx, vy, vz, duration, vehicle_name=...)`로
     world-frame 속도 명령
  5. 목표 도달(수렴) 또는 타임아웃 시 루프 종료, `hoverAsync()`로 고정
- **ORCA 상수** (`server.py` 상단, 그대로 재사용):
  `ORCA_TIME_HORIZON_SEC=2.0`, `ORCA_AGENT_RADIUS_M=1.5`,
  `ORCA_MAX_SPEED_MPS=3.0`, `ORCA_MAX_VZ_MPS=2.0`,
  `FOLLOW_TICK_INTERVAL_SEC=0.1`.
- **충돌 텔레메트리**: 이미 구현되어 있음 (`airsim_worker()`가
  `simGetCollisionInfo` 폴링, `latest_telemetries[d_id]["collided"]`,
  `["collision_count"]`, UI에도 이미 표시됨). **손댈 필요 없음.**

### 2.2 확인된 버그: `_do_rth()`의 좌표계 오류

현재 코드 (`server.py`, `_do_rth` 함수):

```python
def _do_rth(target_drone_id: str):
    with control_lock:
        ctrl = get_control_client()
        v_name = get_real_vehicle_name(ctrl, target_drone_id)
        ensure_api_control(ctrl, v_name)

        state = ctrl.getMultirotorState(vehicle_name=v_name)
        pos = state.kinematics_estimated.position
        home_x, home_y, _ = DRONES_CONFIG[target_drone_id]["spawn_offset"]

        # 1. Climb 15m higher than current position
        safe_climb_z = min(pos.z_val - 15.0, -15.0)
        ctrl.moveToPositionAsync(pos.x_val, pos.y_val, safe_climb_z, 3.0, vehicle_name=v_name).join()

        # 2. Cruise horizontally back to Home Origin for this drone
        ctrl.moveToPositionAsync(home_x, home_y, safe_climb_z, 6.0, vehicle_name=v_name).join()

        # 3. Slow descent to 3m above home point
        ctrl.moveToPositionAsync(home_x, home_y, -3.0, 2.0, vehicle_name=v_name).join()

        # 4. Safe Precision Landing
        ctrl.landAsync(vehicle_name=v_name).join()
        ...
```

**문제**: `pos`(현재 위치)는 `getMultirotorState()`가 반환하는 **원시
로컬 좌표**입니다 - AirSim에서 각 기체의 위치는 그 기체 자신의 스폰
지점을 원점으로 합니다 (Following Mode 작업 때 확인된 사실, `airsim_worker()`가
`world_x = pos.x_val + spawn_offset[0]` 형태로 보정하는 이유입니다).

그런데 2번째 줄(수평 이동)에서 목표로 삼는 `home_x, home_y`는
`DRONES_CONFIG[target_drone_id]["spawn_offset"]` 값을 **그대로**
씁니다. 브라보(스폰 오프셋 Y=3.5)를 예로 들면:

- 브라보의 "진짜 집(월드 좌표)"은 브라보 자신의 로컬 좌표계에서
  `(0, 0)`입니다 (자기 스폰 지점이 곧 자기 로컬 원점이므로).
- 그런데 코드는 브라보에게 로컬 좌표 `(0, 3.5)`로 가라고 명령합니다.
- 즉 브라보는 자기 집에서 **3.5m 벗어난 지점**에 착륙하게 됩니다.

기체마다 스폰 오프셋이 다르므로(브라보 3.5m, 찰리 7.0m, 델타 10.5m),
이 오차도 기체마다 다르게 발생합니다. 알파(오프셋 0,0)만 우연히
정확합니다.

**참고로 확인해 둘 것 (수정 대상 아님, 정보 제공용)**: `_do_takeoff`,
`_do_fleet_takeoff`, `_do_reset`, `_do_rotate`, `_do_joystick_velocity`는
전부 "현재 위치 기준 상대 이동"이거나(이륙 후 제자리에서 3m 상승 등)
좌표 자체를 다루지 않는 방식(바디 프레임 속도, 순수 요 회전, 전체
시뮬레이션 리셋)이라 이 버그가 없는 것으로 보입니다. 작업 중 이
판단이 틀렸다는 걸 발견하면 완료 보고서에 기록해주세요 (수정은 이번
범위 밖입니다).

## 3. 요구사항

### 3.1 좌표계 버그 수정

`home_x, home_y`를 **월드 좌표**로 놓고, 그 월드 좌표를 **RTH 대상
기체 자신의 로컬 좌표로 역변환**해서 `moveToPositionAsync`(또는 아래
3.2의 ORCA 속도 명령)에 넘겨야 합니다. 즉:

```
world_home = (0.0, 0.0, ...)  # 모든 기체의 진짜 집은 맵 원점(알파 스폰 지점)이라고 가정
# 또는: 각 기체의 own spawn point가 각자의 집이라면
local_home = world_home - DRONES_CONFIG[target_drone_id]["spawn_offset"]
```

**설계 판단이 필요한 지점**: "홈"이 (a) 알파의 스폰 지점(맵 원점,
모든 기체가 같은 곳으로 모임) 인지, (b) 각 기체 자신의 스폰 지점(각자
자기 자리로 흩어져서 복귀) 인지는 작업지시서만으로는 명확하지 않습니다.
기존 코드가 `DRONES_CONFIG[target_drone_id]["spawn_offset"]`를 목표로
삼았던 의도를 보면 **(b) "각자 자기 스폰 지점으로 복귀"**가 원래
의도였던 것으로 보입니다 (실수로 좌표 변환을 빠뜨린 것뿐). 이 해석을
따르는 것을 권장합니다 - 그렇다면 목표는 그냥 **로컬 좌표 `(0, 0)`**
(자기 자신의 원점)이 되고, 오히려 코드가 더 단순해집니다:

```python
# 각 기체는 자기 자신의 스폰 지점(로컬 원점)으로 복귀
local_home_x, local_home_y = 0.0, 0.0
```

다른 해석(전 기체가 알파 자리로 모임)이 더 합리적이라고 판단되면
작업계획서에 근거를 남기고 그렇게 구현해도 됩니다 - 다만 `latest_telemetries`의
`x/y/z`는 이미 월드 좌표로 보정되어 있으므로, 그 값을 목표로 쓰고
싶다면 위 예시처럼 자기 자신의 `spawn_offset`을 빼서 다시 로컬 좌표로
변환해야 `moveToPositionAsync`/`moveByVelocityAsync`에 올바르게
전달됩니다.

### 3.2 RTH 비행 경로를 ORCA 속도 루프로 전환

RTH의 3단계 이동(상승 / 수평 복귀 / 하강)을 각각 **바운드된(수렴하면
종료되는) ORCA 속도 루프**로 바꿔주세요 - `_do_formation_assemble()`이
윙맨 이동 단계에 적용한 것과 같은 구조입니다:

1. 각 단계(레그)마다 목표 지점(고도만 바뀌는 상승 레그, 수평 위치만
   바뀌는 복귀 레그, 고도만 바뀌는 하강 레그)을 정하고
2. 매 틱(`FOLLOW_TICK_INTERVAL_SEC`)마다:
   - 목표까지의 거리 비례 선호 속도 계산
   - `latest_telemetries`에서 RTH 대상을 제외한 나머지 3대(비행 중이든
     정지해 있든 전부)를 `neighbors`로 구성 - 이번에는 "리더" 개념이
     없으므로 전부 `weight=0.5`(상호 회피)로 취급하는 것을 권장합니다
     (Following Mode의 알파처럼 "무조건 안 움직이는 장애물"로 취급할
     이유가 없습니다 - RTH 중인 드론과 상대방 둘 다 얼마든지 움직일 수
     있는 상황이므로).
   - `orca.compute_safe_velocity(...)` 호출, `ensure_api_control` 확인,
     `moveByVelocityAsync`로 명령
   - 그 레그의 목표에 수렴(예: 1.0m 이내) 또는 타임아웃(레그별로
     10~15초 정도 권장 - 상승/하강은 짧게, 수평 복귀는 이동 거리에
     따라 달라지므로 넉넉하게)되면 다음 레그로 진행
3. 마지막 레그(3m 고도로 하강) 완료 후에는 기존처럼 `landAsync()`로
   착륙하세요 (착륙 자체는 ORCA 대상이 아닙니다 - 이미 홈 지점 바로
   위에서 수직으로 내려오는 동작이라 다른 기체와 얽힐 가능성이 낮고,
   착륙 중에는 정밀 착륙 로직을 그대로 신뢰하는 것이 더 안전합니다).

**코드 재사용 제안 (선택 사항)**: "목표 지점까지 ORCA 속도 루프로
비행 -> 수렴 또는 타임아웃 -> hoverAsync"라는 동일한 패턴이 이제
편대 집결(1곳)과 RTH(3개 레그)에서 총 4번 필요해집니다. 공용 헬퍼
함수로 뽑아내는 것을 권장하지만 **필수는 아닙니다**. 뽑아낼 경우:
- 이미 검수를 통과한 `_do_formation_assemble()`의 동작이 조금이라도
  달라지지 않도록 각별히 주의하고, 리팩토링 후 `test_orca_formation_assemble.py`가
  여전히 동일한 기준으로 통과하는지 반드시 재확인하세요.
- 자신 없다면 RTH 쪽에 별도로 구현하고 편대 집결 코드는 건드리지
  않는 것이 더 안전한 선택입니다 (중복은 있지만 회귀 위험은 없음).

### 3.3 여러 드론이 동시에 RTH되는 상황 처리

Following Mode의 `formation_assemble_in_progress`처럼, RTH도 진행
중임을 나타내는 상태가 있으면 좋지만 - RTH는 **개별 기체 단위**
명령이라 편대 집결/Following Mode처럼 전체를 잠글 필요는 없습니다.
다만:

- 같은 기체에 대해 RTH가 이미 진행 중인데 또 RTH가 호출되면 어떻게
  할지 결정하세요 (예: 무시, 또는 이전 RTH를 취소하고 새로 시작 -
  구현자 재량, 작업계획서에 근거를 남겨주세요).
- RTH 대상 기체가 Following Mode의 팔로워라면(`is_follower_locked`),
  기존에 이미 `/api/rth` 엔드포인트에서 이 경우를 차단하고 있습니다
  (`is_follower_locked(t_id)` 검사) - 그대로 유지하세요, 손댈 필요
  없습니다.
- 서로 다른 두 기체가 각각 RTH 중인 상황(예: 사용자가 브라보 RTH 누르고
  몇 초 뒤 찰리도 RTH 누름)은 막을 필요 없습니다 - 이게 바로 ORCA가
  풀어야 할 정상적인 시나리오입니다.

## 4. 비기능 요구사항

- Following Mode, 편대 집결 기존 동작과 회귀 테스트에 영향 없어야
  합니다.
- ORCA 계산/명령 전송 중 예외가 발생해도 해당 기체의 RTH만 실패하고
  (`/api/rth` 응답에서 에러로 처리), 서버 전체나 다른 기체에 영향을
  주면 안 됩니다.
- 좌표 변환(3.1절)은 RTH의 데모/시뮬레이션 모드 폴백 코드
  (`/api/rth` 엔드포인트에서 `is_airsim_connected`가 False일 때 쓰는
  `latest_telemetries[t_id]["x"] = home_x` 부분)에도 일관되게
  반영하세요 - 그쪽은 이미 월드 좌표를 다루는 `latest_telemetries`에
  직접 쓰는 코드라 좌표 변환 없이 `home_x, home_y`(스폰 오프셋)를
  그대로 써도 맞습니다만, 3.1절에서 어떤 해석(자기 자리 복귀 vs 알파
  자리로 집결)을 택했는지에 따라 이 폴백 코드도 같이 맞춰주세요.

## 5. 작업 범위 및 제외 사항

- **포함**: `_do_rth()` 좌표계 수정 + ORCA 속도 루프 전환, 데모 모드
  폴백 좌표 일관성 확인.
- **제외**: `orca.py` 수정, Following Mode/편대 집결 로직 변경(공용
  헬퍼로 리팩토링하는 경우가 아니라면), 착륙(`landAsync`) 자체 로직
  변경, "전체 편대 동시 RTH" 같은 새 기능 추가(요청되지 않음 - 개별
  RTH만 대상).

## 6. 검증 및 완료 조건

실제 AirSim 시뮬레이터(Blocks 권장)로 검증하고 결과를 작업완료
보고서에 포함하세요.

1. **좌표 정확성 검증 (신규)**: RTH 완료 후 착륙 지점이 의도한 홈
   좌표(3.1절에서 선택한 해석 기준)에 근접하는지(예: 오차 1.5m 이내)
   확인하는 테스트. 여러 기체(스폰 오프셋이 0이 아닌 브라보/찰리/델타
   중 최소 1대 이상)로 검증해야 이전 버그가 실제로 고쳐졌음을 증명할
   수 있습니다 (알파만 테스트하면 오프셋이 0이라 버그가 있어도 우연히
   통과합니다).
2. **충돌 회피 검증 (신규)**: 두 대 이상을 동시에(또는 몇 초 간격으로
   겹치게) RTH시켜서 서로의 복귀 경로가 교차하는 시나리오를 만들고,
   `collision_count`가 0으로 유지되는지, 최소 이격 거리가
   `2 * ORCA_AGENT_RADIUS_M = 3.0m` 이상인지 측정하세요
   (`test_orca_rth.py` 권장 - 기존 `test_orca_collision_avoidance.py`,
   `test_orca_formation_assemble.py`와 같은 고빈도 샘플링 패턴을
   참고하세요).
3. **기존 회귀**: `test_following_mode.py`, `test_orca_formation_assemble.py`,
   `test_ui_playwright.py` 전부 통과해야 합니다 (RTH 코드 변경이 다른
   기능에 영향을 주지 않았는지 확인).
4. 테스트 전후 시뮬레이터/프로세스 정리를 확인하세요.

## 7. 산출물

- `server.py` (수정 - `_do_rth()` 좌표계 수정 + ORCA 전환, 필요시
  공용 헬퍼 함수 추가)
- 신규 테스트 스크립트 (`test_orca_rth.py` 등)
- `docs/10_implementation_plan_rth_orca.md` (작업계획서 - 특히 3.1절의
  "어느 홈으로 복귀할지" 해석과 그 근거를 명확히 남겨주세요)
- `docs/11_completion_report_rth_orca.md` (작업완료 보고서)
- `docs/00_INDEX.md` 갱신

## 8. 다음 단계 (참고용 - 이번 작업 범위 아님)

Following Mode / 편대 집결 / RTH 세 가지 주요 다중 기체 이동 경로에
모두 ORCA가 적용되면, 시뮬레이션 전용 파라미터(`ORCA_AGENT_RADIUS_M=1.5m`
등)를 실제 물리 드론 운용을 고려해 재검토하는 작업이 이어질 수
있습니다. 참고로 편대 집결 검수(#08)에서 Codex가 "최소 이격 여유가
0.02m로 작다"는 점을 지적하며 더 큰 안전 여유와 다양한 조건(다른 맵,
풍속, 통신 지연 등)에서의 반복 시험을 권고했습니다 - 이번 RTH 작업
검수에서도 비슷한 지적이 나온다면, 다음 작업에서 `ORCA_AGENT_RADIUS_M`
자체를 늘리거나 `ORCA_TIME_HORIZON_SEC`를 조정하는 튜닝 작업으로
다룰 수 있습니다.
