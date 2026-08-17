# 작업지시서 #01: Following Mode ORCA 충돌 회피 적용

- 작성자: Claude (설계 / Plan 역할)
- 구현 담당: Antigravity
- 검수 담당: Codex (독립검수 / Review 역할)
- 대상 레포: 이 레포 (`DEV_Dron_System-ORCA`) - 메인 `DEV_Dron_System` 레포의
  스냅샷이며, 이후 검증이 끝나면 메인 레포로 역이식됩니다.

이 문서는 구현 담당자(Antigravity)가 이 대화의 맥락을 전혀 모른다는
전제로 작성되었습니다. 필요한 배경 정보를 전부 이 문서 안에 포함했습니다.

---

## 1. 배경 및 목적

이 레포는 AirSim 시뮬레이터 위에서 드론 4대(알파/브라보/찰리/델타)를
편대 비행시키는 관제 시스템입니다. 최근 "Following Mode"라는 기능이
추가되었습니다 - 브라보는 알파를, 찰리는 브라보를, 델타는 찰리를 각각
쫓아가는 체인 구조로, 리더가 몇 초 전에 있었던 위치를 목표로 삼아
자연스럽게 뒤따라가는(오리 가족처럼) 자동조종 모드입니다.

**문제**: 이 추격 로직에는 충돌 회피가 전혀 없습니다. 팔로워가 리더를
바짝 쫓다가 리더가 급정지하거나 급선회하면, 또는 여러 팔로워의 경로가
교차하면 그대로 충돌할 수 있습니다.

**목표**: ORCA(Optimal Reciprocal Collision Avoidance) 알고리즘을 Following
Mode의 이동 명령 생성 과정에 적용해서, 팔로워들이 서로(및 알파)와
충돌하지 않으면서도 원래의 "뒤따라가기" 동작은 유지하도록 만듭니다.

ORCA는 여러 이동체가 각자의 "선호 속도(목표 방향으로 가고 싶은 속도)"를
가지고 있을 때, 서로 충돌하지 않는 "안전 속도"로 실시간 보정해주는
지역(local) 충돌 회피 알고리즘입니다. 회피 책임을 상대와 50:50으로
나눈다는 것이 핵심 아이디어이며, 그 덕분에 기존 방식들이 겪던 지그재그
떨림 현상 없이 부드럽게 회피합니다. 매 제어 주기마다 이웃들과의
반평면(half-plane) 속도 제약을 계산하고, 그 제약을 만족하면서 선호
속도에 가장 가까운 속도를 선형계획법(LP)으로 구합니다.

---

## 2. 현재 시스템 구조 (구현 전 반드시 읽어야 함)

### 2.1 파일 구조

- `server.py` - FastAPI 백엔드. AirSim 제어, 텔레메트리, 모든 API
  엔드포인트가 이 한 파일에 있습니다 (약 1500줄).
- `public/app.js`, `public/index.html`, `public/style.css` - 프론트엔드.
  아이콘 없이 텍스트 기반, 3색 시스템(primary=파랑/success=초록/
  critical=빨강 + 드론 식별용 4색 별도)으로 최근 정리되었습니다.
- `test_*.py` - 회귀/진단 테스트 스크립트. 대부분 `airsim` 라이브러리로
  직접 시뮬레이터에 접속해서 고빈도로 위치를 샘플링하고 검증하는 방식을
  씁니다 (HTTP API를 호출하면서 동시에 별도 스레드로 텔레메트리를
  폴링). 새 테스트를 작성할 때 이 패턴을 참고하세요.

### 2.2 핵심 아키텍처

- **두 개의 독립된 AirSim RPC 클라이언트**:
  - `client_telemetry` - 읽기 전용, `airsim_worker()` 백그라운드 스레드가
    소유하고 25Hz(0.04초 간격)로 4대 전부의 `getMultirotorState`와
    (선택된 드론에 한해) `simGetImages`를 폴링합니다.
  - `client_control` - 쓰기/명령 전용, `get_control_client()`로 얻습니다.
    **모든** 제어 액션(이륙/착륙/조이스틱/RTH/편대집결/Following Mode)은
    반드시 `with control_lock:` 블록 안에서 이 클라이언트를 사용해야
    합니다. 이 락은 여러 제어 경로가 동시에 같은 소켓을 건드리지 않도록
    보호합니다.
- **`latest_telemetries`** (dict, key: "Drone1"~"Drone4") - 각 드론의
  최신 위치(x,y,z), 속도(vx,vy,vz), 자세(pitch,roll,yaw), 상태
  (landed_state, armed 등)를 담고 있습니다. `airsim_worker()`가 매 틱
  갱신하므로, ORCA 계산에 필요한 "이웃의 현재 위치/속도"는 여기서 읽으면
  됩니다 (별도로 airsim에 다시 질의할 필요 없음).
- **`position_history`** (dict, key: "Drone1"~"Drone3") - 리더가 될 수
  있는 세 기체(팔로워를 가진 기체)의 최근 위치 이력을 `deque`로 보관
  (튜플 `(timestamp, x, y, z)`, 최대 400개, `airsim_worker()`가 매 틱
  append). `get_lagged_leader_position(leader_id, lag_seconds)`로 "leader가
  lag_seconds초 전에 있었던 위치"를 조회할 수 있습니다.
- **`FOLLOW_CHAIN`** = `{"Drone2": "Drone1", "Drone3": "Drone2", "Drone4": "Drone3"}`
  - 브라보는 알파를, 찰리는 브라보를, 델타는 찰리를 쫓습니다.
- **`following_worker()`** - Following Mode 전용 백그라운드 스레드.
  `following_mode_enabled`가 True이고 AirSim에 연결되어 있을 때만
  0.3초(`FOLLOW_TICK_INTERVAL_SEC`) 간격으로 동작하며, 각 팔로워를
  리더의 지연된 위치로 이동시킵니다. **이 함수를 수정하는 것이 이번
  작업의 핵심입니다.**
- **`ensure_api_control(ctrl, vehicle_name)`** - 이미 server.py에 존재하는
  헬퍼 함수. `enableApiControl(True)`을 이미 활성화된 드론에 무조건
  다시 호출하지 않도록 가드합니다 (아래 2.3의 교훈 A 참고). 새 이동
  명령을 내리기 전에 반드시 이 함수를 통해 API 제어권을 확인하세요.

### 2.3 반드시 지켜야 할 두 가지 교훈 (실제로 발생했던 버그)

이 코드베이스에서 실제로 재현되고 원인까지 격리 확인된 버그 두 가지입니다.
ORCA 구현 시 반드시 아래 패턴을 피해야 합니다.

**교훈 A - `enableApiControl` 재호출 금지**: 이미 API 제어권이 있고
비행 중인 드론에 `enableApiControl(True, ...)`을 다시 호출하면, AirSim이
물리적으로는 공중에 떠 있는데도 내부 `landed_state`를 착륙 상태로
리셋해버려서 드론이 그대로 지면까지 추락했다가 재이륙하는 버그가
있었습니다 (순수 airsim 클라이언트만으로 이 앱과 무관하게 격리
재현했음 - AirSim 자체의 동작이며 우리 코드 로직 버그는 아니지만, 우리
코드가 이 함수를 불필요하게 반복 호출했던 것이 트리거였습니다). 반드시
`ensure_api_control(ctrl, vehicle_name)`를 통해서만 API 제어권을
요청하세요 - 이 헬퍼는 `isApiControlEnabled()`로 이미 켜져 있는지 먼저
확인하고, 꺼져 있을 때만 `enableApiControl(True, ...)`을 호출합니다.

**교훈 B - 동일 기체에 대한 이중 제어 경로 금지**: 같은 드론에 대해
서로 다른 코드 경로가 동시에 이동 명령을 내리면 경쟁 상태(race
condition)가 발생합니다 (과거 텔레메트리 연결 로직에서 이 패턴 때문에
발생한 버그를 수정한 이력이 있습니다). Following Mode가 켜져 있는 동안
브라보/찰리/델타에 대한 개별 조이스틱/이륙/회전/RTH 명령은 이미
`is_follower_locked(drone_id)` 검사로 서버에서 거부되고 있습니다
(`/api/joystick`, `/api/takeoff`, `/api/rotate`, `/api/rth` 엔드포인트
참고) - **이 보호 장치를 절대 제거하거나 우회하지 마세요.** ORCA 루프
자체도 각 틱에서 한 드론당 이동 명령을 정확히 한 번만 내려야 합니다.

---

## 3. 요구사항

### 3.1 ORCA 솔버 모듈 (신규 파일: `orca.py`)

- **순수 Python + numpy만 사용하세요.** `Python-RVO2` 등 C/C++ 확장을
  빌드해야 하는 라이브러리는 사용하지 마세요 - 이 개발 환경에 C++ 빌드
  툴체인이 없을 수 있습니다.
- 최대 4개 에이전트만 다루므로 성능은 걱정할 필요 없습니다. 매 틱
  O(n²) 계산이어도 충분히 빠릅니다.
- 알고리즘 참고: van den Berg, Guy, Lin, Manocha, "Reciprocal n-Body
  Collision Avoidance" (2011) - Velocity Obstacle 기반 반평면 제약을
  세우고, 그 제약들을 만족하며 선호 속도에 가장 가까운 속도를
  선형계획법으로 구하는 표준 ORCA 정식화를 따르세요.
- 좌표계 주의: AirSim은 NED입니다 (X=North, Y=East, **Z=Down**, 즉
  고도가 높을수록 Z는 더 음수). 이 좌표계를 그대로 사용해도 되고,
  내부적으로 편한 좌표계로 변환해서 계산한 뒤 다시 NED로 돌려도
  됩니다 - 다만 어느 쪽이든 명확히 문서화하세요.

권장 인터페이스 (정확한 시그니처는 구현 재량):

```python
def compute_safe_velocity(
    agent_pos: tuple[float, float, float],
    agent_vel: tuple[float, float, float],
    preferred_vel: tuple[float, float, float],
    neighbors: list[dict],  # [{"pos": (x,y,z), "vel": (vx,vy,vz), "radius": float}, ...]
    agent_radius: float,
    time_horizon: float,
    max_speed: float,
) -> tuple[float, float, float]:
    """선호 속도에 가장 가까운, 이웃들과 충돌하지 않는 안전 속도를 반환."""
```

### 3.2 튜닝 파라미터

모두 `server.py` 상단에 조정 가능한 상수로 두세요 (기존 `FOLLOW_*`
상수들 옆에 배치하는 것을 권장):

- `ORCA_TIME_HORIZON_SEC` (기본값 제안: 2.0) - 미래 예측 시간. 크게
  잡을수록 더 일찍, 더 크게 우회합니다.
- `ORCA_AGENT_RADIUS_M` (기본값 제안: 1.5) - 기체 안전 반경. 실제 기체
  크기보다 여유 있게 잡아서 오차를 상쇄합니다.
- `ORCA_MAX_SPEED_MPS` - 기존 `following_velocity`와 동일한 값을
  기본으로 사용하되, 별도로 조정 가능하게 하세요.
- `FOLLOW_TICK_INTERVAL_SEC`는 이미 0.3초로 정의되어 있습니다. ORCA
  루프도 이 주기를 그대로 쓰거나, 반응성을 높이려면 더 짧게(예:
  0.15~0.2초) 조정해도 됩니다. 짧게 할수록 RPC 호출 빈도가 늘어나니
  4대 기준으로 문제없는지 실측 확인하세요.

### 3.3 `following_worker()` 수정

현재 구조 (`server.py`, 함수명으로 검색하면 찾을 수 있습니다):

```python
def following_worker():
    print("[FOLLOW] Following Mode 워커 스레드 시작", flush=True)
    while True:
        if not following_mode_enabled or not is_airsim_connected:
            time.sleep(0.2)
            continue
        try:
            with control_lock:
                ctrl = get_control_client()
                for follower_id, leader_id in FOLLOW_CHAIN.items():
                    target = get_lagged_leader_position(leader_id, following_lag_seconds)
                    if target is None:
                        continue
                    _, tx, ty, tz = target
                    f_vname = get_real_vehicle_name(ctrl, follower_id)
                    state = ctrl.getMultirotorState(vehicle_name=f_vname)
                    if state.landed_state == airsim.LandedState.Landed:
                        continue
                    ensure_api_control(ctrl, f_vname)
                    ctrl.moveToPositionAsync(tx, ty, tz, following_velocity, vehicle_name=f_vname)
        except Exception as e:
            print(f"[FOLLOW] 오류: {type(e).__name__}: {e}", flush=True)
        time.sleep(FOLLOW_TICK_INTERVAL_SEC)
```

**변경 방향**:

1. 목표 지점(`tx, ty, tz`)까지 가고 싶은 "선호 속도"를 계산하세요.
   예: `direction = target - current_pos`를 정규화하고 크기를
   `following_velocity`로 맞추되, 목표까지 남은 거리가 그보다 짧으면
   오버슈트하지 않도록 속도를 줄이는 것을 권장합니다.
2. 나머지 3대(자기 자신 제외)의 **현재 위치와 속도**를
   `latest_telemetries`에서 읽어 ORCA `neighbors` 리스트를 구성하세요.
   **알파도 반드시 포함**해야 합니다 (알파는 사용자가 직접 조종하는
   "움직이는 장애물"로 취급되어야 합니다 - 알파 자신은 ORCA 대상이
   아니지만, 팔로워들의 회피 계산에는 들어가야 합니다).
3. `orca.compute_safe_velocity(...)`로 안전 속도를 계산하세요.
4. `moveToPositionAsync` 대신 **`moveByVelocityAsync(vx, vy, vz, duration,
   vehicle_name=...)`** (world frame)로 명령하세요. `duration`은 다음
   틱까지의 간격보다 약간 여유 있게 (예: `FOLLOW_TICK_INTERVAL_SEC * 1.5`)
   주는 것을 권장합니다 - 다음 틱이 지연되더라도 드론이 갑자기
   멈추지 않도록 하기 위해서입니다.
5. `ensure_api_control` 호출은 그대로 유지하세요 (교훈 A).
6. 착륙 상태인 기체는 건드리지 않는 기존 로직을 유지하세요.

### 3.4 권장 단순화 (선택 사항)

편대 비행은 대체로 같은 고도를 유지하므로, 완전한 3D ORCA 대신 **XY
평면에서만 2D ORCA를 수행하고 Z(고도)는 단순 비례 제어로 분리**해도
실용적으로 충분할 수 있습니다:

- `preferred_vel`과 각 이웃의 속도에서 X, Y 성분만 ORCA에 넣어 안전한
  (vx, vy)를 구하고
- Z축 속도는 `vz = clip((target_z - current_z) / dt, -max_vz, max_vz)`
  같은 별도 로직으로 계산

완전한 3D ORCA를 구현할지, 이 단순화를 선택할지는 **구현자 재량**입니다.
어느 쪽을 선택했는지와 그 이유를 작업완료 보고서(3번 문서)에 명시해
주세요.

### 3.5 충돌 감지 연동 (`simGetCollisionInfo`)

`airsim_worker()`의 텔레메트리 폴링 루프(각 드론마다
`client_telemetry.getMultirotorState(...)`를 호출하는 부분)에
`client_telemetry.simGetCollisionInfo(vehicle_name=v_name)` 호출을
추가하고, 결과를 `latest_telemetries[d_id]`에 다음 필드로 저장하세요:

```python
"collided": bool(collision_info.has_collided),
"collision_count": int(collision_info.collision_count),
```

프론트엔드에 이 값을 표시하는 배지나 로그를 추가해주세요. 최근 UI가
아이콘 없이 텍스트 기반, 3색 시스템(`--accent-primary`=파랑,
`--accent-success`=초록, `--accent-critical`=빨강, `style.css`의
`:root`에 정의됨)으로 정리되었으니 이 톤을 유지하세요. 충돌 발생 시
critical 색상을 사용하는 것을 권장합니다.

---

## 4. 비기능 요구사항

- Following Mode가 꺼져 있을 때는 기존 동작(개별 조종)에 영향이 없어야
  합니다.
- ORCA 계산이나 명령 전송 중 예외가 발생해도 해당 틱만 건너뛰고 다음
  틱에 재시도해야 합니다 (기존 `except Exception as e:` 패턴 유지 -
  한 번의 예외로 워커 스레드 전체가 죽으면 안 됩니다).
- 새 의존성을 추가한다면 `requirements.txt`에 반영하세요 (단, 3.1에서
  언급했듯 C 확장 빌드가 필요한 패키지는 피하세요).

## 5. 작업 범위 및 제외 사항

- **포함**: `orca.py` 신규 작성, `following_worker()` 수정, 충돌 감지
  연동(`airsim_worker()` + `latest_telemetries` + UI 표시).
- **제외 (다음 작업지시서에서 다룰 예정)**: `_do_formation_assemble()`의
  윙맨 이동 단계에 ORCA를 적용하는 것은 이번 범위가 아닙니다. 사람이
  조이스틱으로 알파를 직접 조종하는 로직(`_do_joystick_velocity`)도
  변경하지 마세요 - 다만 3.3의 2번 항목대로, 알파의 실시간 위치/속도는
  팔로워들의 ORCA neighbor로는 반드시 포함되어야 합니다.

## 6. 검증 및 완료 조건

구현 완료 후 아래 항목을 **실제 AirSim 시뮬레이터**로 검증하고, 결과를
작업완료 보고서에 포함해주세요 (Blocks 맵을 권장합니다 - 가장 가볍고
빠르게 켜집니다). 데모/시뮬레이션 모드(시뮬레이터 미연결 상태)는 실제
물리 연산이 없어 충돌 회피 검증에 사용할 수 없습니다.

1. **회귀 검증**: `test_following_mode.py`가 여전히 통과해야 합니다 -
   편대 집결 후 Following Mode를 켜고 알파를 전진시켰을 때 브라보/찰리/
   델타가 같은 방향으로 실제 이동하는지 확인하는 테스트입니다. ORCA가
   추가되어도 "쫓아간다"는 기본 동작 자체는 유지되어야 합니다.
2. **충돌 회피 검증 (신규 테스트 작성 필요)**: ORCA 없이는 충돌이
   발생할 만한 시나리오를 의도적으로 구성하세요 (예: `following_lag_seconds`를
   아주 짧게 설정해 팔로워가 리더를 바짝 쫓게 하거나, 리더를 급격히
   방향 전환시키는 등). `collision_count`가 0으로 유지되거나, 최소한
   ORCA를 끈 비교군보다 현저히 낮은지 확인하는 스크립트를 작성해
   실행하세요 (`test_orca_collision_avoidance.py` 권장 - 기존
   `test_*.py`들처럼 별도 스레드로 고빈도 위치/충돌 샘플링하는 패턴을
   참고하세요).
3. **일반 회귀**: `test_ui_playwright.py` (데모 모드, 실제 시뮬레이터
   불필요) 전체 통과.
4. 테스트 실행 전후로 시뮬레이터/서버 프로세스가 깨끗하게 정리되었는지
   확인하세요 (`taskkill /F /IM Blocks.exe`, python.exe 프로세스 정리 등).
   작업 시작 전 이미 다른 프로세스가 떠 있지 않은지도 확인하세요.

## 7. 산출물

- `orca.py` (신규)
- `server.py` (수정)
- `public/app.js`, `public/index.html`, `public/style.css` (충돌 표시
  UI - 필요한 만큼만 최소로 수정)
- `requirements.txt` (신규 의존성이 있는 경우에만 수정)
- 신규 테스트 스크립트 (`test_orca_collision_avoidance.py` 등)
- `docs/02_implementation_plan_following_mode_orca.md` (작업계획서 -
  구현 착수 전 작성: 선택한 설계(완전 3D vs 2D+Z 단순화), 예상 변경
  파일 목록, 단계별 계획)
- `docs/03_completion_report_following_mode_orca.md` (작업완료 보고서 -
  구현 후 작성: 실제 변경 내역, 6번 검증 항목의 실행 결과/로그, 알려진
  한계나 후속 제안)

## 8. 다음 단계 (참고용 - 이번 작업 범위 아님)

이번 작업이 검수를 통과하면, 같은 ORCA 모듈을 `_do_formation_assemble()`의
윙맨 이동 단계에도 적용하는 2차 작업지시서(#05)가 이어질 예정입니다.
