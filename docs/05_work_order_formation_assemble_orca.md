# 작업지시서 #05: 편대 집결(Formation Assemble) ORCA 충돌 회피 적용

- 작성자: Claude (설계 / Plan 역할)
- 구현 담당: Antigravity
- 검수 담당: Codex (독립검수 / Review 역할)
- 선행 작업: `docs/01~04_*_following_mode_orca.md` (Following Mode ORCA - 승인 완료)
- 대상 레포: `DEV_Dron_System-ORCA`

이 문서는 구현 담당자가 이 대화의 맥락과 #01~#04 문서의 세부 내용을 전부
읽었다는 것을 전제하지 않고, 이번 작업에 필요한 내용을 자체적으로
포함하도록 작성되었습니다. 다만 `orca.py`와 `following_worker()`는 이미
검수를 통과해 동작 중인 **참고 구현체**이므로, 이번 작업은 그 패턴을
최대한 재사용합니다.

---

## 1. 배경 및 목적

Following Mode(#01~#04)에 ORCA를 적용해 팔로워들이 서로 충돌하지 않고
리더를 추격하도록 만드는 작업은 승인 완료되었습니다. 하지만 이 시스템에서
**충돌 위험이 실제로 가장 높은 순간은 따로 있습니다** - 바로
`_do_formation_assemble()`(편대 집결, "알파 호출") 함수가 실행되는
동안입니다.

편대 집결은 흩어져 있던 브라보/찰리/델타를 알파 뒤의 정해진 슬롯(trail
slot)으로 동시에 불러모으는 **일회성(one-shot) 기동**입니다. 지금 이
기동은 각 윙맨에게 `moveToPositionAsync()`를 한 번씩 발사하고 끝나는
방식이라, 세 대가 동시에 알파 근처의 좁은 공간으로 수렴하는데도 서로의
경로를 전혀 고려하지 않습니다. 여러 방향에서 흩어져 있던 기체를 모을
때(특히 사용자가 개별 조종으로 각 기체를 아무 곳에나 이동시켜 놓은
뒤 "편대 집결"을 눌렀을 때) 경로가 교차할 가능성이 following mode보다
오히려 높습니다.

**목표**: 이미 검증된 `orca.py` 솔버와 `following_worker()`의 ORCA 통합
패턴을 재사용해서, `_do_formation_assemble()`의 윙맨 이동 단계를 ORCA
기반 속도 제어 루프로 교체합니다.

## 2. 선행 조건: 알아야 할 기존 구현

### 2.1 재사용할 참고 구현체 (반드시 먼저 읽을 것)

- **`orca.py`**: 순수 Python + NumPy 2D ORCA 솔버. 핵심 함수는
  `compute_safe_velocity(agent_pos, agent_vel, preferred_vel, neighbors,
  agent_radius, time_horizon, max_speed, max_vz, time_step)` - XY 평면
  ORCA + Z축 독립 비례 제어(clip) 조합입니다. **이번 작업에서 이 모듈은
  수정하지 않습니다** (그대로 재사용).
- **`server.py`의 `following_worker()` 함수**: ORCA를 실제 제어 루프에
  통합한 유일한 기존 사례입니다. 이번 작업은 이 함수와 거의 동일한
  패턴을 `_do_formation_assemble()`에 적용합니다:
  1. 목표 지점까지의 "선호 속도(preferred velocity)"를 계산 (거리
     비례 감속 포함)
  2. `latest_telemetries`에서 자신을 제외한 나머지 기체(알파 포함)의
     실시간 위치/속도를 읽어 `neighbors` 리스트 구성
  3. `orca.compute_safe_velocity(...)` 호출로 안전 속도 산출
  4. `ensure_api_control(ctrl, vehicle_name)`으로 API 제어권 확인
  5. `moveByVelocityAsync(vx, vy, vz, duration, vehicle_name=...)`로
     world-frame 속도 명령 발사 (`duration`은 틱 간격의 1.5배 정도)
- **ORCA 관련 기존 상수** (`server.py` 상단, 그대로 재사용):
  `ORCA_TIME_HORIZON_SEC=2.0`, `ORCA_AGENT_RADIUS_M=1.5`,
  `ORCA_MAX_SPEED_MPS=3.0`, `ORCA_MAX_VZ_MPS=2.0`,
  `FOLLOW_TICK_INTERVAL_SEC=0.1`.
- **충돌 텔레메트리**: `airsim_worker()`가 이미 `simGetCollisionInfo`를
  폴링해서 `latest_telemetries[d_id]["collided"]`,
  `["collision_count"]`를 채우고 있고, UI에도 이미 표시됩니다. **이번
  작업에서 이 부분은 손댈 필요가 없습니다.**

### 2.2 중요 - 세계 좌표계(World Coordinate) 정합성 문제

Following Mode 작업 중 다음 사실이 확인되었습니다: **AirSim에서 각
기체의 `getMultirotorState().kinematics_estimated.position`은 그 기체
자신의 스폰 지점을 원점으로 하는 로컬 좌표**입니다 (전체가 공유하는
하나의 월드 원점이 아닙니다). 알파/브라보/찰리/델타의 스폰 오프셋은
`DRONES_CONFIG[drone_id]["spawn_offset"]`에 각각 `(0,0,0)`,
`(0,3.5,0)`, `(0,7.0,0)`, `(0,10.5,0)`로 정의되어 있습니다.

`airsim_worker()`는 이미 이 오프셋을 보정해서 `latest_telemetries`와
`position_history`에 **월드 좌표**로 저장합니다 (`server.py`에서
`world_x = pos.x_val + sp_offset[0]` 형태의 코드를 참고하세요). 따라서
`latest_telemetries`에서 읽는 값은 그대로 써도 됩니다.

**그런데 `_do_formation_assemble()`은 이 보정을 전혀 하지 않습니다.**
`ctrl.getMultirotorState(vehicle_name=...)`으로 얻은 원시(raw) 로컬
좌표를 그대로 트레일 슬롯 계산(`alpha_pos.x_val + back_dir_x *
slot_dist` 등)과 `moveToPositionAsync(tx, ty, ...)` 호출에 사용하고
있습니다. 이건 실제로 편대 집결 결과가 부정확했던 것(윙맨들이 의도한
간격보다 어긋난 위치에 도착하는 현상)의 원인으로 보입니다 - 스폰
오프셋 크기(3.5m/7.0m/10.5m)와 관측된 오차 크기가 비슷한 수준입니다.

**이번 작업의 일부로 이 좌표계 문제도 함께 고쳐주세요.** ORCA 계산 자체가
모든 에이전트가 같은 좌표계를 공유한다는 전제 위에 서 있기 때문에, 이
문제를 고치지 않으면 ORCA를 아무리 잘 구현해도 이웃 위치가 실제와 다르게
계산되어 무의미해집니다. `following_worker()`가 이미 이 문제를 어떻게
풀었는지(`f_offset = DRONES_CONFIG[follower_id].get("spawn_offset", ...)`
+ 오프셋을 더해서 `cur_pos` 구성) 그대로 참고해서, `_do_formation_assemble()`
전체에서 같은 방식을 적용하세요 - 가능하면 `latest_telemetries`에서
직접 읽는 방식(이미 월드 좌표로 보정되어 있으므로)으로 통일하는 것을
권장합니다.

## 3. 요구사항

### 3.1 알파(Alpha) 관련 로직은 변경하지 않음

`_do_formation_assemble()`의 다음 단계는 **그대로 유지**하세요 - 알파는
ORCA 대상이 아닙니다 (다른 기체와 충돌할 상황을 스스로 만들지 않고,
그 자리에서 호버링만 하기 때문입니다):

- 1단계: 알파 지상/비행 상태 판정 및 필요시 이륙
- 2단계: 알파 호버 고정 (`hoverAsync`)
- 6단계: 알파 재확인 호버

### 3.2 윙맨 이동 단계를 ORCA 속도 루프로 교체

현재 3~5단계(윙맨 이륙 -> Stage 1 수직 상승 -> Stage 2 트레일 슬롯
이동)는 `moveToPositionAsync()`를 한 번씩 발사하고 `time.sleep()`으로
기다리는 방식입니다. 이 부분을 다음과 같이 바꿔주세요:

1. 윙맨 이륙(`takeoffAsync`)은 그대로 유지 (ORCA와 무관 - 이륙 자체는
   충돌 회피 대상이 아닙니다).
2. 이륙 완료 후, **최종 목표(트레일 슬롯 좌표, `target_z` 고도)**를
   각 윙맨에 대해 미리 계산해두세요 (기존 Stage 2의 `tx, ty` 계산 로직
   재사용 - 단, 2.2절의 좌표계 보정 적용).
3. Stage 1(수직 상승)과 Stage 2(슬롯 이동)를 **하나의 통합된 ORCA 속도
   루프**로 합치는 것을 권장합니다 - 굳이 두 단계로 나눌 필요 없이,
   "현재 위치에서 최종 목표(트레일 슬롯 + 알파 고도)까지"를 선호 속도로
   계산하면 자연스럽게 상승과 수평 이동이 동시에 일어납니다
   (`following_worker`가 하는 것과 동일한 방식).
4. 이 루프는 `following_worker`와 달리 **무한히 돌지 않고, 수렴하면
   종료되는 유한 루프**여야 합니다 (편대 집결은 "한 번 모이고 끝"이지
   Following Mode처럼 계속 켜져 있는 모드가 아닙니다). 종료 조건 예시:
   - 모든 윙맨이 목표 지점으로부터 일정 거리(예: 1.0m) 이내로 들어오면
     종료
   - 또는 타임아웃(예: 15~20초)에 도달하면 그 시점 상태로 종료 (무한
     대기 방지)
   - 종료 시 각 윙맨에 `hoverAsync()`를 호출해서 제자리에 고정하세요
     (following_worker처럼 계속 속도 명령을 내지 않아도 되도록).
5. 매 틱마다:
   - 자신을 제외한 나머지 3대(알파 포함, 아직 슬롯에 도착 못한 다른
     윙맨 포함)를 `latest_telemetries`에서 읽어 `neighbors`로 구성
   - `orca.compute_safe_velocity(...)` 호출 (기존 상수 재사용)
   - `ensure_api_control` 확인 후 `moveByVelocityAsync`로 명령
6. 이 루프는 `with control_lock:` 블록 안, 그리고 `_do_formation_assemble()`
   함수 자체가 이미 `asyncio.to_thread`로 별도 스레드에서 블로킹
   실행되므로, 함수 내부에 동기적인 `while` 루프 + `time.sleep(...)`를
   그대로 써도 됩니다 (다른 곳에서 쓰는 것과 같은 스타일).

### 3.3 Following Mode와의 상호작용

편대 집결과 Following Mode가 동시에 활성화될 가능성을 반드시 고려하세요:

- 편대 집결 실행 중에는 `following_worker()`가 같은 윙맨들에게 동시에
  속도 명령을 내리지 않아야 합니다 (교훈 B - 이중 제어 경로 금지). 가장
  간단한 방법: 편대 집결 함수가 실행되는 동안 일시적으로 Following
  Mode를 처리하지 않도록 플래그를 두거나(예: 기존
  `following_mode_enabled`와 별개로 `formation_assemble_in_progress`
  같은 짧은 락/플래그), 혹은 편대 집결 시작 시 Following Mode를 강제로
  끄고 완료 후 사용자가 다시 켜도록 안내하는 방법 중 택해서 구현하고,
  선택 이유를 작업계획서에 남기세요.
- 두 로직 모두 `control_lock`을 획득해야 하므로 데드락 없이 서로
  배타적으로 실행되는지 확인하세요 (`with control_lock:` 블록이 중첩되지
  않도록 주의).

## 4. 비기능 요구사항

- Following Mode 관련 기존 동작과 회귀 테스트는 전혀 영향받으면 안
  됩니다.
- ORCA 계산/명령 전송 중 예외가 발생해도 전체 편대 집결이 죽지 않고,
  최소한 예외를 기록하고 다음 틱을 시도하거나 안전하게 종료해야 합니다.
- 좌표계 보정(2.2절)은 `_do_formation_assemble()` 뿐 아니라, 혹시 같은
  방식으로 원시 좌표를 직접 쓰는 다른 곳이 있다면 (`_do_rth` 등) 발견 시
  작업완료 보고서에 기록만 해두고, **수정은 이번 범위에 포함하지
  마세요** (범위 외 - 별도 작업지시서에서 다룸).

## 5. 작업 범위 및 제외 사항

- **포함**: `_do_formation_assemble()`의 윙맨 이동 단계 ORCA 전환,
  월드 좌표계 보정, Following Mode와의 상호배제 처리.
- **제외**: `orca.py` 솔버 자체 수정, Following Mode 로직 변경, 조이스틱
  개별 조종(`_do_joystick_velocity`) 변경, RTH/개별 이착륙 로직 변경,
  UI 변경(충돌 배지는 이미 있으므로 추가 UI 작업 불필요 - 단, 편대
  집결 진행 상태를 표시하고 싶다면 자유롭게 추가해도 좋습니다. 필수는
  아님).

## 6. 검증 및 완료 조건

실제 AirSim 시뮬레이터(Blocks 권장)로 검증하고 결과를 작업완료
보고서에 포함하세요.

1. **회귀 검증**: 기존 편대 집결 동작(알파 근처로 모여서 트레일 대형을
   형성)이 여전히 되는지 확인. 좌표계 보정 후에는 이전보다 오히려 슬롯
   위치 정확도가 개선되어야 합니다 (`test_flight_regression.py`의
   시나리오들, 특히 "전 기체 사방 분산 개별 비행 중 집결" 시나리오를
   참고하거나 재사용하세요).
2. **충돌 회피 검증 (신규)**: `test_orca_collision_avoidance.py`의
   패턴을 참고해서, 흩어진 시작 위치(예: 브라보/찰리/델타를 서로 다른
   방향으로 이동시켜 놓은 뒤)에서 편대 집결을 실행했을 때
   `collision_count`가 0으로 유지되는지, 그리고 최소 이격 거리가
   `2 * ORCA_AGENT_RADIUS_M = 3.0m` 이상 유지되는지 측정하는 스크립트를
   작성하세요 (`test_orca_formation_assemble.py` 권장).
3. **Following Mode 회귀**: `test_following_mode.py`가 여전히 통과하는지
   확인 (편대 집결 로직 변경이 following_worker에 영향을 주지 않았는지).
4. **일반 회귀**: `test_ui_playwright.py` (데모 모드) 전체 통과.
5. 테스트 전후 시뮬레이터/프로세스 정리를 확인하세요.

## 7. 산출물

- `server.py` (수정 - `_do_formation_assemble()` 및 필요시 좌표계 보정
  공통 헬퍼 추가)
- 신규 테스트 스크립트 (`test_orca_formation_assemble.py` 등)
- `docs/06_implementation_plan_formation_assemble_orca.md` (작업계획서)
- `docs/07_completion_report_formation_assemble_orca.md` (작업완료 보고서)
- `docs/00_INDEX.md` 갱신

## 8. 다음 단계 (참고용 - 이번 작업 범위 아님)

이번 작업이 승인되면, RTH/개별 이착륙 등 나머지 이동 경로들의 좌표계
일관성 전수 점검, 그리고 실제 물리 드론 이식을 고려한 ORCA 파라미터
현실성 검토(가속도/저크 제한 등)가 이어질 수 있습니다.
