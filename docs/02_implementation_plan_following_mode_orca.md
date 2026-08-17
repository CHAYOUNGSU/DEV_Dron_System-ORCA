# 작업계획서 #02: Following Mode ORCA 충돌 회피 구현 계획

- 작성자: Antigravity (구현 담당)
- 대상 작업지시서: `docs/01_work_order_following_mode_orca.md` (작성: Claude)
- 검수 예정: Codex (독립검수 / Review 역할)
- 대상 레포: `DEV_Dron_System-ORCA`

---

## 1. 작업지시서 검토 및 이상 유무 점검 결과

`docs/01_work_order_following_mode_orca.md`를 바탕으로 코드베이스 전반(`server.py`, `public/*`, `test_*.py`)을 정밀 점검한 결과, **이상 사항 없으며 기술적 요구사항 및 제약조건이 매우 명확하고 완결성 있게 작성**되었음을 확인했습니다.

### 1.1 주요 검토 및 점검 항목
1. **아키텍처 및 락(Lock) 무결성**:
   - `client_telemetry`(읽기 전용, 25Hz)와 `client_control`(`with control_lock:` 보호)의 분리 원칙이 `server.py`에 잘 정립되어 있으며, `following_worker` 수정 시에도 `control_lock` 내부에서 `get_control_client()`를 사용하는 규칙이 그대로 유지될 수 있음을 확인했습니다.
2. **과거 버그 및 안전 가이드 준수 (교훈 A/B)**:
   - 교훈 A: `ensure_api_control(ctrl, v_name)` 가드가 이미 `server.py`에 존재함을 확인했으며, ORCA 적용 후 속도 명령(`moveByVelocityAsync`)을 내리기 전에도 이를 반드시 유지합니다.
   - 교훈 B: `is_follower_locked` 검사로 인해 수동 조종과의 충돌이 차단되어 있으며, `following_worker` 틱당 기체별 1회 명령 주기를 준수합니다.
3. **알파(Alpha) 기체의 이웃(Neighbor) 포함**:
   - 팔로워(브라보/찰리/델타)의 ORCA 연산 시 `latest_telemetries`에서 알파의 위치/속도를 읽어 장애물 에이전트로 포함시키는 방향이 명확합니다.

---

## 2. 핵심 설계 결정: 2D ORCA (XY 평면) + Z축 독립 비례 제어

작업지시서 3.4절의 권장 단순화 항목을 검토한 결과, **2D ORCA (XY) + Z축 독립 비례 제어(Clamping)** 방식을 채택합니다.

### 2.1 채택 근거
1. **물리적 비행 안정성**: 멀티로터 드론은 수평 회피(가감속 및 요 기동)와 수직 고도 제어(스로틀)의 동특성이 상이합니다. 3D 구(Sphere) 기반 ORCA는 좁은 공간에서 불필요한 고도 급상승/급강하를 유발해 다운워시(Downwash) 간섭 및 지면 충돌 위험을 높일 수 있습니다.
2. **실제 운용 환경 적합성**: Following Mode는 리더의 비행 궤적을 동일 고도(또는 지정 순항 고도)로 추종하는 것이 목적이므로, 수평 2차원 평면에서의 반평면(Half-plane) 제약 최적화가 가장 매끄럽고 안정적인 궤적을 생성합니다.
3. **수학적 안정성 및 성능**: 2D Linear Programming (Half-plane intersection)은 Pure Python + NumPy 환경에서 O(n) (n=4 기체) 시간 복잡도로 수 밀리초 이내에 결정론적(deterministic) 수렴을 보장합니다.

---

## 3. 상세 구현 계획

### 3.1 [신규] `orca.py` (Pure Python + NumPy ORCA 솔버 모듈)
- **외부 C++ 의존성 없음**: 순수 Python 3.10+ 및 NumPy만 사용.
- **주요 구성**:
  - `Line`: 2D 반평면 경계선 (point $p$, direction $d$).
  - `linear_program_1d`, `linear_program_2d`: 2D 선형 계획법을 통한 제약 만족 최적 속도 탐색.
  - `linear_program_3d` (명칭 주의: 3차원 공간이 아닌 **3단계 2D 제약 완화(Relaxation) fallback**):
    - 제약조건들의 교집합이 공집합인 과도 구속(Over-constrained) 상황에서, 2D 반평면 제약선들을 등비율로 외측 평행이동(relaxation)시켜 침투 거리를 최소화하는 2D 완화 LP.
  - `compute_safe_velocity(agent_pos, agent_vel, pref_vel, neighbors, agent_radius, time_horizon, max_speed)`:
    - 2D XY 평면에서 각 이웃과의 Velocity Obstacle (VO/ORCA 반평면) 도출.
    - 상대 속도 $\mathbf{v}_{rel} = \mathbf{v}_A - \mathbf{v}_B$, 상대 위치 $\mathbf{p}_{rel} = \mathbf{p}_B - \mathbf{p}_A$.
    - 충돌 시간(Time Horizon $\tau$) 내 침범 영역 원뿔 및 원형 캡에 대한 $u$ 벡터(최소 속도 보정량) 계산.
    - $50:50$ 분담($\mathbf{u}/2$) 반평면 제약 수립 후 2D LP 수행.
    - Z축 속도는 $v_z = \text{clip}\left(\frac{z_{target} - z_{current}}{\Delta t}, -v_{z\_max}, v_{z\_max}\right)$ 비례 제어로 결합하여 최종 $(v_x, v_y, v_z)$ 반환.

### 3.2 [수정] `server.py`
1. **상수 정의 (상단)**:
   - `ORCA_TIME_HORIZON_SEC = 2.0` (미래 충돌 예측 시간)
   - `ORCA_AGENT_RADIUS_M = 1.5` (기체 안전 반경)
   - `ORCA_MAX_SPEED_MPS = 3.0` (기존 following_velocity 기본값과 동일하게 3.0m/s로 설정)
   - `ORCA_MAX_VZ_MPS = 2.0` (최대 고도 제어 속도)
2. **`airsim_worker()` 충돌 감지 연동**:
   - `simGetCollisionInfo(vehicle_name=v_name)`를 호출하여 `latest_telemetries[d_id]`에 `"collided"`, `"collision_count"` 추가.
3. **`following_worker()` 로직 개선**:
   - 목표 지점(`tx, ty, tz`) 방향으로 거리 비례 감속이 적용된 선호 속도(`preferred_vel`) 계산.
   - `latest_telemetries`에서 다른 3대(Alpha 포함)의 실시간 위치/속도를 추출하여 `neighbors` 리스트 구성.
   - `orca.compute_safe_velocity(...)`를 통해 안전 속도 $(v_x, v_y, v_z)$ 산출.
   - `ctrl.moveByVelocityAsync(vx, vy, vz, duration=FOLLOW_TICK_INTERVAL_SEC * 1.5, vehicle_name=f_vname)`를 통해 World NED 프레임 속도 제어 명령 하달.
   - `ensure_api_control` 가드 유지 및 예외 처리 견고화.

### 3.3 [수정] `public/app.js`, `public/index.html`, `public/style.css` (UI 충돌 상태 표시)
- **디자인 톤 유지**: 텍스트 기반 3색 시스템 유지 (`--accent-critical` 빨강 활용).
- **Fleet Card 및 Status Panel**:
  - 각 드론 미니 카드에 충돌 횟수(`COL: 0`) 뱃지 추가 (충돌 시 빨간색 경고 활성화).
  - 텔레메트리 상세 패널 및 FPV HUD에 충돌 상태 배지 연동.

### 3.4 [신규] 테스트 스크립트 작성
- **`test_orca_collision_avoidance.py`**:
  - 시뮬레이터 연동 테스트.
  - 급격한 추격/급회전/짧은 지연시간(lag=0.8s) 상황에서 팔로워 간 충돌 발생 여부(`collision_count`)를 고빈도 샘플링으로 측정 및 검증.
- **`test_orca_unit.py`**:
  - AirSim 없이 순수 알고리즘의 반평면 계산 및 정면 충돌/교차 상황 회피 속도 계산을 검증하는 빠른 단위 테스트.

---

## 4. 단계별 진행 계획

1. **Phase 1: ORCA 수학 솔버 모듈 개발 및 단위 테스트 (`orca.py`, `test_orca_unit.py`)**
   - 2D 반평면 선형계획법 및 ORCA 제약 생성 구현
   - 2기체 정면충돌, 3기체 교차, 4기체 집결 시나리오 단위 검증
2. **Phase 2: 백엔드 통합 (`server.py`)**
   - 텔레메트리 내 `simGetCollisionInfo` 연동
   - `following_worker()`의 ORCA 안전 속도 계산 및 `moveByVelocityAsync` 연동
3. **Phase 3: 프론트엔드 충돌 모니터링 UI 연동 (`public/*`)**
   - 웹소켓 텔레메트리 데이터 바인딩 및 충돌 카운터/경고 뱃지 표시
4. **Phase 4: 시뮬레이터 기반 회귀 및 충돌 회피 실측 검증**
   - `test_following_mode.py` 회귀 검증
   - `test_ui_playwright.py` 데모 모드 회귀 검증
   - `test_orca_collision_avoidance.py` 충돌 회피 시나리오 검증
5. **Phase 5: 산출물 정리 및 작업완료 보고서(`03_completion_report_...`) 작성**

---

## 5. 예상 변경 파일 목록

| 구분 | 파일 경로 | 변경 내용 |
|---|---|---|
| 신규 | `orca.py` | 순수 Python + NumPy 기반 2D ORCA 충돌 회피 솔버 |
| 신규 | `test_orca_unit.py` | ORCA 솔버 수학적 정확성 단위 테스트 |
| 신규 | `test_orca_collision_avoidance.py` | AirSim 실환경 충돌 회피 및 추격 검증 테스트 |
| 수정 | `server.py` | ORCA 파라미터, 충돌 정보 텔레메트리 수집, `following_worker` 수정 |
| 수정 | `public/index.html` | 드론 카드/텔레메트리에 충돌 상태 표시 영역 추가 |
| 수정 | `public/app.js` | 웹소켓 충돌 정보 수신 및 UI 렌더링 업데이트 |
| 수정 | `public/style.css` | 충돌 뱃지 및 경고 스타일 (3색 시스템 기반) |
| 신규 | `docs/03_completion_report_following_mode_orca.md` | 구현 완료 보고서 (구현 후 작성) |
| 수정 | `docs/00_INDEX.md` | 문서 인덱스 상태 갱신 |
