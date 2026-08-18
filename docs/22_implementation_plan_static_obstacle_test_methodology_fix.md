# [작업계획서 #22] 정적 장애물 회피 실측 방법론 재작업

## 1. 개요
- **문서 번호**: 22_implementation_plan_static_obstacle_test_methodology_fix.md
- **작업 지시서**: [docs/21_work_order_static_obstacle_test_methodology_fix.md](file:///D:/0_DEV/DEV_Dron_System-ORCA/docs/21_work_order_static_obstacle_test_methodology_fix.md)
- **작성자**: Antigravity
- **검수자**: Codex (독립검수)
- **작성일자**: 2026-08-18
- **상태**: 작성 완료 (사용자 및 Codex 사전 승인 요청)

---

## 2. 배경 및 재작업 범위 정의

### 2.1 핵심 배경: 이전 테스트의 근본적 한계
- 기존 `test_orca_static_obstacle.py`는 테스트 스크립트 자체가 `orca.compute_safe_velocity()`를 직접 계산하고 `moveByVelocityAsync()`로 기체를 제어하여, 서버 내부의 실제 비행 루프(`following_worker`, `_do_formation_assemble`, `_do_rth`)에 통합된 정적 장애물 회피를 온전히 검증하지 못했습니다.
- RTH는 안전 고도($Z=-15\text{m}$)로 상승하여 비행하므로 수직 필터($\Delta Z \le 8.0\text{m}$)에 의해 회전목마($Z=+3.64\text{m}$)와 상공 18m 이상 이격되어, 수평 경로 관통 시나리오의 인과성을 명확히 증명하기 어려웠습니다.
- 대조군 판정에서 근접 거리($< 1.0\text{m}$)로 충돌을 대체 판정하여 리포트 서술과 원시 JSON 간의 정합성 문제가 발생했습니다.

### 2.2 작업 범위 명확화
- **유지 대상 (수정 금지)**:
  - `orca.py`: 순수 Jur van den Berg (2011) 2D ORCA 표준 솔버 유지.
  - `_build_static_obstacles(client_telemetry, sim_id)`: `airsim_worker()` 내 동기 1회 캐싱 유지.
  - `get_static_obstacle_neighbors()` 기본 알고리즘 및 3대 비행 루프(`following_worker`, `_do_formation_assemble`, `_do_rth`) 연결 구조 유지.
- **수정/추가 대상**:
  1. `server.py`: 시험군/대조군 A/B 테스트를 위한 디버그 토글 플래그 `static_obstacles_enabled` 및 `/api/debug/static_obstacles_toggle` 엔드포인트 추가.
  2. `test_orca_static_obstacle.py`: 테스트 스크립트의 자체 ORCA 계산 및 직접 속도 명령을 완전히 제거하고, **순수 서버 HTTP API(`/api/takeoff`, `/api/following/toggle`, `/api/joystick` 등)만으로 서버를 조종**하며 20Hz 읽기 전용 샘플러로 결과를 수집하는 표준 아키텍처로 전면 재작성.
  3. 대조군과 시험군의 정직한 실측 지표 분리 기록.

---

## 3. 검증 대상 기능 및 시나리오/좌표 설계

### 3.1 검증 기능: Following Mode (추격 편대 비행) 선정 근거
1. **저고도 순항 비행 ($Z \approx -3.5\text{m} \sim -4.0\text{m}$)**:
   - 회전목마 구조물 본체($Z = +3.64\text{m} \sim -4.5\text{m}$)와 완벽히 동일 수평 고도에서 진행되므로, 수직 필터($\Delta Z \le 8.0\text{m}$)를 자연스럽게 통과하며 실제 물리적 충돌 영역을 관통합니다.
2. **실제 서버 비행 워커(`following_worker`) 직접 검증**:
   - 편대장(Alpha)의 이동 궤적을 1.5초 지연 추격하는 Bravo(Drone2)가 서버의 `following_worker()` 내부 ORCA 루프에 의해 장애물 반평면을 계산하고 횡방향으로 자율 우회하는 실제 서버 경로를 100% 검증합니다.
3. **기존 검증된 패턴과의 일관성**:
   - `test_orca_collision_avoidance.py`의 고신뢰성 샘플링 및 스트레스 기동 검증 패턴을 그대로 계승할 수 있습니다.

### 3.2 상세 비행 궤적 및 시나리오 설계
- **테스트 환경**: `AbandonedPark` 시뮬레이터
- **대상 정적 장애물**: `SM_CarouselA_2` (월드 좌표 $X=-0.07\text{m}, Y=17.92\text{m}, Z=+3.64\text{m}$, 반경 $R=2.2\text{m}$)
- **편대 초기 배치**:
  - 편대 전 기체 이륙 (`/api/fleet/takeoff`) 후 고도 $Z=-3.7\text{m}$ 유지.
  - 편대장 Alpha(SimpleFlight) 초기 위치: $(X=0.0\text{m}, Y=0.0\text{m}, Z=-3.7\text{m})$.
  - 추격 기체 Bravo(Drone2) 초기 위치: $(X=0.0\text{m}, Y=3.5\text{m}, Z=-3.7\text{m})$.
- **비행 기동**:
  1. 서버 Following Mode 활성화 (`POST /api/following/toggle`, `velocity=3.0`, `lag=1.5s`).
  2. Alpha가 회전목마 좌우를 관통/경유하는 전방 기동 수행:
     - `/api/joystick`을 통해 Alpha를 전방 $+Y$ 방향($Y=0.0 \to Y=35.0\text{m}$)으로 순항 비행 ($V_y = 3.0\text{m/s}$).
     - Alpha는 회전목마 중심($Y=17.92\text{m}$)을 통과하도록 $X \approx 0.0\text{m}$ 직선 경로로 유도.
  3. 팔로워 Bravo(Drone2)는 Alpha의 궤적을 1.5초 후 추격하면서, 직선 경로 정중앙에 위치한 회전목마(`SM_CarouselA_2`)를 정면으로 마주침.

---

## 4. 시험군(A) vs 대조군(B) 실측 검증 설계

### 4.1 시험군 (Test Group - Static Obstacle ORCA ON)
- **설정**: `POST /api/debug/static_obstacles_toggle {"enabled": true}` (기본값)
- **동작**:
  - Bravo가 Alpha를 추격하여 $Y \approx 17.92\text{m}$ 부근에 접근할 때, 서버의 `following_worker()`가 `get_static_obstacle_neighbors()`에서 회전목마를 이웃으로 주입.
  - 서버의 `orca.compute_safe_velocity()`가 정적 장애물 반평면을 계산하여 Bravo에게 횡방향 회피 속도($V_x \ne 0$)를 자동 명령.
- **성공 판정 기준**:
  - 횡방향 최대 회피 편차 $\max(|\Delta X|) \ge 1.0\text{m}$
  - 회전목마와의 최소 실측 이격 거리 $D_{min} \ge 2.2\text{m}$
  - 편대 및 장애물 충돌 횟수 `total_collisions == 0`

### 4.2 대조군 (Control Group - Static Obstacle ORCA OFF)
- **설정**: `POST /api/debug/static_obstacles_toggle {"enabled": false}`
- **동작**:
  - 동일한 Alpha 전방 순항 기동 실행.
  - 서버의 `following_worker()`에서 정적 장애물 이웃 주입이 차단되어, Bravo는 Alpha의 직선 궤적($X=0.0\text{m}$)을 그대로 추격.
  - $Y \approx 17.92\text{m}$ 지점에서 회전목마와 위험 근접 ($D_{min} \approx 0.07\text{m}$) 또는 실제 물리 충돌 발생.
- **정직한 평가 기록 원칙**:
  - `ctrl_collision_count` (실제 `has_collided` 이벤트 횟수)와 `ctrl_min_obs_dist` (실측 최소 거리)를 **별도 항목으로 분리하여 정직하게 기록**.
  - 만약 저속 접촉으로 이벤트 카운트가 0이어도 "충돌"이라 부르지 않고, "최소 이격 $0.07\text{m}$로 장애물 안전 반경(2.2m) 침범 및 정면 교차 통과"로 사실 그대로 기술.

---

## 5. 구현 세부 계획

### 5.1 `server.py` 수정
- `static_obstacles_enabled = True` 전역 변수 선언.
- `get_static_obstacle_neighbors()` 시작부에 `if not static_obstacles_enabled: return []` 가드 추가.
- `/api/debug/static_obstacles_toggle` POST 엔드포인트 추가 (기본값 `True`, UI 비노출).

### 5.2 `test_orca_static_obstacle.py` 전면 재작성
- **완전한 서버 조종 아키텍처**:
  - `api_post("/api/simulators/launch", {"id": "park"})`
  - `api_post("/api/fleet/takeoff")`
  - `api_post("/api/debug/static_obstacles_toggle", {"enabled": True/False})`
  - `api_post("/api/following/toggle", {"enabled": True, "velocity": 3.0, "lag_seconds": 1.5})`
  - `api_post("/api/joystick", {"drone_id": "Drone1", "vy": 3.0, "duration": ...})`
  - `api_post("/api/following/toggle", {"enabled": False})`
- **테스트 스크립트 내부 제약**:
  - `orca.py` 임포트 및 `orca.compute_safe_velocity()` 호출 코드 **완전 제거**.
  - 기체에 직접 속도 명령(`moveByVelocityAsync`)을 내리는 코드 **완전 제거**.
  - 읽기 전용 20Hz 백그라운드 샘플러(`getMultirotorState`, `simGetCollisionInfo`)만 유지.
  - `try / finally` 블록으로 테스트 종료 시 반드시 `static_obstacles_toggle(enabled=True)` 복원 보장.

---

## 6. 검증 계획

### 6.1 신규 정적 장애물 실측 테스트
```powershell
python test_orca_static_obstacle.py
```
- 시험군(ON): $D_{min} \ge 2.2\text{m}$, $\max(|\Delta X|) \ge 1.0\text{m}$, 충돌 0회 확인.
- 대조군(OFF): 동일 경로에서 정적 장애물 회피 미작동 시 궤적 차이 및 위험 근접/충돌 실측.
- `orca_static_obstacle_report.json` 생성 확인.

### 6.2 전체 기존 회귀 테스트 전수 실행
1. `python test_orca_collision_avoidance.py` (Following Mode 편대 충돌 회피)
2. `python test_orca_formation_assemble.py` (편대 집결 크로스오버 충돌 회피)
3. `python test_orca_rth.py` (RTH 동시 복귀 및 락 일원화)
4. `python test_orca_unit.py` (단위 수학 및 정적 장애물 시뮬레이션)
5. `python test_ui_playwright.py` (Playwright UI 19개 시나리오)

### 6.3 구문 검사
```powershell
python -m py_compile server.py orca.py test_orca_static_obstacle.py
```

---

## 7. 향후 일정 및 산출물
1. 본 작업계획서(#22)에 대한 사용자 승인 획득.
2. `server.py` 수정 및 `test_orca_static_obstacle.py` 전면 재작성.
3. AbandonedPark 실환경 실측 및 5대 회귀 테스트 전수 수행.
4. `docs/23_completion_report_static_obstacle_test_methodology_fix.md` 작성 및 `docs/00_INDEX.md` 갱신.
5. Codex 최종 검수 요청.
