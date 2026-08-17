# 작업계획서 #18: 정적 장애물(건물/지형/구조물) ORCA 충돌 회피 적용 계획 (개정판)

- 작성자: Antigravity (구현 담당)
- 대상 작업지시서: `docs/17_work_order_static_obstacle_avoidance.md` (작성: Claude)
- 검수 대상: Codex (독립검수 / Review 역할)
- 대상 레포: `DEV_Dron_System-ORCA`

---

## 1. 개요 및 목적

Following Mode(#01~#04), 편대 집결(#05~#08), RTH(#09~#16)의 다중 기체 ORCA 충돌 회피 시스템을 확장하여, **드론 간 상호 회피뿐만 아니라 맵 상의 정적 장애물(건물, 놀이기구 구조물, 기둥 등)에 대해서도 비상호적(Weight=1.0, Vel=(0,0,0)) ORCA 충돌 회피를 수행**하도록 구축합니다.

---

## 2. 사전 조사 및 분석 결과

### 2.1 맵별 씬 오브젝트 프로브 결과
`scratch/probe_static_objects.py` 및 `scratch/probe_other_maps.py`를 통해 맵별 `simListSceneObjects` 및 `simGetObjectPose`를 실측 조사했습니다:

1. **Blocks (기본 비행 훈련장)**:
   - 총 227개 오브젝트 (원점 150m 이내 209개)
   - 주요 장애물: `TemplateCube_Rounded_*`, `Cylinder*`, `Cone_*`, `OrangeBall`
   - 비물리/바닥: `Ground*`, `CameraActor_*`
2. **AbandonedPark (폐허 테마파크)**:
   - 총 231개 오브젝트 (원점 200m 이내 213개)
   - 주요 장애물: `SM_CarouselA_2` (회전목마, 월드 좌표: `(x=-0.07, y=17.92, z=3.64)` - 스폰 원점 정면 18m에 위치하여 테스트에 최적), `SM_DaisyStage*`, `SM_DaisySupport*`, `SM_Fence*`
   - 비물리/바닥: `SM_Asphalt_*`, `CameraActor_*`
3. **CityEnviron (도심 빌딩숲)**:
   - 총 5,707개 오브젝트 (원점 200m 이내 1,096개)
   - 주요 장애물: `Apartment_*`, `Bench*`, `AirDuct_*`, `006_hatchback_*`

### 2.2 좌표계 정합성 확인
- `simGetObjectPose(name).position`은 Unreal 월드 좌표계(미터 단위)를 반환합니다.
- Alpha의 스폰 오프셋이 `(0.0, 0.0, 0.0)`이므로 시뮬레이터 월드 좌표계와 프로젝트의 편대 월드 좌표계(`pos_local + spawn_offset`)가 완벽히 일치합니다.
- `orca.py`의 2D 평면 ORCA 계산은 수평면(XY) 좌표만 사용하므로, Z축 표기 차이와 무관하게 수평 충돌 회피가 정확히 동작합니다.

---

## 3. 세부 설계 및 동시성 아키텍처

### 3.1 비동기 백그라운드 레지스트리 구축 (동시성 안전 보장)
CityEnviron 등 대형 맵(오브젝트 수천 개)에서 `simGetObjectPose()`를 순차 호출하면 수 초~수십 초가 소요될 수 있습니다. 메인 제어 및 텔레메트리 루프의 블로킹을 원천 차단하기 위해 다음 아키텍처를 적용합니다:

1. **전용 비동기 스레드 실행 (Fire-and-Forget)**:
   - `airsim_worker()`에서 새 맵이 로드되고 `spawn_verified = True`가 되는 시점에 메인 루프를 블로킹하지 않고 **별도 백그라운드 스레드(`threading.Thread(target=_build_registry_worker, daemon=True).start()`)**로 실행합니다.
   - `control_lock`을 전혀 점유하지 않으므로 사용자의 실시간 비행 제어(이착륙/조이스틱/RTH 등)에 일체의 지연(0ms)이 발생하지 않습니다.
   - `airsim_worker()`의 25Hz 텔레메트리 폴링 및 FPV 카메라 스트리밍도 전혀 중단되지 않습니다.
2. **독립 RPC 소켓 사용 및 안전 종료**:
   - 백그라운드 구축 스레드 내부에서 전용 `registry_client = airsim.MultirotorClient(timeout_value=5)`를 열어 씬 조회를 완료한 후 소켓을 닫습니다.
3. **원자적 캐시 교체 및 스레드 안전성**:
   - `static_obstacles_lock = threading.Lock()`을 선언하여, 구축이 완료되는 순간 `with static_obstacles_lock: cached_static_obstacles = new_obstacles`로 원자적 교체를 수행합니다.
   - 맵 전환(`is_switching_simulator`) 또는 프로세스 종료 시 `cached_static_obstacles = []`로 즉시 초기화하여 이전 맵의 "유령 장애물"을 방지합니다.
4. **대형 맵 소요 시간 실측 기록**:
   - Blocks, AbandonedPark, CityEnviron에서의 레지스트리 구축 소요 시간(초)을 완료보고서에 실측치로 기록합니다.

### 3.2 필터링 전략
1. **비물리 / 지면 / 조명 / 카메라 필터링**:
   - 제외 키워드 패턴: `['camera', 'ground', 'asphalt', 'light', 'sky', 'particle', 'trigger', 'cine', 'player', 'postprocess', 'fog', 'volume', 'terrain']` (대소문자 무시)
2. **활동 반경 필터링 (연결 시점)**:
   - 스폰 원점 기준 반경 $150m$ 이내 (`abs(x) <= 150` and `abs(y) <= 150`) 오브젝트만 레지스트리에 등록.
3. **매 틱 실시간 근접 필터링 및 개수 제한**:
   - 에이전트 현재 위치 `cur_wpos` 기준 거리 $D \le 25.0m$ 이내의 장애물만 선택.
   - 거리순으로 정렬하여 **가장 가까운 상위 $N=6$개**만 ORCA 입력으로 제한 (10Hz 제어 주기 오버헤드 0.5ms 미만 유지).

### 3.3 정적 장애물 안전 반경 및 한계 명시
- **반경 상수**: `ORCA_STATIC_OBSTACLE_RADIUS_M = 2.2m`
- **ORCA 이웃 속성**:
  ```python
  {
      "pos": (obs_x, obs_y, obs_z),
      "vel": (0.0, 0.0, 0.0),       # 정지 상태
      "radius": ORCA_STATIC_OBSTACLE_RADIUS_M, # 2.2m
      "weight": 1.0                  # 비상호적 (100% 에이전트가 회피 부담)
  }
  ```
- **모델의 한계 및 범위 명시**:
  - 본 "단일 좌표점 + 고정 반경(2.2m)" 모델은 회전목마, 기둥, 동상, 놀이기구 등 **점 형태에 가까운 컴팩트한 구조물**에 최적화된 근사 모델입니다.
  - XY 평면상 수십 미터 폭을 가진 대형 빌딩(CityEnviron급)은 단일 점으로 표현할 경우 모서리 충돌 가능성이 있으며, 대형 복합 건물에 대한 정밀 다각형/바운딩 박스 회피는 이번 작업 범위에서 제외되고 향후 고도화 과제로 관리됩니다.

### 3.4 3대 ORCA 통합 지점 공통 함수 연결
공통 헬퍼 함수 `get_static_obstacle_neighbors(agent_wpos, max_dist=25.0, max_count=6) -> list`를 정의하고, 다음 3곳에 주입:
1. `following_worker()` (Following Mode)
2. `_do_formation_assemble()` (편대 집결)
3. `_do_rth()` 내부 `run_rth_orca_leg()` (RTH 복귀)

기존 드론 이웃 리스트 뒤에 `neighbors.extend(get_static_obstacle_neighbors(cur_wpos))` 형태로 결합합니다.

---

## 4. 검증 계획

### 4.1 핵심 실환경 검증 (`test_orca_static_obstacle.py`)
- **대상 맵**: `AbandonedPark` (폐허 테마파크)
- **시나리오**:
  - Alpha(Drone1)를 `SM_CarouselA_2` (회전목마 구조물, Y=+17.9m) 방향을 관통하는 전진 목표점 `(x=0.0, y=25.0, z=-3.5)`으로 비행 명령.
  - 회피 미적용 시 충돌하는 직선 경로 상에서, ORCA가 구조물 반경 2.2m 밖으로 궤적을 굴절시켜 **무충돌(collision=0) 및 최소 이격 $\ge 2.5m$**를 유지하며 목표점에 도달하는지 20Hz 샘플러로 실측.
  - Following Mode 활성화 상태에서 윙맨(Bravo, Charlie, Delta)도 편대장을 따라 구조물을 안전하게 우회하는지 실측.

### 4.2 기존 Blocks 맵 전수 회귀 테스트
- `test_orca_collision_avoidance.py`: Following Mode 스트레스 실측 회귀 통과 (오탐 없음 확인)
- `test_orca_formation_assemble.py`: 편대 집결 크로스오버 실측 회귀 통과
- `test_orca_rth.py`: RTH 동시성 및 착륙 오버라이드 실측 회귀 통과
- `test_orca_unit.py`: 단위 테스트 6/6 통과
- `test_ui_playwright.py`: UI 관제 테스트 19/19 통과

---

## 5. 산출물
- `server.py` (비동기 정적 장애물 레지스트리 + 3대 ORCA 통합 지점 공통 연결)
- `test_orca_static_obstacle.py` (AbandonedPark 실환경 정적 장애물 회피 실측 스크립트)
- `docs/18_implementation_plan_static_obstacle_avoidance.md` (본 계획서)
- `docs/19_completion_report_static_obstacle_avoidance.md` (완료보고서)
- `docs/00_INDEX.md` 갱신
