# 독립검수 결과보고서 #20: 정적 장애물 ORCA 충돌 회피

- 검수 담당: Codex
- 검수 일시: 2026-08-17
- 검수 대상: `docs/19_completion_report_static_obstacle_avoidance.md`, `orca_static_obstacle_report.json` 및 관련 구현·테스트 코드
- 최종 판정: **승인 보류 (Not Approved)**

---

## 1. 검수 범위와 방법

작업지시서 #17의 정적 장애물 레지스트리·세 ORCA 통합 지점 적용·실환경 회피 검증 요구사항을 기준으로 완료보고서, 실측 JSON, `server.py`, `test_orca_static_obstacle.py`를 교차 검토했다.

독립 실행 검증:

- `python test_orca_unit.py`: 통과 (6개 테스트)
- `python -m py_compile server.py orca.py test_orca_static_obstacle.py`: 통과

AirSim AbandonedPark 비행은 이번 검수 시점에 재실행하지 않았으며, 제출된 원시 리포트와 이를 생성하는 코드의 제어·측정 경로를 대조했다.

## 2. 확인된 구현 사항

- 정적 장애물 캐시와 근접 장애물 선택 함수가 구현되어 있다.
- Following Mode, 편대 집결, RTH의 ORCA 이웃 목록에 정적 장애물이 추가된다.
- 제출 JSON은 AbandonedPark에서 441개 샘플, 충돌 0회, 대상 회전목마와 최소거리 4.89m를 기록한다.

## 3. 승인 보류 사유

### P1 — 핵심 테스트가 ORCA 정적 장애물 회피를 검증하지 못함

`test_orca_static_obstacle.py`는 Alpha의 목표를 장애물 너머로 직접 설정하지 않는다. 대신 Alpha에 대해 `moveByVelocityAsync(0.0, 2.0, ...)` 이후 `moveByVelocityAsync(1.2, 0.8, ...)`를 직접 보내 회전목마 주변을 수동으로 우회시킨다.

Alpha는 Following Mode의 ORCA 제어 대상이 아니므로, 보고된 최소거리 4.89m와 충돌 0회는 수동 조종 경로가 회전목마를 피한 결과일 수 있다. 팔로워가 정적 장애물 때문에 실제로 선호 속도에서 벗어나 회피했는지, 또는 정적 이웃을 제거했을 때 충돌/안전거리 위반이 발생하는지는 검증되지 않는다.

### P1 — 정적 장애물 레지스트리가 별도 AirSim 클라이언트를 생성함

`_build_static_obstacles_worker()`가 `airsim.MultirotorClient()`를 새로 생성해 씬을 조회한다. 작업지시서 #17은 씬 조회도 기존 제어·텔레메트리 클라이언트 수명 주기 안에서 수행하고 별도 독립 AirSim 클라이언트를 만들지 말 것을 명시했다. 이 구현은 해당 제약을 충족하지 않는다.

## 4. 재검수 조건

1. 레지스트리 구축을 기존 `client_telemetry` 또는 `client_control`과 그 보호 규칙을 사용하도록 변경하고, 맵 전환·재연결 시 캐시 무효화와 재구축을 유지한다.
2. 핵심 시험을 ORCA가 제어하는 기체로 구성한다. 예를 들어 팔로워의 지연 목표를 회전목마 너머에 두거나 편대 집결/RTH 목표가 장애물 반대편에 있게 하여, 선호 경로가 실제 장애물을 관통하도록 만든다.
3. 테스트에서 회피 동작을 정량 검증한다. 최소 거리·충돌 0회뿐 아니라 직선 선호 경로 대비 횡방향 편차 또는 회피 속도 성분을 기록하고, 가능하면 정적 장애물 이웃 비활성 비교군을 추가한다.
4. 수정 뒤 AbandonedPark 실환경 시험과 Blocks 회귀 시험을 재실행하고 원시 리포트를 갱신한다.

## 5. 결론 (1차)

정적 장애물 이웃을 세 ORCA 경로에 연결한 구현 자체는 확인됐다. 그러나 제출된 실측은 수동 회피 경로를 사용해 ORCA의 정적 장애물 회피 효과를 입증하지 못하며, 레지스트리 클라이언트 수명 주기도 작업지시서의 안전 제약과 다르다.

따라서 본 작업은 **승인 보류(Not Approved)** 로 판정한다.

---

## 6. 2차 재검수 (Round 2)

- 검수 일시: 2026-08-18
- 검수 대상: 개정된 `docs/19_completion_report_static_obstacle_avoidance.md`, `orca_static_obstacle_report.json`, 개정된 `orca.py`/`server.py`/`test_orca_static_obstacle.py`
- 최종 판정: **승인 보류 (Not Approved)**

### 6.1 개선 확인 사항

- `orca.py`의 정적 장애물 전용 우회 편향(장애물이 전방에 있으면 강제로 큰 횡방향 바이어스를 주던 분기)과 `_do_rth()`의 수동 우회 서브목표(`target_x_nav = ox + 3.8`) 코드가 완전히 제거되고, 순수 van den Berg (2011) 표준 2D ORCA 솔버로 복원됨을 확인.
- 정적 장애물 레지스트리가 `client_telemetry` 기반으로 전환되어(별도 독립 `airsim.MultirotorClient()` 생성 제거) 1차 검수의 P1(레지스트리 클라이언트 수명 주기) 지적은 해소됨.
- 정적 이웃 입력 시 `safe_vel`에 횡방향 성분이 실제로 발생하는 원시 샘플을 확인.
- 구문 검사 및 `test_orca_unit.py` 7건 통과.

### 6.2 승인 보류 사유 (신규/재확인)

1. **대조군이 실제로는 충돌하지 않음**: 제출된 원시 JSON은 `control_group_metrics.total_collisions: 0`, `first_collision_point: null`이다. 그런데 완료보고서 4.1절 표는 대조군 결과를 "정면 물리 교차 충돌 발생"이라고 서술한다 - 원시 데이터와 서술이 직접 모순된다.
2. **테스트의 판정식 자체가 충돌 없이도 통과되도록 작성됨**: `test_orca_static_obstacle.py:305`의 `pass_ctrl = ctrl_collision_count >= 1 or ctrl_min_obs_dist < 1.0`는 실제 `simGetCollisionInfo` 충돌 이벤트가 0회여도 최소 XY 거리가 1.0m 미만이기만 하면(이번 실측은 0.07m) "대조군 충돌 인과관계 입증"으로 PASS 처리한다. 근접 거리와 실제 물리 충돌 이벤트를 하나의 판정으로 섞은 것이 1번 모순의 원인이다.
3. **실측이 실제 `server.py` 통합 경로를 검증하지 않음**: `test_orca_static_obstacle.py`는 서버의 정적 장애물 레지스트리(`_build_static_obstacles`)나 `get_static_obstacle_neighbors()`, 또는 `following_worker()`/`_do_formation_assemble()`/`_do_rth()` 중 어느 것도 거치지 않는다. 테스트가 직접 `client.simGetObjectPose()`로 회전목마 좌표를 얻어 자체 `obstacle_dict`를 만들고, 자체 20Hz 루프에서 `orca.compute_safe_velocity()`를 직접 호출한 뒤 `client.moveByVelocityAsync()`로 Bravo에 직접 명령을 보낸다(182~213행). 이는 1차 검수 P1(핵심 테스트가 ORCA 정적 장애물 회피를 검증하지 못함)과 동일한 지적이며, 2차 제출에서도 해소되지 않았다.

### 6.3 재검수 조건

1. 대조군의 "충돌" 판정은 오직 `simGetCollisionInfo().has_collided` 실이벤트로만 내리고, 근접 거리를 대체 판정 기준으로 쓰지 않는다. 실제 충돌이 없었다면 보고서에도 있는 그대로 기록한다.
2. 시험군/대조군 모두 실제 서버의 `following_worker()`/`_do_formation_assemble()`/`_do_rth()` 중 하나를 HTTP API로 트리거해서 실행하고, 정적 장애물 이웃 주입 여부만 서버 측에서 켜고 끌 수 있어야 한다(테스트 스크립트가 `orca.compute_safe_velocity()`를 직접 호출하거나 드론에 직접 속도 명령을 보내면 안 됨).
3. 위 두 조건을 만족하는 재작업 지시서(`docs/21_work_order_static_obstacle_test_methodology_fix.md`)를 별도로 발행한다.

### 6.4 결론 (2차)

정적 장애물 회피 구현 자체(레지스트리, 3개 통합 지점, 순수 ORCA 솔버 복원)는 유효하다. 그러나 이를 입증하는 실측 방법론이 실제 통합 경로를 우회하고, 대조군 충돌 여부를 근접 거리로 대체 판정하는 문제가 남아있어 **승인 보류(Not Approved)** 를 유지한다.
