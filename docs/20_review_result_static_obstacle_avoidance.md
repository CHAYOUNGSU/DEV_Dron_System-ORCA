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

## 5. 결론

정적 장애물 이웃을 세 ORCA 경로에 연결한 구현 자체는 확인됐다. 그러나 제출된 실측은 수동 회피 경로를 사용해 ORCA의 정적 장애물 회피 효과를 입증하지 못하며, 레지스트리 클라이언트 수명 주기도 작업지시서의 안전 제약과 다르다.

따라서 본 작업은 **승인 보류(Not Approved)** 로 판정한다.
