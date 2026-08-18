# 독립검수 결과보고서 #24: 정적 장애물 회피 검증 방법론 재작업

- 검수 담당: Codex
- 검수 일시: 2026-08-18
- 검수 대상: `docs/23_completion_report_static_obstacle_test_methodology_fix.md`, `orca_static_obstacle_report.json`, `server.py`, `test_orca_static_obstacle.py`
- 최종 판정: **승인 보류 (Not Approved)**

---

## 1. 확인된 사항

- 테스트가 서버 API 기반 Following Mode 검증으로 전환된 점, 정적 장애물 ON/OFF 토글(`static_obstacles_enabled` + `/api/debug/static_obstacles_toggle`), `orca.py` 직접 호출 및 Bravo 직접 속도 제어 제거는 확인됨 - 작업지시서 #21의 3.1절·3.3절 요구사항은 충족.
- 구문 검사 및 `test_orca_unit.py` 7건 통과.

## 2. 승인 보류 사유

핵심 시나리오가 작업지시서 #21의 "장애물을 향한 동일 경로 A/B"를 충족하지 못함.

- 완료보고서와 계획서는 Alpha/Bravo가 X≈0 직선으로 회전목마를 정면 통과한다고 서술하지만, 실제 테스트는 Alpha를 `X=+5.5m` 통로로 조종한다 (`test_orca_static_obstacle.py:241,254`, 주석: "clearing carousel outer roof mesh (X ~ +5.5m)").
- 원시 리포트에서도 대조군(정적 장애물 회피 OFF) 최소 이격 거리는 `5.00m`, 충돌 `0`회. 시험군·대조군 모두 결합 안전반경(드론 1.6m + 장애물 2.2m = 3.8m)을 애초에 침범하지 않았다 - 즉 대조군이 실제로 위험한 경로였다는 증거가 없다.
- 시험군 `6.53m`와 대조군 `5.00m`의 차이, 또는 횡방향 편차 `8.10m` vs `6.09m`의 차이는 정적 장애물 회피가 "필요했는지"를 증명하지 못한다 - Alpha의 우측 통로 유도 자체와 Following Mode 지연·기체 간 상호 회피가 만든 궤적 차이일 가능성을 배제할 수 없다.
- 판정식(`test_orca_static_obstacle.py:324`)이 `pass_ctrl_proof = (ctrl_res['collision_count'] >= 1) or (ctrl_res['min_obs_dist'] < test_res['min_obs_dist']) or (extra_lateral_avoidance >= 1.0)`로, 실제 충돌이나 위험 반경 침범이 전혀 없어도 "시험군이 대조군보다 1m 더 옆으로 이동"하기만 하면 합격 처리된다. 안전 위협이 애초에 없던 경로에서의 임의 변위도 합격시킬 수 있는 판정식이다.

## 3. 재승인 조건

1. Alpha의 기준 경로와 Bravo의 지연 추격 경로가 실제로 회전목마 결합 안전반경(3.8m)을 가로지르도록 구성한다.
2. OFF 대조군에서 최소 이격 거리 `< 3.8m` 또는 실제 충돌(`has_collided`)을 기록해야 한다.
3. ON 시험군은 동일 초기·목표 조건에서 결합 안전반경(3.8m) 이상을 유지해야 한다.
4. 판정식에서 근접 거리·횡방향 편차 차이를 OR로 묶어 충돌 대체 증거로 쓰는 방식을 제거한다.

## 4. 결론

정적 장애물 ON/OFF 토글과 서버 통합 경로 기반 검증 아키텍처 자체는 올바르게 구현됐다. 그러나 실제 비행 경로가 장애물의 위험 반경을 애초에 지나가지 않아 인과관계를 입증하지 못하며, 판정식도 그 사실을 가릴 수 있는 완화된 OR 조건을 쓰고 있다.

따라서 본 작업은 **승인 보류(Not Approved)** 로 판정한다.
