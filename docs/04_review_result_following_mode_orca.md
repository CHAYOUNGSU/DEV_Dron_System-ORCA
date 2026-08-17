# 독립검수 결과보고서 #04: Following Mode ORCA 충돌 회피

- 검수 담당: Codex
- 검수 일시: 2026-08-17
- 검수 대상: `docs/03_completion_report_following_mode_orca.md`, `orca_collision_avoidance_report.json` 및 관련 구현·테스트 코드
- 최종 판정: **승인 (Approved)**

---

## 1. 검수 범위와 방법

완료보고서와 AirSim 실측 JSON의 수치 일관성을 대조하고, `server.py`의 ORCA 설정·Following Mode 제어 경로, `test_orca_collision_avoidance.py`의 성공 기준, `test_orca_unit.py`를 교차 검토하였다.

독립 실행 검증으로 다음을 수행하였다.

- `python test_orca_unit.py`: 통과 (6개 테스트)
- `python -m py_compile orca.py server.py test_orca_unit.py test_orca_collision_avoidance.py`: 통과

AirSim Blocks 실환경 스트레스 결과는 본 검수 환경에서 새로 비행을 실행하지 않고, 제출된 원시 측정 리포트와 그 리포트를 생성하는 테스트 코드의 판정 로직을 대조하여 검증하였다.

## 2. 검수 결과

| 항목 | 기준 | 확인 결과 | 판정 |
|---|---|---|---|
| ORCA 안전 반경 | 기체당 1.5m | `server.py`와 스트레스 테스트가 동일하게 1.5m 사용 | 통과 |
| 최소 이격 거리 | 3.0m 이상 (`2 × radius`) | 3.10m | 통과 |
| 충돌 감지 | 충돌 0회 | 4대 모두 0회, 이벤트 없음 | 통과 |
| 샘플 무결성 | 기체당 40개 이상, 오류 0건 | 기체당 191개, 오류 0건 | 통과 |
| Following 회귀 | 팔로워 이동·방향 정합 | 3대 모두 2m 이상 이동, 정합=true | 통과 |
| 단위 검증 | 회피와 목표 도달 | 최소거리 2.01m, 양측 목표 오차 0.18m | 통과 |

## 3. 주요 확인 사항

- `ORCA_AGENT_RADIUS_M = 1.5`와 스트레스 테스트의 요구 이격 거리 `2 × 1.5 = 3.0m`가 일치한다.
- 제출된 실측 리포트는 총 764개 샘플, 샘플링 오류 0건, 총 충돌 0회, 최소 이격 거리 3.10m를 기록한다.
- 테스트는 샘플 수, 초기 프로브와 반복 샘플링 오류, 충돌, ORCA 안전 이격, 팔로워 이동 방향을 모두 성공 조건에 포함한다.
- Following Mode 제어부는 월드 좌표계를 사용하고, Alpha를 포함한 나머지 기체를 ORCA 이웃으로 구성하며, 안전 속도를 `moveByVelocityAsync`로 전달한다.

## 4. 결론

이전 검수에서 보류했던 안전 이격 기준 불일치, 다중 스텝 목표 도달 미검증, 초기 충돌 프로브 오류 미집계 문제가 해소되었다. 제출 산출물과 코드·테스트 기준은 서로 일관되며, 작업지시서 #01의 Following Mode ORCA 충돌 회피 적용 요구사항을 충족한다.

따라서 본 작업을 **승인(Approved)** 한다.

## 5. 후속 권고

이번 승인은 Blocks 기반의 제출 실측 결과와 코드 검토에 근거한다. 실제 운용 전에는 다른 맵, 더 짧은 추격 지연, 바람·센서 지연 등 조건을 포함한 장시간 반복 시험을 별도로 수행하는 것을 권고한다.
