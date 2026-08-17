# 문서 인덱스

작업 순서대로 번호가 매겨집니다. 각 에이전트는 자신의 역할에 해당하는
문서만 새로 작성하고, 이전 단계 문서는 읽기 전용 참고자료로 취급합니다.

| 순번 | 파일 | 역할 | 담당 | 상태 |
|---|---|---|---|---|
| 01 | `01_work_order_following_mode_orca.md` | 작업지시서 (설계) | Claude | 완료 |
| 02 | `02_implementation_plan_following_mode_orca.md` | 작업계획서 (구현 착수 전) | Antigravity | 완료 |
| 03 | `03_completion_report_following_mode_orca.md` | 작업완료 보고서 (구현 후) | Antigravity | 완료 |
| 04 | `04_review_result_following_mode_orca.md` | 검수결과 (독립검수) | Codex | 승인 |
| 05 | `05_work_order_formation_assemble_orca.md` | 작업지시서 (설계) | Claude | 완료 |
| 06 | `06_implementation_plan_formation_assemble_orca.md` | 작업계획서 (구현 착수 전) | Antigravity | 완료 |
| 07 | `07_completion_report_formation_assemble_orca.md` | 작업완료 보고서 (구현 후) | Antigravity | 완료 |
| 08 | `08_review_result_formation_assemble_orca.md` | 검수결과 (독립검수) | Codex | 승인 |

| 09 | `09_work_order_rth_orca.md` | 작업지시서 (설계) | Claude | 완료 |
| 10 | `10_implementation_plan_rth_orca.md` | 작업계획서 (구현 착수 전) | Antigravity | 완료 |
| 11 | `11_completion_report_rth_orca.md` | 작업완료 보고서 (구현 후) | Antigravity | 완료 (보강 제출) |
| 12 | `12_review_result_rth_orca.md` | 검수결과 (독립검수) | Codex | 승인 보류 (재검수 대기 아님 - 재작업 필요) |
| 13 | `13_work_order_rth_concurrency_fix.md` | 작업지시서 (재작업 - 설계) | Claude | 완료 |
| 14 | `14_implementation_plan_rth_concurrency_fix.md` | 작업계획서 (구현 착수 전) | Antigravity | 완료 |
| 15 | `15_completion_report_rth_concurrency_fix.md` | 작업완료 보고서 (구현 후) | Antigravity | 완료 (보강 재제출) |
| 16 | `16_review_result_rth_concurrency_fix.md` | 검수결과 (독립검수) | Codex | 승인 |

## 작업 #2: 편대 집결(Formation Assemble) ORCA 적용

Following Mode ORCA(#01~#04, 승인 완료)와 동일한 `orca.py` 솔버를
재사용해서, 다수 기체가 동시에 좁은 공간으로 수렴하는 편대 집결
("알파 호출") 기동에도 충돌 회피를 적용하는 작업입니다. 작업지시서
#05에 Following Mode 작업 중 함께 발견된 세계 좌표계 정합성 문제(각
기체의 위치가 자신의 스폰 지점 기준 로컬 좌표라 그대로 쓰면 어긋남)에
대한 보정도 포함되어 있습니다. **승인 완료 (#08).**

## 작업 #3: RTH(자동 복귀) 좌표계 수정 + ORCA 적용

편대 집결과 똑같은 세계 좌표계 버그가 `_do_rth()`에도 그대로 남아있는
것을 발견해서 (스폰 오프셋을 로컬 좌표인 것처럼 잘못 사용 - 알파를
제외한 모든 기체가 자기 홈에서 벗어난 위치에 착륙하게 됨), 이 수정과
RTH 비행 경로(상승/수평복귀/하강)에 대한 ORCA 적용을 함께 다루는
작업입니다. 상세: `docs/09_work_order_rth_orca.md`.

**#12에서 승인 보류** (동시 RTH가 실제로는 `control_lock` 장기 점유로
직렬 실행되어 핵심 요구사항이 검증되지 않음). 이후 제출된 수정판
(#11 재작성)은 `control_lock`/공유 클라이언트를 아예 우회하는 방식으로
"진짜 동시 실행"은 달성했지만, 그 대가로 이 프로젝트에서 이미 여러 번
발생했던 "동일 기체에 대한 이중 제어 경로" 위험을 새로 만들어냈습니다
(알파는 `is_follower_locked` 보호 대상이 아니고, `/api/land` 등 일부
엔드포인트는 애초에 RTH 진행 여부를 확인하지 않음). `docs/13_work_order_rth_concurrency_fix.md`로
재작업 지시 → `following_worker()`처럼 공유 클라이언트를 유지하되 매
제어 틱마다 단일 락(읽기+계산+쓰기)만 잡는 방식으로 일원화하고,
`/api/land`·`/api/reset`이 RTH 중인 기체를 대상으로 호출되면
`rth_cancelled` 플래그로 RTH를 원자적으로 즉시 취소하는 안전장치까지
추가로 구현. ORCA 안전 반경도 1.5m→**1.6m**(결합 안전거리 3.2m)로
상향. **#16에서 최종 승인 완료.**

**작업 #1~#3(Following Mode, 편대 집결, RTH) 전부 승인 완료.**
`orca.py` 솔버가 다중 기체가 동시에 움직이는 3가지 주요 시나리오
전부에 적용되었고, 각각 실제 AirSim 환경에서 무충돌·안전 이격·목표
도달 정확도를 실측 검증받았습니다.

다음 작업이 시작되면 17번부터 이어서 번호를 매깁니다.

## 최초 조사자료 대비 미착수 항목 (참고용)

프로젝트 시작 시 조사했던 원본 자료(ORCA 알고리즘 + `simGetCollisionInfo`
+ 튜닝 프로세스 3단계)와 대조했을 때, 아직 작업지시서로 만들지 않은
항목들입니다. 우선순위는 다음 작업 착수 시점에 논의합니다.

- **정적 장애물 회피 미지원**: 지금 ORCA는 다른 드론끼리만 서로
  회피합니다 - 원본 자료의 "Case B(정적 장애물과 충돌)"에 해당하는
  나무/건물/지형은 전혀 고려하지 않습니다. Blocks 맵에서만 테스트했기
  때문에 드러나지 않았을 뿐, CityEnviron(빌딩)·LandscapeMountains(지형)·
  AbandonedPark(놀이기구)에서 Following Mode/편대 집결/RTH를 실제로
  쓰면 다른 드론은 피하면서 건물에는 그대로 박을 수 있습니다.
- **동역학 제한(Kinematics/Jerk Limiting) 미구현**: 원본 자료가 명시적으로
  요청했던 "ORCA가 계산한 이상적 속도가 모터의 가속도/저크 한계를
  넘지 않도록 후처리 Rate Limiter 추가"가 아직 없습니다. 시뮬레이션
  에서는 AirSim SimpleFlight가 어느 정도 매끄럽게 처리해주지만, 실제
  물리 드론으로 이식할 계획이 있다면 필요합니다.
- **안전 여유 및 다조건 강건성 검증 미흡 (부분 개선)**: RTH 재작업(#13~16)
  과정에서 `ORCA_AGENT_RADIUS_M`이 1.5m→1.6m(결합 안전거리 3.0m→3.2m)로
  상향되었고, 이 상수는 Following Mode/편대 집결도 공유하므로 세
  기능 전부 여유가 더 커졌습니다. 다만 **"다른 맵/풍속/통신 지연
  조건에서 반복 시험"은 여전히 안 됐습니다** - 지금까지 모든 실측
  검증이 Blocks 맵 한 곳에서만 이루어졌습니다.
