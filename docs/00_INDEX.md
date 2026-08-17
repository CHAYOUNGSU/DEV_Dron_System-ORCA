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
| 10 | `10_implementation_plan_rth_orca.md` | 작업계획서 (구현 착수 전) | Antigravity | 대기 |
| 11 | `11_completion_report_rth_orca.md` | 작업완료 보고서 (구현 후) | Antigravity | 대기 |
| 12 | `12_review_result_rth_orca.md` | 검수결과 (독립검수) | Codex | 대기 |

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

다음 작업이 시작되면 13번부터 이어서 번호를 매깁니다.
