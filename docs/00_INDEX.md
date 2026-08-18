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
| 15 | `15_completion_report_rth_concurrency_fix.md` | 작업완료 보고서 (구현 후) | Antigravity | 완료 (최종 승인) |
| 16 | `16_review_result_rth_concurrency_fix.md` | 검수결과 (독립검수) | Codex | 승인 완료 |
| 17 | `17_work_order_static_obstacle_avoidance.md` | 작업지시서 (설계) | Claude | 완료 |
| 18 | `18_implementation_plan_static_obstacle_avoidance.md` | 작업계획서 (구현 착수 전) | Antigravity | 완료 (개정판, 승인) |
| 19 | `19_completion_report_static_obstacle_avoidance.md` | 작업완료 보고서 (구현 후) | Antigravity | 완료 (개정판, 구현 자체는 유효) |
| 20 | `20_review_result_static_obstacle_avoidance.md` | 검수결과 (독립검수) | Codex | 승인 보류 (1차·2차 모두 - 검증 방법론 문제) |
| 21 | `21_work_order_static_obstacle_test_methodology_fix.md` | 작업지시서 (재작업 - 설계) | Claude | 완료 |
| 22 | `22_implementation_plan_static_obstacle_test_methodology_fix.md` | 작업계획서 (구현 착수 전) | Antigravity | 완료 (승인 획득) |
| 23 | `23_completion_report_static_obstacle_test_methodology_fix.md` | 작업완료 보고서 (구현 후) | Antigravity | 완료 (경로 설계 결함 발견) |
| 25 | `25_work_order_static_obstacle_test_hazard_geometry_fix.md` | 작업지시서 (3차 재작업 - 설계) | Claude | 완료 |
| 26 | `26_completion_report_static_obstacle_test_hazard_geometry_fix.md` | 작업완료 보고서 (구현 후) | Antigravity | 완료 (검수 요청) |
| 27 | `27_review_result_static_obstacle_test_hazard_geometry_fix.md` | 검수결과 (독립검수) | Codex | 대기 |

다음 작업이 시작되면 28번부터 이어서 번호를 매깁니다.

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
작업입니다. 상세: `docs/09_work_order_rth_orca.md` 및 `docs/13_work_order_rth_concurrency_fix.md`.
**승인 완료 (#16).**

## 작업 #4: 정적 장애물(건물/지형/구조물) 회피 적용

사용자 요청("정적 장애물 회피 기능을 구현해 보고 싶어..")에 따라
착수. "최초 조사자료 대비 미착수 항목"의 첫 번째 공백을 메우는
작업입니다. 세 기존 ORCA 통합 지점(Following Mode/편대 집결/RTH)이
공유하는 `orca.py`의 `neighbors` 포맷(`{"pos","vel","radius","weight"}`)이
드론인지 아닌지 구분하지 않는다는 점을 이용해서, `simListSceneObjects()`
+ `simGetObjectPose()`로 연결(맵 전환) 시 1회 정적 장애물 후보를
캐시하고 `vel=(0,0,0)`, `weight=1.0`(Following Mode의 알파와 동일한
비상호적 취급)인 이웃으로 세 곳 전부에 편입시키는 "신 시점 정적
레지스트리" 방식을 지시. LiDAR/거리 센서는 머신 전역 `settings.json`에
설정이 전혀 없어 이번 범위에서 제외. 상세: `docs/17_work_order_static_obstacle_avoidance.md`.

작업계획서(#18) 1차 제출본은 레지스트리 구축 시 어떤 클라이언트/락을
쓰는지 불명확해 반려(보완 요청) - CityEnviron처럼 오브젝트가 수천 개인
맵에서 `simGetObjectPose()`를 순차 호출하면 수 초~수십 초가 걸릴 수
있는데, 이걸 `control_lock`으로 감싸거나 `airsim_worker()` 메인
루프에서 동기 실행하면 RTH 동시성 버그(#12)나 맵 전환 카메라 멈춤
버그와 같은 유형의 "장기 블로킹" 문제가 재발할 위험이 있었기 때문.
개정판은 전용 백그라운드 스레드 + 독립 읽기 전용 소켓 +
`static_obstacles_lock`을 통한 원자적 캐시 교체로 `control_lock`과
텔레메트리 루프 어느 쪽도 블로킹하지 않도록 재설계했고, CityEnviron급
대형 건물에는 "단일 좌표점 + 고정 반경" 모델의 한계가 있음을 명시적으로
문서화함. **개정판 승인 완료.**

**구현(#19) 자체는 두 차례 검수(#20)를 거치며 실제로 개선됨** - 1차
지적사항(레지스트리 독립 클라이언트 사용, 테스트가 실제 통합 경로를
안 거침)에 대해 2차 제출에서 레지스트리 클라이언트 문제와 `orca.py`/
`_do_rth()`에 남아있던 수동 우회 하드코딩은 제대로 제거됐음. 다만
**"테스트가 실제 서버 통합 경로를 거치지 않는다"는 1차 지적이 2차
제출에서도 그대로 남아있었고**, 여기에 더해 대조군 충돌 여부를 실제
`simGetCollisionInfo` 이벤트가 아니라 근접 거리(OR 조건)로 대체
판정해서 완료보고서 서술("정면 물리 충돌 발생")이 원시 데이터
(`total_collisions: 0`)와 모순되는 문제가 새로 발견되어 **2차 연속
승인 보류**. 구현 자체를 다시 만드는 게 아니라 **검증 방법론만**
다시 지시하는 재작업지시서 `docs/21_work_order_static_obstacle_test_methodology_fix.md`
발행 - 테스트를 실제 HTTP API로 서버를 조종하고 읽기 전용으로만
샘플링하는 방식(기존 `test_orca_rth.py` 패턴과 동일)으로 전면
재작성하고, 시험군/대조군 A/B 비교용 서버 측 토글
(`static_obstacles_enabled`, 기본값 `True`)을 추가하도록 지시함.

**작업계획서(#22) 조건부 승인 후 제출된 완료보고서(#23)도 3차 승인
보류(#24)** - 검증 아키텍처(서버 API로만 조종, `orca.py` 직접 호출/
Bravo 직접 제어 없음, 토글 정상 동작) 자체는 이번엔 정확히
구현됐지만, 실제 비행 경로가 장애물을 피해 X=+5.5m 통로로 우회하도록
짜여 있어서 대조군조차 결합 안전반경(1.6m+2.2m=3.8m) 안에 들어간
적이 없었음(실측 최소 거리 5.00m) - 즉 "위험한 상황을 실제로
만들었는지"를 증명하지 못했고, 판정식도 이 사실을 가릴 수 있는
OR 조건(근접 거리 차이 또는 횡방향 편차 차이만으로 합격)을 쓰고
있었음. 이 두 문제 다 이전 문서(#21 3.4절, #22 조건부 승인 시
재확인)에서 이미 명시적으로 금지했던 것과 같은 유형이라, 이번엔
재해석 여지를 없애기 위해 경로 코드와 판정식 코드를 거의 그대로
제공하는 `docs/25_work_order_static_obstacle_test_hazard_geometry_fix.md`를
발행 - Alpha의 우회 통로 로직을 완전히 삭제하고 X=0.0 직진으로
고정(장애물이 그 경로상에 있다는 것 자체가 위험 요소), 판정식에서
OR로 묶인 대체 증거(횡방향 편차 차이 등)를 전부 제거하고 "충돌 또는
3.8m 미만 근접" 둘 중 하나만 대조군의 위험성 증거로 인정하도록 지시.

## 최초 조사자료 대비 미착수 항목 (참고용)

프로젝트 시작 시 조사했던 원본 자료(ORCA 알고리즘 + `simGetCollisionInfo`
+ 튜닝 프로세스 3단계)와 대조했을 때, 아직 작업지시서로 만들지 않은
항목들입니다. 우선순위는 다음 작업 착수 시점에 논의합니다.

- **정적 장애물 회피 미지원**: 작업 #4로 착수함 (`docs/17_work_order_static_obstacle_avoidance.md`,
  구현/검수 대기 중). 지금 ORCA는 다른 드론끼리만 서로 회피합니다 -
  원본 자료의 "Case B(정적 장애물과 충돌)"에 해당하는 나무/건물/지형은
  전혀 고려하지 않습니다. Blocks 맵에서만 테스트했기 때문에 드러나지
  않았을 뿐, CityEnviron(빌딩)·LandscapeMountains(지형)·AbandonedPark
  (놀이기구)에서 Following Mode/편대 집결/RTH를 실제로 쓰면 다른
  드론은 피하면서 건물에는 그대로 박을 수 있습니다.
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
