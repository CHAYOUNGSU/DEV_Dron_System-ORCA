# DEV_Dron_System - ORCA Collision Avoidance

AirSim 기반 4대 편대 드론 관제 시스템(`DEV_Dron_System`)에 ORCA(Optimal
Reciprocal Collision Avoidance) 충돌 회피 알고리즘을 추가하기 위한 독립
작업 레포지토리입니다.

이 코드는 [메인 레포](../DEV_Dron_System)의 특정 시점 스냅샷입니다. `assets/`
(AirSim 시뮬레이터 실행 파일, 수 GB)와 `venv/`는 포함되어 있지 않습니다 -
실행하려면 AirSim 시뮬레이터 환경을 별도로 준비해야 합니다 (`server.py`의
`SIMULATORS` 딕셔너리 참고, `Blocks` 맵이 가장 가볍고 빠릅니다).

## 작업 방식: 설계 -> 구현 -> 독립검수

이 레포의 작업은 서로 다른 AI 에이전트가 역할을 나누어 순차적으로 진행합니다.
각 단계는 자신의 산출물을 `docs/`에 문서로 남기고, 다음 단계로 넘깁니다.

| 단계 | 역할 | 도구 | 산출물 |
|---|---|---|---|
| 설계 (Plan) | 작업 범위와 요구사항을 명세 | Claude | `NN_work_order_*.md` (작업지시서) |
| 구현 (Implement) | 명세를 바탕으로 실제 코드 작성 | Antigravity | `NN_implementation_plan_*.md` (작업계획서), `NN_completion_report_*.md` (작업완료 보고서) |
| 검수 (Review) | 구현 결과를 명세와 대조해 독립적으로 검증 | Codex | `NN_review_result_*.md` (검수결과) |

파일명은 `docs/00_INDEX.md`에 등록된 순번을 따릅니다. 각 에이전트는 자신의
산출물만 작성하고, 앞 단계의 문서는 읽기 전용 참고자료로 취급합니다.

## 현재 진행 상황

- [x] `docs/01_work_order_following_mode_orca.md` - Following Mode ORCA 적용 작업지시서 (설계 완료)
- [ ] Antigravity 구현 (작업계획서 + 작업완료 보고서 대기)
- [ ] Codex 독립검수 (검수결과 대기)

## 로컬 실행

```
pip install -r requirements.txt
python server.py
```

`public/index.html`을 통해 `http://127.0.0.1:8000`에서 UI 접속. 실제 비행
검증에는 AirSim 시뮬레이터 실행 파일이 필요합니다 (데모 모드는 물리 연산이
없어 충돌 회피 검증에 사용할 수 없습니다).
