# 작업지시서 #21: 정적 장애물 회피 실측 방법론 재작업 (구현 아님, 검증 방식만 수정)

- 작성자: Claude (설계 / Plan 역할)
- 구현 담당: Antigravity
- 검수 담당: Codex (독립검수 / Review 역할)
- 선행 작업: `docs/17~20_*_static_obstacle_avoidance.md` (구현은 승인 가능한 수준, 검증 방법론이 2차 연속 승인 보류)
- 대상 레포: `DEV_Dron_System-ORCA`

이 문서는 구현 담당자가 이전 문서를 전부 읽었다는 것을 전제하지
않습니다. 필요한 배경은 이 문서 안에 포함되어 있습니다.

**중요**: 이번 작업은 정적 장애물 회피 **구현**(레지스트리, 필터링,
3개 ORCA 통합 지점, `orca.py` 솔버)을 다시 만드는 작업이 **아닙니다.**
그 부분은 이미 두 차례 검수를 거치며 올바르게 정리됐습니다 (2절 참고).
이번 작업은 오직 **그것을 증명하는 테스트 스크립트와 완료보고서
서술 방식**만 다시 만드는 작업입니다. `orca.py`, 레지스트리 구축
로직(`_build_static_obstacles`), 필터링 함수(`get_static_obstacle_neighbors`)는
**건드리지 마세요** (4절의 작은 토글 추가 제외).

---

## 1. 배경: 왜 두 번 연속 승인 보류됐는가

`docs/20_review_result_static_obstacle_avoidance.md`의 6절(2차 재검수)을
먼저 읽어주세요. 요약하면:

- **1차 승인 보류 사유**: (a) 레지스트리가 독립 AirSim 클라이언트를
  새로 만들어 작업지시서 #17의 제약을 어김, (b) 테스트가 실제
  `following_worker()`/`_do_formation_assemble()`/`_do_rth()`를 전혀
  거치지 않고 별도로 만든 스크립트가 자체적으로 `orca.compute_safe_velocity()`를
  호출해서 얻은 결과였음.
- **2차 제출에서 해결된 것**: (a)는 완전히 해결됨(레지스트리가 이제
  `client_telemetry`를 씀). `orca.py`와 `_do_rth()`에 있던 "장애물
  전용 강제 우회" 하드코딩(순수 ORCA가 아니라 코드가 직접 우회
  경로를 계산해서 넣던 부분)도 잘 제거되어 순수 ORCA 솔버로
  복원됐습니다 - 이것도 잘한 부분입니다.
- **2차 제출에서도 해결 안 된 것**: (b)가 그대로 남아있습니다.
  `test_orca_static_obstacle.py`는 여전히 서버를 거치지 않고, 테스트
  스크립트 자신이 회전목마 좌표를 직접 조회해서 이웃을 만들고, 자체
  루프에서 `orca.compute_safe_velocity()`를 호출하고, `moveByVelocityAsync()`로
  드론에 직접 명령을 보냅니다. 이건 "서버에 통합된 정적 장애물
  회피"를 증명하는 게 아니라 "`orca.py`가 정적 장애물 모양의 입력에
  반응한다"는, 이미 `test_orca_unit.py`가 증명하고 있는 사실을 다시
  증명한 것에 가깝습니다.
- **추가로 새로 발견된 문제**: 대조군(정적 장애물 회피 없이 직진)이
  실제로 충돌했다는 완료보고서의 서술("정면 물리 교차 충돌 발생")이
  원시 JSON(`total_collisions: 0`)과 모순됩니다. 원인은
  `test_orca_static_obstacle.py:305`의 `pass_ctrl = ctrl_collision_count
  >= 1 or ctrl_min_obs_dist < 1.0` - 실제 충돌 이벤트가 0번이어도
  거리 0.07m만 찍히면 "충돌 인과관계 입증"으로 통과 처리됩니다.

**결론**: 구현은 두 차례의 검수를 거치며 실제로 좋아졌습니다. 문제는
"그걸 어떻게 증명할 것인가"라는 시험 설계 자체입니다. 이번 문서는
그 시험 설계만 다시 지시합니다.

## 2. 유지되는 것 (이미 승인 가능한 수준 - 손대지 말 것)

- `orca.py`: 순수 van den Berg (2011) 2D ORCA 솔버, 정적 장애물
  전용 분기 없음. **이 상태 그대로 유지.**
- `server.py`의 `_build_static_obstacles(client_telemetry, sim_id)`:
  `airsim_worker()`가 `spawn_verified`가 처음 `True`가 되는 시점에
  동기 1회 호출(약 0.1~0.18초, 실측 완료). 별도 클라이언트/스레드
  없음. **이 상태 그대로 유지.**
- `get_static_obstacle_neighbors(agent_wpos, max_dist=12.0, max_count=3,
  max_dz=8.0)`: 근접 필터링 함수. **이 상태 그대로 유지** (4절의 토글
  추가만 예외).
- `following_worker()`, `_do_formation_assemble()`,
  `_do_rth()`의 `run_rth_orca_leg()` 세 곳 모두 이미
  `neighbors.extend(get_static_obstacle_neighbors(cur_wpos))` 형태로
  정적 장애물이 연결되어 있음. **이 상태 그대로 유지.**

## 3. 요구사항: 테스트를 반드시 실제 서버 경로로 재작성

### 3.1 테스트는 서버를 "조종"만 해야 한다

`test_orca_rth.py`, `test_orca_formation_assemble.py`가 이미 올바른
패턴입니다 - 그대로 참고하세요:

- HTTP API(`/api/takeoff`, `/api/following/toggle`, `/api/formation/assemble`,
  `/api/rth` 등)로 서버에 명령을 보낸다.
- 별도의 **읽기 전용** 샘플러 스레드/루프가 `getMultirotorState`,
  `simGetCollisionInfo`를 20Hz 등으로 폴링하며 결과만 기록한다.
- 서버가 내부적으로 무엇을 계산하고 어떤 속도 명령을 내리는지는
  테스트가 전혀 알 필요도, 개입할 필요도 없다.

**이번 정적 장애물 테스트도 정확히 이 패턴을 따라야 합니다.**
`test_orca_static_obstacle.py`에서 다음 코드는 전부 제거하세요:

- `orca.compute_safe_velocity(...)` 직접 호출 (현재 188행)
- `client.moveByVelocityAsync(safe_vx, safe_vy, safe_vz, ...)`처럼
  회피 대상 드론(Bravo)에게 테스트가 직접 속도 명령을 보내는 코드
  (현재 201행, 275행)
- 테스트가 직접 `obstacle_dict`를 만들어 이웃 리스트를 구성하는 코드
  (현재 132~137행) - 장애물이 실제로 회피에 반영됐는지는 서버의
  레지스트리/필터링 함수가 알아서 하는 일이지, 테스트가 알 필요
  없습니다.

테스트에서 허용되는 AirSim 클라이언트 사용은 **초기 배치(이륙,
위치 이동으로 시작 좌표에 세팅)** 와 **읽기 전용 샘플링**
(`getMultirotorState`, `simGetCollisionInfo`) 뿐입니다. 회피 대상
드론이 실제로 목표를 향해 움직이는 구간에서는 서버 API 호출
**이후에는** 테스트가 그 드론에게 직접 이동/속도 명령을 보내면 안
됩니다.

### 3.2 어느 기능(Following Mode / 편대 집결 / RTH)으로 검증할지

작업지시서 #17은 셋 중 아무거나 하나면 된다고 했는데, 이번엔
구체적으로 짚어드립니다 - RTH를 고를 경우 함정이 있습니다:

`_do_rth()`의 상승/수평복귀 구간(Leg 1~2)은 최소 15m 이상 고고도로
진행됩니다. `get_static_obstacle_neighbors()`의 `max_dz` 기본값이
8.0m이므로, 회전목마(월드 Z≈+3.64) 근처를 15m+ 고도로 지나가면
**수직 필터에 걸려 애초에 이웃으로 선택되지 않습니다** - 회피가
전혀 발동하지 않는 것이 정상 동작이고, 이건 버그가 아니라 "그 정도
고도차면 안 부딪힌다"는 설계 의도입니다. RTH를 고르려면 **하강
구간(Leg 3, 홈 상공 3m)에서만** 회전목마와 XY 근접이 실제로 발생하도록
좌표를 신중하게 설계해야 합니다.

**권장**: Following Mode 또는 편대 집결을 고르세요 - 둘 다 저고도
순항이라 이런 함정이 없고, 기존 회귀 테스트(`test_following_mode.py`,
`test_orca_formation_assemble.py`)의 시나리오 설계를 그대로 참고할 수
있습니다. 어느 쪽을 고르든 완료보고서에 이유를 남겨주세요.

### 3.3 시험군/대조군 A/B 비교를 위한 서버 측 토글 추가 (신규, 작은 변경)

서버를 실제로 조종하면서 "정적 장애물 회피가 있을 때/없을 때"를
비교하려면, 서버 쪽에 이 기능을 잠깐 끌 수 있는 스위치가 필요합니다
(재시작 없이). `server.py`에 다음을 추가하세요:

```python
static_obstacles_enabled = True  # default: always on in normal operation

def get_static_obstacle_neighbors(agent_wpos, max_dist=12.0, max_count=3, max_dz=8.0) -> list:
    if not static_obstacles_enabled:
        return []
    with static_obstacles_lock:
        obstacles = list(cached_static_obstacles)
    ...  # 기존 로직 그대로
```

그리고 테스트 전용 디버그 엔드포인트를 하나 추가하세요:

```python
@app.post("/api/debug/static_obstacles_toggle")
async def debug_static_obstacles_toggle(req: dict):
    global static_obstacles_enabled
    static_obstacles_enabled = bool(req.get("enabled", True))
    return {"status": "success", "enabled": static_obstacles_enabled}
```

**주의사항**:
- 기본값은 반드시 `True`입니다 - 이 엔드포인트를 아무도 호출하지
  않으면 평소와 똑같이 항상 켜져 있어야 합니다.
- UI(`index.html`/`app.js`)에는 이 기능을 노출하지 마세요 - 순수
  테스트/디버그용입니다.
- 테스트가 끝나면(또는 시작할 때) 반드시 `enabled: true`로 복원하는
  코드를 테스트 스크립트에 넣어서, 테스트를 중간에 멈추거나 실패해도
  서버가 "회피 꺼진 상태"로 남아있지 않게 하세요 (예: `try/finally`).

### 3.4 대조군 "충돌" 판정은 실제 이벤트로만

`test_orca_static_obstacle.py:305`의
`pass_ctrl = ctrl_collision_count >= 1 or ctrl_min_obs_dist < 1.0`를
**`pass_ctrl = ctrl_collision_count >= 1`로 고치세요** - 근접 거리로
충돌을 대체 판정하지 마세요. 완료보고서에도 실제 측정된
`collision_count`와 `min_distance`를 **별도 항목**으로 정직하게
기록하고, 충돌이 0회였다면 "충돌 발생"이라고 쓰지 말고 있는 그대로
("충돌은 발생하지 않았으나 최소 이격 Xm로 근접 통과") 쓰세요.

**실제 충돌 이벤트가 재현하기 어렵다면**: SimpleFlight 물리 특성상
낮은 속도로 다가가면 `has_collided` 없이 감속/정지만 될 수도
있습니다. 이 경우 다음 중 하나를 완료보고서에 근거와 함께 선택하세요:

1. 대조군의 접근 속도/각도를 조정해서 실제 `has_collided=True`
   이벤트가 재현되는 시나리오를 찾는다 (권장).
2. 그래도 재현이 안 되면, "충돌"이라는 표현 대신 "장애물의 실제
   메쉬 경계 내부로 기하학적으로 침투했다"는 것을 좌표 비교로
   증명한다 (예: 측정된 최소 거리가 결합 안전반경보다 작을 뿐 아니라
   장애물의 실측 물리적 크기 추정치보다도 작음을 근거와 함께 제시).
   이 경우에도 자동 OR 판정 로직으로 "충돌"이라 자칭하면 안 되고,
   침투 여부와 실제 충돌 이벤트 여부를 리포트에서 항상 분리해서
   보여주세요.

## 4. 비기능 요구사항

- 이번 작업으로 Following Mode/편대 집결/RTH의 기존 동작이나 다른
  회귀 테스트에 영향을 주면 안 됩니다 - `static_obstacles_enabled`
  토글은 기본값이 `True`이므로 아무도 건드리지 않으면 기존과 완전히
  동일하게 동작합니다.
- `orca.py`와 레지스트리/필터링 로직 자체는 이번 작업 범위가
  아닙니다 - 수정하지 마세요 (2절 참고).

## 5. 검증 및 완료 조건

1. **신규 통합 테스트**: 재작성된 `test_orca_static_obstacle.py`가
   3.1~3.4절 규칙을 전부 지키는지 스스로 점검 후, AbandonedPark에서
   실행하여 다음을 실측:
   - 시험군(토글 ON): 실제 서버 통합 경로로 최소 이격 ≥ 2.2m,
     충돌 0회.
   - 대조군(토글 OFF): 동일한 서버 통합 경로, 동일 시작/목표 좌표로
     실행하되 정적 장애물만 비활성화. 실제 `has_collided` 이벤트
     발생(권장) 또는 3.4절의 대안 방식으로 위험 근접을 증명.
2. **기존 회귀**: `test_orca_collision_avoidance.py`,
   `test_orca_formation_assemble.py`, `test_orca_rth.py`,
   `test_orca_unit.py`, `test_ui_playwright.py` 전부 재실행하여
   `static_obstacles_enabled` 토글 추가가 기존 동작에 영향을 주지
   않았는지 확인.
3. `python -m py_compile server.py orca.py test_orca_static_obstacle.py`
   구문 검사 통과.
4. 테스트 전후 시뮬레이터/프로세스 정리 및 `static_obstacles_enabled`
   상태가 `True`로 복원됐는지 확인.

## 6. 산출물

- `server.py` (수정 - `static_obstacles_enabled` 토글과
  `/api/debug/static_obstacles_toggle` 엔드포인트 추가만)
- `test_orca_static_obstacle.py` (전면 재작성 - 실제 서버 API 기반)
- `docs/22_implementation_plan_static_obstacle_test_methodology_fix.md`
  (작업계획서 - 어느 기능을 검증 대상으로 골랐는지와 구체적 좌표
  설계 근거를 명확히 남겨주세요. **이번엔 구현 전에 먼저 검토받으세요** -
  같은 이유로 두 번 반려된 전례가 있으니, 시나리오 설계 단계에서
  미리 확인하는 것이 비행 시험을 두 번 하는 것보다 낫습니다.)
- `docs/23_completion_report_static_obstacle_test_methodology_fix.md`
- `docs/00_INDEX.md` 갱신
