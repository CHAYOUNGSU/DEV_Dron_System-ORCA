# 작업지시서 #13: RTH 동시성 수정 재작업 (control_lock 우회 문제 해결)

- 작성자: Claude (설계 / Plan 역할)
- 구현 담당: Antigravity
- 검수 담당: Codex (독립검수 / Review 역할)
- 선행 작업: `docs/09~12_*_rth_orca.md` (RTH ORCA - **승인 보류**, 이후 코드 수정됨 - 재검수 전)
- 대상 레포: `DEV_Dron_System-ORCA`

이 문서는 구현 담당자가 이전 문서(#09~#12)를 전부 읽었다는 것을 전제하지
않습니다. 필요한 배경은 이 문서에 포함되어 있습니다.

---

## 1. 배경

`docs/12_review_result_rth_orca.md`(Codex)에서 RTH 작업이 **승인 보류**
판정을 받았습니다. 이유: `_do_rth()`가 RTH 전체(3구간 비행 + 착륙, 실측
약 30초 이상)를 `control_lock`으로 통째로 점유하고 있어서, 두 기체를
"동시에" RTH시켜도 실제로는 완전히 직렬(순차) 실행되고 있었습니다.
작업지시서 #09의 핵심 요구사항("여러 기체가 동시에 RTH될 때 ORCA로
서로 충돌을 피한다")이 검증되지 못한 상태였습니다.

이후 `_do_rth()`가 수정되어 `docs/11_completion_report_rth_orca.md`가
재작성되었고, 실측으로 두 기체의 RTH 비행이 30.01초간 실제로 겹쳤음을
보고했습니다. **그런데 이 재검토(Claude)에서, 그 수정 방식 자체가 이
프로젝트에서 이미 여러 번 실제로 발생했던 종류의 버그를 다시 만들어낸
것을 확인했습니다.** 이 문서는 그 문제를 짚고, 올바른 해결 방향을
제시합니다. **이 개정판은 아직 Codex의 재검수를 받지 않았습니다** -
재검수 전에 먼저 이 작업지시서대로 수정해주세요.

## 2. 현재 코드의 문제 (직접 확인함)

`server.py`의 `_do_rth()`:

```python
def _do_rth(target_drone_id: str):
    with rth_lock:
        if target_drone_id in rth_in_progress:
            return False
        rth_in_progress.add(target_drone_id)
    try:
        # Create thread-isolated AirSim client to avoid msgpack socket buffer collision across threads
        ctrl = airsim.MultirotorClient(timeout_value=5)
        ctrl.confirmConnection()
        ...
        # (이후 3개 레그 + 착륙까지 이 독립 ctrl로 전부 진행, control_lock 전혀 사용 안 함)
```

**문제**: `control_lock`/`get_control_client()`(공유 `client_control`)를
전혀 쓰지 않고, RTH 호출마다 완전히 독립된 `airsim.MultirotorClient()`를
새로 만들어서 문제를 "우회"했습니다. 이러면 두 RTH가 실제로 동시에
진행되는 것은 맞지만, **이 프로젝트의 다른 모든 제어 경로(이착륙,
회전, 조이스틱, 리셋, 편대 집결, 전체 이착륙)는 여전히 공유
`client_control`을 씁니다.** RTH 대상 기체에게 그 사이에 다른 명령이
들어오면, 두 개의 서로 다른(그리고 서로 존재를 모르는) AirSim RPC
클라이언트가 **같은 기체에 동시에 명령을 내리는 경쟁 상태**가
발생합니다:

- `is_follower_locked()`가 이착륙(`/api/takeoff`)/회전(`/api/rotate`)/
  조이스틱(`/api/joystick`)을 막긴 하지만, 이 함수는
  `drone_id in FOLLOW_CHAIN`(브라보/찰리/델타)일 때만 작동합니다.
  **알파(Drone1)는 `FOLLOW_CHAIN`에 없으므로 이 보호를 전혀 받지
  못합니다** - 알파가 RTH 중이어도 알파에게 이착륙/회전/조이스틱
  명령을 그대로 보낼 수 있습니다.
- `/api/land`, `/api/reset`, `/api/formation/assemble`,
  `/api/fleet/takeoff`, `/api/fleet/land`는 **애초에 `is_follower_locked`나
  `rth_in_progress` 검사를 전혀 하지 않습니다.** 어떤 기체든 RTH 중에
  이 엔드포인트들이 호출되면 곧바로 이중 제어 경로 충돌이 발생합니다.

이건 이 세션에서 이미 세 번 실제로 만나서 고쳤던 것과 정확히 같은
버그 유형입니다(`client_telemetry`를 두 곳에서 동시에 만들던 문제,
텔레메트리 연결 경쟁 상태 등). RTH가 "진짜 동시 실행"이라는 목표를
달성한 건 맞지만, 대가로 안전장치를 통째로 우회해버린 것입니다.

## 3. 요구사항: 공유 클라이언트를 유지하면서 동시성 확보

**핵심 방향**: `client_control`/`control_lock`을 계속 쓰되, **RTH 호출
전체가 아니라 매 제어 틱(tick)마다만** 잠깐 락을 잡도록 바꾸세요. 이
패턴은 이미 `following_worker()`에 정확히 구현되어 있고 검증도
끝났습니다 - 그 함수를 참고하세요:

```python
# following_worker()의 패턴 (참고용, 그대로 복사하라는 뜻 아님)
while ...:
    with control_lock:
        ctrl = get_control_client()
        # 이번 틱에 필요한 모든 기체에 대해 상태 조회 + 속도 명령까지 전부 수행
        ...
    time.sleep(FOLLOW_TICK_INTERVAL_SEC)
```

`following_worker()`는 "한 번의 락 안에서 여러 기체를 순회"하는
방식이지만, RTH는 서로 다른 스레드에서 각자 자기 기체 하나만 다루므로
조금 다릅니다. RTH의 `run_rth_orca_leg()` 내부 while 루프를 다음과
같이 바꿔주세요:

```python
def run_rth_orca_leg(target_wpos, max_speed, max_vz, tol_3d, max_sec, leg_name):
    t_start = time.time()
    tick_dt = FOLLOW_TICK_INTERVAL_SEC
    while time.time() - t_start < max_sec:
        with control_lock:
            ctrl = get_control_client()
            s_curr = ctrl.getMultirotorState(vehicle_name=v_name)
            # ... 거리/선호속도/ORCA 계산 ...
            ensure_api_control(ctrl, v_name)
            ctrl.moveByVelocityAsync(safe_vx, safe_vy, safe_vz, tick_dt * 1.5, vehicle_name=v_name)
        time.sleep(tick_dt)  # 락 밖에서 대기 - 이 틈에 다른 RTH 스레드나 다른 API 요청이 락을 잡을 수 있음
```

이렇게 하면:
1. 두 개의 RTH 스레드(각각 다른 기체 대상)가 매 0.1초마다 번갈아
   락을 잡으며 진행하므로, **실질적으로 동시에 비행**합니다(완전히
   같은 나노초에 실행되는 건 아니지만, 각자 매 틱마다 확실히
   전진하므로 "동시 병렬 비행"이라는 실측 목표는 그대로 달성됩니다).
2. 다른 모든 제어 경로(이착륙/착륙/회전/조이스틱/리셋/편대집결/전체
   이착륙)와 **같은 공유 클라이언트, 같은 락**을 쓰므로, RTH 중인
   기체에 실수로 다른 명령이 들어와도 서로 순서를 기다리게 되지
   먼저 것처럼 경쟁하지 않습니다 - 이 프로젝트 전체에서 지켜온
   "동일 기체에 대한 이중 제어 경로 금지" 원칙이 RTH에도 그대로
   적용됩니다.
3. RTH 호출마다 새 `airsim.MultirotorClient()`를 만들 필요가 없어져서,
   연결 누수/소켓 경합 위험도 함께 사라집니다.

### 3.1 세부 변경 사항

1. `_do_rth()` 도입부의 `ctrl = airsim.MultirotorClient(timeout_value=5); ctrl.confirmConnection()`
   줄을 제거하세요. 대신 각 틱에서 `get_control_client()`로 공유
   클라이언트를 얻으세요 (이 함수는 이미 `server.py`에 존재하고
   ping 실패 시 자동 재연결도 처리합니다).
2. `ensure_api_control(ctrl, v_name)` 호출은 유지하되, `control_lock`
   블록 안에서 이루어지도록 하세요 (이미 위 예시에 반영됨).
3. `rth_lock = threading.Lock()`과 `rth_in_progress` 원자적 중복 방지
   로직은 **그대로 유지**하세요 - 이 부분은 Codex 검수(#12)에서도
   문제 삼지 않았고 여전히 유효합니다. 이건 `control_lock`과는
   별개의, "같은 기체에 대한 RTH 중복 호출 방지"용 락이라 둘을
   혼동하지 마세요.
4. `landAsync(vehicle_name=v_name).join()` 등 마지막 착륙 단계도
   `get_control_client()` + `control_lock`을 통해 이루어지도록
   맞춰주세요 (지금은 독립 `ctrl`로 되어 있음).
5. `airsim.MultirotorClient(timeout_value=5)`처럼 새로 만든 매직
   넘버 타임아웃은 제거됩니다 (공유 클라이언트를 쓰면 이미
   `WORKER_RPC_TIMEOUT_SEC` 상수로 통일되어 있는 타임아웃을 그대로
   물려받습니다 - 일관성 유지).

### 3.2 왜 `control_lock`을 프로젝트 전체에서 계속 공유해야 하는가

혹시 "RTH만이라도 독립 클라이언트를 쓰는 게 더 간단하지 않냐"는
생각이 들 수 있는데, 그렇지 않습니다 - 이 프로젝트는 지금 알파를
사람이 조이스틱으로 직접 조종하면서 Following Mode로 나머지가
자동비행하는 상황, 편대 집결 중에 개별 명령이 들어오는 상황 등
"이 기체가 지금 정확히 어떤 제어 하에 있는가"를 하나의 공유 자원
(client_control)과 하나의 락으로 통제하는 것이 안전성의 핵심
전제입니다. RTH만 예외로 두면 그 전제가 깨집니다.

## 4. 비기능 요구사항

- `following_worker()`, `_do_formation_assemble()`의 기존 동작에
  영향이 없어야 합니다 (둘 다 여전히 `control_lock`을 쓰므로, RTH가
  틱 단위로 짧게 락을 쥐고 놓는 방식으로 바뀌면 이들과도 자연스럽게
  공존합니다 - 서로 매 0.1초 안팎으로 번갈아 실행될 뿐입니다).
- 매 틱마다 락을 획득/해제하는 오버헤드는 무시할 수준입니다 (기존
  `following_worker`가 이미 같은 빈도로 하고 있음).

## 5. 검증 및 완료 조건

`docs/09_work_order_rth_orca.md`의 6절 기준을 그대로 따르되, 이번엔
**동시성이 진짜인지와 이중 제어 경로가 없는지**를 반드시 추가로
확인하세요:

1. `test_orca_rth.py`를 그대로(또는 필요시 보강해서) 재실행 - 이전과
   같은 "비행 중첩 시간 ≥ 5.0초", 무충돌, 최소 이격 3.0m 이상, 홈
   착륙 오차 1.5m 이하 기준을 이번에도 통과해야 합니다.
2. **신규**: 한 기체가 RTH 중일 때 **알파를 대상으로** 다른 명령
   (예: `/api/land` 또는 `/api/takeoff`)을 보내도 오류 없이 정상적으로
   순서대로 처리되는지(즉시 경쟁 상태로 죽거나 두 명령이 동시에
   같은 기체를 건드리지 않는지) 확인하는 간단한 테스트나 수동 검증을
   추가하고, 완료 보고서에 결과를 남겨주세요.
3. `test_orca_formation_assemble.py`, `test_following_mode.py`,
   `test_ui_playwright.py` 전체 회귀 통과.
4. 테스트 전후 시뮬레이터/프로세스 정리 확인.

## 6. 산출물

- `server.py` (수정 - `_do_rth()`를 공유 클라이언트 + 틱 단위 락으로
  재작성)
- `docs/14_completion_report_rth_concurrency_fix.md` (작업완료 보고서)
- `docs/15_review_result_rth_concurrency_fix.md` (Codex 검수결과 -
  이번엔 "진짜 동시성"과 "이중 제어 경로 부재"를 둘 다 확인해야
  승인)
- `docs/00_INDEX.md` 갱신
