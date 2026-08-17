# 작업계획서 #10: RTH(자동 복귀) 좌표계 수정 및 ORCA 충돌 회피 적용 계획

- 작성자: Antigravity (구현 담당)
- 대상 작업지시서: `docs/09_work_order_rth_orca.md` (작성: Claude)
- 검수 담당: Codex (독립검수 / Review 역할)
- 선행 작업: `docs/01~04_*_following_mode_orca.md`, `docs/05~08_*_formation_assemble_orca.md` (둘 다 승인 완료)
- 대상 레포: `DEV_Dron_System-ORCA`

---

## 1. 개요 및 목적

### 1.1 배경 및 문제점
1. **좌표계 버그**:
   - 기존 `_do_rth()`는 AirSim의 `moveToPositionAsync()`에 `DRONES_CONFIG[target_drone_id]["spawn_offset"]`를 목표 좌표로 직접 전달했습니다.
   - AirSim의 기체별 위치 제어 명령은 기체 고유의 로컬 좌표계(스폰 지점 원점)를 기준으로 동작하므로, 브라보(오프셋 Y=3.5m), 찰리(Y=7.0m), 델타(Y=10.5m)는 각자의 홈 원점에서 오프셋 거리만큼 벗어난 엉뚱한 위치에 착륙하는 치명적 오차가 발생합니다.
2. **충돌 회피 부재**:
   - RTH 3단계 이동(15m 상승 $\rightarrow$ 홈 좌표 수평 복귀 $\rightarrow$ 3m 하강 $\rightarrow$ 착륙) 중 타 기체의 위치/속도를 고려하지 않아, 다중 기체 동시 RTH 또는 Following Mode/편대 정지 기체와의 교차 경로에서 충돌 위험이 존재합니다.

### 1.2 목적
1. **홈 복귀 좌표계 정합성 확립**:
   - "홈(Home)"의 정의를 **"각 기체 자신의 고유 스폰 지점(Own Spawn Point)"**으로 확정합니다.
   - 월드 좌표계 기준 $World\_Home_i = Spawn\_Offset_i$, 로컬 좌표계 기준 $Local\_Home_i = (0.0, 0.0)$으로 정합성을 완벽히 수정합니다.
2. **ORCA 기반 3단계 RTH 속도 제어 루프 전환**:
   - 상승(Climb), 수평 복귀(Horizontal Cruise), 하강(Descent) 3단계 이동을 `orca.compute_safe_velocity()` 기반 10Hz 유한 속도 제어 루프로 구현하여 완전한 충돌 회피를 보장합니다.
3. **독립 실환경 검증 (`test_orca_rth.py`)**:
   - 다중 기체 교차 RTH 시나리오에서 **샘플링 에러 0건, 무충돌(0회), 최소 안전 이격($\ge 3.0m$), 전 기체 홈 착륙 오차($\le 1.5m$)**를 정밀 실측 검증합니다.

---

## 2. 주요 설계 및 아키텍처

```mermaid
flowchart TD
    A[RTH API Call (/api/rth)] --> B{Following Mode 중?}
    B -- Yes --> C[에러 응답 거절 (is_follower_locked)]
    B -- No --> D{해당 기체 RTH 진행 중?}
    D -- Yes --> E[중복 실행 방지 (Ignored/Pass)]
    D -- No --> F[rth_in_progress.add(target_drone_id)]
    
    F --> G[Leg 1: Safe Climb 15m (ORCA 10Hz Loop)]
    G --> H[Leg 2: Horizontal Return to Own Home (ORCA 10Hz Loop)]
    H --> I[Leg 3: Descent to Altitude 3m (ORCA 10Hz Loop)]
    I --> J[Final: Precision Landing (landAsync)]
    J --> K[rth_in_progress.remove(target_drone_id)]
```

### 2.1 홈(Home) 좌표계 해석 및 근거
- **해석**: RTH의 "홈"은 **각 기체 고유의 스폰 지점(Own Spawn Origin)**으로 정의합니다.
  - 전 기체가 맵 원점(알파 자리)으로 모이게 되면 착륙 패드가 겹쳐 최종 착륙 단계에서 물리적 충돌이 불가피합니다.
  - 따라서 각 기체 $i$는 월드 기준 $Spawn\_Offset_i$ (즉 로컬 기준 $(0, 0)$) 지점으로 안전하게 개별 복귀해야 합니다.
- **좌표 변환**:
  - 기체 $i$의 월드 현재 위치: $P_{world} = P_{local} + Spawn\_Offset_i$
  - 기체 $i$의 월드 홈 목표: $Home_{world} = Spawn\_Offset_i$
  - 데모 모드 폴백: `latest_telemetries[t_id]["x"] = home_x`, `["y"] = home_y` (월드 좌표계이므로 일관성 유지).

### 2.2 RTH 3단계 ORCA 속도 제어 루프 설계
1. **Leg 1 (상승)**:
   - 목표 월드 좌표: $(cur\_wx, cur\_wy, \min(cur\_wz - 15.0, -15.0))$
   - 수평 위치는 현재 위치를 유지하며 $ORCA\_MAX\_VZ\_MPS=2.0m/s$로 안전 상승.
   - 수렴 조건: 목표 고도 오차 $\le 0.8m$ (타임아웃 10초).
2. **Leg 2 (수평 복귀)**:
   - 목표 월드 좌표: $(home\_wx, home\_wy, safe\_climb\_z)$
   - $safe\_climb\_z$를 유지하며 홈 수평 좌표로 순항 (거리 비례 감속, 최대 속도 $3.5m/s$).
   - 수렴 조건: 수평 거리 오차 $\le 0.8m$ (타임아웃 20초).
3. **Leg 3 (하강)**:
   - 목표 월드 좌표: $(home\_wx, home\_wy, -3.0)$
   - 홈 상공에서 $ORCA\_MAX\_VZ\_MPS=1.5m/s$로 3m 고도까지 서서히 감속 하강.
   - 수렴 조건: 3D 거리 오차 $\le 0.8m$ (타임아웃 10초).
4. **Final (착륙)**:
   - `ctrl.landAsync(vehicle_name=v_name).join()` 호출 후 disarm/API control 해제.

### 2.3 이웃(`neighbors`) 및 ORCA 파라미터
- 타 3대 기체 전부를 `neighbors`로 등록:
  - `weight = 0.5` (상호 회피 균등 분배)
  - `radius = 1.5m` ($2 \times r = 3.0m$ 안전 분리 거리)
  - `time_horizon = 2.0s`, `tick_dt = 0.1s`

### 2.4 동시 RTH 및 상호작용 처리
- `rth_in_progress = set()` 전역 집합으로 관리.
- 특정 기체가 이미 RTH 수행 중일 때 추가 요청이 오면 중복 기동을 방지.
- 서로 다른 기체들의 동시 RTH는 자유롭게 병렬 실행 허용 (ORCA가 실시간 상호 회피 수행).

---

## 3. 구현 단계별 세부 계획

### Step 1: `server.py` 백엔드 수정
- [ ] `rth_in_progress = set()` 전역 집합 선언.
- [ ] `_do_rth(target_drone_id)` 리팩토링:
  - 기체별 `spawn_offset` 기반 월드 홈 좌표 $(home\_wx, home\_wy)$ 산출.
  - Leg 1(상승), Leg 2(수평 복귀), Leg 3(하강) 3단계 10Hz ORCA 속도 제어 루프 적용.
  - 종료 후 `ctrl.landAsync()` 호출 및 `rth_in_progress` 정리.
- [ ] `/api/rth` 엔드포인트에 `is_follower_locked` 검사 및 중복 실행 방어 추가.

### Step 2: 신규 테스트 스크립트 작성 (`test_orca_rth.py`)
- [ ] 4대 동시 이륙 후 Bravo(Drone2)와 Charlie(Drone3)를 상호 교차 위치로 사전 전진 이동:
  - Bravo: $(X=+20m, Y=+10m)$ $\rightarrow$ 복귀 경로가 Charlie의 복귀 경로와 정면/대각 교차.
  - Charlie: $(X=+20m, Y=-5m)$
- [ ] Bravo와 Charlie에 대해 동시 RTH 호출.
- [ ] 20Hz 독립 샘플러를 통해 충돌 횟수, 기체 간 최소 이격 거리, 각 기체의 홈 착륙 오차 실측.
- [ ] 엄격한 검증 기준 적용:
  - `sampling_errors == 0`, `samples_count >= 40`
  - `total_collisions == 0` (무충돌 달성)
  - `min_pairwise_distance >= 3.0m` (안전 이격)
  - Bravo/Charlie의 최종 착륙 지점과 자기 스폰 좌표 간 오차 $\le 1.5m$ (좌표계 버그 해결 검증)

### Step 3: 전체 회귀 테스트
- [ ] `python test_orca_unit.py` (6/6 Pass)
- [ ] `python test_orca_formation_assemble.py` (Pass)
- [ ] `python test_orca_collision_avoidance.py` (Pass)
- [ ] `python test_following_mode.py` (Pass)
- [ ] `python test_ui_playwright.py` (19/19 Pass)

### Step 4: 산출물 문서화
- [ ] `docs/10_implementation_plan_rth_orca.md` (작업계획서)
- [ ] `docs/11_completion_report_rth_orca.md` (작업완료 보고서)
- [ ] `docs/00_INDEX.md` 갱신

---

## 4. 검증 계획

### 4.1 자동화 검증 항목
| 테스트 파일 | 목적 | 판정 기준 |
| :--- | :--- | :--- |
| `test_orca_rth.py` | 다중 기체 교차 RTH 충돌 회피 및 홈 착륙 정합성 | 충돌 0회, 최소 거리 $\ge 3.0m$, 홈 착륙 오차 $\le 1.5m$ |
| `test_orca_unit.py` | ORCA 수학 솔버 단위 테스트 | 6/6 All Passed |
| `test_orca_formation_assemble.py` | 편대 집결 회귀 | 무충돌, 최소거리 $\ge 3.0m$, 정렬 오차 $\le 1.5m$ |
| `test_orca_collision_avoidance.py` | Following Mode 스트레스 회귀 | 무충돌, 최소거리 $\ge 3.0m$ |
| `test_following_mode.py` | Following Mode 기본 기능 회귀 | 전 기체 정상 추종 |
| `test_ui_playwright.py` | 웹 관제 UI 전체 회귀 | 19/19 All Passed |

---

## 5. 결론 및 승인 요청

본 작업계획서는 작업지시서 #09의 요구사항에 따라 RTH의 기체별 고유 홈 복귀 좌표계 버그를 바로잡고, 3단계 비행 전 과정에 ORCA 충돌 회피를 완벽하게 적용하도록 설계되었습니다.
검토 후 승인해 주시면 구현을 진행하겠습니다.
