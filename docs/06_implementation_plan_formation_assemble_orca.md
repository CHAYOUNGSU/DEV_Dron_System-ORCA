# 작업계획서 #06: 편대 집결(Formation Assemble) ORCA 충돌 회피 적용 계획

- 작성자: Antigravity (구현 담당)
- 대상 작업지시서: `docs/05_work_order_formation_assemble_orca.md` (작성: Claude)
- 검수 담당: Codex (독립검수 / Review 역할)
- 선행 작업: `docs/01~04_*_following_mode_orca.md` (Following Mode ORCA - 검수 승인 완료)
- 대상 레포: `DEV_Dron_System-ORCA`

---

## 1. 개요 및 목적

### 1.1 배경
`Following Mode`에 ORCA를 적용하여 편대 추격 중의 충돌을 방지하는 작업은 성공적으로 검증 및 승인되었습니다.
그러나 사방으로 흩어져 있던 윙맨들(Drone2 Bravo, Drone3 Charlie, Drone4 Delta)을 알파 뒤의 정해진 트레일 슬롯(Trail Slot)으로 일시에 수렴시키는 **편대 집결(`_do_formation_assemble`)** 단계는 경로 교차로 인한 충돌 위험이 가장 높은 순간입니다.
기존 구현은 단순 일회성 `moveToPositionAsync()` 호출로 인해 윙맨 상호 간 경로 간섭을 고려하지 않으며, AirSim 기체별 로컬 좌표계(`SPAWN_OFFSETS`)를 보정하지 않아 트레일 슬롯 위치가 스폰 오프셋만큼 왜곡되는 문제가 있었습니다.

### 1.2 목적
1. 이미 검증된 `orca.py` 최적 속도 솔버를 `_do_formation_assemble()`의 윙맨 이동 단계에 적용하여, 윙맨들이 상호 간 및 알파와의 충돌을 회피하며 안전하게 정렬하도록 구현.
2. 각 기체의 스폰 오프셋(`SPAWN_OFFSETS`)을 반영하여 통일된 월드 좌표계(World Coordinate System) 기준으로 정확한 트레일 슬롯 위치 계산 및 비행 제어.
3. Following Mode와의 상호 배제(`formation_assemble_in_progress`)를 통해 이중 제어 명령 간섭 방지.
4. AirSim 실환경 정밀 스트레스 테스트(`test_orca_formation_assemble.py`)를 통해 무충돌(0회), 안전 이격($\ge 3.0m$), 슬롯 정렬 성공을 실측 검증.

---

## 2. 주요 설계 및 아키텍처

```mermaid
flowchart TD
    A[Formation Assemble API Call] --> B[Alpha 상태 확인 및 호버 고정]
    B --> C[Landed 윙맨 병렬 이륙]
    C --> D[월드 트레일 슬롯 좌표 계산 (Alpha 기준)]
    D --> E[formation_assemble_in_progress = True]
    E --> F[ORCA 속도 제어 루프 (10Hz, Max 20초)]
    
    subgraph ORCA_Loop [10Hz ORCA Velocity Loop]
        F1[윙맨별 월드 위치/속도 측정] --> F2[슬롯 선호 속도 pref_vel 계산]
        F2 --> F3[Alpha + 타 윙맨 neighbors 리스트 구성]
        F3 --> F4[orca.compute_safe_velocity 계산]
        F4 --> F5[ctrl.moveByVelocityAsync 명령 발사]
        F5 --> F6{모든 윙맨 슬롯 도달? < 1.0m}
    end
    
    F --> F1
    F6 -- Yes --> G[루프 정상 수렴 종료]
    F6 -- Timeout (20s) --> G
    G --> H[전 윙맨 hoverAsync 고정]
    H --> I[formation_assemble_in_progress = False]
```

### 2.1 월드 좌표계(World Coordinate System) 정합성
- `DRONES_CONFIG`의 `spawn_offset`:
  - `Drone1` (Alpha): `(0.0, 0.0, 0.0)`
  - `Drone2` (Bravo): `(0.0, 3.5, 0.0)`
  - `Drone3` (Charlie): `(0.0, 7.0, 0.0)`
  - `Drone4` (Delta): `(0.0, 10.5, 0.0)`
- 각 윙맨의 월드 위치: `cur_pos = (pos.x_val + offset_x, pos.y_val + offset_y, pos.z_val + offset_z)`
- 트레일 슬롯 월드 좌표:
  - $back\_dir_x = -\cos(alpha\_yaw)$
  - $back\_dir_y = -\sin(alpha\_yaw)$
  - $slot\_wx_i = alpha\_wx + back\_dir_x \times (i \times spacing)$
  - $slot\_wy_i = alpha\_wy + back\_dir_y \times (i \times spacing)$
  - $slot\_wz_i = target\_z$

### 2.2 ORCA 속도 제어 루프 설계
1. **반복 주기 및 제한**: `tick = 0.1s` (10Hz), `max_duration = 20.0s`
2. **선호 속도(`preferred_vel`) 계산**:
   - $dist_{2d} = \sqrt{(slot\_wx - cur\_wx)^2 + (slot\_wy - cur\_wy)^2}$
   - $dist_{3d} = \sqrt{dist_{2d}^2 + (slot\_wz - cur\_wz)^2}$
   - 비례 감속: $speed = \min(velocity, dist_{2d} / 0.8)$
   - $pref\_vx = (dx / dist_{2d}) \times speed$, $pref\_vy = (dy / dist_{2d}) \times speed$
   - $pref\_vz = (dz / 0.5)$ (최대 $ORCA\_MAX\_VZ\_MPS = 2.0m/s$ 클램핑)
3. **이웃(`neighbors`) 리스트**:
   - Alpha: 정지 장애물 (`weight = 1.0`, `radius = 1.5m`)
   - 타 윙맨: 상호 회피 에이전트 (`weight = 0.5`, `radius = 1.5m`)
4. **수렴 및 종료 조건**:
   - 모든 윙맨의 $dist_{3d} \le 0.8m$ 이고 수평 속도 $\le 0.5m/s$ 일 때 조기 수렴 종료.
   - 타임아웃 20초 도달 시 강제 종료 방어.
   - 루프 종료 시 전 윙맨 `ctrl.hoverAsync(vehicle_name=w_vname)` 호출로 제자리 고정.

### 2.3 Following Mode와의 상호배제
- `formation_assemble_in_progress = False` 전역 플래그 도입.
- `_do_formation_assemble()` 진입 시:
  - `following_mode_enabled = False`로 해제 (편대 집결 후 정렬된 상태에서 사용자가 필요 시 다시 활성화하도록 명확히 분리).
  - `formation_assemble_in_progress = True` 설정.
- `following_worker()` 루프 시작부에 `if formation_assemble_in_progress: time.sleep(0.1); continue` 가드 추가.
- `_do_formation_assemble()` 종료 시 `finally:` 블록에서 `formation_assemble_in_progress = False` 안전 해제.

---

## 3. 구현 단계별 세부 계획

### Step 1: `server.py` 백엔드 수정
- [ ] `formation_assemble_in_progress = False` 전역 플래그 선언.
- [ ] `following_worker()` 루프에 `formation_assemble_in_progress` 가드 추가.
- [ ] `_do_formation_assemble(spacing, velocity)` 함수 리팩토링:
  - Alpha 상태 확인 및 `hoverAsync` 고정 (유지).
  - 윙맨 병렬 이륙 (`takeoffAsync`) (유지).
  - 월드 좌표계 기반 트레일 슬롯 $(slot\_wx, slot\_wy, slot\_wz)$ 사전 계산.
  - 10Hz 유한 ORCA 속도 제어 루프 구현 (Alpha 및 타 윙맨 neighbors 등록, `compute_safe_velocity` 호출, `moveByVelocityAsync` 발사, 수렴 판정).
  - 루프 종료 시 `hoverAsync` 호출 및 플래그 복구.

### Step 2: 신규 테스트 스크립트 작성 (`test_orca_formation_assemble.py`)
- [ ] 4대 동시 이륙 후 윙맨 3대를 사방으로 강제 분산 이동 (Cross-over 충돌 위험 시나리오 구성):
  - Drone2: $+15m$ 전방 우측 ($X=+15, Y=+10$)
  - Drone3: $+15m$ 전방 좌측 ($X=+15, Y=-10$)
  - Drone4: $-10m$ 후방 좌측 ($X=-10, Y=-10$)
- [ ] `api_post("/api/formation/assemble", {"spacing": 8.0, "velocity": 3.5})` 호출.
- [ ] 20Hz 독립 샘플러를 통한 고빈도 실측 (샘플 수, 에러 카운트, 충돌 횟수, 기체 간 최소 이격 거리, 슬롯 최종 오차 측정).
- [ ] 엄격한 검증 기준 적용:
  - `sampling_errors == 0`, `samples_count >= 40`
  - `total_collisions == 0`
  - `min_pairwise_distance >= 2 * ORCA_AGENT_RADIUS_M = 3.0m`
  - 모든 윙맨의 슬롯 정렬 오차 $\le 1.5m$

### Step 3: 회귀 테스트 및 실환경 검증
- [ ] `python test_orca_unit.py`: 단위 테스트 (6/6 통과 확인).
- [ ] `python test_orca_formation_assemble.py`: 신규 분산 집결 충돌 회피 실측 (무충돌, 3.0m 안전 이격 통과 확인).
- [ ] `python test_following_mode.py`: Following Mode 회귀 검증 통과 확인.
- [ ] `python test_ui_playwright.py`: 관제 UI 자동화 회귀 (19/19 통과 확인).

### Step 4: 산출물 문서화
- [ ] `docs/06_implementation_plan_formation_assemble_orca.md` (작업계획서)
- [ ] `docs/07_completion_report_formation_assemble_orca.md` (작업완료 보고서)
- [ ] `docs/00_INDEX.md` 갱신

---

## 4. 검증 계획

### 4.1 자동화 검증 항목
| 테스트 파일 | 목적 | 판정 기준 |
| :--- | :--- | :--- |
| `test_orca_unit.py` | ORCA 수학 솔버 및 전방 시뮬레이션 | 6/6 All Passed |
| `test_orca_formation_assemble.py` | 사방 분산 상태에서 동시 집결 시 충돌 회피 | 충돌 0회, 최소 거리 $\ge 3.0m$, 슬롯 오차 $\le 1.5m$, 샘플러 에러 0건 |
| `test_following_mode.py` | 기존 Following Mode 기능 회귀 | 전 기체 정상 추종 Pass |
| `test_ui_playwright.py` | 웹 관제 UI 전체 회귀 | 19/19 All Passed |

---

## 5. 결론 및 승인 요청

본 작업계획서는 작업지시서 #05의 요구사항을 엄밀히 준수하여 편대 집결 시의 월드 좌표계 정합성과 ORCA 기반 안전 수렴을 완벽하게 구현하도록 설계되었습니다.
검토 후 승인해 주시면 구현을 진행하겠습니다.
