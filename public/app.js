/**
 * AirSim 4-UAV Fleet Command & Alpha Trail Formation Cockpit
 */

document.addEventListener('DOMContentLoaded', () => {
    // Current Active Selected Drone ('Drone1', 'Drone2', 'Drone3', 'Drone4')
    let currentDroneId = 'Drone1';
    let followingModeEnabled = false;

    const DRONE_META = {
        'Drone1': { tag: 'ALPHA-01',   name: 'Drone 1 (알파 / 편대장)',  role: '편대장', color: '#38bdf8', class: 'card-alpha',   key: 'F1' },
        'Drone2': { tag: 'BRAVO-02',   name: 'Drone 2 (브라보 / 2호기)', role: '2번기',  color: '#f59e0b', class: 'card-bravo',   key: 'F2' },
        'Drone3': { tag: 'CHARLIE-03', name: 'Drone 3 (찰리 / 3호기)',   role: '3번기',  color: '#10b981', class: 'card-charlie', key: 'F3' },
        'Drone4': { tag: 'DELTA-04',   name: 'Drone 4 (델타 / 4호기)',   role: '4번기',  color: '#a855f7', class: 'card-delta',   key: 'F4' }
    };

    // DOM Elements
    const connDot = document.getElementById('conn-dot');
    const connStatusText = document.getElementById('conn-status-text');
    const liveClock = document.getElementById('live-clock');

    // Drone Selection Buttons
    const btnDrone1 = document.getElementById('btn-drone-1');
    const btnDrone2 = document.getElementById('btn-drone-2');
    const btnDrone3 = document.getElementById('btn-drone-3');
    const btnDrone4 = document.getElementById('btn-drone-4');

    const tabStatusDrone1 = document.getElementById('tab-status-drone1');
    const tabStatusDrone2 = document.getElementById('tab-status-drone2');
    const tabStatusDrone3 = document.getElementById('tab-status-drone3');
    const tabStatusDrone4 = document.getElementById('tab-status-drone4');

    // Formation Call Buttons
    const btnFormationAssemble = document.getElementById('btn-formation-assemble');
    const btnCallFormationBar = document.getElementById('btn-call-formation-bar');

    // Following Mode Toggle
    const btnFollowingToggle = document.getElementById('btn-following-toggle');
    const followingToggleText = document.getElementById('following-toggle-text');

    // Bulk Fleet Buttons
    const btnFleetAllTakeoff = document.getElementById('btn-fleet-all-takeoff');
    const btnFleetAllLand = document.getElementById('btn-fleet-all-land');

    // Active Drone Labels
    const activeDroneTitleTag = document.getElementById('active-drone-title-tag');
    const cameraDroneBadge = document.getElementById('camera-drone-badge');
    const joystickDroneTag = document.getElementById('joystick-drone-tag');
    const actionDroneLabel = document.getElementById('action-drone-label');
    const telemetryDroneTitle = document.getElementById('telemetry-drone-title');
    const coordDroneTag = document.getElementById('coord-drone-tag');
    const fallbackStatusText = document.getElementById('fallback-status-text');

    // Fleet Mini Cards
    const fleetCards = {
        'Drone1': document.getElementById('fleet-card-drone1'),
        'Drone2': document.getElementById('fleet-card-drone2'),
        'Drone3': document.getElementById('fleet-card-drone3'),
        'Drone4': document.getElementById('fleet-card-drone4')
    };

    const fleetStates = {
        'Drone1': document.getElementById('fleet-state-drone1'),
        'Drone2': document.getElementById('fleet-state-drone2'),
        'Drone3': document.getElementById('fleet-state-drone3'),
        'Drone4': document.getElementById('fleet-state-drone4')
    };

    const fleetAlts = {
        'Drone1': document.getElementById('fleet-alt-drone1'),
        'Drone2': document.getElementById('fleet-alt-drone2'),
        'Drone3': document.getElementById('fleet-alt-drone3'),
        'Drone4': document.getElementById('fleet-alt-drone4')
    };

    const fleetSpds = {
        'Drone1': document.getElementById('fleet-spd-drone1'),
        'Drone2': document.getElementById('fleet-spd-drone2'),
        'Drone3': document.getElementById('fleet-spd-drone3'),
        'Drone4': document.getElementById('fleet-spd-drone4')
    };

    const fleetPoss = {
        'Drone1': document.getElementById('fleet-pos-drone1'),
        'Drone2': document.getElementById('fleet-pos-drone2'),
        'Drone3': document.getElementById('fleet-pos-drone3'),
        'Drone4': document.getElementById('fleet-pos-drone4')
    };

    // Simulator Environment Selector Header
    const btnOpenSimModal = document.getElementById('btn-open-sim-modal');
    const simActiveDot = document.getElementById('sim-active-dot');
    const simActiveText = document.getElementById('sim-active-text');

    // Simulator Selection Modal Elements
    const simModal = document.getElementById('sim-modal');
    const btnCloseSimModal = document.getElementById('btn-close-sim-modal');
    const btnCancelSimModal = document.getElementById('btn-cancel-sim-modal');
    const btnStopAllSims = document.getElementById('btn-stop-all-sims');
    const simCardsContainer = document.getElementById('sim-cards-container');

    const fpvImg = document.getElementById('fpv-stream-img');
    const cameraFallback = document.getElementById('camera-fallback');
    const cameraFps = document.getElementById('camera-fps');
    const cameraLatency = document.getElementById('camera-latency');

    // Aviation HUD Canvas & Controls
    const hudCanvas = document.getElementById('hud-canvas');
    const hudCtx = hudCanvas.getContext('2d');
    const btnToggleHud = document.getElementById('btn-toggle-hud');
    const hudToggleText = document.getElementById('hud-toggle-text');
    let isHudEnabled = true;

    const armedBadge = document.getElementById('armed-badge');
    const apiBadge = document.getElementById('api-badge');

    const valAltitude = document.getElementById('val-altitude');
    const barAltitude = document.getElementById('bar-altitude');

    const valSpeed = document.getElementById('val-speed');
    const barSpeed = document.getElementById('bar-speed');

    const valX = document.getElementById('val-x');
    const valY = document.getElementById('val-y');
    const valZ = document.getElementById('val-z');

    const valPitch = document.getElementById('val-pitch');
    const valRoll = document.getElementById('val-roll');
    const valYaw = document.getElementById('val-yaw');

    const valGps = document.getElementById('val-gps');

    const logTerminal = document.getElementById('log-terminal');

    // Joysticks Elements
    const stickLeftPad = document.getElementById('stick-left-pad');
    const stickLeftKnob = document.getElementById('stick-left-knob');
    const stickLeftInfo = document.getElementById('stick-left-info');

    const stickRightPad = document.getElementById('stick-right-pad');
    const stickRightKnob = document.getElementById('stick-right-knob');
    const stickRightInfo = document.getElementById('stick-right-info');

    // Speed Rate Mode Buttons
    const btnRateLow = document.getElementById('btn-rate-low');
    const btnRateMid = document.getElementById('btn-rate-mid');
    const btnRateHigh = document.getElementById('btn-rate-high');

    // Action Buttons
    const btnTakeoff = document.getElementById('btn-takeoff');
    const btnLand = document.getElementById('btn-land');
    const btnRth = document.getElementById('btn-rth');
    const btnRotate = document.getElementById('btn-rotate');
    const btnEmergency = document.getElementById('btn-emergency');
    const btnReset = document.getElementById('btn-reset');
    const btnSnapshot = document.getElementById('btn-snapshot');
    const btnReconnect = document.getElementById('btn-reconnect');
    const btnClearLog = document.getElementById('btn-clear-log');
    const btnCopyLog = document.getElementById('btn-copy-log');
    const btnExportLog = document.getElementById('btn-export-log');

    let ws = null;
    let frameCount = 0;
    let lastFpsCheck = Date.now();
    let rawLogHistory = [];

    // Speed Rate Presets
    const SPEED_RATES = {
        low:  { name: 'LOW',  multiplier: 0.3, label: '30% 정밀 저속' },
        mid:  { name: 'MID',  multiplier: 0.6, label: '60% 표준 중속' },
        high: { name: 'HIGH', multiplier: 1.0, label: '100% 다이내믹 고속' }
    };
    let currentRateKey = 'mid';

    // Joystick Velocity State
    let stickLeft = { x: 0, y: 0 };   // Yaw (X), Vz (Y)
    let stickRight = { x: 0, y: 0 };  // Vy (X), Vx (Y)
    const activeKeys = {};
    let isJoystickActive = false;
    let joystickInterval = null;

    // 1. Live Clock
    function updateClock() {
        const now = new Date();
        liveClock.textContent = now.toTimeString().split(' ')[0];
    }
    setInterval(updateClock, 1000);
    updateClock();

    // 2. Add Log Entry
    function appendLog(msg, type = 'info') {
        const now = new Date();
        const timeStr = now.toTimeString().split(' ')[0];

        rawLogHistory.push(`[${timeStr}] [${type.toUpperCase()}] ${msg}`);
        if (rawLogHistory.length > 500) rawLogHistory.shift();

        const entry = document.createElement('div');
        entry.className = `log-entry log-${type}`;
        entry.innerHTML = `<span class="log-time">[${timeStr}]</span> <span class="log-msg">${msg}</span>`;
        logTerminal.appendChild(entry);
        logTerminal.scrollTop = logTerminal.scrollHeight;
    }

    // 3. Switch Active Controlled Drone (Drone1, Drone2, Drone3, Drone4)
    async function selectTargetDrone(droneId) {
        if (!DRONE_META[droneId]) return;
        currentDroneId = droneId;
        const meta = DRONE_META[droneId];

        // Update Tab Active States
        [btnDrone1, btnDrone2, btnDrone3, btnDrone4].forEach((btn, idx) => {
            const d = `Drone${idx+1}`;
            if (btn) btn.classList.toggle('active', d === droneId);
        });

        // Update Mini Cards Active States
        Object.keys(fleetCards).forEach(k => {
            if (fleetCards[k]) fleetCards[k].classList.toggle('active', k === droneId);
        });

        activeDroneTitleTag.textContent = `${meta.name} (${meta.tag})`;
        activeDroneTitleTag.style.color = meta.color;

        cameraDroneBadge.textContent = `TARGET: ${meta.tag}`;
        cameraDroneBadge.style.borderColor = meta.color;
        cameraDroneBadge.style.color = meta.color;

        joystickDroneTag.textContent = `${meta.tag} (${meta.name})`;
        joystickDroneTag.style.color = meta.color;

        actionDroneLabel.textContent = meta.name;
        telemetryDroneTitle.textContent = meta.name;
        coordDroneTag.textContent = meta.tag;
        coordDroneTag.style.color = meta.color;
        fallbackStatusText.textContent = `AirSim ${meta.name} FPV Camera Stream Ready`;

        appendLog(`[기체 전환] 제어 대상 드론을 <strong>[${meta.tag} - ${meta.name}]</strong>으로 변경했습니다. [단축키: ${meta.key}]`, 'info');

        try {
            await fetch('/api/drones/select', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ drone_id: droneId })
            });
        } catch (e) {
            console.error("Failed to notify backend drone selection:", e);
        }
    }

    if (btnDrone1) btnDrone1.addEventListener('click', () => selectTargetDrone('Drone1'));
    if (btnDrone2) btnDrone2.addEventListener('click', () => selectTargetDrone('Drone2'));
    if (btnDrone3) btnDrone3.addEventListener('click', () => selectTargetDrone('Drone3'));
    if (btnDrone4) btnDrone4.addEventListener('click', () => selectTargetDrone('Drone4'));

    Object.keys(fleetCards).forEach(k => {
        if (fleetCards[k]) fleetCards[k].addEventListener('click', () => selectTargetDrone(k));
    });

    // 4. Alpha Leader Call & Trail Formation Assembly API
    async function triggerFormationAssembly() {
        appendLog(`[편대장 ALPHA 호출] 전 편대기(브라보/찰리/델타)를 알파 고도로 호출하여 일렬 종대(Trail) 편대 포메이션 집결을 시작합니다.`, 'cmd');
        try {
            const res = await fetch('/api/formation/assemble', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ spacing: 12.0, velocity: 4.0 })
            });
            const data = await res.json();
            if (data.status === 'success' || data.status === 'simulated') {
                appendLog(`${data.message}`, 'success');
            } else {
                appendLog(`편대 집결 실패: ${data.message}`, 'warn');
            }
        } catch (e) {
            appendLog(`편대 집결 요청 오류: ${e}`, 'warn');
        }
    }

    if (btnFormationAssemble) btnFormationAssemble.addEventListener('click', triggerFormationAssembly);
    if (btnCallFormationBar) btnCallFormationBar.addEventListener('click', triggerFormationAssembly);

    // 4b. Following Mode Toggle (Duckling Chain Autopilot: Bravo->Alpha, Charlie->Bravo, Delta->Charlie)
    function updateFollowingButtonUI() {
        if (!btnFollowingToggle) return;
        btnFollowingToggle.classList.toggle('active', followingModeEnabled);
        followingToggleText.textContent = followingModeEnabled ? 'Following Mode: ON [F6]' : 'Following Mode: OFF [F6]';
    }

    async function setFollowingMode(enabled) {
        try {
            const res = await fetch('/api/following/toggle', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ enabled, lag_seconds: 2.0, velocity: 3.0 })
            });
            const data = await res.json();
            followingModeEnabled = !!data.enabled;
            updateFollowingButtonUI();
            appendLog(`${data.message}`, (data.status === 'error') ? 'warn' : 'success');
            if (followingModeEnabled) {
                selectTargetDrone('Drone1');
            }
        } catch (e) {
            appendLog(`Following Mode 전환 오류: ${e}`, 'warn');
        }
    }

    if (btnFollowingToggle) {
        btnFollowingToggle.addEventListener('click', () => setFollowingMode(!followingModeEnabled));
    }

    // Fleet Bulk Takeoff & Land
    if (btnFleetAllTakeoff) {
        btnFleetAllTakeoff.addEventListener('click', async () => {
            appendLog(`[전체 편대 동시 이륙] 알파/브라보/찰리/델타 4대 동시 이륙 명령 전송 중...`, 'cmd');
            try {
                const res = await fetch('/api/fleet/takeoff', { method: 'POST' });
                const data = await res.json();
                appendLog(`${data.message}`, 'success');
            } catch (e) {
                appendLog(`전체 이륙 요청 오류: ${e}`, 'warn');
            }
        });
    }

    if (btnFleetAllLand) {
        btnFleetAllLand.addEventListener('click', async () => {
            appendLog(`[전체 편대 동시 착륙] 4대 전 기체 안전 착륙 명령 전송 중...`, 'cmd');
            try {
                const res = await fetch('/api/fleet/land', { method: 'POST' });
                const data = await res.json();
                appendLog(`${data.message}`, 'success');
            } catch (e) {
                appendLog(`전체 착륙 요청 오류: ${e}`, 'warn');
            }
        });
    }

    // 5. Simulator Environment Selector Modal & Process Lifecycle Manager
    async function loadSimulatorCards() {
        if (!simCardsContainer) return;
        simCardsContainer.innerHTML = `
            <div style="grid-column: 1 / -1; text-align: center; padding: 24px; color: var(--text-muted);">
                시뮬레이터 환경 목록 및 프로세스 상태 조회 중...
            </div>
        `;

        try {
            const res = await fetch('/api/simulators');
            const data = await res.json();
            const activeId = data.active ? data.active.id : null;
            const simList = data.simulators || [];

            simCardsContainer.innerHTML = '';

            simList.forEach(sim => {
                const isRunning = (sim.id === activeId) || sim.is_running;
                const card = document.createElement('div');
                card.className = `sim-card ${isRunning ? 'active' : ''}`;
                card.id = `sim-card-${sim.id}`;

                const badgeHtml = isRunning
                    ? `<span class="badge-pill pill-active">현재 실행 중</span>`
                    : (sim.exists ? `<span class="badge-pill pill-neutral">대기</span>` : `<span class="badge-pill pill-warn">파일 없음</span>`);

                const btnHtml = isRunning
                    ? `<button class="btn btn-action btn-sm" id="btn-launch-${sim.id}">재시작</button>`
                    : `<button class="btn btn-primary btn-sm" id="btn-launch-${sim.id}" ${sim.exists ? '' : 'disabled'}>맵 실행</button>`;

                card.innerHTML = `
                    <div class="sim-card-header">
                        <div class="sim-card-title">
                            <span>${sim.name}</span>
                        </div>
                        ${badgeHtml}
                    </div>
                    <div class="sim-card-desc">
                        ${sim.desc}
                    </div>
                    <div class="sim-card-footer">
                        ${btnHtml}
                    </div>
                `;

                simCardsContainer.appendChild(card);

                const btnLaunch = card.querySelector(`#btn-launch-${sim.id}`);
                if (btnLaunch && sim.exists) {
                    btnLaunch.addEventListener('click', () => launchSimulator(sim.id, sim.name, btnLaunch));
                }
            });
        } catch (err) {
            simCardsContainer.innerHTML = `
                <div style="grid-column: 1 / -1; text-align: center; padding: 20px; color: var(--accent-critical);">
                    시뮬레이터 목록을 불러오지 못했습니다: ${err.message}
                </div>
            `;
        }
    }

    async function launchSimulator(simId, simName, btnEl) {
        if (btnEl) {
            btnEl.disabled = true;
            btnEl.textContent = '기동 중...';
        }

        appendLog(`[시뮬레이터 전환] 기존 프로세스를 정리하고 [${simName || simId}] 환경을 시작합니다...`, 'cmd');

        try {
            const res = await fetch('/api/simulators/launch', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ id: simId, resolution: '1280x720' })
            });
            const data = await res.json();

            if (data.status === 'success') {
                appendLog(`${data.message}`, 'success');
                closeSimModal();

                // Auto-poll connection status until the background worker thread
                // finishes connecting + spawning the fleet (large maps can take a
                // while to boot). This only reads status - it does not open its own
                // AirSim client, since doing so used to race with the worker thread.
                let attempts = 0;
                const maxAttempts = 40; // 40 x 1.5s = up to 60s for slow-loading maps
                const pollConnect = setInterval(async () => {
                    attempts++;
                    try {
                        const cRes = await fetch('/api/connect', { method: 'POST' });
                        const cData = await cRes.json();
                        if (cData.status === 'success') {
                            clearInterval(pollConnect);
                            appendLog(`${cData.message}`, 'success');
                        } else if (attempts >= maxAttempts) {
                            clearInterval(pollConnect);
                            appendLog(`연결 확인 시간 초과 (${maxAttempts * 1.5}초). 텔레메트리가 계속 업데이트되면 정상 연결된 것입니다.`, 'warn');
                        }
                    } catch (e) {
                        if (attempts >= maxAttempts) clearInterval(pollConnect);
                    }
                }, 1500);
            } else {
                appendLog(`${data.message}`, 'warn');
                if (btnEl) {
                    btnEl.disabled = false;
                    btnEl.textContent = '맵 실행';
                }
            }
        } catch (err) {
            appendLog(`시뮬레이터 실행 통신 실패: ${err}`, 'warn');
            if (btnEl) {
                btnEl.disabled = false;
                btnEl.textContent = '맵 실행';
            }
        }
    }

    async function stopAllSimulatorsAction() {
        if (btnStopAllSims) {
            btnStopAllSims.disabled = true;
            btnStopAllSims.textContent = '종료 중...';
        }

        appendLog('[시뮬레이터 완전 종료] 모든 AirSim 3D 시뮬레이터 프로세스를 종료합니다...', 'warn');

        try {
            const res = await fetch('/api/simulators/stop', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });
            const data = await res.json();
            appendLog(`${data.message}`, 'success');
            setTimeout(() => {
                if (btnStopAllSims) {
                    btnStopAllSims.disabled = false;
                    btnStopAllSims.textContent = '시뮬레이터 완전 종료';
                }
                loadSimulatorCards();
            }, 600);
        } catch (err) {
            appendLog(`시뮬레이터 종료 통신 실패: ${err}`, 'warn');
            if (btnStopAllSims) {
                btnStopAllSims.disabled = false;
                btnStopAllSims.textContent = '시뮬레이터 완전 종료';
            }
        }
    }

    function openSimModal() {
        if (simModal) {
            simModal.style.display = 'flex';
            loadSimulatorCards();
        }
    }

    function closeSimModal() {
        if (simModal) {
            simModal.style.display = 'none';
        }
    }

    if (btnOpenSimModal) btnOpenSimModal.addEventListener('click', openSimModal);
    if (btnCloseSimModal) btnCloseSimModal.addEventListener('click', closeSimModal);
    if (btnCancelSimModal) btnCancelSimModal.addEventListener('click', closeSimModal);
    if (btnStopAllSims) btnStopAllSims.addEventListener('click', stopAllSimulatorsAction);

    // Close modal on backdrop click
    if (simModal) {
        simModal.addEventListener('click', (e) => {
            if (e.target === simModal) closeSimModal();
        });
    }

    // Close modal on Escape key
    window.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && simModal && simModal.style.display === 'flex') {
            closeSimModal();
        }
    });

    // 6. Aviation HUD ON/OFF Toggle
    function toggleHud() {
        isHudEnabled = !isHudEnabled;
        if (btnToggleHud) {
            btnToggleHud.classList.toggle('off', !isHudEnabled);
            hudToggleText.textContent = isHudEnabled ? 'HUD: ON [H]' : 'HUD: OFF [H]';
        }
        if (!isHudEnabled) {
            hudCtx.clearRect(0, 0, hudCanvas.width, hudCanvas.height);
        }
        appendLog(`비행 HUD 표시 상태: [${isHudEnabled ? 'ON (활성화)' : 'OFF (비활성화)'}]`, 'info');
    }

    if (btnToggleHud) {
        btnToggleHud.addEventListener('click', toggleHud);
    }

    // 7. Tactical Aviation HUD Canvas Rendering Engine
    function renderAviationHud(t) {
        const rect = hudCanvas.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        const width = Math.round(rect.width * dpr);
        const height = Math.round(rect.height * dpr);

        if (hudCanvas.width !== width || hudCanvas.height !== height) {
            hudCanvas.width = width;
            hudCanvas.height = height;
        }

        hudCtx.clearRect(0, 0, width, height);

        if (!isHudEnabled || !t) return;

        hudCtx.save();
        hudCtx.scale(dpr, dpr);

        const w = rect.width;
        const h = rect.height;
        const cx = w / 2;
        const cy = h / 2;

        const pitch = t.pitch || 0;
        const roll = t.roll || 0;
        const yaw = (t.yaw !== undefined ? ((t.yaw % 360) + 360) % 360 : 0);
        const alt = Math.abs(t.z || 0);
        const spd = t.speed || 0;
        const vz = -(t.vz || 0);

        const meta = DRONE_META[currentDroneId] || DRONE_META['Drone1'];
        const hudColor = meta.color;
        const hudColorDim = 'rgba(255, 255, 255, 0.35)';
        const hudColorBg = 'rgba(6, 15, 25, 0.65)';

        hudCtx.strokeStyle = hudColor;
        hudCtx.fillStyle = hudColor;
        hudCtx.lineWidth = 1.5;
        hudCtx.font = '11px "JetBrains Mono", Consolas, monospace';
        hudCtx.shadowColor = 'rgba(0, 0, 0, 0.8)';
        hudCtx.shadowBlur = 4;

        // A. Center Boresight Reticle
        hudCtx.save();
        hudCtx.beginPath();
        hudCtx.arc(cx, cy, 4, 0, Math.PI * 2);
        hudCtx.stroke();

        hudCtx.beginPath();
        hudCtx.moveTo(cx - 24, cy);
        hudCtx.lineTo(cx - 8, cy);
        hudCtx.lineTo(cx - 8, cy + 4);
        hudCtx.stroke();

        hudCtx.beginPath();
        hudCtx.moveTo(cx + 8, cy + 4);
        hudCtx.lineTo(cx + 8, cy);
        hudCtx.lineTo(cx + 24, cy);
        hudCtx.stroke();
        hudCtx.restore();

        // B. Pitch Ladder & Artificial Horizon
        hudCtx.save();
        hudCtx.translate(cx, cy);
        hudCtx.rotate((roll * Math.PI) / 180);

        const pitchPixelsPerDeg = 3.5;
        const pitchOffsetY = pitch * pitchPixelsPerDeg;

        hudCtx.beginPath();
        hudCtx.rect(-160, -140, 320, 280);
        hudCtx.clip();

        // 0 deg Horizon
        hudCtx.lineWidth = 2.0;
        hudCtx.strokeStyle = hudColor;
        hudCtx.beginPath();
        hudCtx.moveTo(-110, pitchOffsetY);
        hudCtx.lineTo(-30, pitchOffsetY);
        hudCtx.moveTo(30, pitchOffsetY);
        hudCtx.lineTo(110, pitchOffsetY);
        hudCtx.stroke();

        // Pitch bars
        hudCtx.lineWidth = 1.5;
        const pitchAngles = [10, 20, 30, 40, -10, -20, -30, -40];

        pitchAngles.forEach(deg => {
            const y = pitchOffsetY - (deg * pitchPixelsPerDeg);
            const isPos = deg > 0;
            const barWidth = 60;
            const tickH = 6;

            hudCtx.beginPath();
            if (isPos) {
                hudCtx.setLineDash([]);
                hudCtx.moveTo(-barWidth, y);
                hudCtx.lineTo(-24, y);
                hudCtx.lineTo(-24, y + tickH);

                hudCtx.moveTo(24, y + tickH);
                hudCtx.lineTo(24, y);
                hudCtx.lineTo(barWidth, y);
            } else {
                hudCtx.setLineDash([4, 4]);
                hudCtx.moveTo(-barWidth, y);
                hudCtx.lineTo(-24, y);
                hudCtx.lineTo(-24, y - tickH);

                hudCtx.moveTo(24, y - tickH);
                hudCtx.lineTo(24, y);
                hudCtx.lineTo(barWidth, y);
            }
            hudCtx.stroke();
            hudCtx.setLineDash([]);

            hudCtx.fillText(`${Math.abs(deg)}`, -barWidth - 18, y + 4);
            hudCtx.fillText(`${Math.abs(deg)}`, barWidth + 6, y + 4);
        });
        hudCtx.restore();

        // C. Heading Compass Tape
        const compassW = 240;
        const compassH = 28;
        const compassX = cx - compassW / 2;
        const compassY = 14;

        hudCtx.fillStyle = hudColorBg;
        hudCtx.strokeStyle = hudColorDim;
        hudCtx.fillRect(compassX, compassY, compassW, compassH);
        hudCtx.strokeRect(compassX, compassY, compassW, compassH);

        hudCtx.save();
        hudCtx.beginPath();
        hudCtx.rect(compassX, compassY, compassW, compassH);
        hudCtx.clip();

        const degPixels = compassW / 60;
        const startDeg = Math.floor((yaw - 30) / 5) * 5;
        const endDeg = startDeg + 65;

        for (let d = startDeg; d <= endDeg; d += 5) {
            const screenX = cx + (d - yaw) * degPixels;
            const normDeg = ((d % 360) + 360) % 360;
            const isMajor = (normDeg % 15 === 0);

            hudCtx.strokeStyle = isMajor ? hudColor : hudColorDim;
            hudCtx.beginPath();
            hudCtx.moveTo(screenX, compassY + compassH);
            hudCtx.lineTo(screenX, compassY + compassH - (isMajor ? 10 : 5));
            hudCtx.stroke();

            if (isMajor) {
                let label = `${Math.floor(normDeg / 10)}`;
                if (normDeg === 0 || normDeg === 360) label = 'N';
                else if (normDeg === 90) label = 'E';
                else if (normDeg === 180) label = 'S';
                else if (normDeg === 270) label = 'W';

                hudCtx.fillStyle = (['N','E','S','W'].includes(label)) ? meta.color : hudColor;
                hudCtx.textAlign = 'center';
                hudCtx.fillText(label, screenX, compassY + 12);
            }
        }
        hudCtx.restore();

        // Compass Center Pip
        hudCtx.fillStyle = hudColor;
        hudCtx.beginPath();
        hudCtx.moveTo(cx, compassY + compassH + 4);
        hudCtx.lineTo(cx - 5, compassY + compassH + 9);
        hudCtx.lineTo(cx + 5, compassY + compassH + 9);
        hudCtx.closePath();
        hudCtx.fill();

        hudCtx.fillStyle = hudColor;
        hudCtx.textAlign = 'center';
        hudCtx.fillText(`HDG: ${Math.round(yaw).toString().padStart(3, '0')}°`, cx, compassY + compassH + 22);

        // D. Left Airspeed Tape
        const spdW = 64;
        const spdH = 180;
        const spdX = 30;
        const spdY = cy - spdH / 2;

        hudCtx.fillStyle = hudColorBg;
        hudCtx.strokeStyle = hudColorDim;
        hudCtx.fillRect(spdX, spdY, spdW, spdH);
        hudCtx.strokeRect(spdX, spdY, spdW, spdH);

        hudCtx.save();
        hudCtx.beginPath();
        hudCtx.rect(spdX, spdY, spdW, spdH);
        hudCtx.clip();

        const spdPixPerUnit = spdH / 16;
        const minSpdTick = Math.max(0, Math.floor(spd - 8));
        const maxSpdTick = minSpdTick + 18;

        for (let s = minSpdTick; s <= maxSpdTick; s += 1) {
            const tickY = cy - (s - spd) * spdPixPerUnit;
            const isMajor = (s % 2 === 0);

            hudCtx.strokeStyle = isMajor ? hudColor : hudColorDim;
            hudCtx.beginPath();
            hudCtx.moveTo(spdX + spdW, tickY);
            hudCtx.lineTo(spdX + spdW - (isMajor ? 12 : 6), tickY);
            hudCtx.stroke();

            if (isMajor) {
                hudCtx.fillStyle = hudColor;
                hudCtx.textAlign = 'left';
                hudCtx.fillText(`${s}`, spdX + 8, tickY + 4);
            }
        }
        hudCtx.restore();

        // Digital Speed Box
        hudCtx.fillStyle = '#0f172a';
        hudCtx.strokeStyle = hudColor;
        hudCtx.lineWidth = 1.5;
        hudCtx.fillRect(spdX - 4, cy - 14, spdW + 8, 28);
        hudCtx.strokeRect(spdX - 4, cy - 14, spdW + 8, 28);

        hudCtx.fillStyle = hudColor;
        hudCtx.textAlign = 'center';
        hudCtx.font = 'bold 12px "JetBrains Mono", monospace';
        hudCtx.fillText(`${spd.toFixed(1)}`, spdX + spdW / 2, cy + 4);
        hudCtx.font = '9px "JetBrains Mono", monospace';
        hudCtx.fillText(`m/s`, spdX + spdW / 2, cy + 12);

        // E. Right Altitude Tape
        const altW = 68;
        const altH = 180;
        const altX = w - altW - 30;
        const altY = cy - altH / 2;

        hudCtx.fillStyle = hudColorBg;
        hudCtx.strokeStyle = hudColorDim;
        hudCtx.fillRect(altX, altY, altW, altH);
        hudCtx.strokeRect(altX, altY, altW, altH);

        hudCtx.save();
        hudCtx.beginPath();
        hudCtx.rect(altX, altY, altW, altH);
        hudCtx.clip();

        const altPixPerUnit = altH / 20;
        const minAltTick = Math.floor((alt - 10) / 2) * 2;
        const maxAltTick = minAltTick + 24;

        for (let a = minAltTick; a <= maxAltTick; a += 2) {
            const tickY = cy - (a - alt) * altPixPerUnit;
            const isMajor = (a % 10 === 0);

            hudCtx.strokeStyle = isMajor ? hudColor : hudColorDim;
            hudCtx.beginPath();
            hudCtx.moveTo(altX, tickY);
            hudCtx.lineTo(altX + (isMajor ? 12 : 6), tickY);
            hudCtx.stroke();

            if (isMajor) {
                hudCtx.fillStyle = hudColor;
                hudCtx.textAlign = 'right';
                hudCtx.fillText(`${a}`, altX + altW - 8, tickY + 4);
            }
        }
        hudCtx.restore();

        // Digital Altitude Box
        hudCtx.fillStyle = '#0f172a';
        hudCtx.strokeStyle = hudColor;
        hudCtx.fillRect(altX - 4, cy - 14, altW + 8, 28);
        hudCtx.strokeRect(altX - 4, cy - 14, altW + 8, 28);

        hudCtx.fillStyle = hudColor;
        hudCtx.textAlign = 'center';
        hudCtx.font = 'bold 12px "JetBrains Mono", monospace';
        hudCtx.fillText(`${alt.toFixed(1)}m`, altX + altW / 2, cy + 4);
        hudCtx.font = '9px "JetBrains Mono", monospace';
        hudCtx.fillText(`ALT`, altX + altW / 2, cy + 12);

        // VSI
        hudCtx.font = '10px "JetBrains Mono", monospace';
        hudCtx.fillStyle = vz >= 0 ? '#34d399' : '#f87171';
        hudCtx.textAlign = 'right';
        hudCtx.fillText(`VSI: ${vz >= 0 ? '+' : ''}${vz.toFixed(1)} m/s`, w - 30, altY + altH + 16);

        // F. Tactical Corner Badges
        hudCtx.font = '11px "JetBrains Mono", monospace';
        hudCtx.fillStyle = hudColor;

        const activeSimTag = t.active_sim_name || 'AIRSIM';
        hudCtx.textAlign = 'left';
        hudCtx.fillText(`TARGET: [${meta.tag} ${meta.role}] | MAP: ${activeSimTag} [${SPEED_RATES[currentRateKey].name} ${SPEED_RATES[currentRateKey].multiplier * 100}%]`, 20, 26);
        hudCtx.fillText(`GPS: ${t.lat || 47.641468}, ${t.lon || -122.140165}`, 20, 42);

        hudCtx.fillText(`STATE: ${t.landed_state || 'Flying'} | ARMED: ${t.armed ? 'YES' : 'NO'}`, 20, h - 26);
        hudCtx.fillText(`BATTERY: ${t.battery || 98}% (4S LiPo)`, 20, h - 12);

        hudCtx.textAlign = 'right';
        hudCtx.fillText(`POS X: ${(t.x || 0).toFixed(2)}  Y: ${(t.y || 0).toFixed(2)}`, w - 20, h - 26);
        hudCtx.fillText(`POS Z: ${(t.z || 0).toFixed(2)}m (AGL)`, w - 20, h - 12);

        hudCtx.restore();
    }

    // 8. Connect Zero-Latency 4-UAV Fleet WebSocket
    function connectWebSocket() {
        const wsUrl = `ws://${window.location.host}/ws/telemetry`;
        appendLog(`웹 대시보드 통신 시작... (${wsUrl})`, 'info');

        ws = new WebSocket(wsUrl);

        ws.onopen = () => {
            connDot.className = 'chip-dot dot-online';
            connStatusText.textContent = '웹 대시보드 서버 연결됨';
            appendLog('[웹서버] 4대 편대 초저지연 조이스틱 & 관제 스트림 연결 완료', 'success');
        };

        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                const t = data.telemetry;
                const allT = data.all_telemetries || {};
                const nowMs = Date.now();

                // Keep the Following Mode button in sync with server state (e.g.
                // after a page refresh, or if it was toggled from elsewhere)
                if (typeof data.following_mode_enabled === 'boolean' && data.following_mode_enabled !== followingModeEnabled) {
                    followingModeEnabled = data.following_mode_enabled;
                    updateFollowingButtonUI();
                }

                // Update 4-UAV Fleet Overview Cards
                ['Drone1', 'Drone2', 'Drone3', 'Drone4'].forEach((dId, idx) => {
                    if (allT[dId]) {
                        const td = allT[dId];
                        if (fleetStates[dId]) fleetStates[dId].textContent = td.landed_state || 'Landed';
                        if (fleetAlts[dId]) fleetAlts[dId].textContent = `${Math.abs(td.z || 0).toFixed(1)}m`;
                        if (fleetSpds[dId]) fleetSpds[dId].textContent = `${(td.speed || 0).toFixed(1)}m/s`;
                        if (fleetPoss[dId]) fleetPoss[dId].textContent = `(${td.x.toFixed(1)}, ${td.y.toFixed(1)}, ${td.z.toFixed(1)})`;
                    }
                });

                if (allT['Drone1'] && tabStatusDrone1) tabStatusDrone1.textContent = allT['Drone1'].landed_state || 'Landed';
                if (allT['Drone2'] && tabStatusDrone2) tabStatusDrone2.textContent = allT['Drone2'].landed_state || 'Landed';
                if (allT['Drone3'] && tabStatusDrone3) tabStatusDrone3.textContent = allT['Drone3'].landed_state || 'Landed';
                if (allT['Drone4'] && tabStatusDrone4) tabStatusDrone4.textContent = allT['Drone4'].landed_state || 'Landed';

                // End-to-End Latency
                if (data.capture_time && data.capture_time > 0) {
                    const latencyMs = Math.max(1, Math.round(nowMs - (data.capture_time * 1000)));
                    if (cameraLatency) {
                        cameraLatency.textContent = `Latency: ${latencyMs} ms`;
                        if (latencyMs < 100) {
                            cameraLatency.className = 'camera-badge latency-badge latency-low';
                        } else {
                            cameraLatency.className = 'camera-badge latency-badge latency-high';
                        }
                    }
                }

                // Update Active Simulator Header Chip
                if (t.active_sim_name && t.active_sim_name !== '시뮬레이터 미실행') {
                    simActiveText.textContent = t.active_sim_name;
                    simActiveDot.className = t.connected ? 'chip-dot dot-online' : 'chip-dot dot-simulated';
                } else {
                    simActiveText.textContent = '시뮬레이터 미실행 (클릭하여 선택)';
                    simActiveDot.className = 'chip-dot dot-offline';
                }

                // Connection status
                if (t.connected) {
                    connDot.className = 'chip-dot dot-online';
                    connStatusText.textContent = `3D 시뮬레이터 연결됨 (${t.active_sim_name || 'AirSim'})`;
                } else if (t.simulated) {
                    connDot.className = 'chip-dot dot-simulated';
                    connStatusText.textContent = '가상 데모 모드 (맵 미실행)';
                }

                // Armed & API badges
                if (t.armed) {
                    armedBadge.className = 'badge-pill pill-active';
                    armedBadge.textContent = t.landed_state === 'Flying' ? 'FLYING (ARMED)' : 'ARMED';
                } else {
                    armedBadge.className = 'badge-pill pill-neutral';
                    armedBadge.textContent = 'DISARMED (LANDED)';
                }

                if (t.api_control) {
                    apiBadge.className = 'badge-pill pill-active';
                    apiBadge.textContent = 'API: ACTIVE';
                } else {
                    apiBadge.className = 'badge-pill pill-neutral';
                    apiBadge.textContent = 'API: OFF';
                }

                // Altitude
                const alt = Math.abs(t.z);
                valAltitude.textContent = alt.toFixed(1);
                const altPercent = Math.min(100, (alt / 30) * 100);
                barAltitude.style.width = `${altPercent}%`;

                // Speed
                valSpeed.textContent = t.speed.toFixed(1);
                const speedPercent = Math.min(100, (t.speed / 15) * 100);
                barSpeed.style.width = `${speedPercent}%`;

                // Position Coordinates
                valX.textContent = t.x.toFixed(2);
                valY.textContent = t.y.toFixed(2);
                valZ.textContent = t.z.toFixed(2);

                // Attitude
                valPitch.textContent = `${t.pitch.toFixed(1)}°`;
                valRoll.textContent = `${t.roll.toFixed(1)}°`;
                valYaw.textContent = `${t.yaw.toFixed(1)}°`;

                // GPS
                valGps.textContent = `LAT: ${t.lat.toFixed(6)} | LON: ${t.lon.toFixed(6)} | ALT: ${t.alt.toFixed(1)}m`;

                // Live Tactical Aviation HUD Rendering
                renderAviationHud(t);

                // Live Camera Frame Stream
                if (data.frame && data.frame.length > 0) {
                    fpvImg.src = data.frame.startsWith('data:') ? data.frame : `data:image/png;base64,${data.frame}`;
                    fpvImg.style.display = 'block';
                    cameraFallback.style.display = 'none';

                    frameCount++;
                    if (nowMs - lastFpsCheck >= 1000) {
                        cameraFps.textContent = `FPS: ${frameCount}`;
                        frameCount = 0;
                        lastFpsCheck = nowMs;
                    }
                } else {
                    if (!t.connected) {
                        fpvImg.style.display = 'none';
                        cameraFallback.style.display = 'flex';
                        cameraFps.textContent = 'FPS: --';
                        if (cameraLatency) cameraLatency.textContent = 'Latency: -- ms';
                    }
                }
            } catch (e) {
                console.error("Error parsing telemetry WebSocket data:", e);
            }
        };

        ws.onclose = () => {
            connDot.className = 'chip-dot dot-offline';
            connStatusText.textContent = '웹서버 연결 해제 (재연결 시도 중)';
            appendLog('웹서버 연결 끊김. 3초 후 재연결...', 'warn');
            setTimeout(connectWebSocket, 3000);
        };

        ws.onerror = (err) => {
            console.error("WebSocket Error:", err);
        };
    }

    connectWebSocket();

    // 9. API Action Helper
    async function sendApiCommand(endpoint, body = {}, actionName = '') {
        const meta = DRONE_META[currentDroneId] || DRONE_META['Drone1'];
        body.drone_id = currentDroneId;

        const curSnapshot = `[${meta.tag} | 고도: ${valAltitude.textContent}m | X:${valX.textContent}, Y:${valY.textContent}, Z:${valZ.textContent}]`;
        appendLog(`[명령 전송] ${actionName} (${meta.tag}) ${curSnapshot}...`, 'cmd');
        try {
            const res = await fetch(endpoint, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(body)
            });
            const data = await res.json();

            if (data.status === 'success') {
                appendLog(`[응답 완료] ${data.message}`, 'success');
            } else if (data.status === 'simulated') {
                appendLog(`[응답 완료] ${data.message}`, 'warn');
            } else {
                appendLog(`[응답 실패] ${data.message}`, 'warn');
            }
        } catch (err) {
            appendLog(`[통신 오류] 백엔드 서버 응답 실패: ${err}`, 'warn');
        }
    }

    // 10. Speed Rate Switcher (LOW / MID / HIGH)
    function setSpeedRate(rateKey) {
        if (!SPEED_RATES[rateKey]) return;
        currentRateKey = rateKey;

        document.querySelectorAll('.btn-rate').forEach(btn => {
            btn.classList.toggle('active', btn.dataset.rate === rateKey);
        });

        const rate = SPEED_RATES[rateKey];
        appendLog(`조종 속도 모드 변경: [${rate.name}] ${rate.label}`, 'info');

        stickLeftInfo.textContent = `[${rate.name}] Vz: 0.0 m/s | Yaw: 0°/s`;
        stickRightInfo.textContent = `[${rate.name}] Vx: 0.0 m/s | Vy: 0.0 m/s`;
    }

    if (btnRateLow) btnRateLow.addEventListener('click', () => setSpeedRate('low'));
    if (btnRateMid) btnRateMid.addEventListener('click', () => setSpeedRate('mid'));
    if (btnRateHigh) btnRateHigh.addEventListener('click', () => setSpeedRate('high'));

    // 11. Dual Joystick Virtual Flight Controller Engine
    function setupVirtualJoystick(padEl, knobEl, onChange) {
        const maxRadius = 45;
        let isDragging = false;

        function handlePointer(clientX, clientY) {
            const rect = padEl.getBoundingClientRect();
            const centerX = rect.left + rect.width / 2;
            const centerY = rect.top + rect.height / 2;

            let dx = clientX - centerX;
            let dy = clientY - centerY;
            const dist = Math.sqrt(dx * dx + dy * dy);

            if (dist > maxRadius) {
                dx = (dx / dist) * maxRadius;
                dy = (dy / dist) * maxRadius;
            }

            knobEl.style.transform = `translate(${dx}px, ${dy}px)`;

            const normX = dx / maxRadius;
            const normY = dy / maxRadius;
            onChange(normX, normY);
        }

        function onPointerDown(e) {
            isDragging = true;
            padEl.setPointerCapture(e.pointerId);
            handlePointer(e.clientX, e.clientY);
        }

        function onPointerMove(e) {
            if (!isDragging) return;
            handlePointer(e.clientX, e.clientY);
        }

        function onPointerUp(e) {
            if (!isDragging) return;
            isDragging = false;
            knobEl.style.transform = 'translate(0px, 0px)';
            onChange(0, 0);
        }

        padEl.addEventListener('pointerdown', onPointerDown);
        padEl.addEventListener('pointermove', onPointerMove);
        padEl.addEventListener('pointerup', onPointerUp);
        padEl.addEventListener('pointercancel', onPointerUp);
    }

    setupVirtualJoystick(stickLeftPad, stickLeftKnob, (x, y) => {
        stickLeft.x = x;
        stickLeft.y = y;
        updateJoystickStream();
    });

    setupVirtualJoystick(stickRightPad, stickRightKnob, (x, y) => {
        stickRight.x = x;
        stickRight.y = y;
        updateJoystickStream();
    });

    // Keyboard Shortcuts Handler (F1~F4: Drone Select, F5: Formation Assemble, F6: Following Mode, WASD + Arrows: Flight, 1/2/3: Speed, H: HUD)
    window.addEventListener('keydown', (e) => {
        if (e.code === 'F1') {
            e.preventDefault();
            selectTargetDrone('Drone1');
            return;
        }
        if (e.code === 'F2') {
            e.preventDefault();
            selectTargetDrone('Drone2');
            return;
        }
        if (e.code === 'F3') {
            e.preventDefault();
            selectTargetDrone('Drone3');
            return;
        }
        if (e.code === 'F4') {
            e.preventDefault();
            selectTargetDrone('Drone4');
            return;
        }
        if (e.code === 'F5') {
            e.preventDefault();
            triggerFormationAssembly();
            return;
        }
        if (e.code === 'F6') {
            e.preventDefault();
            setFollowingMode(!followingModeEnabled);
            return;
        }

        if (e.code === 'KeyH') {
            e.preventDefault();
            toggleHud();
            return;
        }

        if (e.code === 'Digit1') {
            e.preventDefault();
            setSpeedRate('low');
            return;
        }
        if (e.code === 'Digit2') {
            e.preventDefault();
            setSpeedRate('mid');
            return;
        }
        if (e.code === 'Digit3') {
            e.preventDefault();
            setSpeedRate('high');
            return;
        }

        if (['KeyW', 'KeyS', 'KeyA', 'KeyD', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'].includes(e.code)) {
            e.preventDefault();
            activeKeys[e.code] = true;
            evalKeyboardInput();
        }
    });

    window.addEventListener('keyup', (e) => {
        if (activeKeys[e.code]) {
            delete activeKeys[e.code];
            evalKeyboardInput();
        }
    });

    function evalKeyboardInput() {
        let lx = 0, ly = 0;
        let rx = 0, ry = 0;

        if (activeKeys['KeyW']) ly -= 1.0;
        if (activeKeys['KeyS']) ly += 1.0;
        if (activeKeys['KeyA']) lx -= 1.0;
        if (activeKeys['KeyD']) lx += 1.0;

        if (activeKeys['ArrowUp']) ry -= 1.0;
        if (activeKeys['ArrowDown']) ry += 1.0;
        if (activeKeys['ArrowLeft']) rx -= 1.0;
        if (activeKeys['ArrowRight']) rx += 1.0;

        const rate = SPEED_RATES[currentRateKey];
        const visualScale = 25 + (rate.multiplier * 20);
        stickLeftKnob.style.transform = `translate(${lx * visualScale}px, ${ly * visualScale}px)`;
        stickRightKnob.style.transform = `translate(${rx * visualScale}px, ${ry * visualScale}px)`;

        stickLeft.x = lx;
        stickLeft.y = ly;
        stickRight.x = rx;
        stickRight.y = ry;

        if (activeKeys['Space']) {
            stickLeft = { x: 0, y: 0 };
            stickRight = { x: 0, y: 0 };
            stickLeftKnob.style.transform = 'translate(0px, 0px)';
            stickRightKnob.style.transform = 'translate(0px, 0px)';
            sendApiCommand('/api/joystick', { vx: 0, vy: 0, vz: 0, yaw_rate: 0, duration: 0.2 }, '긴급 제자리 정지 (Hover)');
            return;
        }

        updateJoystickStream();
    }

    function sendJoystickPacket() {
        const rate = SPEED_RATES[currentRateKey];
        const mult = rate.multiplier;

        const vx = -stickRight.y * (6.0 * mult);
        const vy = stickRight.x * (6.0 * mult);
        const vz = stickLeft.y * (3.5 * mult);
        const yaw_rate = stickLeft.x * (50.0 * mult);

        stickLeftInfo.textContent = `[${rate.name}] Vz: ${(-vz).toFixed(1)} m/s | Yaw: ${yaw_rate.toFixed(0)}°/s`;
        stickRightInfo.textContent = `[${rate.name}] Vx: ${vx.toFixed(1)} m/s | Vy: ${vy.toFixed(1)} m/s`;

        fetch('/api/joystick', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                drone_id: currentDroneId,
                vx: vx,
                vy: vy,
                vz: vz,
                yaw_rate: yaw_rate,
                duration: 0.25
            })
        }).catch(() => {});
    }

    function updateJoystickStream() {
        const isDeflected = (Math.abs(stickLeft.x) > 0.05 || Math.abs(stickLeft.y) > 0.05 ||
                             Math.abs(stickRight.x) > 0.05 || Math.abs(stickRight.y) > 0.05);

        if (isDeflected) {
            if (!isJoystickActive) {
                isJoystickActive = true;
                sendJoystickPacket();
                joystickInterval = setInterval(sendJoystickPacket, 120);
            }
        } else {
            if (isJoystickActive) {
                isJoystickActive = false;
                clearInterval(joystickInterval);
                joystickInterval = null;
                sendJoystickPacket();
            }
        }
    }

    // 12. Action Button Click Handlers
    btnTakeoff.addEventListener('click', () => sendApiCommand('/api/takeoff', {}, '수직 이륙 (Takeoff)'));
    btnLand.addEventListener('click', () => sendApiCommand('/api/land', {}, '안전 착륙 (Land)'));
    btnRotate.addEventListener('click', () => sendApiCommand('/api/rotate', {}, '360도 스캔 회전'));
    btnEmergency.addEventListener('click', () => sendApiCommand('/api/joystick', { vx: 0, vy: 0, vz: 0, yaw_rate: 0, duration: 0.2 }, '제자리 정지 (Hover)'));
    btnSnapshot.addEventListener('click', () => sendApiCommand('/api/capture', {}, '고해상도 사진 캡처'));
    btnReconnect.addEventListener('click', () => sendApiCommand('/api/connect', {}, '3D 시뮬레이터 연결 상태 점검'));

    if (btnRth) {
        btnRth.addEventListener('click', () => sendApiCommand('/api/rth', {}, 'RTH 홈 자동 복귀 & 착륙'));
    }

    if (btnReset) {
        btnReset.addEventListener('click', () => sendApiCommand('/api/reset', {}, '시뮬레이션 및 편대 위치 리셋'));
    }

    // 13. Diagnostic Log Actions (Copy & Export)
    btnClearLog.addEventListener('click', () => {
        logTerminal.innerHTML = '';
        rawLogHistory = [];
        appendLog('로그 콘솔이 초기화되었습니다.', 'info');
    });

    btnCopyLog.addEventListener('click', () => {
        const text = rawLogHistory.join('\n');
        navigator.clipboard.writeText(text).then(() => {
            appendLog('전체 진단 로그가 클립보드에 복사되었습니다.', 'success');
        }).catch(err => {
            appendLog(`클립보드 복사 실패: ${err}`, 'warn');
        });
    });

    btnExportLog.addEventListener('click', () => {
        const text = rawLogHistory.join('\n');
        const blob = new Blob([text], { type: 'text/plain;charset=utf-8' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = `airsim_fleet4_log_${Date.now()}.txt`;
        a.click();
        appendLog(`진단 로그 파일 다운로드 완료 (${a.download})`, 'success');
    });
});
