"""
Playwright UI Regression Test - AirSim 4-UAV Fleet Command Cockpit

Drives the real browser UI (public/index.html + app.js) against the running
FastAPI backend (server.py) and verifies that clicks/keyboard shortcuts
produce the expected DOM/state changes.

Does NOT require the AirSim/Unreal simulator executable to be running -
server.py falls back to a built-in "simulated" demo telemetry mode whenever
no simulator is connected, and that is enough state for full UI coverage.
If a simulator IS actually connected, the same assertions still hold.

Usage:
    python server.py                    # start the backend in one terminal
    python test_ui_playwright.py        # run headless (default)
    python test_ui_playwright.py --headed   # watch the browser drive itself
"""
import sys
import io
import os
import re
import json
import time
import argparse
import urllib.request

if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

try:
    from playwright.sync_api import sync_playwright, expect
except ImportError:
    print("[ERROR] playwright가 설치되어 있지 않습니다.", flush=True)
    print("  설치: pip install playwright && playwright install chromium", flush=True)
    sys.exit(1)

BASE_URL = "http://127.0.0.1:8000"
REPORT_PATH = "ui_test_report.json"
ACTIVE_CLASS_RE = re.compile(r"\bactive\b")

DRONES = [
    {"id": "Drone1", "tab": "#btn-drone-1", "card": "#fleet-card-drone1", "tag": "ALPHA-01", "key": "F1"},
    {"id": "Drone2", "tab": "#btn-drone-2", "card": "#fleet-card-drone2", "tag": "BRAVO-02", "key": "F2"},
    {"id": "Drone3", "tab": "#btn-drone-3", "card": "#fleet-card-drone3", "tag": "CHARLIE-03", "key": "F3"},
    {"id": "Drone4", "tab": "#btn-drone-4", "card": "#fleet-card-drone4", "tag": "DELTA-04", "key": "F4"},
]

SPAWN_OFFSETS = {
    "Drone1": (0.0, 0.0, 0.0),
    "Drone2": (0.0, 3.5, 0.0),
    "Drone3": (0.0, 7.0, 0.0),
    "Drone4": (0.0, 10.5, 0.0),
}


def check_server_alive() -> bool:
    try:
        with urllib.request.urlopen(f"{BASE_URL}/api/status", timeout=3) as res:
            return res.status == 200
    except Exception:
        return False


class UITestRunner:
    def __init__(self, page):
        self.page = page
        self.steps = []
        self.all_passed = True

    def step(self, name, fn):
        print(f"\n[TEST] {name}", flush=True)
        entry = {"name": name, "passed": False, "error": None}
        try:
            fn()
            entry["passed"] = True
            print("  - PASS", flush=True)
        except Exception as e:
            entry["error"] = str(e)
            self.all_passed = False
            print(f"  - FAIL: {e}", flush=True)
        self.steps.append(entry)
        return entry["passed"]

    # ------------------------------------------------------------------
    # Individual UI checks
    # ------------------------------------------------------------------
    def check_page_loads(self):
        p = self.page
        p.goto(BASE_URL, wait_until="domcontentloaded")
        expect(p.locator(".brand-title")).to_have_text("AIRSIM 4-UAV FLEET COCKPIT")
        # WebSocket telemetry should replace the initial "connecting" placeholder
        expect(p.locator("#conn-status-text")).not_to_have_text("시뮬레이터 연결 확인 중...", timeout=10000)

    def check_drone_selection_via_tabs(self):
        p = self.page
        for d in DRONES:
            p.locator(d["tab"]).click()
            expect(p.locator(d["tab"])).to_have_class(ACTIVE_CLASS_RE)
            expect(p.locator(d["card"])).to_have_class(ACTIVE_CLASS_RE)
            expect(p.locator("#coord-drone-tag")).to_have_text(d["tag"])

    def check_drone_selection_via_fleet_cards(self):
        p = self.page
        for d in reversed(DRONES):
            p.locator(d["card"]).click()
            expect(p.locator(d["tab"])).to_have_class(ACTIVE_CLASS_RE)
            expect(p.locator(d["card"])).to_have_class(ACTIVE_CLASS_RE)

    def check_drone_selection_via_keyboard(self):
        p = self.page
        for d in DRONES:
            p.keyboard.press(d["key"])
            expect(p.locator(d["tab"])).to_have_class(ACTIVE_CLASS_RE)
        # leave Drone1 selected for the flight-command checks below
        p.locator("#btn-drone-1").click()
        expect(p.locator("#btn-drone-1")).to_have_class(ACTIVE_CLASS_RE)

    def check_speed_rate_buttons(self):
        p = self.page
        for rate_id, label in [("#btn-rate-low", "LOW"), ("#btn-rate-high", "HIGH"), ("#btn-rate-mid", "MID")]:
            p.locator(rate_id).click()
            expect(p.locator(rate_id)).to_have_class(ACTIVE_CLASS_RE)
            expect(p.locator("#stick-left-info")).to_contain_text(label)

    def _last_log_text(self):
        return self.page.locator(".log-entry .log-msg").last.inner_text()

    def check_takeoff(self):
        p = self.page
        before = p.locator(".log-entry").count()
        p.locator("#btn-takeoff").click()
        expect(p.locator(".log-entry")).to_have_count(before + 2)  # [명령 전송] + [응답 완료]
        msg = self._last_log_text()
        assert "실패" not in msg and "오류" not in msg, f"이륙 명령이 실패로 응답함: {msg}"
        expect(p.locator("#armed-badge")).to_have_text(re.compile("ARMED"), timeout=5000)
        expect(p.locator("#fleet-state-drone1")).to_have_text("Flying", timeout=5000)
        expect(p.locator("#tab-status-drone1")).to_have_text("Flying", timeout=5000)

    def check_rotate(self):
        p = self.page
        before = p.locator(".log-entry").count()
        p.locator("#btn-rotate").click()
        expect(p.locator(".log-entry")).to_have_count(before + 2)
        msg = self._last_log_text()
        assert "실패" not in msg and "오류" not in msg, f"회전 명령이 실패로 응답함: {msg}"

    def check_emergency_hover(self):
        p = self.page
        before = p.locator(".log-entry").count()
        p.locator("#btn-emergency").click()
        expect(p.locator(".log-entry")).to_have_count(before + 2)
        msg = self._last_log_text()
        assert "실패" not in msg and "오류" not in msg, f"긴급 정지 명령이 실패로 응답함: {msg}"

    def check_land(self):
        p = self.page
        p.locator("#btn-land").click()
        expect(p.locator("#armed-badge")).to_have_text("DISARMED (LANDED)", timeout=5000)
        expect(p.locator("#fleet-state-drone1")).to_have_text("Landed", timeout=5000)

    def check_rth(self):
        p = self.page
        p.locator("#btn-takeoff").click()
        expect(p.locator("#fleet-state-drone1")).to_have_text("Flying", timeout=5000)
        before = p.locator(".log-entry").count()
        p.locator("#btn-rth").click()
        expect(p.locator(".log-entry")).to_have_count(before + 2)
        msg = self._last_log_text()
        assert "실패" not in msg and "오류" not in msg, f"RTH 명령이 실패로 응답함: {msg}"
        expect(p.locator("#fleet-state-drone1")).to_have_text("Landed", timeout=5000)

    def check_reset(self):
        p = self.page
        p.locator("#btn-reset").click()
        for d_id, (x, y, z) in SPAWN_OFFSETS.items():
            idx = d_id[-1]
            expect(p.locator(f"#fleet-state-drone{idx}")).to_have_text("Landed", timeout=5000)
            expect(p.locator(f"#fleet-pos-drone{idx}")).to_have_text(
                f"({x:.1f}, {y:.1f}, {z:.1f})", timeout=5000
            )

    def check_formation_assemble(self):
        p = self.page
        p.locator("#btn-formation-assemble").click()
        expect(p.locator(".log-entry.log-success").last).to_contain_text("편대", timeout=8000)
        for idx in ["1", "2", "3", "4"]:
            expect(p.locator(f"#fleet-state-drone{idx}")).to_have_text("Flying", timeout=8000)

    def check_following_mode_toggle(self):
        p = self.page
        expect(p.locator("#btn-following-toggle")).not_to_have_class(ACTIVE_CLASS_RE)
        expect(p.locator("#following-toggle-text")).to_contain_text("OFF")

        p.locator("#btn-following-toggle").click()
        expect(p.locator("#btn-following-toggle")).to_have_class(ACTIVE_CLASS_RE, timeout=5000)
        expect(p.locator("#following-toggle-text")).to_contain_text("ON")
        # turning it on should auto-select Alpha (the drone you fly while the others autopilot)
        expect(p.locator("#btn-drone-1")).to_have_class(ACTIVE_CLASS_RE)

        p.locator("#btn-following-toggle").click()
        expect(p.locator("#btn-following-toggle")).not_to_have_class(ACTIVE_CLASS_RE, timeout=5000)
        expect(p.locator("#following-toggle-text")).to_contain_text("OFF")

    def check_fleet_bulk_land_then_takeoff(self):
        p = self.page
        p.locator("#btn-fleet-all-land").click()
        for idx in ["1", "2", "3", "4"]:
            expect(p.locator(f"#fleet-state-drone{idx}")).to_have_text("Landed", timeout=8000)

        p.locator("#btn-fleet-all-takeoff").click()
        for idx in ["1", "2", "3", "4"]:
            expect(p.locator(f"#fleet-state-drone{idx}")).to_have_text("Flying", timeout=8000)

        # cleanup: leave the fleet landed for subsequent runs
        p.locator("#btn-reset").click()
        for idx in ["1", "2", "3", "4"]:
            expect(p.locator(f"#fleet-state-drone{idx}")).to_have_text("Landed", timeout=8000)

    def check_log_clear(self):
        p = self.page
        p.locator("#btn-clear-log").click()
        expect(p.locator(".log-entry")).to_have_count(1)
        expect(p.locator(".log-entry .log-msg").last).to_contain_text("초기화")

    def check_log_export_download(self):
        p = self.page
        with p.expect_download() as dl_info:
            p.locator("#btn-export-log").click()
        download = dl_info.value
        assert download.suggested_filename.startswith("airsim_fleet4_log_"), download.suggested_filename
        assert download.suggested_filename.endswith(".txt"), download.suggested_filename

    def check_log_copy_clipboard(self):
        p = self.page
        p.context.grant_permissions(["clipboard-read", "clipboard-write"])
        p.locator("#btn-copy-log").click()
        expect(p.locator(".log-entry .log-msg").last).to_contain_text("클립보드", timeout=5000)
        clip_text = p.evaluate("navigator.clipboard.readText()")
        assert len(clip_text) > 0, "클립보드가 비어 있음"

    def check_simulator_modal(self):
        p = self.page
        p.locator("#btn-open-sim-modal").click()
        expect(p.locator("#sim-modal")).to_be_visible()
        expect(p.locator("#sim-cards-container .sim-card")).to_have_count(4, timeout=8000)
        p.locator("#btn-close-sim-modal").click()
        expect(p.locator("#sim-modal")).to_be_hidden()

        p.locator("#btn-open-sim-modal").click()
        expect(p.locator("#sim-modal")).to_be_visible()
        p.locator("#btn-cancel-sim-modal").click()
        expect(p.locator("#sim-modal")).to_be_hidden()

    def check_hud_toggle(self):
        p = self.page
        expect(p.locator("#hud-toggle-text")).to_contain_text("ON")
        p.keyboard.press("h")
        expect(p.locator("#hud-toggle-text")).to_contain_text("OFF")
        p.keyboard.press("h")
        expect(p.locator("#hud-toggle-text")).to_contain_text("ON")


def run(headed: bool):
    if not check_server_alive():
        print(f"[ERROR] 백엔드 서버({BASE_URL})가 응답하지 않습니다.", flush=True)
        print("  먼저 다른 터미널에서 'python server.py'를 실행하세요.", flush=True)
        sys.exit(1)

    print("=" * 80, flush=True)
    print("[Playwright UI 회귀 테스트] AirSim 4-UAV Fleet Cockpit", flush=True)
    print("=" * 80, flush=True)

    with sync_playwright() as pw:
        browser = pw.chromium.launch(headless=not headed)
        page = browser.new_page(viewport={"width": 1600, "height": 950})
        runner = UITestRunner(page)

        runner.step("페이지 로드 및 WebSocket 텔레메트리 연결", runner.check_page_loads)
        runner.step("드론 선택 - 상단 탭(F1~F4 버튼) 클릭", runner.check_drone_selection_via_tabs)
        runner.step("드론 선택 - 편대 현황 카드 클릭", runner.check_drone_selection_via_fleet_cards)
        runner.step("드론 선택 - 키보드 단축키 (F1~F4)", runner.check_drone_selection_via_keyboard)
        runner.step("조종 속도 감도 전환 (LOW/MID/HIGH)", runner.check_speed_rate_buttons)
        runner.step("Drone1 수직 이륙 명령", runner.check_takeoff)
        runner.step("360도 스캔 회전 명령", runner.check_rotate)
        runner.step("긴급 정지(제자리 호버) 명령", runner.check_emergency_hover)
        runner.step("안전 착륙 명령", runner.check_land)
        runner.step("RTH 자동 복귀 명령", runner.check_rth)
        runner.step("편대 위치 초기화(Reset)", runner.check_reset)
        runner.step("편대 집결(알파 호출, Formation Assemble)", runner.check_formation_assemble)
        runner.step("Following Mode 토글 (F6)", runner.check_following_mode_toggle)
        runner.step("전체 편대 동시 착륙 -> 동시 이륙", runner.check_fleet_bulk_land_then_takeoff)
        runner.step("미션 로그 - 콘솔 초기화", runner.check_log_clear)
        runner.step("미션 로그 - 파일 내보내기(다운로드)", runner.check_log_export_download)
        runner.step("미션 로그 - 클립보드 복사", runner.check_log_copy_clipboard)
        runner.step("시뮬레이터 맵 선택 모달 열기/닫기", runner.check_simulator_modal)
        runner.step("HUD 오버레이 토글 (단축키 H)", runner.check_hud_toggle)

        browser.close()

    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "total_passed": runner.all_passed,
        "steps": runner.steps,
    }
    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    print("\n" + "=" * 80, flush=True)
    print("[Playwright UI 회귀 테스트 결과 보고]", flush=True)
    passed = sum(1 for s in runner.steps if s["passed"])
    print(f"총 {len(runner.steps)}개 중 {passed}개 통과", flush=True)
    for s in runner.steps:
        sym = "PASS" if s["passed"] else "FAIL"
        print(f"  - [{sym}] {s['name']}" + (f" :: {s['error']}" if s["error"] else ""), flush=True)
    print(f"\n최종 결과: {'ALL TESTS PASSED' if runner.all_passed else 'SOME TESTS FAILED'}", flush=True)
    print("=" * 80, flush=True)

    sys.exit(0 if runner.all_passed else 1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AirSim Fleet Cockpit Playwright UI regression test")
    parser.add_argument("--headed", action="store_true", help="브라우저 창을 띄워서 실행 (기본은 headless)")
    args = parser.parse_args()
    run(headed=args.headed)
