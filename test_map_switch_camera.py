"""
Playwright regression test for the "camera/control dead after switching maps" bug.

Reproduces the reported failure end-to-end through the real browser UI:
  1. Launch a simulator map from the UI's map-selector modal.
  2. Verify the FPV camera is actually streaming LIVE frames (not just that the
     status chip claims "connected") and that takeoff/land control works.
  3. Switch to a DIFFERENT map through the same UI flow.
  4. Verify camera + control work there too.
  5. Repeat once more to make sure the fix holds across repeated switches,
     not just the first one.

Requires the real AirSim/Unreal simulator executables (assets/) and a running
backend (`python server.py`). Each map takes real time to boot - this is a
slow, heavyweight integration test, not something to run on every commit.

Usage:
    python server.py
    python test_map_switch_camera.py [--headed]
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
    print("[ERROR] playwright가 설치되어 있지 않습니다. pip install playwright && playwright install chromium", flush=True)
    sys.exit(1)

BASE_URL = "http://127.0.0.1:8000"
REPORT_PATH = "map_switch_test_report.json"

# Order mirrors the reported bug: first map should always work: the interesting
# question is whether the 2nd/3rd map (a *different* map each time) also works.
MAP_SEQUENCE = [
    {"id": "blocks", "name": "Blocks"},
    {"id": "park", "name": "AbandonedPark"},
    {"id": "city", "name": "CityEnviron"},
]

CONNECT_TIMEOUT_MS = 120_000   # Unreal cold boot can be slow, especially for large maps
CAMERA_TIMEOUT_MS = 60_000
CONTROL_TIMEOUT_MS = 30_000    # real landAsync() physics can take longer than the demo-mode instant fallback


def check_server_alive() -> bool:
    try:
        with urllib.request.urlopen(f"{BASE_URL}/api/status", timeout=3) as res:
            return res.status == 200
    except Exception:
        return False


def verify_one_map(page, map_info, step_no):
    """Launch map_info via the UI and verify camera + control both actually work."""
    detail = {}

    # 1. Open the selector modal and launch this map
    page.locator("#btn-open-sim-modal").click()
    expect(page.locator("#sim-modal")).to_be_visible()
    launch_btn = page.locator(f"#btn-launch-{map_info['id']}")
    expect(launch_btn).to_be_visible(timeout=10_000)
    t_launch = time.time()
    launch_btn.click()

    # 2. Wait for the connection status chip to confirm a real AirSim connection
    expect(page.locator("#conn-status-text")).to_contain_text("연결됨", timeout=CONNECT_TIMEOUT_MS)
    detail["connect_elapsed_sec"] = round(time.time() - t_launch, 1)

    # 3. Camera must show a REAL, LIVE frame - not just the fallback placeholder
    fpv_img = page.locator("#fpv-stream-img")
    expect(fpv_img).to_be_visible(timeout=CAMERA_TIMEOUT_MS)
    expect(fpv_img).to_have_attribute("src", re.compile(r"^data:image"), timeout=CAMERA_TIMEOUT_MS)
    expect(page.locator("#camera-fallback")).to_be_hidden(timeout=5_000)

    # The functional bar for "camera works": real frame bytes must keep changing.
    # Give a freshly-switched (esp. geometrically heavy) map a few retries, since
    # the render target and the connection itself can both take a moment to
    # settle right after a switch.
    src_1 = fpv_img.get_attribute("src")
    frame_updated = False
    for _ in range(4):
        page.wait_for_timeout(2500)
        src_2 = fpv_img.get_attribute("src")
        if src_1 != src_2:
            frame_updated = True
            break
        src_1 = src_2
    assert frame_updated, "카메라 프레임이 10초 동안 전혀 갱신되지 않음 (스트림이 멈춰있음 - '먹통' 재현됨)"
    detail["camera_frame_updates"] = True

    # The on-screen FPS counter is a cosmetic widget on top of that - useful to
    # record, but not something a slow first connection should hard-fail on.
    detail["camera_fps_text"] = page.locator("#camera-fps").inner_text()

    # 4. Control must actually work: takeoff should raise altitude and flip state
    page.locator("#btn-takeoff").click()
    expect(page.locator("#armed-badge")).to_have_text(re.compile("ARMED"), timeout=CONTROL_TIMEOUT_MS)
    expect(page.locator("#fleet-state-drone1")).to_have_text("Flying", timeout=CONTROL_TIMEOUT_MS)

    page.wait_for_function(
        "() => { const v = parseFloat(document.getElementById('val-altitude').innerText); return v > 0.3; }",
        timeout=CONTROL_TIMEOUT_MS,
    )
    detail["altitude_after_takeoff"] = page.locator("#val-altitude").inner_text()

    page.locator("#btn-land").click()
    expect(page.locator("#fleet-state-drone1")).to_have_text("Landed", timeout=CONTROL_TIMEOUT_MS)

    return detail


def run(headed: bool):
    if not check_server_alive():
        print(f"[ERROR] 백엔드 서버({BASE_URL})가 응답하지 않습니다. 먼저 'python server.py'를 실행하세요.", flush=True)
        sys.exit(1)

    print("=" * 80, flush=True)
    print("[맵 전환 카메라/조종 회귀 테스트] - 실제 시뮬레이터로 재현 + 검증", flush=True)
    print("=" * 80, flush=True)

    results = []
    all_passed = True

    with sync_playwright() as pw:
        browser = pw.chromium.launch(headless=not headed)
        page = browser.new_page(viewport={"width": 1600, "height": 950})
        page.goto(BASE_URL, wait_until="domcontentloaded")

        for i, map_info in enumerate(MAP_SEQUENCE, start=1):
            label = f"[{i}/{len(MAP_SEQUENCE)}] {map_info['name']} ({map_info['id']}) 전환 후 카메라/조종 검증"
            print(f"\n[TEST] {label}", flush=True)
            entry = {"step": i, "map": map_info["id"], "passed": False, "error": None, "detail": {}}
            try:
                entry["detail"] = verify_one_map(page, map_info, i)
                entry["passed"] = True
                print(f"  - PASS | {entry['detail']}", flush=True)
            except Exception as e:
                entry["error"] = str(e)
                all_passed = False
                print(f"  - FAIL: {e}", flush=True)
            results.append(entry)

        # Cleanup: stop whatever simulator is left running
        try:
            page.locator("#btn-open-sim-modal").click()
            expect(page.locator("#sim-modal")).to_be_visible(timeout=5_000)
            page.locator("#btn-stop-all-sims").click()
            page.wait_for_timeout(1500)
        except Exception:
            pass

        browser.close()

    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "total_passed": all_passed,
        "steps": results,
    }
    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    print("\n" + "=" * 80, flush=True)
    print("[결과 보고]", flush=True)
    for r in results:
        sym = "PASS" if r["passed"] else "FAIL"
        print(f"  - [{sym}] Step {r['step']}: {r['map']}" + (f" :: {r['error']}" if r["error"] else ""), flush=True)
    print(f"\n최종 결과: {'ALL PASSED' if all_passed else 'SOME FAILED'}", flush=True)
    print("=" * 80, flush=True)

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Map-switch camera/control regression test")
    parser.add_argument("--headed", action="store_true")
    args = parser.parse_args()
    run(headed=args.headed)
