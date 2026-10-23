"""Render the simulated demo logs to web MP4s with a poster frame each.

Frames are drawn by render.html (three.js) in headless Chrome, one per logged sample, then
encoded with ffmpeg. Writes assets/videos/vehicles/<id>.mp4 and assets/images/vehicles/<id>.jpg.

    python tools/vehicle_demos/render.py [--only brake] [--size 1280x720]
"""

import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

from playwright.sync_api import sync_playwright

HERE = Path(__file__).resolve().parent
SITE = HERE.parents[1]
HOLD = 1.0  # seconds of still frame at the end


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", type=Path, default=HERE / "logs")
    ap.add_argument("--only", default="")
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--poster", type=float, default=0.55, help="poster frame as a fraction of the clip")
    ap.add_argument("--chrome", default="/usr/bin/google-chrome")
    args = ap.parse_args()
    w, h = map(int, args.size.split("x"))
    videos, posters = SITE / "assets/videos/vehicles", SITE / "assets/images/vehicles"
    videos.mkdir(parents=True, exist_ok=True)
    posters.mkdir(parents=True, exist_ok=True)
    logs = sorted(args.logs.glob("*.json"))
    with sync_playwright() as p:
        browser = p.chromium.launch(
            executable_path=args.chrome, args=["--use-angle=swiftshader", "--enable-unsafe-swiftshader"]
        )
        for log in logs:
            name = log.stem
            if args.only and name not in args.only.split(","):
                continue
            page = browser.new_page(viewport={"width": w, "height": h}, device_scale_factor=1)
            errors = []
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto((HERE / "render.html").as_uri())
            page.wait_for_function("window.__ready === true", timeout=60000)
            info = page.evaluate("d => window.__demo.load(d)", json.loads(log.read_text()))
            work = Path(tempfile.mkdtemp(prefix=f"veh_{name}_"))
            n = info["n"]
            for k in range(n + int(HOLD * info["hz"])):
                page.evaluate("k => window.__demo.frame(k)", k)
                page.screenshot(path=str(work / f"{k:05d}.jpg"), type="jpeg", quality=93)
            if errors:
                raise SystemExit(f"{name}: page errors {errors[:3]}")
            shutil.copy(work / f"{int(args.poster * (n - 1)):05d}.jpg", posters / f"{name}.jpg")
            target = videos / f"{name}.mp4"
            subprocess.run(
                ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(info["hz"]), "-i", str(work / "%05d.jpg"),
                 "-c:v", "libx264", "-preset", "slow", "-crf", "24", "-pix_fmt", "yuv420p", "-an",
                 "-movflags", "+faststart", str(target)],
                check=True,
            )
            shutil.rmtree(work)
            page.close()
            print(f"{target.relative_to(SITE)}  {n / info['hz']:.1f} s  {target.stat().st_size / 1e6:.2f} MB", flush=True)
        browser.close()


if __name__ == "__main__":
    main()
