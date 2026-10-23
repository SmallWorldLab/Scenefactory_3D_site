"""Simulate the vehicle demo clips with the released SceneFactory-3D vehicle model.

Each scenario runs the release's DrivingPhysics (Newton XPBD + per-wheel tire forces, unchanged)
and writes a compact trajectory log that render.py turns into a video. These are hand-authored
vehicle demos, not benchmark episodes.

    python tools/vehicle_demos/simulate.py --release ~/Github/scenefactory3d [--only brake]
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
LOG_HZ = 30
DRY, LOW = 1.0, 0.18  # the paper's reference and most severe friction levels


def setup(release):
    sys.path.insert(0, str(release))
    import warp as wp

    wp.config.log_level = wp.LOG_WARNING
    from astra_drive.physics import DrivingPhysics
    from astra_drive.terrain import Terrain
    from astra_drive.vehicle import VEHICLES, DriverInput, rotation

    return DrivingPhysics, Terrain, VEHICLES, DriverInput, rotation


def place(sim, rotation, starts):
    """Put vehicle i at (x, y, yaw, speed) resting on its suspension, wheels already rolling."""
    poses, vel = sim.poses, sim.velocities
    for i, (x, y, yaw, v) in enumerate(starts):
        spec = sim.specs[i]
        q = np.array([0, 0, math.sin(yaw / 2), math.cos(yaw / 2)])
        # Raise until every wheel ray reaches the ground at static sag.
        z = max(sim.terrain.height(*(np.array([x, y]) + rotation(q)[:2, :2] @ a[:2])) for a in spec.anchors)
        poses[i] = [x, y, z + spec.ride_height, *q]
        vel[i] = 0
        vel[i, :3] = rotation(q)[:, 0] * v
        sim.vehicles[i].omega[:] = v / spec.radius
    for state in (sim.state, sim.next_state):
        state.body_q.assign(poses.astype(np.float32))
        state.body_qd.assign(vel.astype(np.float32))


def run(sim, policy, duration, rotation, DriverInput):
    """Step at the release's 240 Hz; log pose, steer, wheel spin and suspension at LOG_HZ."""
    frames, every = [], round(1 / sim.DT / LOG_HZ)
    for k in range(int(duration / sim.DT) + 1):
        t = k * sim.DT
        poses, vel = sim.poses, sim.velocities
        speeds = [float(np.dot(vel[i, :3], rotation(poses[i, 3:])[:, 0])) for i in range(len(sim.specs))]
        cmds = [DriverInput(*policy(i, t, poses[i], speeds[i])) for i in range(len(sim.specs))]
        if k % every == 0:
            frames.append(
                {
                    "t": round(t, 4),
                    "cars": [
                        {
                            "p": [round(float(v), 4) for v in poses[i]],
                            "steer": [round(float(a), 4) for a in vh.steer_angles],
                            "spin": [round(float(a), 3) for a in vh.wheel_angle],
                            "susp": [round(float(a), 4) for a in vh.lengths],
                            "omega": [round(float(a), 2) for a in vh.omega],
                            "kmh": round(speeds[i] * 3.6, 2),
                            "cmd": [round(cmds[i].throttle, 2), round(cmds[i].brake, 2), round(cmds[i].steer, 2)],
                        }
                        for i, vh in enumerate(sim.vehicles)
                    ],
                }
            )
        sim.step(cmds)
    return frames


def cruise(target, v, gain=0.5):
    """Throttle/brake for a speed target (m/s)."""
    e = target - v
    return (min(1.0, gain * e), 0.0) if e > 0 else (0.0, min(1.0, -gain * e))


def lateral(y_target, pose, rotation, k=0.12, kd=0.9):
    """Steer to hold a lateral offset with heading damping (straight +x roads)."""
    yaw = math.atan2(rotation(pose[3:])[1, 0], rotation(pose[3:])[0, 0])
    return float(np.clip(k * (y_target - pose[1]) - kd * yaw, -1, 1))


def scenario_brake(mods):
    """Matched counterfactual: same car, same speed, same commands; only the road friction differs."""
    DrivingPhysics, Terrain, VEHICLES, DriverInput, rotation = mods
    gt, trail = VEHICLES
    lanes = [("Astra GT", DRY, -15.0), ("Astra GT", LOW, -5.0), ("Astra Trail", DRY, 5.0), ("Astra Trail", LOW, 15.0)]
    terrain = Terrain(nx=513, ny=89, spacing=0.5, preset="Flat")
    terrain.add_surface("gravel", DRY, shape="rect", x=0.0, y=0.0, w=256.0, h=44.0)
    for _, mu, y in lanes:
        if mu != DRY:
            terrain.add_surface("ice", mu, shape="rect", x=20.0, y=y, w=210.0, h=5.0)
    sim = DrivingPhysics(terrain, specs=(gt, gt, trail, trail), device="cpu")
    v0, brake_x = 60 / 3.6, -70.0
    place(sim, rotation, [(-115.0, y, 0.0, v0) for _, _, y in lanes])
    braking = {}

    def policy(i, t, pose, v):
        y = lanes[i][2]
        if pose[0] < brake_x and i not in braking:
            return (*cruise(v0, v), lateral(y, pose, rotation))
        braking.setdefault(i, t)
        # Emergency stop with a lane-change swerve, identical timing for every car.
        s = t - braking[i]
        steer = 0.3 if s < 0.9 else (-0.3 if s < 1.8 else 0.0)
        return (0.0, 0.0 if v < 0.05 else 0.8, steer)

    frames = run(sim, policy, 13.0, rotation, DriverInput)
    meta = {
        "title": "Emergency stop on dry vs. icy asphalt",
        "caption": "Four copies of the same manoeuvre at 60 km/h: identical speed, brake and steering inputs. Only the road "
        f"friction under the right-hand car of each pair changes (μ {DRY:g} → {LOW:g}).",
        "lanes": [{"label": f"{n} · μ {mu:g}", "y": y} for n, mu, y in lanes],
        "markers": [{"x": brake_x, "label": "brake + swerve"}],
        "camera": {"mode": "overview", "target": [-30, 0, 0], "follow": False},
    }
    return terrain, sim, frames, meta


def scenario_grade(mods):
    """Hill start on an icy 8 % grade: rear-wheel drive vs. all-wheel drive, same half throttle."""
    DrivingPhysics, Terrain, VEHICLES, DriverInput, rotation = mods
    terrain = Terrain(nx=241, ny=41, spacing=0.5, preset="Flat")
    x = terrain.x
    # Flat run-in, 10 m transition, then a constant +8 % grade.
    ramp = np.clip((x + 20) / 10, 0, 1)
    terrain.z = np.where(x < -10, 0.08 * 10 * ramp**2 / 2, 0.08 * (x + 15))
    terrain.add_surface("gravel", DRY, shape="rect", x=0.0, y=0.0, w=120.0, h=20.0)
    terrain.add_surface("ice", 0.25, shape="rect", x=20.0, y=0.0, w=80.0, h=12.0)
    lanes = [("Astra GT · RWD", -2.6), ("Astra Trail · AWD", 2.6)]
    sim = DrivingPhysics(terrain, specs=VEHICLES, device="cpu")
    place(sim, rotation, [(-4.0, y, 0.0, 0.0) for _, y in lanes])

    def policy(i, t, pose, v):
        throttle = 0.0 if t < 1.0 else 0.5
        if v > 50 / 3.6:
            throttle = 0.0
        return (throttle, 1.0 if t < 1.0 else 0.0, lateral(lanes[i][1], pose, rotation))

    frames = run(sim, policy, 9.0, rotation, DriverInput)
    meta = {
        "title": "Hill start on an icy +8 % grade",
        "caption": "Both cars apply the same half throttle from rest on μ 0.25. The rear-driven GT spins its driven wheels; the "
        "all-wheel-drive Trail spreads the same traction demand over four tyres.",
        "lanes": [{"label": n, "y": y} for n, y in lanes],
        "markers": [],
        "camera": {"mode": "chase_pair", "follow": True},
        "show_wheel_speed": True,
    }
    return terrain, sim, frames, meta


def scenario_hills(mods):
    """Rolling terrain at speed: suspension travel, pitch and roll on a per-world heightfield."""
    DrivingPhysics, Terrain, VEHICLES, DriverInput, rotation = mods
    terrain = Terrain(nx=361, ny=81, spacing=0.5, preset="Flat")
    x, y = terrain.x, terrain.y
    terrain.z = (
        0.7 * np.sin(x / 8.0) * np.cos(y / 9.0)
        + 0.16 * np.sin(x / 3.0 + y / 2.5)
        + 0.03 * np.sin(x / 1.1)
    ) * np.clip((x + 80) / 15, 0, 1)
    terrain.add_surface("gravel", DRY, shape="rect", x=0.0, y=0.0, w=180.0, h=40.0)
    lanes = [("Astra GT", -3.0), ("Astra Trail", 3.0)]
    sim = DrivingPhysics(terrain, specs=VEHICLES, device="cpu")
    target = 45 / 3.6
    place(sim, rotation, [(-82.0, y, 0.0, target) for _, y in lanes])

    def policy(i, t, pose, v):
        return (*cruise(target, v), lateral(lanes[i][1], pose, rotation))

    frames = run(sim, policy, 12.0, rotation, DriverInput)
    meta = {
        "title": "Rolling heightfield at 45 km/h",
        "caption": "Each wheel finds its own contact on the 3D heightfield; suspension compression, pitch and roll "
        "follow from the four contact forces.",
        "lanes": [{"label": n, "y": y} for n, y in lanes],
        "markers": [],
        "camera": {"mode": "chase_pair", "follow": True},
    }
    return terrain, sim, frames, meta


SCENARIOS = {"brake": scenario_brake, "grade": scenario_grade, "hills": scenario_hills}


def terrain_json(terrain):
    return {
        "nx": terrain.nx,
        "ny": terrain.ny,
        "x0": float(terrain.x.min()),
        "y0": float(terrain.y.min()),
        "spacing": terrain.spacing,
        "z": np.round(terrain.z, 3).ravel().tolist(),
        "surfaces": terrain.surfaces,
    }


def spec_json(spec):
    return {
        "name": spec.name,
        "length": spec.length,
        "width": spec.width,
        "body_height": spec.body_height,
        "wheelbase": spec.wheelbase,
        "track": spec.track,
        "radius": spec.radius,
        "rest": spec.rest,
        "anchors": spec.anchors.tolist(),
        "color": list(spec.color),
        "awd": spec.awd,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--release", type=Path, default=Path.home() / "Github/scenefactory3d")
    ap.add_argument("--out", type=Path, default=HERE / "logs")
    ap.add_argument("--only", default="")
    args = ap.parse_args()
    mods = setup(args.release.resolve())
    rev = subprocess.run(
        ["git", "-C", str(args.release), "rev-parse", "--short", "HEAD"], capture_output=True, text=True
    ).stdout.strip()
    args.out.mkdir(parents=True, exist_ok=True)
    for name, fn in SCENARIOS.items():
        if args.only and name not in args.only.split(","):
            continue
        terrain, sim, frames, meta = fn(mods)
        data = {
            "id": name,
            "release_rev": rev,
            "hz": LOG_HZ,
            **meta,
            "terrain": terrain_json(terrain),
            "specs": [spec_json(s) for s in sim.specs],
            "frames": frames,
        }
        (args.out / f"{name}.json").write_text(json.dumps(data, separators=(",", ":")))
        last = frames[-1]["cars"]
        print(name, f"{len(frames)} frames", [(round(c["p"][0], 1), round(c["p"][1], 1), c["kmh"]) for c in last])


if __name__ == "__main__":
    main()
