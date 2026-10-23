"""Simulate the vehicle demo clips with the SceneFactory-3D vehicle family.

Every clip runs ``astra_drive.vehicle_family.FamilyPhysics``: the v1 per-wheel vehicle model
(Newton XPBD, raycast suspension, v1 tire law) generalized to N axles, dual tires, a fifth-wheel
trailer joint and skid steering. For the SF3D Coupe and Crossover it reproduces the released
``vehicle.py``/``physics.py`` bit for bit (tests/test_vehicle_family.py on branch vehicle-family).
The Pickup's loose crates are free bodies that touch the truck only through contact.
These are hand-authored vehicle demos, not benchmark episodes.

    python tools/vehicle_demos/simulate.py --code ~/Github/astraexplore-vehicles [--only brake,hauler]
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
SETTLE = 1.5  # s on the springs before a clip starts (not logged)


def setup(code):
    sys.path.insert(0, str(code))
    import warp as wp

    wp.config.log_level = wp.LOG_WARNING
    from astra_drive.terrain import Terrain
    from astra_drive.vehicle import DriverInput, rotation
    from astra_drive.vehicle_family import spec as S
    from astra_drive.vehicle_family.physics import FamilyPhysics

    return dict(Terrain=Terrain, DriverInput=DriverInput, rotation=rotation, S=S, Physics=FamilyPhysics)


# ------------------------------------------------------------------ helpers
def heading(M, pose):
    r = M["rotation"](pose[3:])
    return math.atan2(r[1, 0], r[0, 0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def cruise(target, v, gain=0.5):
    e = target - v
    return (min(1.0, gain * e), 0.0) if e > 0 else (0.0, min(1.0, -gain * e))


def lateral(M, y_target, pose, k=0.12, kd=0.9):
    """Ackermann lane keeping on +x roads."""
    return float(np.clip(k * (y_target - pose[1]) - kd * heading(M, pose), -1, 1))


class Seek:
    """Skid-steer waypoint follower: pivot toward the next point, then drive; stop at the end."""

    def __init__(self, points, speed, final_heading=None, start=0.0, tol=0.25):
        self.points, self.speed, self.final, self.start, self.tol = list(points), speed, final_heading, start, tol
        self.i, self.done = 0, False

    def __call__(self, M, t, pose, v, yaw_rate):
        if t < self.start or self.done:
            return (0.0, 1.0, 0.0)
        if self.i >= len(self.points):
            if self.final is None:
                self.done = True
                return (0.0, 1.0, 0.0)
            if abs(v) > 0.15:  # stop before the final pivot
                return (0.0, 1.0, 0.0)
            err = wrap(self.final - heading(M, pose))
            if abs(err) < 0.04 and abs(yaw_rate) < 0.15:
                self.done = True
                return (0.0, 1.0, 0.0)
            return (0.0, 0.0, float(np.clip(1.6 * err - 0.4 * yaw_rate, -1, 1)))
        tx, ty = self.points[self.i]
        dx, dy = tx - pose[0], ty - pose[1]
        dist = math.hypot(dx, dy)
        last = self.i == len(self.points) - 1
        if dist < (self.tol if last else 0.8):
            self.i += 1
            return (0.0, 0.6 if last else 0.0, 0.0)
        err = wrap(math.atan2(dy, dx) - heading(M, pose))
        steer = float(np.clip(2.2 * err - 0.35 * yaw_rate, -1, 1))
        if abs(err) > 0.5:  # stop, then pivot in place
            return (0.0, 1.0, 0.0) if abs(v) > 0.2 else (0.0, 0.0, steer)
        target = min(self.speed, 0.7 * math.sqrt(2 * 1.0 * dist) if last else self.speed) * math.cos(err)
        th, br = cruise(target, v, 1.2)
        return (th, br, steer)


def build(M, terrain, placements):
    """placements: [(model, x, y, yaw)]. Returns a settled sim (brakes on, SETTLE seconds)."""
    P = M["Physics"]
    poses = [P.body_poses(m, x, y, yaw, P.touch_height(terrain, m, x, y, yaw) + 0.02) for m, x, y, yaw in placements]
    sim = P([p[0] for p in placements], terrain, poses=poses, device="cpu")
    hold = [M["DriverInput"](0, 1.0, 0) for _ in placements]
    for _ in range(int(SETTLE / sim.DT)):
        sim.step(hold)
    sim.time = 0.0
    return sim


def launch(M, sim, speeds):
    """Give vehicle i forward speed speeds[i] (all its bodies, wheels rolling)."""
    vel = sim.velocities
    for vi, (f, sl) in enumerate(zip(sim.forces, sim.body_slices)):
        if not speeds[vi]:
            continue
        for b in range(sl.start, sl.stop):
            vel[b, :3] = M["rotation"](sim.poses[b][3:])[:, 0] * speeds[vi]
            vel[b, 3:] = 0
        f.omega[:] = [speeds[vi] / w[1].radius for w in f.wheels]
    sim.set_velocities(vel)


def run(M, sim, policy, duration, note=None):
    """note(vi, poses of vehicle vi's bodies and crates) -> short text shown in its label."""
    frames, every = [], round(1 / sim.DT / LOG_HZ)
    for k in range(int(duration / sim.DT) + 1):
        t = k * sim.DT
        poses, vel = sim.poses, sim.velocities
        cmds, info = [], []
        for vi, sl in enumerate(sim.body_slices):
            p0, v0 = poses[sl.start], vel[sl.start]
            speed = float(np.dot(v0[:3], M["rotation"](p0[3:])[:, 0]))
            cmd = M["DriverInput"](*policy(vi, t, p0, speed, float(v0[5])))
            cmds.append(cmd)
            info.append((speed, cmd))
        if k % every == 0:
            frames.append({
                "t": round(t, 4),
                "v": [
                    {
                        "b": [[round(float(x), 4) for x in poses[b]] for b in range(sl.start, sl.stop)],
                        "steer": [round(float(a), 4) for a in f.steer_angles],
                        "spin": [round(float(a), 3) for a in f.wheel_angle],
                        "susp": [round(float(a), 4) for a in f.lengths],
                        "omega": [round(float(a), 2) for a in f.omega],
                        "kmh": round(info[vi][0] * 3.6, 2),
                        "cmd": [round(info[vi][1].throttle, 2), round(info[vi][1].brake, 2), round(info[vi][1].steer, 2)],
                        **({"note": note(vi, poses[sl])} if note else {}),
                    }
                    for vi, (f, sl) in enumerate(zip(sim.forces, sim.body_slices))
                ],
            })
        sim.step(cmds)
    return frames


# ------------------------------------------------------------------ scenarios
def scenario_brake(M):
    """Matched counterfactual: same car, same speed, same commands; only the road friction differs."""
    S = M["S"]
    lanes = [(S.COUPE, DRY, -15.0), (S.COUPE, LOW, -5.0), (S.CROSSOVER, DRY, 5.0), (S.CROSSOVER, LOW, 15.0)]
    terrain = M["Terrain"](nx=513, ny=89, spacing=0.5, preset="Flat")
    terrain.add_surface("gravel", DRY, shape="rect", x=0.0, y=0.0, w=256.0, h=44.0)
    for _, mu, y in lanes:
        if mu != DRY:
            terrain.add_surface("ice", mu, shape="rect", x=20.0, y=y, w=210.0, h=5.0)
    sim = build(M, terrain, [(m, -115.0, y, 0.0) for m, _, y in lanes])
    v0, brake_x = 60 / 3.6, -70.0
    launch(M, sim, [v0] * 4)
    braking = {}

    def policy(i, t, pose, v, wz):
        if pose[0] < brake_x and i not in braking:
            return (*cruise(v0, v), lateral(M, lanes[i][2], pose))
        braking.setdefault(i, t)
        s = t - braking[i]
        steer = 0.3 if s < 0.9 else (-0.3 if s < 1.8 else 0.0)
        return (0.0, 0.0 if v < 0.05 else 0.8, steer)

    return terrain, sim, run(M, sim, policy, 13.0), {
        "lanes": [{"y": y} for _, _, y in lanes],
        "labels": [f"{m.name.replace('SF3D ', '')} · μ {mu:g}" for m, mu, _ in lanes],
        "markers": [{"x": brake_x}],
        "camera": {"mode": "fit"},
    }


def scenario_grade(M):
    """Hill start on an icy 8 % grade: rear-wheel drive vs. all-wheel drive, same half throttle."""
    S = M["S"]
    terrain = M["Terrain"](nx=241, ny=41, spacing=0.5, preset="Flat")
    x = terrain.x
    ramp = np.clip((x + 20) / 10, 0, 1)
    terrain.z = np.where(x < -10, 0.08 * 10 * ramp**2 / 2, 0.08 * (x + 15))
    terrain.add_surface("gravel", DRY, shape="rect", x=0.0, y=0.0, w=120.0, h=20.0)
    terrain.add_surface("ice", 0.25, shape="rect", x=20.0, y=0.0, w=80.0, h=12.0)
    lanes = [(S.COUPE, -2.6), (S.CROSSOVER, 2.6)]
    sim = build(M, terrain, [(m, -4.0, y, 0.0) for m, y in lanes])

    def policy(i, t, pose, v, wz):
        throttle = 0.0 if t < 1.0 else 0.5
        if v > 50 / 3.6:
            throttle = 0.0
        return (throttle, 1.0 if t < 1.0 else 0.0, lateral(M, lanes[i][1], pose))

    return terrain, sim, run(M, sim, policy, 9.0), {
        "lanes": [{"y": y} for _, y in lanes],
        "labels": ["Coupe · RWD", "Crossover · AWD"],
        "markers": [],
        "camera": {"mode": "chase", "offset": [-9, -15, 4.5]},
        "show_wheel_speed": True,
    }


def scenario_hills(M):
    """Rolling terrain at speed: suspension travel, pitch and roll on a per-world heightfield."""
    S = M["S"]
    terrain = M["Terrain"](nx=361, ny=81, spacing=0.5, preset="Flat")
    x, y = terrain.x, terrain.y
    terrain.z = (
        0.7 * np.sin(x / 8.0) * np.cos(y / 9.0) + 0.16 * np.sin(x / 3.0 + y / 2.5) + 0.03 * np.sin(x / 1.1)
    ) * np.clip((x + 80) / 15, 0, 1)
    terrain.add_surface("gravel", DRY, shape="rect", x=0.0, y=0.0, w=180.0, h=40.0)
    lanes = [(S.COUPE, -3.0), (S.CROSSOVER, 3.0)]
    sim = build(M, terrain, [(m, -82.0, y, 0.0) for m, y in lanes])
    target = 45 / 3.6
    launch(M, sim, [target, target])

    def policy(i, t, pose, v, wz):
        return (*cruise(target, v), lateral(M, lanes[i][1], pose))

    return terrain, sim, run(M, sim, policy, 12.0), {
        "lanes": [{"y": y} for _, y in lanes],
        "labels": ["Coupe", "Crossover"],
        "markers": [],
        "camera": {"mode": "chase", "offset": [-9, -15, 4.5]},
    }


def scenario_hauler(M):
    """Loaded 18-wheeler, lane change while braking at 72 km/h: road friction and brake distribution."""
    S = M["S"]
    terrain = M["Terrain"](nx=513, ny=81, spacing=1.0, preset="Flat")
    runs = [("dry", DRY, "all axles", -18.0), ("ice", LOW, "all axles", 2.0), ("ice", LOW, "drive axles locked", 22.0)]
    terrain.add_surface("gravel", DRY, shape="rect", x=0.0, y=0.0, w=500.0, h=80.0)
    terrain.add_surface("ice", LOW, shape="rect", x=60.0, y=12.0, w=360.0, h=27.0)
    sim = build(M, terrain, [(S.HAULER, -220.0, y, 0.0) for *_, y in runs])
    v0, start_x = 20.0, -110.0
    launch(M, sim, [v0] * 3)
    began = {}

    def policy(i, t, pose, v, wz):
        if pose[0] < start_x and i not in began:
            return (*cruise(v0, v), lateral(M, runs[i][3], pose, k=0.03, kd=0.6))
        began.setdefault(i, t)
        s = t - began[i]
        steer = 0.11 * math.sin(2 * math.pi * s / 4.0) if s < 4.0 else 0.0  # same lane change for all
        moving = v > 0.05
        if runs[i][2] == "all axles":
            return (0.0, 0.6 if moving else 0.0, steer)
        return (0.0, 0.0, steer, 1, moving)  # handbrake path: tractor drive axles only

    return terrain, sim, run(M, sim, policy, 13.0), {
        "lanes": [{"y": y} for *_, y in runs] + [{"y": y + 3.5} for *_, y in runs],
        "labels": [f"μ {mu:g} · {how}" for _, mu, how, _ in runs],
        "markers": [{"x": start_x}],
        "camera": {"mode": "fit", "min_dist": 80},
    }


def scenario_closure(M):
    """Robots deploy a lane closure: five Cone Bots form a taper, the Barrier Bot closes the lane."""
    S = M["S"]
    terrain = M["Terrain"](nx=513, ny=49, spacing=0.5, preset="Flat")
    terrain.add_surface("gravel", DRY, shape="rect", x=0.0, y=0.0, w=256.0, h=7.2)
    cone_x = [0.0, 7.0, 14.0, 21.0, 28.0]
    cone_y = [-3.3 + 3.0 * k / 4 for k in range(5)]
    place, seeks = [], []
    for k, (cx, cy) in enumerate(zip(cone_x, cone_y)):
        place.append((S.CONE_BOT, cx - 5.0, -5.4, 0.0))
        seeks.append(Seek([(cx - 1.5, -4.6), (cx, cy)], 2.2, final_heading=0.0, start=0.6 * k))
    place.append((S.BARRIER_BOT, 30.0, -6.0, 0.0))
    seeks.append(Seek([(34.0, -4.2), (38.0, -1.8)], 2.0, final_heading=0.0, start=2.0, tol=0.6))
    # Traffic: a Coupe in the closing lane merges left before the taper; a Crossover in the open lane.
    place.append((S.COUPE, -122.0, -1.8, 0.0))
    place.append((S.CROSSOVER, -110.0, 1.8, 0.0))
    sim = build(M, terrain, place)
    n_bots = len(seeks)
    launch(M, sim, [0.0] * n_bots + [30 / 3.6, 30 / 3.6])

    def policy(i, t, pose, v, wz):
        if i < n_bots:
            return seeks[i](M, t, pose, v, wz)
        lane = 1.8 if (i == n_bots + 1 or pose[0] > -40.0) else -1.8
        return (*cruise(30 / 3.6, v), lateral(M, lane, pose, k=0.08, kd=0.9))

    return terrain, sim, run(M, sim, policy, 17.0), {
        "lanes": [{"y": -1.8}, {"y": 1.8}],
        "lane_width": 3.6,
        "shoulder": -3.6,
        "labels": [None] * 5 + ["Barrier Bot", "Coupe", "Crossover"],
        "group_labels": [{"text": "Cone Bots ×5", "vehicles": [0, 1, 2, 3, 4]}],
        "markers": [],
        "camera": {"mode": "fixed", "pos": [-9, -19, 9], "look": [17, -1.0, 0.3]},
    }


def scenario_courier(M):
    """Six-wheel Courier on a sidewalk route: pivot turns and a curb ramp."""
    S = M["S"]
    terrain = M["Terrain"](nx=161, ny=161, spacing=0.25, preset="Flat")
    x, y = terrain.x, terrain.y
    # A raised sidewalk (12 cm) with a 1.6 m wide curb ramp at x = 2..3.6 down to the y < -3 street.
    curb = np.where(y > -3.0, 0.12, 0.0)
    ramp = np.where((x > 2.0) & (x < 3.6) & (y <= -3.0) & (y > -6.0), 0.12 * np.clip((y + 6.0) / 3.0, 0, 1), 0)
    terrain.z = np.maximum(curb, ramp)
    terrain.add_surface("gravel", 0.9, shape="rect", x=0.0, y=0.0, w=40.0, h=40.0)
    route = [(-8.0, 0.0), (2.8, 0.0), (2.8, -8.0), (12.0, -8.0)]
    seek = Seek(route[1:], 1.6, final_heading=None)
    sim = build(M, terrain, [(S.COURIER, *route[0], 0.0)])

    def policy(i, t, pose, v, wz):
        return seek(M, t, pose, v, wz)

    return terrain, sim, run(M, sim, policy, 20.0), {
        "lanes": [],
        "labels": ["Courier"],
        "show_speed": True,
        "markers": [],
        "camera": {"mode": "chase", "offset": [-3.2, -4.2, 2.4], "lag": 1.2, "look_ahead": 0.6},
        "sidewalk": {"y": -3.0},
    }


def crate_offsets(M, model, poses):
    """Each crate's displacement from its loading place, in the truck frame (m)."""
    rot = M["rotation"](poses[0][3:])
    start = M["Physics"].cargo_poses(model, np.array([0, 0, 0, 0, 0, 0, 1.0]))
    return [rot.T @ (poses[len(model.bodies) + i][:3] - poses[0][:3]) - start[i][:3] for i in range(len(model.cargo))]


def tilt_deg(M, pose):
    return math.degrees(math.acos(min(1.0, M["rotation"](pose[3:])[2, 2])))


def scenario_cargo_stop(M):
    """Hard stop from 72 km/h with three loads in the bed: one loose crate, three loose crates, one fitted crate."""
    S = M["S"]
    loads = [("300 kg crate, loose", (S.CRATE,)), ("410 kg in 3 loose crates", S.MIXED_CRATES),
             ("410 kg crate filling the bed", (S.FITTED_CRATE,))]
    models = [S.PICKUP.with_cargo(*c) for _, c in loads]
    ys = [-5.0, 0.0, 5.0]
    terrain = M["Terrain"](nx=321, ny=41, spacing=0.5, preset="Flat")
    terrain.add_surface("gravel", DRY, shape="rect", x=0.0, y=0.0, w=160.0, h=20.0)
    sim = build(M, terrain, [(m, -60.0, y, 0.0) for m, y in zip(models, ys)])
    v0, brake_at = 20.0, 0.8
    launch(M, sim, [v0] * 3)

    def policy(i, t, pose, v, wz):
        if t < brake_at:
            return (0.0, 0.0, lateral(M, ys[i], pose))
        return (0.0, 0.85 if v > 0.05 else 1.0, 0.0)

    def note(i, poses):
        far = max(crate_offsets(M, models[i], poses), key=lambda d: abs(d[0]))
        return f"crate slid {far[0]:+.2f} m"

    return terrain, sim, run(M, sim, policy, 5.0, note), {
        "lanes": [{"y": y} for y in ys],
        "lane_width": 5.0,
        "labels": [label for label, _ in loads],
        "markers": [{"x": -60.0 + v0 * brake_at}],
        "camera": {"mode": "chase", "offset": [-6.0, -12.5, 9.0], "lag": 0.4},
    }


def scenario_cargo_fishhook(M):
    """Fishhook at 50 km/h: empty, 410 kg fitted crate, 410 kg in three loose crates."""
    S = M["S"]
    loads = [("Empty", ()), ("410 kg crate filling the bed", (S.FITTED_CRATE,)), ("410 kg in 3 loose crates", S.MIXED_CRATES)]
    models = [S.PICKUP.with_cargo(*c) for _, c in loads]
    # Identical inputs give translated copies of one path, so trucks 8 m apart never meet; the
    # truck that rolls is outermost, clear of where the others go after the reversal.
    ys = [-8.0, 0.0, 8.0]
    terrain = M["Terrain"](nx=241, ny=161, spacing=0.5, preset="Flat")
    terrain.add_surface("gravel", DRY, shape="rect", x=0.0, y=0.0, w=120.0, h=80.0)
    sim = build(M, terrain, [(m, -50.0, y, 0.0) for m, y in zip(models, ys)])
    v0 = 14.0
    launch(M, sim, [v0] * 3)

    def policy(i, t, pose, v, wz):
        steer = 0.0 if t < 0.5 else (1.0 if t < 1.4 else (-1.0 if t < 3.5 else 0.0))
        return (0.0, 0.0, steer)

    peak = [0.0] * 3

    def note(i, poses):
        peak[i] = max(peak[i], tilt_deg(M, poses[0]))
        return f"tilt {tilt_deg(M, poses[0]):.0f}° (max {peak[i]:.0f}°)"

    return terrain, sim, run(M, sim, policy, 5.5, note), {
        "lanes": [],
        "labels": [label for label, _ in loads],
        "show_speed": False,
        "markers": [],
        "camera": {"mode": "fit", "min_dist": 20, "margin": 2, "lag": 0.15},
    }


SCENARIOS = {
    "brake": scenario_brake, "grade": scenario_grade, "hills": scenario_hills,
    "hauler": scenario_hauler, "closure": scenario_closure, "courier": scenario_courier,
    "cargo_stop": scenario_cargo_stop, "cargo_fishhook": scenario_cargo_fishhook,
}


def terrain_json(t):
    return {"nx": t.nx, "ny": t.ny, "x0": float(t.x.min()), "y0": float(t.y.min()), "spacing": t.spacing,
            "z": np.round(t.z, 3).ravel().tolist(), "surfaces": t.surfaces}


def model_json(m):
    return {
        "name": m.name, "kind": m.kind, "drive": m.drive, "color": list(m.color),
        "bodies": [
            {
                "shapes": [{"kind": s.kind, "center": list(s.center), "size": list(s.size), "color": list(s.color),
                            "role": s.role} for s in b.shapes],
                "wheels": [{"anchor": [float(v) for v in a], "radius": ax.radius, "dual": ax.dual}
                           for ax, a in zip(b.wheels(), b.anchors())],
            }
            for b in m.bodies
        ] + [  # crates: free bodies drawn as boxes, logged after the vehicle's bodies
            {"shapes": [{"kind": "box", "center": [0, 0, 0], "size": list(c.size), "color": list(c.color), "role": "crate"}],
             "wheels": []}
            for c in m.cargo
        ],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--code", type=Path, default=Path.home() / "Github/astraexplore-vehicles")
    ap.add_argument("--out", type=Path, default=HERE / "logs")
    ap.add_argument("--only", default="")
    args = ap.parse_args()
    M = setup(args.code.resolve())
    rev = subprocess.run(["git", "-C", str(args.code), "rev-parse", "--short", "HEAD"],
                         capture_output=True, text=True).stdout.strip()
    args.out.mkdir(parents=True, exist_ok=True)
    for name, fn in SCENARIOS.items():
        if args.only and name not in args.only.split(","):
            continue
        terrain, sim, frames, meta = fn(M)
        data = {"id": name, "code_rev": rev, "hz": LOG_HZ, **meta, "terrain": terrain_json(terrain),
                "models": [model_json(m) for m in sim.models], "frames": frames}
        (args.out / f"{name}.json").write_text(json.dumps(data, separators=(",", ":")))
        last = frames[-1]["v"]
        print(name, f"{len(frames)} frames", [(round(v["b"][0][0], 1), round(v["b"][0][1], 1), v["kmh"]) for v in last])


if __name__ == "__main__":
    main()
