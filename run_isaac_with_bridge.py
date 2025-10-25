# run_isaac_with_bridge.py
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import time, threading
import numpy as np
from queue import Empty
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations.articulation_view import ArticulationView

# ----- bridge queue coming from your gRPC server -----
from rby_stub_server_ok import command_q  # each item: {"cmds":[{"segment","positions","minimum_time"}, ...], "done":Event, ...}

# ---- load or attach the robot ----
# USD_PATH = "/absolute/path/to/rby1.usd"   # set this if you are NOT opening a stage with the robot already in it
PRIM_PATH = "/RBY1"                 # make sure this matches the robot’s root prim in your stage

world = World(stage_units_in_meters=1.0)
# If your stage already has the robot, comment the next line:
# add_reference_to_stage(USD_PATH, PRIM_PATH)

# Use an ArticulationView (works great for single or multiple robots)
rby = ArticulationView(prim_paths_expr=PRIM_PATH, name="rby1_view")
world.scene.add(rby)
world.reset()

controller = rby.get_articulation_controller()
dof_names = list(rby.dof_names)

def _pick(prefix):
    sel = [n for n in dof_names if n.startswith(prefix)]
    sel.sort(key=lambda s: int(s.split("_")[-1]))
    return [dof_names.index(n) for n in sel]

TORSO_IDX = _pick("torso_")       # 6
RIGHT_IDX = _pick("right_arm_")   # 7
LEFT_IDX  = _pick("left_arm_")    # 7

assert len(TORSO_IDX)==6 and len(RIGHT_IDX)==7 and len(LEFT_IDX)==7, f"unexpected DOF grouping, got lens: {len(TORSO_IDX)}, {len(RIGHT_IDX)}, {len(LEFT_IDX)}"

SEG2IDX = {"torso": TORSO_IDX, "right_arm": RIGHT_IDX, "left_arm": LEFT_IDX}

# ---- shared state (SDK order: torso(6), right(7), left(7)) ----
dof_count = len(dof_names)
qpos_full = np.zeros(dof_count, dtype=np.float64)
_state_lock = threading.Lock()

def _goal_from_cmds(cmds):
    """merge torso/right/left targets into a full-length goal vector"""
    goal = qpos_full.copy()
    for c in cmds:
        idxs = SEG2IDX[c["segment"]]
        vals = np.asarray(c["positions"], dtype=np.float64)
        n = min(len(idxs), len(vals))
        goal[idxs[:n]] = vals[:n]
    return goal

# ---- trajectory state machine (main thread only) ----
active = None  # None or dict with start, goal, t0, T, ticket

def maybe_start_next():
    global active
    try:
        ticket = command_q.get_nowait()
    except Empty:
        return
    with _state_lock:
        start = qpos_full.copy()
    goal = _goal_from_cmds(ticket["cmds"])
    T = max((c["minimum_time"] for c in ticket["cmds"]), default=0.0)
    T = max(T, 1e-3)  # avoid zero
    active = {"start": start, "goal": goal, "t0": time.perf_counter(), "T": T, "ticket": ticket}

def tick(dt):
    """advance one frame; returns when current trajectory completes"""
    global active
    if active is None:
        return
    t = time.perf_counter()
    a = (t - active["t0"]) / active["T"]
    if a >= 1.0:
        a = 1.0
    q = (1.0 - a) * active["start"] + a * active["goal"]

    with _state_lock:
        qpos_full[:] = q

    controller.set_joint_positions(qpos_full)

    if a >= 1.0:
        # mark done
        active["ticket"]["finish"] = "OK"
        active["ticket"]["done"].set()
        active = None

# ---- main loop ----
while simulation_app.is_running():
    if active is None:
        maybe_start_next()
    tick(world.get_physics_dt())  # or just tick(0)
    world.step(render=True)

simulation_app.close()