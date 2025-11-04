# === RBY1 Bridge — main-thread update callback, no world.step ===
PRIM_PATH    = "/World/rby1a/base"
VIEW_NAME    = "rby1_view"
PROJECT_ROOT = "/home/clear/IsaacLab/isaac-rby1-bridge"
SDK_GEN      = "/home/clear/IsaacLab/rby1-sdk/generated/python"

import os, sys, time, threading, queue, numpy as np

# ---- paths so rb.api and your files import
for p in (PROJECT_ROOT, SDK_GEN):
    if os.path.exists(p) and p not in sys.path:
        sys.path.append(p)

# ---- server (same-process) ----
import rby1_stub_server as stub
from rby1_stub_server import command_q

# Start server thread only once (safe on re-run)
_srv = globals().get("RBY_SERVER_THREAD")
if not _srv or not _srv.is_alive():
    def _serve_once():
        try:
            stub.serve()
        except Exception as e:
            import traceback
            print("[Bridge] serve() crashed:", e)
            traceback.print_exc()
    RBY_SERVER_THREAD = threading.Thread(target=_serve_once, daemon=True)
    RBY_SERVER_THREAD.start()
    print("[Bridge] started stub server thread")
else:
    print("[Bridge] server already running")

# ---- Isaac / Kit setup ----
import omni.kit.app, omni.usd
from omni.timeline import get_timeline_interface
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.physx")
enable_extension("omni.isaac.core")

from omni.isaac.core import World
from omni.isaac.core.articulations.articulation_view import ArticulationView
from omni.isaac.core.utils.prims import is_prim_path_valid

# Clean any previous subscription from this script
_sub = globals().get("RBY_UPDATE_SUB")
if _sub:
    try: _sub.unsubscribe()
    except Exception: pass
    del globals()["RBY_UPDATE_SUB"]

timeline = get_timeline_interface()
timeline.stop()

stage = omni.usd.get_context().get_stage()
if not stage.GetPrimAtPath(PRIM_PATH):
    raise RuntimeError(f"[Bridge] Prim not found: {PRIM_PATH}. Open your USD first.")

WORLD = World.instance() or World(stage_units_in_meters=1.0)
SCENE = WORLD.scene

# (re)create a fresh view each run
try:
    SCENE.remove_object(VIEW_NAME)
except Exception:
    pass
RBY = ArticulationView(prim_paths_expr=PRIM_PATH, name=VIEW_NAME)
SCENE.add(RBY)
WORLD.reset()
RBY.initialize()

dof_names = list(RBY.dof_names)
if not dof_names:
    raise RuntimeError("[Bridge] No DOFs found. Is /World/rby1a/base an ArticulationRoot with joints under it?")

# Build indices (prefix-based; fallback to sequential slices if names differ)
def _idx(prefix):
    sel = [i for i, n in enumerate(dof_names) if n.startswith(prefix)]
    sel.sort(key=lambda i: int(dof_names[i].split("_")[-1]) if "_" in dof_names[i] else i)
    return sel

TORSO_IDX     = _idx("torso_")
RIGHT_ARM_IDX = _idx("right_arm_")
LEFT_ARM_IDX  = _idx("left_arm_")
if not RIGHT_ARM_IDX or not LEFT_ARM_IDX:
    n = len(dof_names)
    RIGHT_ARM_IDX = list(range(min(7, n)))
    LEFT_ARM_IDX  = list(range(len(RIGHT_ARM_IDX), min(14, n)))
    TORSO_IDX     = [i for i in range(n) if i not in RIGHT_ARM_IDX + LEFT_ARM_IDX]
    print("[Bridge] Using sequential indices:",
          f"torso={TORSO_IDX}, right={RIGHT_ARM_IDX}, left={LEFT_ARM_IDX}")
else:
    print("[Bridge] Using prefix indices.")

# Initial joint state
q = RBY.get_joint_positions()
if getattr(q, "ndim", 1) == 2: q = q[0]
q = np.array(q if (q is not None and np.size(q)) else np.zeros(len(dof_names)), dtype=np.float64)

# Tell server how to pack state (torso+right+left order)
stub.set_joint_layout(
    dof_names,
    {"torso": TORSO_IDX, "right_arm": RIGHT_ARM_IDX, "left_arm": LEFT_ARM_IDX},
)

# ---- Trajectory state (all on main thread) ----
active = None   # dict: {"q0", "goal", "T", "t", "ticket"}
last_t = None   # wall-clock timestamp

def _start_from_ticket(ticket):
    nonlocal_q = q.copy()
    goal = nonlocal_q.copy(); T = 0.0
    for c in ticket.get("cmds", []):
        seg = c.get("segment"); pos = np.asarray(c.get("positions", []), dtype=np.float64)
        if seg == "torso" and TORSO_IDX:
            if len(TORSO_IDX) != pos.size: raise ValueError("Torso DOF mismatch.")
            goal[TORSO_IDX] = pos
        elif seg == "right_arm":
            if len(RIGHT_ARM_IDX) != pos.size: raise ValueError(f"Right-arm DOF mismatch: {pos.size} != {len(RIGHT_ARM_IDX)}")
            goal[RIGHT_ARM_IDX] = pos
        elif seg == "left_arm":
            if len(LEFT_ARM_IDX) != pos.size: raise ValueError("Left-arm DOF mismatch.")
            goal[LEFT_ARM_IDX] = pos
        T = max(T, float(c.get("minimum_time", 0.0)))
    return {"q0": nonlocal_q, "goal": goal, "T": max(T, 0.0), "t": 0.0, "ticket": ticket}

def _finish(ok=True, err=None):
    global active
    if not active: return
    ticket = active["ticket"]
    ticket["finish"] = "OK" if ok else f"ERROR: {err}"
    ticket["done"].set()
    active = None

# ---- Main-thread frame-update callback (no world.step here)
app = omni.kit.app.get_app()
def _on_update(e):
    global q, active, last_t

    now = time.perf_counter()
    dt = 0.0 if last_t is None else (now - last_t)
    last_t = now

    # If idle, try to fetch a ticket (non-blocking)
    if active is None:
        try:
            ticket = command_q.get_nowait()
        except queue.Empty:
            # publish current state and return
            stub.update_qpos_from_sim(q)
            return
        try:
            active = _start_from_ticket(ticket)
        except Exception as ex:
            import traceback
            print("[Bridge] ticket error:", ex)
            traceback.print_exc()
            ticket["finish"] = f"ERROR: {ex}"
            ticket["done"].set()
            return

    # Advance active trajectory
    T = active["T"]
    if T <= 0.0:
        q = active["goal"].copy()
        RBY.set_joint_positions(q)
        stub.update_qpos_from_sim(q)
        _finish(ok=True)
        return

    active["t"] = min(T, active["t"] + dt)
    a = active["t"] / T
    q = (1.0 - a) * active["q0"] + a * active["goal"]
    RBY.set_joint_positions(q)
    stub.update_qpos_from_sim(q)
    if active["t"] >= T:
        _finish(ok=True)

# Unsubscribe old, then subscribe to the app’s update stream (main thread)
RBY_UPDATE_SUB = app.get_update_event_stream().create_subscription_to_pop(_on_update)
timeline.play()
print("RBY1 bridge ready (main-thread update callback).")
