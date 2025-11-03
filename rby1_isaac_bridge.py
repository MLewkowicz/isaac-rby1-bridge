# rby1_isaac_bridge_min.py  — paste into Script Editor

# --- config ---
PRIM_PATH   = "/World/rby1a/base"
SIM_FPS     = 60.0
VIEW_NAME   = "rby1_view"
PROJECT_ROOT= "/home/clear/IsaacLab/isaac-rby1-bridge"
SDK_GEN     = "/home/clear/IsaacLab/rby1-sdk/generated/python"

# --- imports/paths ---
import os, sys, time, threading, queue, numpy as np
for p in (PROJECT_ROOT, SDK_GEN):
    if os.path.exists(p) and p not in sys.path:
        sys.path.append(p)

import rby1_stub_server as stub
from rby1_stub_server import command_q

# --- start gRPC server (once) ---
import os, threading, inspect
print("[Bridge] using server module:", inspect.getsourcefile(stub))

os.environ.setdefault("RBY_PORT", "50051")

def _serve_once():
    try:
        stub.serve()
    except Exception as e:
        import traceback
        print("[Bridge] serve() crashed:", e)
        traceback.print_exc()

_server_thread = globals().get("RBY_SERVER_THREAD")  # may be None first run
if not _server_thread or not _server_thread.is_alive():
    RBY_SERVER_THREAD = threading.Thread(target=_serve_once, daemon=True)
    RBY_SERVER_THREAD.start()
    print("[Bridge] started stub server thread")
else:
    print("[Bridge] server thread already running")



from omni.timeline import get_timeline_interface
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.core")
from omni.isaac.core import World
from omni.isaac.core.articulations.articulation_view import ArticulationView
from omni.isaac.core.utils.prims import is_prim_path_valid

# avoid duplicates on re-run
if "RBY_BRIDGE_RUNNING" in globals():
    print("[Bridge] already running.")
else:
    get_timeline_interface().stop()

    # World & articulation
    WORLD = World.instance() or World(stage_units_in_meters=1.0)
    SCENE = WORLD.scene

    if not is_prim_path_valid(PRIM_PATH):
        raise RuntimeError(f"[Bridge] prim not found: {PRIM_PATH}. Open your kitchen USD first.")

    # (re)create view
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
        raise RuntimeError("[Bridge] no DOFs found under articulation root.")

    # Index mapping: try prefixes, else fall back to simple slices
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
        TORSO_IDX     = [i for i in range(n) if i not in RIGHT_ARM_IDX+LEFT_ARM_IDX]
        print("[Bridge] using sequential indices:",
              f"torso={TORSO_IDX}, right={RIGHT_ARM_IDX}, left={LEFT_ARM_IDX}")
    else:
        print("[Bridge] using prefix indices.")

    # initial q
    q_full = RBY.get_joint_positions()
    if getattr(q_full, "ndim", 1) == 2:
        q_full = q_full[0]
    q_full = np.array(q_full if (q_full is not None and np.size(q_full))
                      else np.zeros(len(dof_names)), dtype=np.float64)
    _Q = q_full.copy()

    # tell server how to pack state
    stub.set_joint_layout(
        dof_names,
        {"torso": TORSO_IDX, "right_arm": RIGHT_ARM_IDX, "left_arm": LEFT_ARM_IDX},
    )

    DT = 1.0 / SIM_FPS  # <— fixed dt; no world.get_physics_dt()

    def _interpolate_to(goal_full, min_time_sec):
        nonlocal_q = globals().get("_Q", _Q)
        steps = max(1, int(max(0.0, float(min_time_sec)) / DT))
        for k in range(steps):
            a = (k + 1) / steps
            q_step = (1.0 - a) * nonlocal_q + a * goal_full
            RBY.set_joint_positions(q_step)
            WORLD.step(render=True)
            stub.update_qpos_from_sim(q_step)
            time.sleep(DT)
        RBY.set_joint_positions(goal_full)
        WORLD.step(render=True)
        globals()["_Q"] = goal_full.copy()
        stub.update_qpos_from_sim(globals()["_Q"])

    def _apply_ticket(ticket):
        goal = globals().get("_Q", _Q).copy()
        T = 0.0
        for c in ticket.get("cmds", []):
            seg = c.get("segment")
            pos = np.asarray(c.get("positions", []), dtype=np.float64)
            if seg == "torso" and TORSO_IDX:
                if len(TORSO_IDX) != pos.size:  raise ValueError("Torso DOF mismatch.")
                goal[TORSO_IDX] = pos
            elif seg == "right_arm":
                if len(RIGHT_ARM_IDX) != pos.size: raise ValueError("Right-arm DOF mismatch.")
                goal[RIGHT_ARM_IDX] = pos
            elif seg == "left_arm":
                if len(LEFT_ARM_IDX) != pos.size:  raise ValueError("Left-arm DOF mismatch.")
                goal[LEFT_ARM_IDX] = pos
            T = max(T, float(c.get("minimum_time", 0.0)))
        _interpolate_to(goal, T)
        ticket["finish"] = "OK"; ticket["done"].set()

    def _loop():
        print("[Bridge] loop @ ~{} Hz; qid={}".format(SIM_FPS, id(command_q)))
        while True:
            try:
                ticket = command_q.get_nowait()
            except queue.Empty:
                ticket = None
            if ticket is not None:
                try:
                    _apply_ticket(ticket)
                except Exception as e:
                    import traceback
                    print("[Bridge] ERROR:", e); traceback.print_exc()
                    ticket["finish"] = f"ERROR: {e}"
                    ticket["done"].set()
            else:
                # idle: just keep sim alive and publish current q
                try:
                    WORLD.step(render=True)
                except Exception as e:
                    # if WORLD got cleared somehow, try to re-attach once
                    w = World.instance()
                    if w: 
                        globals()["WORLD"] = w
                stub.update_qpos_from_sim(globals().get("_Q", _Q))
                time.sleep(DT)

    threading.Thread(target=_loop, daemon=True).start()
    RBY_BRIDGE_RUNNING = True
    print("RBY1 bridge ready.")
