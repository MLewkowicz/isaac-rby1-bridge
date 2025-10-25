# decoder.py
import rb.api.robot_command_pb2 as rc_pb

from numbers import Real
try:
    from google.protobuf.duration_pb2 import Duration as _PB_Duration
except Exception:
    _PB_Duration = None

def as_seconds(value) -> float:
    """Accepts numeric or google.protobuf.Duration; returns seconds as float."""
    # already numeric
    if isinstance(value, Real):
        return float(value)
    # google.protobuf.Duration instance
    if _PB_Duration is not None and isinstance(value, _PB_Duration):
        return float(value.seconds) + float(value.nanos) / 1e9
    # generic message with seconds/nanos (defensive)
    if hasattr(value, "seconds") and hasattr(value, "nanos"):
        return float(value.seconds) + float(value.nanos) / 1e9
    # last resort (will raise if truly unsupported)
    return float(value)

def decode_joint_position_commands(req: rc_pb.RobotCommandRequest):
    out = []
    if not req.HasField("robot_command"):
        return out
    rcmd = req.robot_command

    if rcmd.WhichOneof("command") != "component_based_command":
        return out
    cbc_req = rcmd.component_based_command

    if not cbc_req.HasField("body_command"):
        return out
    body_req = cbc_req.body_command
    bcbc_req = (body_req.body_component_based_command
                if body_req.HasField("body_component_based_command")
                else body_req)

    def pull(seg_field, label):
        if bcbc_req.HasField(seg_field):
            seg = getattr(bcbc_req, seg_field)
            if seg.HasField("joint_position_command"):
                jpc = seg.joint_position_command
                out.append({
                    "segment": label,
                    "positions": list(jpc.position),
                    "minimum_time": as_seconds(jpc.minimum_time),  # <-- changed
                })

    pull("torso_command", "torso")
    pull("right_arm_command", "right_arm")
    pull("left_arm_command",  "left_arm")
    return out
