# rby_stub_server_ok.py
import time
import grpc
from concurrent import futures

# ----- SERVICE STUBS (services live here) -----
import rb.api.robot_command_service_pb2_grpc      as rc_svc_grpc
import rb.api.robot_state_service_pb2_grpc        as rs_svc_grpc
import rb.api.control_manager_service_pb2_grpc    as cm_svc_grpc
import rb.api.power_service_pb2_grpc              as pw_svc_grpc
import rb.api.joint_operation_service_pb2_grpc    as jo_svc_grpc
import rb.api.parameter_service_pb2_grpc          as pr_svc_grpc

# ----- MESSAGE TYPES (requests/responses live here) -----
import rb.api.robot_command_pb2                   as rc_pb
import rb.api.robot_state_pb2                     as rs_pb
import rb.api.control_manager_pb2                 as cm_pb
import rb.api.power_pb2                           as pw_pb
import rb.api.joint_operation_pb2                 as jo_pb
import rb.api.parameter_pb2                       as pr_pb

from decoder import decode_joint_position_commands

class LoggingInterceptor(grpc.ServerInterceptor):
    def intercept_service(self, continuation, handler_call_details):
        print(f"[gRPC] call -> {handler_call_details.method}")
        return continuation(handler_call_details)

# When you create the server:
server = grpc.server(
    futures.ThreadPoolExecutor(max_workers=16),
    interceptors=[LoggingInterceptor()],
)

from google.protobuf.message import Message

# server side (same process or separate — up to you)
import queue, threading
command_q: "queue.Queue[dict]" = queue.Queue(maxsize=128)

def enqueue_and_wait(cmd_list, timeout=None):
    """
    cmd_list: [{'segment': 'torso|right_arm|left_arm', 'positions': [...], 'minimum_time': float}, ...]
    """
    ticket = {"cmds": cmd_list, "done": threading.Event(), "finish": "OK"}
    command_q.put(ticket)
    ticket["done"].wait(timeout)
    return ticket.get("finish", "OK")


# ----------------- helpers to set fields safely -----------------
def set_bool_if_present(msg, *field_names, value=True):
    for name in field_names:
        f = msg.DESCRIPTOR.fields_by_name.get(name)
        if f and f.type == f.TYPE_BOOL:
            setattr(msg, name, value)
            return True
    return False

def set_enum_by_name(msg, field_name, enum_name):
    f = msg.DESCRIPTOR.fields_by_name.get(field_name)
    if not f or not f.enum_type:
        return False
    enum_vals = f.enum_type.values_by_name
    if enum_name not in enum_vals:
        return False
    setattr(msg, field_name, enum_vals[enum_name].number)
    return True

def set_float_if_present(msg, field_name, value):
    f = msg.DESCRIPTOR.fields_by_name.get(field_name)
    if f and f.type in (f.TYPE_FLOAT, f.TYPE_DOUBLE):
        setattr(msg, field_name, float(value))
        return True
    return False

def extend_repeated_floats_if_present(msg, field_name, values):
    f = msg.DESCRIPTOR.fields_by_name.get(field_name)
    if f and f.type in (f.TYPE_FLOAT, f.TYPE_DOUBLE) and f.label == f.LABEL_REPEATED:
        getattr(msg, field_name).extend([float(v) for v in values])
        return True
    return False

def pack_sdk_order():
    with _state_lock:
        t = qpos_full[TORSO_IDX]
        r = qpos_full[RIGHT_IDX]
        l = qpos_full[LEFT_IDX]
        return np.concatenate([t, r, l]).tolist()

# ----------------- tiny in-memory "state" -----------------
powered = False
servos = False
control_enabled = True
time_scale = 1.0

# Pick a safe DOF count so robot_state.* arrays exist
# (torso 6 + right 7 + left 7 = 20)
ROBOT_DOF = 24

class RobotCommandService(rc_svc_grpc.RobotCommandServiceServicer):
    def RobotCommand(self, request, context):
        cmds = decode_joint_position_commands(request)
        if cmds:
            enqueue_and_wait(cmds)
        resp = rc_pb.RobotCommandResponse()
        fb = resp.feedback
        set_enum_by_name(fb, "status", "STATUS_FINISHED")
        set_enum_by_name(fb, "finish_code", "FINISH_CODE_OK")
        return resp

    def RobotCommandStream(self, request_iterator, context):
        first = next(request_iterator, None)
        if first:
            cmds = decode_joint_position_commands(first)
            if cmds:
                enqueue_and_wait(cmds)
        # send terminal feedback
        resp = rc_pb.RobotCommandResponse()
        fb = resp.feedback
        set_enum_by_name(fb, "status", "STATUS_FINISHED")
        set_enum_by_name(fb, "finish_code", "FINISH_CODE_OK")
        yield resp


class RobotStateService(rs_svc_grpc.RobotStateServiceServicer):
    def GetRobotState(self, request, context):
        resp = rs_pb.GetRobotStateResponse()
        s = resp.robot_state
        sdk_pos = pack_sdk_order()
        s.position.extend(sdk_pos)
        # You can keep velocity/current zeros for now
        s.velocity.extend([0.0]*len(sdk_pos))
        s.current.extend([0.0]*len(sdk_pos))
        s.target_position.extend(sdk_pos)
        # Control manager state as before (ENABLED/IDLE)
        cms = resp.control_manager_state
        set_enum_by_name(cms, "state", "CONTROL_MANAGER_STATE_ENABLED")
        set_enum_by_name(cms, "control_state", "CONTROL_STATE_IDLE")
        cms.unlimited_mode_enabled = True
        cms.time_scale = 1.0
        return resp

    def GetRobotStateStream(self, request, context):
        import time
        while True:
            resp = rs_pb.GetRobotStateStreamResponse()
            if hasattr(resp, "robot_state"):
                s = resp.robot_state
                sdk_pos = pack_sdk_order()
                s.position.extend(sdk_pos)
                s.velocity.extend([0.0]*len(sdk_pos))
                s.current.extend([0.0]*len(sdk_pos))
                s.target_position.extend(sdk_pos)
            yield resp
            time.sleep(0.02)


    def GetControlManagerState(self, request, context):
        return rs_pb.GetControlManagerStateResponse()

    def ResetOdometry(self, request, context):
        return rs_pb.ResetOdometryResponse()

class ControlManagerService(cm_svc_grpc.ControlManagerServiceServicer):
    def _enabled_state(self):
        s = cm_pb.ControlManagerState()
        set_enum_by_name(s, "state", "CONTROL_MANAGER_STATE_ENABLED")
        set_enum_by_name(s, "control_state", "CONTROL_STATE_IDLE")
        s.unlimited_mode_enabled = True
        s.time_scale = 1.0
        # leave enabled_joint_idx empty unless you want to enumerate all joints
        return s

    def ControlManagerCommand(self, request, context):
        resp = cm_pb.ControlManagerCommandResponse()
        resp.control_manager_state.CopyFrom(self._enabled_state())
        return resp

    def WaitForControlReady(self, request, context):
        resp = cm_pb.WaitForControlReadyResponse()
        resp.ready = True
        return resp

    def CancelControl(self, request, context):
        return cm_pb.CancelControlResponse()

    def GetTimeScale(self, request, context):
        return cm_pb.GetTimeScaleResponse(time_scale=1.0)

    def SetTimeScale(self, request, context):
        # honor request.time_scale if present
        ts = getattr(request, "time_scale", None)
        return cm_pb.SetTimeScaleResponse()

class PowerService(pw_svc_grpc.PowerServiceServicer):
    def PowerCommand(self, request, context):
        resp = pw_pb.PowerCommandResponse()
        set_enum_by_name(resp, "status", "STATUS_SUCCESS")
        return resp
    def JointCommand(self, request, context):
        resp = pw_pb.JointCommandResponse()
        if "status" in resp.DESCRIPTOR.fields_by_name:
            set_enum_by_name(resp, "status", "STATUS_SUCCESS")
        return resp
    def ToolFlangePowerCommand(self, request, context):
        resp = pw_pb.ToolFlangePowerCommandResponse()
        if "status" in resp.DESCRIPTOR.fields_by_name:
            set_enum_by_name(resp, "status", "STATUS_SUCCESS")
        return resp

class JointOperationService(jo_svc_grpc.JointOperationServiceServicer):
    def ServoOn(self, request, context):
        resp = jo_pb.ServoOnResponse()
        set_enum_by_name(resp, "status", "STATUS_SUCCESS")
        return resp
    def ServoOff(self, request, context):
        resp = jo_pb.ServoOffResponse()
        set_enum_by_name(resp, "status", "STATUS_SUCCESS")
        return resp

    def BrakeEngage(self, request, context):  return jo_pb.BrakeEngageResponse()
    def BrakeRelease(self, request, context): return jo_pb.BrakeReleaseResponse()
    def HomeOffsetReset(self, request, context): return jo_pb.HomeOffsetResetResponse()
    def GetPositionPIDGain(self, request, context): return jo_pb.GetPositionPIDGainResponse()
    def SetPositionPIDGain(self, request, context): return jo_pb.SetPositionPIDGainResponse()

class ParameterService(pr_svc_grpc.ParameterServiceServicer):
    def __init__(self):
        self.params = {"joint_position_command.cutoff_frequency": "5.0"}

    def GetParameterList(self, request, context):
        items = [pr_pb.ParameterListItem(name=k) for k in self.params.keys()]
        return pr_pb.GetParameterListResponse(parameters=items)

    def GetParameter(self, request, context):
        return pr_pb.GetParameterResponse(value=self.params.get(request.name, ""))

    def SetParameter(self, request, context):
        self.params[request.name] = request.value
        return pr_pb.SetParameterResponse()

    # No-ops for factory/reset variants:
    def FactoryResetAllParameters(self, req, ctx): return pr_pb.FactoryResetAllParametersResponse()
    def FactoryResetParameter(self, req, ctx):     return pr_pb.FactoryResetParameterResponse()
    def ResetAllParameters(self, req, ctx):        return pr_pb.ResetAllParametersResponse()
    def ResetParameter(self, req, ctx):            return pr_pb.ResetParameterResponse()
    def ResetAllParametersToDefault(self, req, ctx): return pr_pb.ResetAllParametersToDefaultResponse()
    def ResetParameterToDefault(self, req, ctx):     return pr_pb.ResetParameterToDefaultResponse()

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=16))
    rc_svc_grpc.add_RobotCommandServiceServicer_to_server(RobotCommandService(), server)
    rs_svc_grpc.add_RobotStateServiceServicer_to_server(RobotStateService(), server)
    cm_svc_grpc.add_ControlManagerServiceServicer_to_server(ControlManagerService(), server)
    pw_svc_grpc.add_PowerServiceServicer_to_server(PowerService(), server)
    jo_svc_grpc.add_JointOperationServiceServicer_to_server(JointOperationService(), server)
    pr_svc_grpc.add_ParameterServiceServicer_to_server(ParameterService(), server)
    server.add_insecure_port("[::]:50051")
    server.start()
    print("Stub RBY server (OK responses) on :50051")
    server.wait_for_termination()

if __name__ == "__main__":
    serve()
