# send_right_arm_pose.py
import math
import grpc
from google.protobuf.duration_pb2 import Duration

import rb.api.robot_command_service_pb2_grpc as rc_svc_grpc
import rb.api.robot_command_pb2              as rc_pb

def build_right_arm_req(rad7, seconds=3.0):
    # Root request
    req = rc_pb.RobotCommandRequest()
    rc  = req.robot_command

    # Select the "component_based_command" oneof
    cbc = rc.component_based_command

    # Body -> BodyComponentBasedCommand
    bcbc = cbc.body_command.body_component_based_command

    # Right arm -> joint_position_command
    rac = bcbc.right_arm_command
    jpc = rac.joint_position_command

    # Fill fields
    jpc.position.extend(float(x) for x in rad7)
    d = Duration()
    # seconds may be float; handle fractional seconds cleanly:
    d.seconds = int(seconds)
    d.nanos   = int((seconds - int(seconds)) * 1e9)
    jpc.minimum_time.CopyFrom(d)

    return req

def main():
    # Example target: 7 DOFs (radians)
    target = [0, math.radians(-120), 0.0, 0.0, 0.0, math.radians(70), 0.0]
    # target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    channel = grpc.insecure_channel("localhost:50051")
    stub = rc_svc_grpc.RobotCommandServiceStub(channel)

    req = build_right_arm_req(target, seconds=3.0)
    resp = stub.RobotCommand(req, timeout=10.0)
    print("Feedback status:", getattr(resp.feedback, "status", None))
    print("Finish code:", getattr(resp.feedback, "finish_code", None))

if __name__ == "__main__":
    main()
