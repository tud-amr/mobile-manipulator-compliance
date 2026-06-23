import importlib.resources as pkg_resources
import os
import sys
import numpy as np
import casadi
import pinocchio
import pinocchio.casadi as cpin
import compliant_control.mujoco.models as models
import compliant_control.control.symbolics as symbolics


def load_robot() -> pinocchio.RobotWrapper:
    """Load the pinocchio robot."""
    urdf = os.path.abspath(sys.argv[1])
    urdf_package = os.path.dirname(urdf)
    robot = pinocchio.RobotWrapper.BuildFromURDF(urdf, urdf_package)
    frame_id = robot.model.getFrameId("GRIPPER_FRAME")
    joint_id = robot.model.getJointId("5")
    location = pinocchio.SE3(1)
    location.translation = np.array([0, 0, 0.156])
    frame = pinocchio.Frame(
        "END_EFFECTOR",
        joint_id,
        frame_id,
        location,
        pinocchio.OP_FRAME,
    )
    robot.model.addFrame(frame)
    robot.data = robot.model.createData()
    return robot


def define_matrices(robot: pinocchio.RobotWrapper) -> None:
    """Create the symbolic matrices using Casadi."""
    model = cpin.Model(robot.model)
    data = model.createData()

    q = casadi.SX.sym("q", model.nq, 1)
    dq = casadi.SX.sym("dq", model.nq, 1)

    g = cpin.computeGeneralizedGravity(model, data, q)

    M = cpin.crba(model, data, q)
    Minv = casadi.solve(M, casadi.SX.eye(M.size1()))

    C = cpin.computeCoriolisMatrix(model, data, q, dq)

    frame_id = robot.model.getFrameId("END_EFFECTOR")
    cpin.framesForwardKinematics(model, data, q)
    x = data.oMf[frame_id].translation
    # rotation as quaternion
    rot_matrix = data.oMf[frame_id].rotation





    J = cpin.getFrameJacobian(
        model,
        data,
        frame_id,
        pinocchio.LOCAL_WORLD_ALIGNED,
    )[:3, :]

    dx = J @ dq

    dJ = cpin.getFrameJacobianTimeVariation(
        model,
        data,
        frame_id,
        pinocchio.LOCAL_WORLD_ALIGNED,
    )[:3, :]

    Jinv = Minv @ J.T @ casadi.solve((J @ Minv @ J.T), casadi.SX.eye(3))

    dx_sym = casadi.SX.sym("x", 3, 1)
    dq_inv = Jinv @ dx_sym

    lam = Jinv.T @ M @ Jinv
    mu = Jinv.T @ (C - M @ Jinv @ dJ) @ Jinv

    f = casadi.SX.sym("f", 3, 1)
    T = J.T @ f

    t = casadi.SX.sym("t", 6, 1)
    F = Jinv.T @ t

    N = casadi.SX.eye(6) - J.T @ Jinv.T
    Nv = casadi.SX.eye(6) - Jinv @ J

    functions: list[casadi.Function] = [
        casadi.Function("g", [q], [g]),
        casadi.Function("x", [q], [x]),
        casadi.Function("rot", [q], [rot_matrix]),
        casadi.Function("dx", [q, dq], [dx]),
        casadi.Function("J", [q], [J]),
        casadi.Function("JT", [q], [J.T]),
        casadi.Function("lam", [q], [lam]),
        casadi.Function("mu", [q, dq], [mu]),
        casadi.Function("T", [q, f], [T]),
        casadi.Function("F", [q, t], [F]),
        casadi.Function("dq", [q, dx_sym], [dq_inv]),
        casadi.Function("N", [q], [N]),
        casadi.Function("Nv", [q], [Nv]),
    ]

    current_dir = os.getcwd()
    output_dir = "."
    os.chdir(output_dir)

    for function in functions:
        function.generate(function.name())

    os.chdir(current_dir)


def main() -> None:
    """Generate symbolics."""
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <path/to/urdf>")
        return
    robot = load_robot()
    define_matrices(robot)


if __name__ == "__main__":
    main()
