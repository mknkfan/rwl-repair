"""Replay a GA layout's cycle in MuJoCo, and ask it three questions the
study's own evaluator answers only approximately or not at all.

1. **Does the arm clear the machines?**  :func:`mesh_clearance` walks the
   planned cycle kinematically and measures the exact distance between each
   link's convex solid and each machine box.  The evaluator bounds the arm by
   spheres, which are conservative, so this gap should never be the smaller
   of the two; if it is, the sphere model is not bounding the solid.
2. **Can the motors deliver it?**  :func:`required_torques` runs inverse
   dynamics along the planned motion -- the trapezoid gives ``q``, ``q_dot``
   and ``q_ddot`` exactly -- and adds Coulomb friction.  The GA limits joint
   *rates and accelerations*; whether those accelerations are affordable at
   every posture is a torque question it never asks.
3. **Does a controller actually follow it?**  :func:`track_cycle` simulates
   the robot under physics -- gravity, reflected motor inertia, friction,
   contact with the machines, torques clipped at the limits -- driven by a
   computed-torque controller, and reports how far the tool strays from the
   plan and what it touches.

Everything is in the manufacturer's six angles inside MuJoCo; the study's
four map across by :meth:`Puma560Arm.to_puma_angles`, with the two roll
joints held at zero.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import mujoco
import numpy as np

from geometry import Machine, box_frames, points_over_boxes
from puma_mjcf import PUMA_TORQUE_LIMIT, TOOL_PARTS, build_mjcf, link_side_friction
from robot import PUMA_JOINT_INDEX, Puma560Arm
from trajectory import plan_cycle, dwell_indices

JOINT_NAMES = ("waist", "shoulder", "elbow", "wrist roll", "wrist pitch", "flange roll")


# ----------------------------------------------------------------------
# the reference motion
# ----------------------------------------------------------------------


@dataclass
class Reference:
    """The planned cycle sampled on a fixed clock, in manufacturer angles."""

    t: np.ndarray        # (n,)
    q: np.ndarray        # (n, 6)
    qd: np.ndarray       # (n, 6)
    qdd: np.ndarray      # (n, 6)
    #: Index of the tour segment each sample belongs to, -1 during a dwell.
    segment: np.ndarray  # (n,)

    @property
    def dt(self) -> float:
        return float(self.t[1] - self.t[0])


def _to_six(arm: Puma560Arm, q4: np.ndarray, is_angle: bool) -> np.ndarray:
    """Four model joints -> six manufacturer joints.

    Angles pick up the fixed offsets; rates and accelerations do not, since
    the offsets are constant.
    """
    if is_angle:
        return arm.to_puma_angles(q4)
    six = np.zeros(q4.shape[:-1] + (6,))
    for k, joint in enumerate(PUMA_JOINT_INDEX):
        six[..., joint] = q4[..., k]
    return six


def sample_reference(
    arm: Puma560Arm,
    tour: np.ndarray,
    velocity_limit: np.ndarray,
    acceleration_limit: np.ndarray,
    dwell: float,
    station_index: Optional[Sequence[int]],
    dt: float = 0.001,
) -> Reference:
    """The trapezoidal cycle of :mod:`trajectory`, with its acceleration.

    :func:`trajectory.sample_cycle` returns ``q`` and ``q_dot``; the dynamics
    also need ``q_ddot``, which on a trapezoid is ``+a``, ``0`` or ``-a`` of
    the path parameter times the joint displacement.
    """
    segments = plan_cycle(tour, velocity_limit, acceleration_limit)
    holds = set(int(k) for k in dwell_indices(tour, station_index))

    # Lay the segments and dwells end to end on one clock.
    pieces: List[Tuple[float, float, int]] = []   # (start, end, segment or -1)
    clock = 0.0
    for k, segment in enumerate(segments):
        pieces.append((clock, clock + segment.duration, k))
        clock += segment.duration
        if dwell > 0.0 and (k + 1) in holds:
            pieces.append((clock, clock + dwell, -1 - (k + 1)))
            clock += dwell

    t = np.arange(0.0, clock + 0.5 * dt, dt)
    q = np.empty((t.size, 4))
    qd = np.zeros((t.size, 4))
    qdd = np.zeros((t.size, 4))
    which = np.full(t.size, -1, dtype=int)

    for start, end, tag in pieces:
        mask = (t >= start) & (t < end) if end < clock else (t >= start)
        if tag < 0:
            q[mask] = tour[-1 - tag]
            continue
        segment = segments[tag]
        local = t[mask] - start
        s, s_dot = segment.path_parameter(local)
        a = segment.acceleration
        ramp = segment.duration / 2.0 if segment.triangular else segment.peak_rate / a
        s_ddot = np.where(local < ramp, a, np.where(local > segment.duration - ramp, -a, 0.0))
        q[mask] = tour[tag] + s[:, None] * segment.delta
        qd[mask] = s_dot[:, None] * segment.delta
        qdd[mask] = s_ddot[:, None] * segment.delta
        which[mask] = tag

    return Reference(
        t=t,
        q=_to_six(arm, q, True),
        qd=_to_six(arm, qd, False),
        qdd=_to_six(arm, qdd, False),
        segment=which,
    )


# ----------------------------------------------------------------------
# the scene
# ----------------------------------------------------------------------


@dataclass
class Cell:
    """A compiled MuJoCo scene and the bookkeeping the checks need."""

    arm: Puma560Arm
    machines: List[Machine]
    xml: str
    model: mujoco.MjModel
    data: mujoco.MjData
    #: Geom ids of the moving robot, split into links and the tool.
    arm_geoms: List[int]
    tool_geoms: List[int]
    machine_geoms: List[int]
    tip_site: int
    #: Same scene with every constraint off: what the controller's internal
    #: model and the torque audit run on, so contacts and friction loss do not
    #: leak into a "required torque".
    ideal: mujoco.MjModel = field(repr=False, default=None)


def build_cell(arm: Puma560Arm, machines: Sequence[Machine],
               ports: Optional[np.ndarray] = None, friction: bool = True) -> Cell:
    xml = build_mjcf(arm, machines, ports, friction=friction)
    model = mujoco.MjModel.from_xml_string(xml)
    ideal = mujoco.MjModel.from_xml_string(xml)
    ideal.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT

    arm_geoms, tool_geoms = [], []
    for g in range(model.ngeom):
        if model.geom_bodyid[g] == 0:
            continue   # the pedestal and the floor are static
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
        (tool_geoms if name in TOOL_PARTS else arm_geoms).append(g)
    machine_geoms = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, m.name) for m in machines]
    return Cell(
        arm=arm, machines=list(machines), xml=xml, model=model,
        data=mujoco.MjData(model), arm_geoms=arm_geoms, tool_geoms=tool_geoms,
        machine_geoms=machine_geoms,
        tip_site=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tool_tip"),
        ideal=ideal,
    )


def _geom_name(cell: Cell, g: int) -> str:
    return mujoco.mj_id2name(cell.model, mujoco.mjtObj.mjOBJ_GEOM, g)


def _served(cell: Cell, tips: np.ndarray, sag: float = 0.0) -> np.ndarray:
    """``(n, n_machines)``: is the tool tip over each machine's footprint?

    The approach corridor, exactly as the evaluator defines it: the gripper
    may touch the machine it is directly above, and nothing else.  ``sag``
    lets the tip sit that far *below* the top face and still count -- under
    physics a gripper resting on a surface presses a few micrometres into it,
    which the kinematic test (tip at or above the top) would call a collision.
    """
    centres, half, rotations = box_frames(cell.machines)
    lifted = np.array(tips, dtype=float, copy=True)
    lifted[..., 2] += sag
    return points_over_boxes(lifted, centres, half, rotations)

#: How far below a machine's top the tool tip may be pressed under physics and
#: still be in the approach corridor.  Tracking error is well under this.
CONTACT_SAG = 0.001


# ----------------------------------------------------------------------
# 1. clearance, against the solid rather than its bounding spheres
# ----------------------------------------------------------------------


@dataclass
class Clearance:
    worst_gap: float
    worst_part: str
    worst_machine: str
    worst_time: float
    #: Smallest gap per robot part over the whole cycle, metres.
    per_part: Dict[str, float]


def mesh_clearance(cell: Cell, reference: Reference, stride: int = 10,
                   distmax: float = 0.30) -> Clearance:
    """Exact solid-to-box distance along the cycle, every ``stride`` samples.

    Gaps beyond ``distmax`` are reported as ``distmax``: past that nothing
    is close enough to matter, and capping it is what keeps the distance
    queries cheap.
    """
    m, d = cell.model, cell.data
    per_part: Dict[str, float] = {}
    worst = (math.inf, "", "", 0.0)
    for i in range(0, reference.t.size, stride):
        d.qpos[:] = reference.q[i]
        mujoco.mj_kinematics(m, d)
        over = _served(cell, d.site_xpos[cell.tip_site][None, :])[0]
        for g in cell.arm_geoms + cell.tool_geoms:
            name = _geom_name(cell, g)
            for k, mg in enumerate(cell.machine_geoms):
                if g in cell.tool_geoms and over[k]:
                    continue
                gap = mujoco.mj_geomDistance(m, d, g, mg, distmax, None)
                per_part[name] = min(per_part.get(name, distmax), gap)
                if gap < worst[0]:
                    worst = (gap, name, cell.machines[k].name, float(reference.t[i]))
    return Clearance(worst[0], worst[1], worst[2], worst[3], per_part)


# ----------------------------------------------------------------------
# 2. the torque the plan demands
# ----------------------------------------------------------------------


@dataclass
class TorqueDemand:
    tau: np.ndarray            # (n, 6), N m, rigid body + friction
    peak: np.ndarray           # (6,)
    limit: np.ndarray          # (6,)
    peak_time: np.ndarray      # (6,)

    @property
    def utilisation(self) -> np.ndarray:
        """Peak over limit, per joint: above 1 the motor cannot do it."""
        return self.peak / self.limit


def required_torques(cell: Cell, reference: Reference) -> TorqueDemand:
    """Inverse dynamics along the plan, plus Coulomb friction.

    Rigid-body terms, reflected motor inertia and viscous friction come from
    ``mj_inverse`` on the constraint-free model; Coulomb friction is added as
    ``Tc sign(q_dot)``, which is what a real drive must overcome and what the
    symmetric ``frictionloss`` in the physics model resists.
    """
    m = cell.ideal
    d = mujoco.MjData(m)
    _, coulomb = link_side_friction()
    tau = np.empty_like(reference.q)
    for i in range(reference.t.size):
        d.qpos[:] = reference.q[i]
        d.qvel[:] = reference.qd[i]
        d.qacc[:] = reference.qdd[i]
        mujoco.mj_inverse(m, d)
        tau[i] = d.qfrc_inverse + coulomb * np.sign(reference.qd[i])
    peak_index = np.abs(tau).argmax(axis=0)
    return TorqueDemand(
        tau=tau,
        peak=np.abs(tau).max(axis=0),
        limit=np.asarray(PUMA_TORQUE_LIMIT, dtype=float),
        peak_time=reference.t[peak_index],
    )


# ----------------------------------------------------------------------
# 3. a controller following it under physics
# ----------------------------------------------------------------------


@dataclass
class Tracking:
    t: np.ndarray
    q: np.ndarray               # (n, 6) achieved
    tau: np.ndarray             # (n, 6) commanded, after clipping
    tip_error: np.ndarray       # (n,) metres, achieved tool tip vs planned
    joint_error_max: np.ndarray  # (6,) rad
    saturated_fraction: np.ndarray  # (6,) share of steps at the torque limit
    #: Every contact between the robot and a machine outside the approach
    #: corridor: (time, part, machine, penetration in metres).
    collisions: List[Tuple[float, str, str, float]]
    #: Contacts the gripper makes with the top of the machine it is serving.
    approach_contacts: int
    station_tip_error: np.ndarray  # (n_stations,) at the end of each dwell


def track_cycle(cell: Cell, reference: Reference, natural_frequency: float = 25.0,
                damping_ratio: float = 1.0, friction_compensation: float = 1.0) -> Tracking:
    """Computed-torque control of the full physics model along the plan.

    The controller asks its own *ideal* model (no contacts, no friction loss)
    for the torque that gives ``q_ddot_ref + Kp e + Kd e_dot`` and adds a
    Coulomb feed-forward scaled by ``friction_compensation`` -- 1 is a
    perfectly identified drive, 0 an uncompensated one.  The plant is the
    real model: friction loss, contact with the machines, and the torque
    clipped at the motor limits.  One control update per physics step, at the
    reference's own clock.
    """
    m, d = cell.model, cell.data
    ctrl_model = cell.ideal
    ctrl_data = mujoco.MjData(ctrl_model)
    if abs(m.opt.timestep - reference.dt) > 1e-12:
        m.opt.timestep = reference.dt

    kp = natural_frequency**2
    kd = 2.0 * damping_ratio * natural_frequency
    _, coulomb = link_side_friction()
    limit = np.asarray(PUMA_TORQUE_LIMIT, dtype=float)

    mujoco.mj_resetData(m, d)
    d.qpos[:] = reference.q[0]
    d.qvel[:] = reference.qd[0]
    mujoco.mj_forward(m, d)

    n = reference.t.size
    q_log = np.empty((n, 6))
    tau_log = np.empty((n, 6))
    tips = np.empty((n, 3))
    saturated = np.zeros(6)
    collisions: List[Tuple[float, str, str, float]] = []
    approach = 0
    arm_set = set(cell.arm_geoms) | set(cell.tool_geoms)
    tool_set = set(cell.tool_geoms)
    machine_index = {g: k for k, g in enumerate(cell.machine_geoms)}

    # Planned tip, for the error: kinematics of the reference itself.
    ref_data = mujoco.MjData(ctrl_model)
    planned_tips = np.empty((n, 3))
    for i in range(n):
        ref_data.qpos[:] = reference.q[i]
        mujoco.mj_kinematics(ctrl_model, ref_data)
        planned_tips[i] = ref_data.site_xpos[cell.tip_site]

    for i in range(n):
        error = reference.q[i] - d.qpos
        error_rate = reference.qd[i] - d.qvel
        ctrl_data.qpos[:] = d.qpos
        ctrl_data.qvel[:] = d.qvel
        ctrl_data.qacc[:] = reference.qdd[i] + kp * error + kd * error_rate
        mujoco.mj_inverse(ctrl_model, ctrl_data)
        tau = ctrl_data.qfrc_inverse + friction_compensation * coulomb * np.sign(reference.qd[i])
        clipped = np.clip(tau, -limit, limit)
        saturated += np.abs(tau) > limit
        d.ctrl[:] = clipped

        q_log[i] = d.qpos
        tau_log[i] = clipped
        tips[i] = d.site_xpos[cell.tip_site]

        # Robot-machine contacts, sorted into the corridor and everything else.
        over = None
        for c in range(d.ncon):
            contact = d.contact[c]
            pair = {contact.geom1, contact.geom2}
            robot = pair & arm_set
            others = [g for g in pair if g in machine_index]
            if not robot or not others:
                continue
            part, machine = robot.pop(), others[0]
            if over is None:
                over = _served(cell, tips[i][None, :], CONTACT_SAG)[0]
            if part in tool_set and over[machine_index[machine]]:
                approach += 1
                continue
            collisions.append((float(reference.t[i]), _geom_name(cell, part),
                               cell.machines[machine_index[machine]].name, float(-contact.dist)))

        mujoco.mj_step(m, d)

    tip_error = np.linalg.norm(tips - planned_tips, axis=1)
    # The tip error that matters most is where the tool is working: at the
    # end of each dwell, when the part would be picked or placed.
    dwell_ends = [i for i in range(1, n) if reference.segment[i - 1] < 0 <= reference.segment[i]]
    return Tracking(
        t=reference.t,
        q=q_log,
        tau=tau_log,
        tip_error=tip_error,
        joint_error_max=np.abs(q_log - reference.q).max(axis=0),
        saturated_fraction=saturated / n,
        collisions=collisions,
        approach_contacts=approach,
        station_tip_error=tip_error[np.asarray(dwell_ends, dtype=int) - 1],
    )


# ----------------------------------------------------------------------
# pictures
# ----------------------------------------------------------------------


def render_frames(cell: Cell, q_path: np.ndarray, every: int, width: int = 480,
                  height: int = 360, azimuth: float = 135.0, elevation: float = -38.0,
                  distance: float = 2.1) -> List[np.ndarray]:
    """Offscreen frames of the cell at every ``every``-th configuration."""
    m, d = cell.model, cell.data
    renderer = mujoco.Renderer(m, height, width)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (cell.arm.base[0], cell.arm.base[1], 0.30)
    camera.distance, camera.azimuth, camera.elevation = distance, azimuth, elevation
    frames = []
    for i in range(0, q_path.shape[0], every):
        d.qpos[:] = q_path[i]
        mujoco.mj_kinematics(m, d)
        renderer.update_scene(d, camera)
        frames.append(renderer.render().copy())
    renderer.close()
    return frames
