"""The Puma 560 and a workcell as a MuJoCo model, built from the same numbers
the rest of the study uses.

**Kinematics come from the DH table, not from a second hand-built chain.**
Each of the manufacturer's six joints is one MuJoCo body whose frame is the
previous DH frame turned by that joint -- ``B_i = DH_{i-1} Rz(theta_i)`` --
so the hinge is the body's own ``z`` axis and the fixed part of the DH row,
``Tz(d) Tx(a) Rx(alpha)``, is simply where the next body sits.  ``qpos`` is
therefore in the *manufacturer's* angles; :meth:`Puma560Arm.to_puma_angles`
converts from this study's four.

**The solid model is the one the figures draw.**  :func:`armmesh.build_parts`
gives each part in one of :meth:`Puma560Arm.link_frames`' five frames.  The
transform from such a frame to the MuJoCo body it rides on is *measured*
rather than derived: :func:`frame_to_body` computes it at one configuration
and checks it is the same at a spread of others, which is the statement that
the study's 4-DOF model and the DH chain are one mechanism.  MuJoCo collides
the convex hull of each part; every part is convex already, so the hull is
the part.

**Dynamics are Armstrong, Khatib and Burdick (1986) as Corke's Robotics
Toolbox ships them** (``mdl_puma560``): link masses, centres of mass and
inertias in each DH frame, plus motor inertia ``Jm`` and gear ratio ``G``,
which enter as the joint armature ``Jm G^2`` -- at the Puma's gear ratios the
reflected motor inertia rivals the link's own, so leaving it out would
understate every torque.  Viscous friction becomes joint damping and the
Coulomb term a symmetric ``frictionloss``; both are optional, because the
cross-check against the toolbox's recursive Newton-Euler wants them off.

Link 1 has no measured mass (only the inertia about its own axis, 0.35 kg m^2)
and MuJoCo needs a positive one, so it gets a token gram.  Its other two
principal moments are unobservable -- the body turns about one fixed axis
only -- and are set to the smallest values MuJoCo's triangle inequality
admits.

Link 3's published principal moments, ``(0.066, 0.086, 0.0125)``, violate the
triangle inequality every rigid body satisfies (``0.066 + 0.0125 < 0.086``) --
a known flaw of the measured set -- and MuJoCo refuses them.  ``Izz`` is
raised to 0.020, the least value that is physical.  That axis is the forearm's
own roll axis, which only the locked joint 4 turns, so the change reaches the
driven joints only through the small off-axis projection of that moment.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from armmesh import Part, build_parts
from geometry import Machine
from robot import PUMA_DH, PUMA_TRAVEL, Puma560Arm

#: Per link: mass (kg), centre of mass in its DH frame (m), principal
#: inertia about the centre of mass along the DH axes (kg m^2).
#: Armstrong, Khatib & Burdick (1986), via Corke's ``mdl_puma560``.
PUMA_LINKS: Tuple[Tuple[float, Tuple[float, ...], Tuple[float, ...]], ...] = (
    (0.001, (0.0, 0.0, 0.0), (0.175, 0.35, 0.175)),   # see module docstring
    (17.40, (-0.3638, 0.0060, 0.2275), (0.130, 0.524, 0.539)),
    (4.80, (-0.0203, -0.0141, 0.0700), (0.066, 0.086, 0.0200)),   # Izz: see docstring
    (0.82, (0.0, 0.019, 0.0), (1.8e-3, 1.3e-3, 1.8e-3)),
    (0.34, (0.0, 0.0, 0.0), (0.3e-3, 0.4e-3, 0.3e-3)),
    (0.09, (0.0, 0.0, 0.032), (0.15e-3, 0.15e-3, 0.04e-3)),
)

#: Motor rotor inertia (kg m^2) and gear ratio, joints 1..6.
PUMA_MOTOR_INERTIA = (200e-6, 200e-6, 200e-6, 33e-6, 33e-6, 33e-6)
PUMA_GEAR = (-62.6111, 107.815, -53.7063, 76.0364, 71.923, 76.686)

#: Motor-side viscous friction (N m s/rad) and Coulomb friction (N m,
#: positive and negative direction), joints 1..6.  Same source.
PUMA_VISCOUS = (1.48e-3, 0.817e-3, 1.38e-3, 71.2e-6, 82.6e-6, 36.7e-6)
PUMA_COULOMB = (
    (0.395, -0.435), (0.126, -0.071), (0.132, -0.105),
    (11.2e-3, -16.9e-3), (9.26e-3, -14.5e-3), (3.96e-3, -10.5e-3),
)

#: Joint torque limits, N m, joints 1..6.  **Not verified against a Unimation
#: document**: these are the figures commonly quoted in the time-optimal
#: trajectory literature for the Puma 560.  Treat a margin against them as
#: indicative, and substitute your controller's real limits if you have them.
PUMA_TORQUE_LIMIT = (97.6, 186.4, 89.4, 24.2, 20.1, 21.3)

#: Which link frame of :meth:`Puma560Arm.link_frames` rides on which MuJoCo
#: body.  Frame 0 is the fixed pedestal (body 0, the world); the wrist frame
#: follows joint 5, since joint 4 is held at zero.
FRAME_BODY = {0: 0, 1: 1, 2: 2, 3: 3, 4: 5}

#: Geoms whose parts belong to the tool, for the approach-corridor exemption
#: :class:`CycleTimeEvaluator` also makes: the gripper is *meant* to touch the
#: top of the machine it is serving.
TOOL_PARTS = ("wrist", "flange", "palm", "finger_a", "finger_b")


def reflected_armature() -> np.ndarray:
    """``Jm G^2`` per joint: the rotor inertia as the link side sees it."""
    return np.array([j * g * g for j, g in zip(PUMA_MOTOR_INERTIA, PUMA_GEAR)])


def link_side_friction() -> Tuple[np.ndarray, np.ndarray]:
    """``(viscous, coulomb)`` at the link: ``B G^2`` and mean ``|Tc| |G|``.

    MuJoCo's ``frictionloss`` is symmetric; the measured Coulomb terms are
    not, so the mean magnitude of the two directions is used.
    """
    gear = np.abs(np.asarray(PUMA_GEAR))
    viscous = np.asarray(PUMA_VISCOUS) * gear**2
    coulomb = np.array([0.5 * (abs(p) + abs(n)) for p, n in PUMA_COULOMB]) * gear
    return viscous, coulomb


# ----------------------------------------------------------------------
# frames
# ----------------------------------------------------------------------


def _dh_row(d: float, a: float, alpha: float) -> np.ndarray:
    """The fixed part of one DH row, ``Tz(d) Tx(a) Rx(alpha)``."""
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [[1.0, 0.0, 0.0, a], [0.0, ca, -sa, 0.0], [0.0, sa, ca, d], [0.0, 0.0, 0.0, 1.0]]
    )


def _rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    frame = np.eye(4)
    frame[:2, :2] = [[c, -s], [s, c]]
    return frame


def body_frames(arm: Puma560Arm, theta: Sequence[float]) -> np.ndarray:
    """World poses of the six joint bodies at manufacturer angles ``theta``.

    Index 0 is the world itself, so the result is ``(7, 4, 4)`` and index
    ``i`` is the body joint ``i`` turns.
    """
    frames = [np.eye(4)]
    dh = np.eye(4)
    dh[:3, 3] = (arm.base[0], arm.base[1], arm.d1)
    for angle, (d, a, alpha) in zip(theta, PUMA_DH):
        body = dh @ _rot_z(float(angle))
        frames.append(body)
        dh = body @ _dh_row(d, a, alpha)
    return np.stack(frames)


def frame_to_body(arm: Puma560Arm, trials: int = 25, seed: int = 0) -> Dict[int, np.ndarray]:
    """The constant transform from each link frame to the body it rides on.

    Measured at the zero pose and then checked at ``trials`` random ones: if
    the study's kinematics and the DH chain ever disagreed, the transform
    would change with the pose and this raises instead of building a robot
    whose drawing and physics part company.
    """
    def relative(q: np.ndarray) -> Dict[int, np.ndarray]:
        links = arm.link_frames(q)
        bodies = body_frames(arm, arm.to_puma_angles(q))
        return {f: np.linalg.solve(bodies[b], links[f]) for f, b in FRAME_BODY.items()}

    reference = relative(np.zeros(4))
    rng = np.random.default_rng(seed)
    low, high, _, _ = arm.limits.as_arrays()
    for _ in range(trials):
        other = relative(low + rng.random(4) * (high - low))
        for frame, transform in reference.items():
            if not np.allclose(other[frame], transform, atol=1e-9):
                raise AssertionError(
                    f"link frame {frame} does not ride rigidly on body "
                    f"{FRAME_BODY[frame]}: the model and the DH table disagree"
                )
    return reference


# ----------------------------------------------------------------------
# MJCF
# ----------------------------------------------------------------------


def _fmt(values: Sequence[float]) -> str:
    return " ".join(f"{float(v):.6g}" for v in values)


def _quat(rotation: np.ndarray) -> np.ndarray:
    """Rotation matrix -> MuJoCo's ``(w, x, y, z)``."""
    r = rotation
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    return np.array([w, x, y, z])


def _mesh_vertices(part: Part, transform: np.ndarray) -> np.ndarray:
    """A part's corner points in its body's frame, duplicates dropped."""
    points = np.unique(np.round(np.vstack(part.polys), 9), axis=0)
    homogeneous = np.hstack([points, np.ones((points.shape[0], 1))])
    return (homogeneous @ transform.T)[:, :3]


def build_mjcf(
    arm: Puma560Arm,
    machines: Sequence[Machine] = (),
    ports: Optional[np.ndarray] = None,
    friction: bool = True,
    timestep: float = 0.001,
) -> str:
    """The whole scene as an MJCF string: robot, machines, floor, ports.

    ``friction=False`` drops the joint damping and friction loss, which is
    what a comparison against a friction-free inverse-dynamics model needs.
    The robot is torque-driven -- one ``motor`` per joint, clipped at
    :data:`PUMA_TORQUE_LIMIT` -- so whatever drives it has to supply the
    torque a real controller would.
    """
    transforms = frame_to_body(arm)
    parts = build_parts(arm, sides=20)
    armature = reflected_armature()
    viscous, coulomb = link_side_friction()
    if not friction:
        viscous = np.zeros(6)
        coulomb = np.zeros(6)

    assets: List[str] = []
    on_body: Dict[int, List[str]] = {b: [] for b in range(7)}
    for part in parts:
        body = FRAME_BODY[part.frame]
        vertices = _mesh_vertices(part, transforms[part.frame])
        assets.append(f'    <mesh name="{part.name}" vertex="{_fmt(vertices.ravel())}"/>')
        rgba = _fmt(tuple(part.colour) + (1.0,))
        group = "tool" if part.name in TOOL_PARTS else "arm"
        on_body[body].append(
            f'<geom name="{part.name}" class="{group}" type="mesh" mesh="{part.name}" rgba="{rgba}"/>'
        )

    # The chain: body i sits where the previous DH row left off.
    def body_xml(i: int, indent: str) -> str:
        if i > 6:
            return ""
        if i == 1:
            placement = np.eye(4)
            placement[:3, 3] = (arm.base[0], arm.base[1], arm.d1)
        else:
            placement = _dh_row(*PUMA_DH[i - 2])
        mass, com, inertia = PUMA_LINKS[i - 1]
        # Link inertia is published in the DH frame at the *end* of the link,
        # which is this body's frame carried through its own DH row.
        row = _dh_row(*PUMA_DH[i - 1])
        low, high = PUMA_TRAVEL[i - 1]
        lines = [
            f'{indent}<body name="link{i}" pos="{_fmt(placement[:3, 3])}" '
            f'quat="{_fmt(_quat(placement[:3, :3]))}">',
            f'{indent}  <joint name="j{i}" range="{low} {high}" armature="{armature[i - 1]:.6g}" '
            f'damping="{viscous[i - 1]:.6g}" frictionloss="{coulomb[i - 1]:.6g}"/>',
            f'{indent}  <inertial pos="{_fmt(row[:3, :3] @ np.asarray(com) + row[:3, 3])}" '
            f'quat="{_fmt(_quat(row[:3, :3]))}" mass="{mass}" diaginertia="{_fmt(inertia)}"/>',
        ]
        lines += [f"{indent}  {geom}" for geom in on_body[i]]
        if i == 5:
            # The tool tip: (tool, 0, 0) of the wrist frame, which rides on body 5.
            tip = transforms[4] @ np.array([arm.tool, 0.0, 0.0, 1.0])
            lines.append(f'{indent}  <site name="tool_tip" pos="{_fmt(tip[:3])}" size="0.008" rgba="1 0 0 1"/>')
        inner = body_xml(i + 1, indent + "  ")
        if inner:
            lines.append(inner)
        lines.append(f"{indent}</body>")
        return "\n".join(lines)

    # Frame 0 is the world moved to the base, so these vertices already carry
    # the base offset and the pedestal goes straight into the world.
    pedestal = [f"    {geom}" for geom in on_body[0]]

    scene: List[str] = []
    for k, machine in enumerate(machines):
        scene.append(
            f'    <geom name="{machine.name}" class="machine" type="box" '
            f'pos="{_fmt((machine.x, machine.y, machine.height / 2.0))}" '
            f'euler="0 0 {math.radians(machine.rotation):.6g}" '
            f'size="{_fmt((machine.width / 2.0, machine.depth / 2.0, machine.height / 2.0))}"/>'
        )
    if ports is not None:
        for k, port in enumerate(np.atleast_2d(ports)):
            scene.append(
                f'    <site name="port{k + 1}" pos="{_fmt(port)}" size="0.012" rgba="0.1 0.6 1 1"/>'
            )

    motors = "\n".join(
        f'    <motor name="m{i}" joint="j{i}" ctrlrange="{-lim} {lim}"/>'
        for i, lim in enumerate(PUMA_TORQUE_LIMIT, start=1)
    )

    return f"""<mujoco model="puma560_cell">
  <compiler angle="degree" inertiafromgeom="false" autolimits="true"/>
  <option timestep="{timestep}" integrator="implicitfast" gravity="0 0 -9.81"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.6 0.6 0.6"/>
  </visual>
  <default>
    <!-- Robot geoms collide with machines and the floor, never each other. -->
    <default class="arm"><geom contype="1" conaffinity="2" group="1"/></default>
    <default class="tool"><geom contype="1" conaffinity="2" group="1"/></default>
    <default class="machine"><geom contype="2" conaffinity="1" group="2" rgba="0.55 0.62 0.72 1"/></default>
  </default>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.93 0.95 0.97" rgb2="0.62 0.68 0.75" width="64" height="64"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.84 0.86" rgb2="0.74 0.76 0.79" width="256" height="256"/>
    <material name="floor" texture="grid" texrepeat="8 8"/>
{chr(10).join(assets)}
  </asset>
  <worldbody>
    <light pos="1 -1 3" dir="-0.3 0.3 -1" diffuse="0.5 0.5 0.5"/>
    <geom name="floor" type="plane" size="1.5 1.5 0.05" material="floor" contype="2" conaffinity="1"/>
{chr(10).join(pedestal)}
{chr(10).join(scene)}
{body_xml(1, "    ")}
  </worldbody>
  <actuator>
{motors}
  </actuator>
</mujoco>
"""
