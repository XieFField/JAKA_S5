"""Build the MuJoCo model from the canonical project Xacro description."""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import mujoco
import numpy as np
import yaml
import xacro


JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))
PRONE_BACK_FLEX_NAME = "prone_back_soft_tissue"


@dataclass(frozen=True)
class ContactPadConfiguration:
    """Contact fixture values and MuJoCo-specific solver parameters."""

    position: Tuple[float, float, float]
    size: Tuple[float, float, float]
    mass: float
    axis: Tuple[float, float, float]
    joint_range: Tuple[float, float]
    stiffness: float
    damping: float
    friction: Tuple[float, float, float]
    solref: Tuple[float, float]
    solimp: Tuple[float, float, float, float, float]


@dataclass(frozen=True)
class ProneMannequinConfiguration:
    """Fixed prone-body layout and uncalibrated local back-tissue model."""

    back_center: Tuple[float, float, float]
    back_size: Tuple[float, float, float]
    back_grid_count: Tuple[int, int, int]
    back_mass: float
    back_radius: float
    back_young: float
    back_poisson: float
    back_damping: float
    back_friction: Tuple[float, float, float]
    table_center_offset: Tuple[float, float, float]
    table_half_size: Tuple[float, float, float]
    torso_center_offset: Tuple[float, float, float]
    torso_half_size: Tuple[float, float, float]
    pelvis_center_offset: Tuple[float, float, float]
    pelvis_half_size: Tuple[float, float, float]
    head_center_offset: Tuple[float, float, float]
    head_half_size: Tuple[float, float, float]
    arm_x_offsets: Tuple[float, float]
    arm_start_offset: Tuple[float, float, float]
    arm_end_offset: Tuple[float, float, float]
    arm_radius: float
    leg_x_offsets: Tuple[float, float]
    leg_start_offset: Tuple[float, float, float]
    leg_end_offset: Tuple[float, float, float]
    leg_radius: float


@dataclass(frozen=True)
class ModelConfiguration:
    """MuJoCo-only settings kept separate from the manufacturer URDF."""

    timestep: float
    integrator: str
    solver: str
    iterations: int
    gravity_compensation: bool
    damping_ratio: float
    position_kp: Dict[str, float]
    contact_pad: ContactPadConfiguration
    prone_mannequin: ProneMannequinConfiguration
    collision_geometry: str
    contact_monitor: dict


@dataclass(frozen=True)
class UrdfContract:
    """Values extracted from the expanded canonical URDF."""

    joint_ranges: Dict[str, Tuple[float, float]]
    effort_limits: Dict[str, float]
    velocity_limits: Dict[str, float]
    initial_positions: Dict[str, float]


def default_config_path() -> Path:
    """Return the installed MuJoCo configuration path."""
    return Path(get_package_share_directory("massage_mujoco")) / "config/mujoco.yaml"


def load_configuration(path: Optional[Path] = None) -> ModelConfiguration:
    """Load and validate MuJoCo-only model parameters."""
    config_path = Path(path) if path is not None else default_config_path()
    with config_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)

    simulation = raw["simulation"]
    actuators = raw["position_actuators"]
    contact_pad = raw["contact_pad"]
    prone_mannequin = raw.get("prone_mannequin", {
        "back_center": [0.65, -0.10, 0.425],
        "back_size": [0.32, 0.45, 0.05],
        "back_grid_count": [5, 7, 3],
        "back_mass": 10.0,
        "back_radius": 0.008,
        "back_young": 1000.0,
        "back_poisson": 0.30,
        "back_damping": 0.10,
        "back_friction": [0.8, 0.01, 0.001],
    })
    layout = prone_mannequin.get("layout", {
        "massage_table": {
            "center_offset": [0.0, -0.18, -0.185],
            "half_size": [0.43, 0.82, 0.055],
        },
        "torso_core": {
            "center_offset": [0.0, 0.0, -0.075],
            "half_size": [0.19, 0.29, 0.075],
        },
        "pelvis": {
            "center_offset": [0.0, -0.39, -0.075],
            "half_size": [0.20, 0.16, 0.075],
        },
        "head": {
            "center_offset": [0.0, 0.37, -0.055],
            "half_size": [0.105, 0.13, 0.095],
        },
        "arms": {
            "x_offsets": [-0.245, 0.245],
            "start_offset": [0.0, 0.20, -0.09],
            "end_offset": [0.0, -0.30, -0.09],
            "radius": 0.052,
        },
        "legs": {
            "x_offsets": [-0.105, 0.105],
            "start_offset": [0.0, -0.48, -0.08],
            "end_offset": [0.0, -0.95, -0.10],
            "radius": 0.068,
        },
    })
    table = layout["massage_table"]
    torso = layout["torso_core"]
    pelvis = layout["pelvis"]
    head = layout["head"]
    arms = layout["arms"]
    legs = layout["legs"]
    kp = {name: float(actuators["kp"][name]) for name in JOINT_NAMES}
    pad_configuration = ContactPadConfiguration(
        position=tuple(float(value) for value in contact_pad["position"]),
        size=tuple(float(value) for value in contact_pad["size"]),
        mass=float(contact_pad["mass"]),
        axis=tuple(float(value) for value in contact_pad["axis"]),
        joint_range=tuple(float(value) for value in contact_pad["joint_range"]),
        stiffness=float(contact_pad["stiffness"]),
        damping=float(contact_pad["damping"]),
        friction=tuple(float(value) for value in contact_pad["friction"]),
        solref=tuple(float(value) for value in contact_pad["solref"]),
        solimp=tuple(float(value) for value in contact_pad["solimp"]),
    )
    mannequin_configuration = ProneMannequinConfiguration(
        back_center=tuple(float(value) for value in prone_mannequin["back_center"]),
        back_size=tuple(float(value) for value in prone_mannequin["back_size"]),
        back_grid_count=tuple(
            int(value) for value in prone_mannequin["back_grid_count"]
        ),
        back_mass=float(prone_mannequin["back_mass"]),
        back_radius=float(prone_mannequin["back_radius"]),
        back_young=float(prone_mannequin["back_young"]),
        back_poisson=float(prone_mannequin["back_poisson"]),
        back_damping=float(prone_mannequin["back_damping"]),
        back_friction=tuple(
            float(value) for value in prone_mannequin["back_friction"]
        ),
        table_center_offset=tuple(float(value) for value in table["center_offset"]),
        table_half_size=tuple(float(value) for value in table["half_size"]),
        torso_center_offset=tuple(float(value) for value in torso["center_offset"]),
        torso_half_size=tuple(float(value) for value in torso["half_size"]),
        pelvis_center_offset=tuple(float(value) for value in pelvis["center_offset"]),
        pelvis_half_size=tuple(float(value) for value in pelvis["half_size"]),
        head_center_offset=tuple(float(value) for value in head["center_offset"]),
        head_half_size=tuple(float(value) for value in head["half_size"]),
        arm_x_offsets=tuple(float(value) for value in arms["x_offsets"]),
        arm_start_offset=tuple(float(value) for value in arms["start_offset"]),
        arm_end_offset=tuple(float(value) for value in arms["end_offset"]),
        arm_radius=float(arms["radius"]),
        leg_x_offsets=tuple(float(value) for value in legs["x_offsets"]),
        leg_start_offset=tuple(float(value) for value in legs["start_offset"]),
        leg_end_offset=tuple(float(value) for value in legs["end_offset"]),
        leg_radius=float(legs["radius"]),
    )
    configuration = ModelConfiguration(
        timestep=float(simulation["timestep"]),
        integrator=str(simulation["integrator"]),
        solver=str(simulation["solver"]),
        iterations=int(simulation["iterations"]),
        gravity_compensation=bool(actuators["gravity_compensation"]),
        damping_ratio=float(actuators["damping_ratio"]),
        position_kp=kp,
        contact_pad=pad_configuration,
        prone_mannequin=mannequin_configuration,
        collision_geometry=raw.get("collision_geometry", "mesh"),
        contact_monitor=raw.get("contact_monitor", {
            "tool_body": "massage_head_link", "target_body": "contact_pad",
            "contact_force": 0.05, "over_force": 20.0,
            "impact_force_rate": 200.0,
        }),
    )
    if configuration.collision_geometry not in ("mesh", "box"):
        raise ValueError("collision_geometry must be mesh or box")
    if configuration.timestep <= 0.0:
        raise ValueError("simulation.timestep must be positive")
    if configuration.iterations <= 0:
        raise ValueError("simulation.iterations must be positive")
    if configuration.damping_ratio <= 0.0:
        raise ValueError("position_actuators.damping_ratio must be positive")
    if any(value <= 0.0 for value in configuration.position_kp.values()):
        raise ValueError("position actuator gains must be positive")
    if (
        len(pad_configuration.position) != 3
        or len(pad_configuration.size) != 3
        or len(pad_configuration.axis) != 3
        or len(pad_configuration.joint_range) != 2
        or len(pad_configuration.friction) != 3
        or len(pad_configuration.solref) != 2
        or len(pad_configuration.solimp) != 5
    ):
        raise ValueError("contact_pad vectors have invalid dimensions")
    if (
        pad_configuration.mass <= 0.0
        or any(value <= 0.0 for value in pad_configuration.size)
        or pad_configuration.stiffness <= 0.0
        or pad_configuration.damping <= 0.0
        or pad_configuration.joint_range[0]
        >= pad_configuration.joint_range[1]
    ):
        raise ValueError("contact_pad physical parameters are invalid")
    if (
        len(mannequin_configuration.back_center) != 3
        or len(mannequin_configuration.back_size) != 3
        or len(mannequin_configuration.back_grid_count) != 3
        or len(mannequin_configuration.back_friction) != 3
    ):
        raise ValueError("prone_mannequin vectors have invalid dimensions")
    layout_vectors = (
        mannequin_configuration.table_center_offset,
        mannequin_configuration.table_half_size,
        mannequin_configuration.torso_center_offset,
        mannequin_configuration.torso_half_size,
        mannequin_configuration.pelvis_center_offset,
        mannequin_configuration.pelvis_half_size,
        mannequin_configuration.head_center_offset,
        mannequin_configuration.head_half_size,
        mannequin_configuration.arm_start_offset,
        mannequin_configuration.arm_end_offset,
        mannequin_configuration.leg_start_offset,
        mannequin_configuration.leg_end_offset,
    )
    if any(len(vector) != 3 for vector in layout_vectors):
        raise ValueError("prone_mannequin.layout vectors have invalid dimensions")
    if (
        len(mannequin_configuration.arm_x_offsets) != 2
        or len(mannequin_configuration.leg_x_offsets) != 2
        or any(value <= 0.0 for value in mannequin_configuration.table_half_size)
        or any(value <= 0.0 for value in mannequin_configuration.torso_half_size)
        or any(value <= 0.0 for value in mannequin_configuration.pelvis_half_size)
        or any(value <= 0.0 for value in mannequin_configuration.head_half_size)
        or mannequin_configuration.arm_radius <= 0.0
        or mannequin_configuration.leg_radius <= 0.0
    ):
        raise ValueError("prone_mannequin.layout geometry is invalid")
    if (
        any(value <= 0.0 for value in mannequin_configuration.back_size)
        or any(value < 2 for value in mannequin_configuration.back_grid_count)
        or mannequin_configuration.back_mass <= 0.0
        or mannequin_configuration.back_radius <= 0.0
        or mannequin_configuration.back_young <= 0.0
        or not 0.0 < mannequin_configuration.back_poisson < 0.5
        or mannequin_configuration.back_damping <= 0.0
    ):
        raise ValueError("prone_mannequin physical parameters are invalid")
    spacing = tuple(
        size / (count - 1)
        for size, count in zip(
            mannequin_configuration.back_size,
            mannequin_configuration.back_grid_count,
        )
    )
    if any(value <= 2.0 * mannequin_configuration.back_radius for value in spacing):
        raise ValueError("prone_mannequin grid spacing must exceed its diameter")
    return configuration


def expanded_urdf_xml(initial_positions_path: Optional[Path] = None) -> str:
    """Expand the project Xacro and resolve ROS package mesh paths."""
    description_share = Path(get_package_share_directory("massage_description"))
    mujoco_share = Path(get_package_share_directory("massage_mujoco"))
    xacro_path = description_share / "urdf/jaka_s5_massage.urdf.xacro"
    initial_positions = (
        Path(initial_positions_path)
        if initial_positions_path is not None
        else mujoco_share / "config/initial_positions.yaml"
    )
    document = xacro.process_file(
        str(xacro_path),
        mappings={
            "use_gazebo": "false",
            "use_rviz_sim": "false",
            "use_massage_head": "true",
            "initial_positions_file": str(initial_positions),
        },
    )

    root = ET.fromstring(document.toxml())
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename is None or not filename.startswith("package://"):
            continue
        package_path = filename[len("package://"):]
        package_name, separator, relative_path = package_path.partition("/")
        if not separator:
            raise ValueError(f"invalid ROS package URI: {filename}")
        resolved = Path(get_package_share_directory(package_name)) / relative_path
        mesh.set("filename", str(resolved))
    return ET.tostring(root, encoding="unicode")


def parse_urdf_contract(urdf_xml: str) -> UrdfContract:
    """Extract joint limits needed by the MuJoCo actuator layer."""
    root = ET.fromstring(urdf_xml)
    joint_ranges: Dict[str, Tuple[float, float]] = {}
    effort_limits: Dict[str, float] = {}
    velocity_limits: Dict[str, float] = {}
    initial_positions: Dict[str, float] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in JOINT_NAMES:
            continue
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"{name} is missing its URDF limit element")
        joint_ranges[name] = (
            float(limit.attrib["lower"]),
            float(limit.attrib["upper"]),
        )
        effort_limits[name] = float(limit.attrib["effort"])
        velocity_limits[name] = float(limit.attrib["velocity"])

        initial_value = root.find(
            "./ros2_control/joint[@name='{}']/state_interface[@name='position']/"
            "param[@name='initial_value']".format(name)
        )
        if initial_value is None or initial_value.text is None:
            raise ValueError(f"{name} is missing its simulation initial position")
        initial_positions[name] = float(initial_value.text)

    if set(joint_ranges) != set(JOINT_NAMES):
        raise ValueError("expanded URDF does not contain all six JAKA joints")
    return UrdfContract(
        joint_ranges,
        effort_limits,
        velocity_limits,
        initial_positions,
    )


def _set_physics_options(spec: mujoco.MjSpec, config: ModelConfiguration) -> None:
    integrators = {
        "Euler": mujoco.mjtIntegrator.mjINT_EULER,
        "RK4": mujoco.mjtIntegrator.mjINT_RK4,
        "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
        "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    }
    solvers = {
        "PGS": mujoco.mjtSolver.mjSOL_PGS,
        "CG": mujoco.mjtSolver.mjSOL_CG,
        "Newton": mujoco.mjtSolver.mjSOL_NEWTON,
    }
    if config.integrator not in integrators:
        raise ValueError(f"unsupported MuJoCo integrator: {config.integrator}")
    if config.solver not in solvers:
        raise ValueError(f"unsupported MuJoCo solver: {config.solver}")
    spec.option.timestep = config.timestep
    spec.option.integrator = integrators[config.integrator]
    spec.option.solver = solvers[config.solver]
    spec.option.iterations = config.iterations


def _add_contact_pad(
    spec: mujoco.MjSpec,
    configuration: ContactPadConfiguration,
) -> None:
    pad = spec.worldbody.add_body(
        name="contact_pad",
        pos=configuration.position,
    )
    pad.add_joint(
        name="contact_pad_slide",
        type=mujoco.mjtJoint.mjJNT_SLIDE,
        axis=configuration.axis,
        limited=mujoco.mjtLimited.mjLIMITED_TRUE,
        range=configuration.joint_range,
        stiffness=configuration.stiffness,
        damping=configuration.damping,
        springref=0.0,
    )
    pad.add_geom(
        name="contact_pad_geom",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[value / 2.0 for value in configuration.size],
        mass=configuration.mass,
        friction=configuration.friction,
        condim=3,
        solref=configuration.solref,
        solimp=configuration.solimp,
        rgba=[0.25, 0.45, 0.65, 1.0],
    )


def _add_prone_mannequin(
    spec: mujoco.MjSpec,
    configuration: ProneMannequinConfiguration,
) -> list:
    """Add a prone rigid-body outline with a locally deformable back volume."""
    center = np.asarray(configuration.back_center)
    skin = [0.72, 0.45, 0.34, 1.0]
    fabric = [0.10, 0.42, 0.48, 1.0]
    mannequin = spec.worldbody.add_body(name="prone_mannequin")
    elements = [mannequin]

    def add_geom(name, geom_type, *, pos=None, size=None, fromto=None,
                 rgba=skin, collidable=True):
        geom = mannequin.add_geom(
            name=name,
            type=geom_type,
            pos=pos,
            size=size,
            fromto=fromto,
            mass=0.0,
            rgba=rgba,
            contype=1 if collidable else 0,
            conaffinity=1 if collidable else 0,
            friction=configuration.back_friction,
        )
        elements.append(geom)

    # The bed and body outline establish scale and collision context. The rigid
    # torso core is non-colliding because the flex volume occupies its surface.
    add_geom(
        "massage_table",
        mujoco.mjtGeom.mjGEOM_BOX,
        pos=center + np.asarray(configuration.table_center_offset),
        size=configuration.table_half_size,
        rgba=fabric,
        collidable=False,
    )
    add_geom(
        "prone_torso_core",
        mujoco.mjtGeom.mjGEOM_ELLIPSOID,
        pos=center + np.asarray(configuration.torso_center_offset),
        size=configuration.torso_half_size,
        collidable=False,
    )
    add_geom(
        "prone_pelvis",
        mujoco.mjtGeom.mjGEOM_ELLIPSOID,
        pos=center + np.asarray(configuration.pelvis_center_offset),
        size=configuration.pelvis_half_size,
    )
    add_geom(
        "prone_head",
        mujoco.mjtGeom.mjGEOM_ELLIPSOID,
        pos=center + np.asarray(configuration.head_center_offset),
        size=configuration.head_half_size,
    )
    for side, x_offset in zip(("left", "right"), configuration.arm_x_offsets):
        start = center + np.asarray(configuration.arm_start_offset)
        end = center + np.asarray(configuration.arm_end_offset)
        start[0] += x_offset
        end[0] += x_offset
        add_geom(
            f"prone_{side}_arm",
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[*start, *end],
            size=[configuration.arm_radius, 0.0, 0.0],
        )
    for side, x_offset in zip(("left", "right"), configuration.leg_x_offsets):
        start = center + np.asarray(configuration.leg_start_offset)
        end = center + np.asarray(configuration.leg_end_offset)
        start[0] += x_offset
        end[0] += x_offset
        add_geom(
            f"prone_{side}_leg",
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[*start, *end],
            size=[configuration.leg_radius, 0.0, 0.0],
        )

    counts = configuration.back_grid_count
    spacing = [
        size / (count - 1)
        for size, count in zip(configuration.back_size, counts)
    ]
    flex = spec.worldbody.make_flex(
        name=PRONE_BACK_FLEX_NAME,
        type="grid",
        dim=3,
        dof="full",
        count=counts,
        spacing=spacing,
        mass=configuration.back_mass,
        radius=configuration.back_radius,
        pos=configuration.back_center,
    )
    flex.contype = 1
    flex.conaffinity = 1
    flex.condim = 3
    flex.selfcollide = mujoco.mjtFlexSelf.mjFLEXSELF_NONE
    flex.friction = configuration.back_friction
    flex.young = configuration.back_young
    flex.poisson = configuration.back_poisson
    flex.damping = configuration.back_damping
    flex.thickness = configuration.back_size[2]
    flex.rgba = [0.78, 0.48, 0.36, 0.96]
    elements.append(flex)

    # Pin the underside to the rigid torso. The two upper layers remain free to
    # deform under the massage head while the whole body stays on the table.
    nx, ny, nz = counts
    for x_index in range(nx):
        for y_index in range(ny):
            for z_index in range(nz):
                body_index = (x_index * ny + y_index) * nz + z_index
                node_body = spec.body(f"{PRONE_BACK_FLEX_NAME}_{body_index}")
                node_body.gravcomp = 1.0
                if z_index == 0:
                    for joint in list(node_body.joints):
                        spec.delete(joint)
    return elements


def _add_visual_scene(spec: mujoco.MjSpec) -> list:
    """Add a lit lab-style backdrop without changing collision dynamics."""
    skybox = spec.add_texture(
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.20, 0.31, 0.45],
        rgb2=[0.58, 0.68, 0.78],
        width=512,
        height=3072,
    )
    floor_texture = spec.add_texture(
        name="lab_floor_grid",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        mark=mujoco.mjtMark.mjMARK_EDGE,
        rgb1=[0.48, 0.53, 0.58],
        rgb2=[0.27, 0.32, 0.38],
        markrgb=[0.12, 0.17, 0.22],
        width=512,
        height=512,
    )
    texture_slots = [""] * int(mujoco.mjtTextureRole.mjNTEXROLE)
    texture_slots[int(mujoco.mjtTextureRole.mjTEXROLE_RGB)] = (
        "lab_floor_grid"
    )
    floor_material = spec.add_material(
        name="lab_floor_material",
        textures=texture_slots,
        texuniform=True,
        texrepeat=[8.0, 8.0],
        reflectance=0.08,
        roughness=0.85,
    )
    floor = spec.worldbody.add_geom(
        name="visual_floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        pos=[0.0, 0.0, -0.015],
        size=[3.0, 3.0, 0.05],
        material="lab_floor_material",
        contype=0,
        conaffinity=0,
    )
    plinth = spec.worldbody.add_geom(
        name="visual_robot_plinth",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0.0, 0.0, -0.04],
        size=[0.24, 0.025, 0.0],
        rgba=[0.18, 0.23, 0.29, 1.0],
        contype=0,
        conaffinity=0,
    )
    key_light = spec.worldbody.add_light(
        name="lab_key_light",
        pos=[-1.5, -1.0, 2.8],
        dir=[0.45, 0.30, -1.0],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        castshadow=True,
        ambient=[0.08, 0.09, 0.10],
        diffuse=[0.56, 0.58, 0.62],
        specular=[0.08, 0.08, 0.08],
    )
    fill_light = spec.worldbody.add_light(
        name="lab_fill_light",
        pos=[1.5, 0.8, 1.8],
        dir=[-0.75, -0.25, -0.65],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        castshadow=False,
        ambient=[0.03, 0.04, 0.05],
        diffuse=[0.18, 0.22, 0.30],
        specular=[0.03, 0.03, 0.04],
    )
    spec.visual.headlight.ambient = [0.12, 0.12, 0.14]
    spec.visual.headlight.diffuse = [0.36, 0.37, 0.40]
    spec.visual.headlight.specular = [0.05, 0.05, 0.05]
    spec.visual.rgba.haze = [0.40, 0.50, 0.62, 1.0]
    return [
        skybox,
        floor_texture,
        floor_material,
        floor,
        plinth,
        key_light,
        fill_light,
    ]


def _add_tcp_control_ball(spec: mujoco.MjSpec) -> list:
    """Add a collision-free mocap target for MuJoCo Viewer mouse control."""
    target = spec.worldbody.add_body(
        name="tcp_control_target",
        mocap=True,
    )
    elements = [target]
    elements.append(target.add_geom(
        name="tcp_control_ball",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[0.035, 0.0, 0.0],
        rgba=[0.05, 0.85, 0.95, 0.72],
        contype=0,
        conaffinity=0,
        group=4,
    ))
    axes = (
        ("x", [0.10, 0.0, 0.0], [0.95, 0.15, 0.12, 0.92]),
        ("y", [0.0, 0.10, 0.0], [0.20, 0.85, 0.25, 0.92]),
        ("z", [0.0, 0.0, 0.10], [0.15, 0.35, 0.95, 0.92]),
    )
    for axis_name, endpoint, color in axes:
        elements.append(target.add_geom(
            name=f"tcp_control_axis_{axis_name}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            size=[0.004, 0.0, 0.0],
            fromto=[0.0, 0.0, 0.0, *endpoint],
            rgba=color,
            contype=0,
            conaffinity=0,
            group=4,
        ))
    return elements


def build_model(
    config_path: Optional[Path] = None,
    use_prone_mannequin: bool = False,
    initial_positions_path: Optional[Path] = None,
) -> Tuple[mujoco.MjModel, UrdfContract]:
    """Compile a MuJoCo model while preserving the URDF as source of truth."""
    config = load_configuration(config_path)
    urdf_xml = expanded_urdf_xml(initial_positions_path)
    contract = parse_urdf_contract(urdf_xml)
    spec = mujoco.MjSpec.from_string(urdf_xml)
    # The FT sensor needs the fixed massage-head body to remain distinct.
    spec.compiler.fusestatic = False
    # URDF import enables this flag; disable it for MuJoCo-only scene decor.
    spec.compiler.discardvisual = False
    _set_physics_options(spec, config)
    visual_elements = _add_visual_scene(spec)
    visual_elements.extend(_add_tcp_control_ball(spec))

    tool_body = spec.body("massage_head_link")
    if tool_body is None:
        raise ValueError("expanded URDF does not contain massage_head_link")
    tool_body.add_site(
        name="massage_tool_tip",
        pos=[0.0, 0.0, 0.08],
        size=[0.005, 0.005, 0.005],
        rgba=[0.9, 0.2, 0.1, 1.0],
    )
    tool_body.add_site(
        name="massage_ft_site",
        pos=[0.0, 0.0, 0.0],
        size=[0.003, 0.003, 0.003],
    )
    spec.add_sensor(
        name="massage_ft_force",
        type=mujoco.mjtSensor.mjSENS_FORCE,
        objtype=mujoco.mjtObj.mjOBJ_SITE,
        objname="massage_ft_site",
    )
    spec.add_sensor(
        name="massage_ft_torque",
        type=mujoco.mjtSensor.mjSENS_TORQUE,
        objtype=mujoco.mjtObj.mjOBJ_SITE,
        objname="massage_ft_site",
    )
    if use_prone_mannequin:
        visual_elements.extend(_add_prone_mannequin(spec, config.prone_mannequin))
    else:
        _add_contact_pad(spec, config.contact_pad)

    for name in JOINT_NAMES:
        lower, upper = contract.joint_ranges[name]
        effort = contract.effort_limits[name]
        actuator = spec.add_actuator(
            name=f"{name}_position",
            target=name,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
            ctrlrange=[lower, upper],
            forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE,
            forcerange=[-effort, effort],
        )
        actuator.set_to_position(
            kp=config.position_kp[name],
            dampratio=config.damping_ratio,
        )

    # Direct torque actuators appear as independent J1-J6 inputs in the
    # native Viewer Control panel. Runtime ownership prevents them from
    # competing with the position actuator on the same joint.
    for name in JOINT_NAMES:
        effort = contract.effort_limits[name]
        actuator = spec.add_actuator(
            name=f"{name}_torque_Nm",
            target=name,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
            ctrlrange=[-effort, effort],
            forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE,
            forcerange=[-effort, effort],
        )
        actuator.set_to_motor()

    for index, geom in enumerate(spec.geoms):
        if not geom.name:
            geom.name = f"urdf_geom_{index}"
    model = spec.compile()
    if config.collision_geometry == "box":
        visual_elements.extend(_add_collision_boxes(spec, model))
        model = spec.compile()
    # MjSpec elements must stay referenced until compilation in MuJoCo 3.12.
    del visual_elements
    return model, contract


def _add_collision_boxes(spec, model):
    """Enclose every collidable mesh vertex; keep STL visuals and URDF inertia."""
    proxies = []
    records = []
    for geom_id in range(model.ngeom):
        if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        if not (model.geom_contype[geom_id] or model.geom_conaffinity[geom_id]):
            continue
        mesh_id = model.geom_dataid[geom_id]
        start = model.mesh_vertadr[mesh_id]
        count = model.mesh_vertnum[mesh_id]
        vertices = model.mesh_vert[start:start + count].astype(float)
        low, high = vertices.min(axis=0), vertices.max(axis=0)
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, model.geom_quat[geom_id])
        center = model.geom_pos[geom_id] + rotation.reshape(3, 3) @ ((low + high) / 2)
        records.append(dict(
            name=model.geom(geom_id).name,
            body=model.body(int(model.geom_bodyid[geom_id])).name,
            pos=center.copy(), quat=model.geom_quat[geom_id].copy(),
            size=(high - low) / 2 + 1e-6,
            contype=int(model.geom_contype[geom_id]),
            conaffinity=int(model.geom_conaffinity[geom_id]),
            friction=model.geom_friction[geom_id].copy(),
            solref=model.geom_solref[geom_id].copy(),
            solimp=model.geom_solimp[geom_id].copy(),
        ))
    # Resolve and disable originals before additions change MjSpec geom indices.
    for record in records:
        name = record['name']
        spec.geom(name).contype = 0
        spec.geom(name).conaffinity = 0
    mesh_bodies = {record['body'] for record in records}
    exclusions = []
    for body in sorted(mesh_bodies):
        parent = model.body(int(model.body(body).parentid[0])).name
        if parent in mesh_bodies:
            # Box envelopes overlap at joints. Explicitly include the fixed
            # base pair, which MuJoCo's world-body parent filter exempts.
            exclusions.append(spec.add_exclude(bodyname1=parent, bodyname2=body))
    for record in records:
        name = record.pop('name')
        body = record.pop('body')
        proxies.append(spec.body(body).add_geom(
            name=f"proxy_{name}", type=mujoco.mjtGeom.mjGEOM_BOX,
            mass=0.0, group=3, rgba=[0.95, 0.55, 0.1, 0.3], **record,
        ))
    return proxies + exclusions
