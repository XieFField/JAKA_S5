"""Independent URDF-to-MuJoCo kinematics checks."""

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from massage_mujoco.model import JOINT_NAMES, expanded_urdf_xml
from massage_mujoco.runtime import MujocoRuntime


def _vector(value):
    return np.fromstring(value, sep=" ")


def _origin_transform(origin):
    transform = np.eye(4)
    if origin is None:
        return transform
    xyz = _vector(origin.get("xyz", "0 0 0"))
    roll, pitch, yaw = _vector(origin.get("rpy", "0 0 0"))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    rotation_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rotation_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    transform[:3, :3] = rotation_z @ rotation_y @ rotation_x
    transform[:3, 3] = xyz
    return transform


def _axis_rotation(axis, angle):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    cosine = math.cos(angle)
    sine = math.sin(angle)
    cross = 1.0 - cosine
    rotation = np.array([
        [cosine + x * x * cross, x * y * cross - z * sine, x * z * cross + y * sine],
        [y * x * cross + z * sine, cosine + y * y * cross, y * z * cross - x * sine],
        [z * x * cross - y * sine, z * y * cross + x * sine, cosine + z * z * cross],
    ])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    return transform


def _urdf_link_transforms(positions):
    root = ET.fromstring(expanded_urdf_xml())
    pending = list(root.findall("joint"))
    transforms = {"world": np.eye(4)}
    joint_positions = dict(zip(JOINT_NAMES, positions))
    while pending:
        progressed = False
        for joint in list(pending):
            parent = joint.find("parent").get("link")
            if parent not in transforms:
                continue
            child = joint.find("child").get("link")
            transform = transforms[parent] @ _origin_transform(joint.find("origin"))
            if joint.get("type") == "revolute":
                axis = _vector(joint.find("axis").get("xyz"))
                transform = transform @ _axis_rotation(
                    axis, joint_positions[joint.get("name")]
                )
            transforms[child] = transform
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise AssertionError("URDF joint tree could not be resolved")
    return transforms


@pytest.mark.parametrize(
    "positions",
    [
        np.zeros(6),
        np.array([0.2, 1.0, -1.2, 0.7, -0.5, 0.3]),
    ],
)
def test_compiled_model_matches_urdf_forward_kinematics(positions):
    runtime = MujocoRuntime()
    runtime.reset(positions)
    expected = _urdf_link_transforms(positions)
    for link_name in [f"Link_0{index}" for index in range(1, 7)]:
        body_id = runtime.model.body(link_name).id
        np.testing.assert_allclose(
            runtime.data.xpos[body_id], expected[link_name][:3, 3], atol=1e-9
        )
        np.testing.assert_allclose(
            runtime.data.xmat[body_id].reshape(3, 3),
            expected[link_name][:3, :3],
            atol=1e-9,
        )

    tool_site_id = mujoco.mj_name2id(
        runtime.model, mujoco.mjtObj.mjOBJ_SITE, "massage_tool_tip"
    )
    np.testing.assert_allclose(
        runtime.data.site_xpos[tool_site_id],
        expected["massage_tool_tip"][:3, 3],
        atol=1e-9,
    )


def test_compiler_preserves_moving_link_mass_and_inertia():
    runtime = MujocoRuntime()
    root = ET.fromstring(expanded_urdf_xml())
    links = {link.get("name"): link for link in root.findall("link")}
    dynamic_links = [f"Link_0{index}" for index in range(1, 7)]
    dynamic_links.append("massage_head_link")
    for link_name in dynamic_links:
        inertial = links[link_name].find("inertial")
        expected_mass = float(inertial.find("mass").get("value"))
        expected_position = _vector(inertial.find("origin").get("xyz"))
        inertia = inertial.find("inertia")
        expected_inertia = np.array([
            [float(inertia.get("ixx")), float(inertia.get("ixy")), float(inertia.get("ixz"))],
            [float(inertia.get("ixy")), float(inertia.get("iyy")), float(inertia.get("iyz"))],
            [float(inertia.get("ixz")), float(inertia.get("iyz")), float(inertia.get("izz"))],
        ])

        body_id = runtime.model.body(link_name).id
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, runtime.model.body_iquat[body_id])
        rotation = rotation.reshape(3, 3)
        compiled_inertia = (
            rotation
            @ np.diag(runtime.model.body_inertia[body_id])
            @ rotation.T
        )
        assert runtime.model.body_mass[body_id] == pytest.approx(expected_mass)
        np.testing.assert_allclose(
            runtime.model.body_ipos[body_id], expected_position, atol=1e-9
        )
        np.testing.assert_allclose(
            compiled_inertia,
            expected_inertia,
            rtol=1e-6,
            atol=2e-6,
        )
