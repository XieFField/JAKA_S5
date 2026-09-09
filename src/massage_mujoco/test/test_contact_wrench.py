"""Contact fixture and force-torque semantics tests."""

from pathlib import Path

import mujoco
import numpy as np

from massage_mujoco.runtime import MujocoRuntime


PRECONTACT_POSITIONS = np.array([
    -0.0150542703,
    0.439740469,
    -0.667101116,
    0.227360592,
    -0.0150542703,
    0.0000009447,
])
CONTACT_POSITIONS = np.array([
    -0.0224084984,
    0.442422977,
    -0.672689985,
    0.230266926,
    -0.0224084984,
    0.0000009178,
])
PRONE_CONTACT_POSITIONS = np.array([
    -0.3261385299660454,
    1.1008184598506219,
    -0.6994338024152722,
    1.169415342551158,
    1.5707926536032857,
    1.2446614700204621,
])
PRONE_INITIAL_POSITIONS = (
    Path(__file__).parents[2]
    / "massage_bringup/config/prone_back_initial_positions.yaml"
)


def test_contact_pad_matches_gazebo_mechanical_contract():
    runtime = MujocoRuntime()
    configuration = runtime.config.contact_pad
    joint = runtime.model.joint("contact_pad_slide")
    geom = runtime.model.geom("contact_pad_geom")
    body = runtime.model.body("contact_pad")

    np.testing.assert_allclose(body.pos, configuration.position)
    np.testing.assert_allclose(joint.axis, configuration.axis)
    np.testing.assert_allclose(joint.range, configuration.joint_range)
    np.testing.assert_allclose(joint.stiffness, configuration.stiffness)
    np.testing.assert_allclose(joint.damping, configuration.damping)
    np.testing.assert_allclose(geom.size, np.asarray(configuration.size) / 2.0)
    np.testing.assert_allclose(geom.friction, configuration.friction)
    np.testing.assert_allclose(body.mass, configuration.mass)


def test_known_tool_load_uses_child_to_parent_sensor_axes():
    runtime = MujocoRuntime()
    runtime.model.opt.gravity[:] = 0.0

    loads = [
        (np.eye(3)[axis], np.zeros(3)) for axis in range(3)
    ] + [
        (np.zeros(3), np.eye(3)[axis]) for axis in range(3)
    ]
    for force, torque in loads:
        runtime.clear_tool_external_wrench()
        runtime.set_tool_external_wrench(force, torque)
        runtime.data.qvel[:] = 0.0
        runtime.data.qacc[:] = 0.0
        mujoco.mj_inverse(runtime.model, runtime.data)
        mujoco.mj_sensorAcc(runtime.model, runtime.data)
        np.testing.assert_allclose(
            runtime.state().tool_wrench,
            np.concatenate((force, torque)),
            atol=1e-12,
        )


def test_slow_pad_contact_matches_spring_force_and_sensor_axis():
    runtime = MujocoRuntime()
    runtime.reset(PRECONTACT_POSITIONS)
    runtime.set_target(PRECONTACT_POSITIONS)
    baseline = runtime.step(3000)
    assert runtime.data.ncon == 0

    for step_index in range(3000):
        alpha = (step_index + 1) / 3000
        runtime.set_target(
            PRECONTACT_POSITIONS
            + alpha * (CONTACT_POSITIONS - PRECONTACT_POSITIONS)
        )
        runtime.step()
    contact = runtime.step(3000)

    assert runtime.data.ncon == 5
    assert contact.contact_pad_position < -0.0002
    spring_force = (
        -runtime.config.contact_pad.stiffness * contact.contact_pad_position
    )
    contact_wrench = contact.tool_wrench - baseline.tool_wrench
    assert contact_wrench[2] < 0.0
    assert abs(contact_wrench[2] + spring_force) < 0.005
    assert np.max(np.abs(contact.velocity)) < 0.001
    points = runtime.contacts()
    assert points and all(point.massage_pair for point in points)
    assert all(point.penetration > 0.0 for point in points)
    np.testing.assert_allclose(
        [np.linalg.norm(point.normal) for point in points], 1.0,
    )
    assert abs(sum(point.normal_force for point in points) - spring_force) < 0.005


def test_prone_back_flex_accepts_stable_massage_contact():
    runtime = MujocoRuntime(
        use_prone_mannequin=True,
        initial_positions_path=PRONE_INITIAL_POSITIONS,
    )
    start = runtime.state().position
    for step_index in range(1000):
        alpha = (step_index + 1) / 1000
        runtime.set_target(
            start + alpha * (PRONE_CONTACT_POSITIONS - start)
        )
        runtime.step()
    runtime.step(1000)

    points = runtime.contacts()
    massage_points = [point for point in points if point.massage_pair]
    assert massage_points
    assert all(
        {point.body_a, point.body_b}
        == {"massage_head_link", "prone_back_soft_tissue"}
        for point in massage_points
    )
    assert not [point for point in points if not point.massage_pair]
    assert sum(point.normal_force for point in massage_points) > 0.05
    assert np.all(np.isfinite(runtime.data.qpos))
