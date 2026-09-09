"""Contact classification and conservative collision proxy contracts."""

from dataclasses import replace

import mujoco
import numpy as np
import yaml

from massage_mujoco.contact import ContactMonitor, ContactPoint
from massage_mujoco.model import default_config_path
from massage_mujoco.runtime import MujocoRuntime


def test_contact_pair_threshold_edges_and_reset():
    monitor = ContactMonitor(dict(contact_force=0.05, over_force=2.0,
                                  impact_force_rate=100.0))
    point = ContactPoint('tool', 'pad', 'a', 'b', [0., 0., 0.],
                         [0., 1., 0.], 0.001, 0.01, True)
    assert not monitor.update(0., [point])['in_contact']
    other = replace(point, massage_pair=False, normal_force=10.)
    state = monitor.update(0.01, [other])
    assert not state['in_contact']
    assert state['unexpected_contact']
    state = monitor.update(0.02, [replace(point, normal_force=3.)])
    assert state['in_contact'] and state['over_force'] and state['impact']
    assert 'CONTACT_STARTED' in state['events']
    state = monitor.update(0.03, [replace(point, normal_force=3.)])
    assert 'CONTACT_STARTED' not in state['events']
    assert not state['impact']
    assert 'CONTACT_ENDED' in monitor.update(0.04, [])['events']
    assert not monitor.update(0., [])['events']


def test_box_proxies_enclose_mesh_and_preserve_inertia(tmp_path):
    raw = yaml.safe_load(default_config_path().read_text())
    reference = MujocoRuntime()
    raw['collision_geometry'] = 'box'
    path = tmp_path / 'proxy.yaml'
    path.write_text(yaml.safe_dump(raw))
    proxy = MujocoRuntime(path)
    np.testing.assert_allclose(proxy.model.body_mass, reference.model.body_mass)
    np.testing.assert_allclose(proxy.model.body_inertia, reference.model.body_inertia)
    np.testing.assert_allclose(proxy.state().tool_position, reference.state().tool_position)
    for g in range(reference.model.ngeom):
        if reference.model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        if not reference.model.geom_contype[g]:
            continue
        name = reference.model.geom(g).name
        box = proxy.model.geom('proxy_' + name)
        mid = reference.model.geom_dataid[g]
        start, count = reference.model.mesh_vertadr[mid], reference.model.mesh_vertnum[mid]
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, reference.model.geom_quat[g])
        vertices = reference.model.mesh_vert[start:start + count] @ rotation.reshape(3, 3).T
        vertices += reference.model.geom_pos[g]
        box_rotation = np.zeros(9)
        mujoco.mju_quat2Mat(box_rotation, box.quat)
        local = (vertices - box.pos) @ box_rotation.reshape(3, 3)
        assert np.all(np.abs(local) <= box.size + 1e-7)
        assert proxy.model.geom(name).contype == 0
    assert proxy.data.ncon == 0
