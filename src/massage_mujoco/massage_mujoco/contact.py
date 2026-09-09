"""Contact geometry and sampled massage-contact events, independent of ROS."""

from dataclasses import asdict, dataclass

import mujoco
import numpy as np


@dataclass(frozen=True)
class ContactPoint:
    """One solver contact; normal points from body A to B in world axes."""

    body_a: str
    body_b: str
    geom_a: str
    geom_b: str
    position: list
    normal: list
    penetration: float
    normal_force: float
    massage_pair: bool


def read_contacts(model, data, tool_body, target_body):
    """Copy active solver contacts without confusing unrelated link collisions."""
    result = []
    wrench = np.zeros(6)

    def entity(contact, side):
        geom_id = int(contact.geom1 if side == 0 else contact.geom2)
        if geom_id >= 0:
            body = model.body(int(model.geom_bodyid[geom_id])).name
            geom = model.geom(geom_id).name or f"geom_{geom_id}"
            return body, geom
        flex_id = int(contact.flex[side])
        if flex_id >= 0:
            name = (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_FLEX, flex_id)
                or f"flex_{flex_id}"
            )
            element = int(contact.elem[side])
            return name, f"{name}_element_{element}"
        return "world", "world"

    for index in range(data.ncon):
        contact = data.contact[index]
        if contact.efc_address < 0:
            continue
        body_a, geom_a = entity(contact, 0)
        body_b, geom_b = entity(contact, 1)
        mujoco.mj_contactForce(model, data, index, wrench)
        result.append(ContactPoint(
            body_a, body_b, geom_a, geom_b, contact.pos.copy().tolist(),
            contact.frame[:3].copy().tolist(), max(0.0, -float(contact.dist)),
            max(0.0, float(wrench[0])),
            {body_a, body_b} == {tool_body, target_body},
        ))
    return result


class ContactMonitor:
    """Classify contact at the caller's sampling rate, with edge events."""

    def __init__(self, configuration):
        self.config = configuration
        for name in ('contact_force', 'over_force', 'impact_force_rate'):
            value = float(configuration[name])
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if configuration['over_force'] <= configuration['contact_force']:
            raise ValueError('over_force must exceed contact_force')
        self.reset()

    def reset(self):
        """Clear event history after an explicit or native reset."""
        self.previous_time = None
        self.previous_force = 0.0
        self.flags = (False, False, False, False)

    def update(self, timestamp, contacts):
        """Return a serializable snapshot; force rate is sampled, not FT impulse."""
        if self.previous_time is not None and timestamp < self.previous_time:
            self.reset()
        force = sum((c.normal_force for c in contacts if c.massage_pair), 0.0)
        dt = 0 if self.previous_time is None else timestamp - self.previous_time
        rate = (force - self.previous_force) / dt if dt > 0 else 0.0
        active = force >= self.config['contact_force'] and any(
            c.massage_pair for c in contacts)
        flags = (
            active, active and force >= self.config['over_force'],
            active and rate >= self.config['impact_force_rate'],
            any(not c.massage_pair and c.normal_force > 0 for c in contacts),
        )
        edges = (
            ('CONTACT_STARTED', 'CONTACT_ENDED'),
            ('OVER_FORCE', 'OVER_FORCE_CLEARED'),
            ('IMPACT', 'IMPACT_CLEARED'),
            ('UNEXPECTED_CONTACT', 'UNEXPECTED_CONTACT_CLEARED'),
        )
        events = [edges[i][0 if flag else 1] for i, flag in enumerate(flags)
                  if flag != self.flags[i]]
        self.previous_time, self.previous_force, self.flags = timestamp, force, flags
        return dict(
            time=timestamp, contacts=[asdict(c) for c in contacts],
            in_contact=flags[0], over_force=flags[1], impact=flags[2],
            unexpected_contact=flags[3], normal_force=force, force_rate=rate,
            events=events,
        )
