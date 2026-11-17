"""Immutable constant-velocity sphere snapshots; no future simulator queries."""
import copy
import math
from ..common.vectors import add, sub, mul, dot, norm
from .world import Environment


def finite_vector(value):
    return (isinstance(value, (list, tuple)) and len(value) == 3
            and all(isinstance(x, (int, float)) and math.isfinite(x) for x in value))


def point_segment_distance(p, a, b):
    delta = sub(b, a)
    squared = dot(delta, delta)
    fraction = max(0., min(1., dot(sub(p, a), delta) / squared)) if squared else 0.
    return norm(sub(p, add(a, mul(delta, fraction))))


class SweptEnvironment(Environment):
    """Min clearance to static world and inflated swept capsules.

    Exact free-space clearance; inside unions the value is not necessarily the
    Euclidean signed distance to the union boundary. All queries are 1-Lipschitz.
    ``center`` is measured at each record's ``timestamp``. Old records are
    propagated causally, not silently treated as current measurements.
    """
    def __init__(self, world, obstacles, now, horizon, margin=0., max_age=0.):
        values = (now, horizon, margin, max_age)
        if not all(math.isfinite(x) for x in values) or horizon <= 0 or margin < 0 or max_age < 0:
            raise ValueError('Invalid snapshot time/horizon/margin/max_age')
        super().__init__(copy.deepcopy(world))
        self.now, self.horizon = now, horizon
        self.capsules = []
        identifiers = set()
        for record in copy.deepcopy(obstacles):
            identifier = record['id']
            if identifier in identifiers:
                raise ValueError('Duplicate obstacle ID')
            identifiers.add(identifier)
            if record['shape'] != 'sphere':
                raise ValueError('Only fixed-radius spheres are supported')
            if not finite_vector(record['center']) or not finite_vector(record['velocity']):
                raise ValueError('Invalid obstacle vectors')
            radius, timestamp = record['radius'], record['timestamp']
            if not math.isfinite(radius) or radius <= 0 or not math.isfinite(timestamp):
                raise ValueError('Invalid obstacle radius/timestamp')
            age = now - timestamp
            if age < -1e-9 or age > max_age + 1e-9:
                raise ValueError('Future or stale obstacle observation')
            a = add(record['center'], mul(record['velocity'], max(0., age)))
            b = add(a, mul(record['velocity'], horizon))
            self.capsules.append(dict(id=identifier, a=a, b=b, radius=radius + margin,
                                      physical_radius=radius, prediction_margin=margin,
                                      observation_timestamp=timestamp))

    def clearance(self, p):
        return min([super().clearance(p)] + [
            point_segment_distance(p, c['a'], c['b']) - c['radius'] for c in self.capsules])

    def describe(self):
        return dict(valid_from=self.now, valid_until=self.now + self.horizon,
                    model='constant velocity, fixed-radius spheres', capsules=self.capsules)
