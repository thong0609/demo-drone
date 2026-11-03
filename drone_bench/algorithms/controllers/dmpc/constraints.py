"""Convex norm/tube cuts and collision cuts consistent with segment validation.

Last decision variable relaxes ONLY the preferred pair buffer. Its upper bound
leaves the physical safety distance plus pair_hard_margin as a hard constraint.
"""
import math
from ....common.vectors import dot, sub, mul, norm, unit, cross


def row(kernel, direction):
    return [coefficient * component for coefficient in kernel for component in direction] + [0.0]


def append_bound(rows, bounds, kernel, direction, limit, drift=None, buffer=False):
    coefficients = row(kernel, direction)
    if buffer:
        coefficients[-1] = -float(buffer)
    rows.append(coefficients)
    bounds.append(limit - (dot(direction, drift) if drift is not None else 0))


def closest_fraction(a, b, c, d):
    start = sub(a, c)
    change = sub(sub(b, a), sub(d, c))
    denominator = dot(change, change)
    return max(0, min(1, -dot(start, change) / denominator)) if denominator > 1e-12 else 0.0


def interpolate(a, b, fraction):
    return [x + fraction * (y-x) for x, y in zip(a, b)]


def collision_cuts(predictor, nominal, drift, other, own_start, other_start):
    """Yield end-point AND closest-interior cuts for the same PL model as validation."""
    zeros = [0.0] * predictor.M
    for k in range(predictor.N):
        previous_a = own_start if k == 0 else nominal[k-1]
        previous_b = other_start if k == 0 else other[k-1]
        fraction = closest_fraction(previous_a, nominal[k], previous_b, other[k])
        fractions = [1.0]
        if 1e-6 < fraction < 1-1e-6:
            fractions.append(fraction)
        for t in fractions:
            own = interpolate(previous_a, nominal[k], t)
            peer = interpolate(previous_b, other[k], t)
            separation = sub(own, peer)
            distance = norm(separation)
            if distance < 1e-8:
                separation = sub(previous_a, previous_b)
                if norm(separation) < 1e-8:
                    separation = [1, 0, 0]
            direction = mul(unit(separation), -1)
            previous_kernel = zeros if k == 0 else predictor.P[k-1]
            kernel = interpolate(previous_kernel, predictor.P[k], t)
            previous_drift = own_start if k == 0 else drift[k-1]
            base = interpolate(previous_drift, drift[k], t)
            yield kernel, direction, peer, base, distance, (k+t)*predictor.h


def build(predictor, geometry, nominal, drift_p, drift_v, neighbors, last_command,
          cfg, outer_cfg, directions, norm_factor, agent, location, states):
    rows, bounds = [], []
    M, N = predictor.M, predictor.N
    preferred = cfg['pair_margin']
    hard = cfg['pair_hard_margin']
    if not 0 <= hard <= preferred:
        raise ValueError('Pair hard margin must lie between zero and preferred margin')
    slack = [0.0] * (3*M) + [1.0]
    rows.extend([slack, [-x for x in slack]])
    bounds.extend([preferred-hard, 0.0])
    for j in range(M):
        basis = [float(i == j) for i in range(M)]
        for direction in directions:
            append_bound(rows, bounds, basis, direction, outer_cfg['vm'] * norm_factor)
        delta = basis[:] if not j else [float(i == j)-float(i == j-1) for i in range(M)]
        for direction in directions:
            append_bound(rows, bounds, delta, direction,
                         cfg['command_rate']*predictor.h*predictor.block_length*norm_factor,
                         mul(last_command, -1) if j == 0 else None)
    for k in range(N):
        for direction in directions:
            append_bound(rows, bounds, predictor.V[k], direction,
                         (outer_cfg['vm']-cfg['speed_margin'])*norm_factor, drift_v[k])
        geo = geometry.predicted_geometry(nominal[k], key=(agent,k), hint=location['segment'])
        tangent = geo['tangent']
        normal = unit(cross(tangent, [0,0,1] if abs(tangent[2]) < .9 else [0,1,0]))
        binormal = cross(tangent, normal)
        facets = cfg['tube_facets']
        radius = geo['radius']-outer_cfg['rs']-cfg['boundary_margin']
        for j in range(facets):
            angle = 2*math.pi*j/facets
            radial = [normal[a]*math.cos(angle)+binormal[a]*math.sin(angle) for a in range(3)]
            direction = [radial[a]-geo['radius_slope']*math.cos(math.pi/facets)*tangent[a] for a in range(3)]
            limit = radius*math.cos(math.pi/facets)+dot(direction, geo['point'])
            append_bound(rows, bounds, predictor.P[k], direction, limit, drift_p[k])
    for other, plan in neighbors.items():
        if other == agent:
            continue
        for kernel, direction, peer, base, distance, elapsed in collision_cuts(
                predictor, nominal, drift_p, plan, states[agent][:3], states[other][:3]):
            if distance > cfg['collision_activation']:
                continue
            limit = dot(direction, peer)-(2*outer_cfg['rs']+preferred)
            append_bound(rows, bounds, kernel, direction, limit, base, buffer=math.exp(-elapsed/cfg['pair_buffer_decay']))
    return rows, bounds


def segment_pair_distance(a, b, c, d):
    t = closest_fraction(a, b, c, d)
    return norm(sub(interpolate(a, b, t), interpolate(c, d, t)))
