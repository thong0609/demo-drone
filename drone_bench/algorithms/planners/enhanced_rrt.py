"""Algorithms 2–3 and Eqs. 15–16; static analytical SDF queries."""
import math
import random
from ...common.vectors import add, sub, mul, norm


def edge_cost(env, a, b, cfg, re):
    distance = norm(sub(b, a))
    n = max(1, math.ceil(distance / cfg['edge_spacing']))
    penalty = 0.0
    for j in range(n + 1):
        d = env.clearance(add(a, mul(sub(b, a), j / n)))
        if d < 0: return float('inf')
        if d < re: penalty += (re - d)**3 / (d**3 + cfg['sdf_kappa'])
    return cfg['lambda1'] * distance + cfg['lambda2'] * penalty


def plan(env, cfg, seed):
    rng = random.Random(seed)
    world = env.config
    start, goal = world['start'], world['goal']
    re = world['tube']['max_radius']
    vertices, parents, costs, children = [start], [-1], [0.0], [set()]
    solutions = set()
    cache = {}

    def connection(a, b):
        key = tuple(sorted((tuple(a), tuple(b))))
        if key not in cache:
            cache[key] = (edge_cost(env, a, b, cfg, re)
                          if env.segment_free(a, b, cfg['clearance'], cfg['edge_spacing']) else None)
        return cache[key]

    def visual(parent, p, total):
        # Algorithm 3: keep walking through ancestors only while cost improves.
        ancestor = parents[parent]
        while ancestor >= 0:
            edge = connection(vertices[ancestor], p)
            if edge is None or costs[ancestor] + edge >= total - 1e-9: break
            parent, total = ancestor, costs[ancestor] + edge
            ancestor = parents[ancestor]
        return parent, total

    def propagate(index, delta):
        costs[index] += delta
        for child in children[index]: propagate(child, delta)

    lo, hi = world['bounds']
    for _ in range(cfg['iterations']):
        sample = goal if rng.random() < cfg['goal_bias'] else [rng.uniform(a,b) for a,b in zip(lo,hi)]
        nearest = min(range(len(vertices)), key=lambda i: norm(sub(vertices[i], sample)))
        delta = sub(sample, vertices[nearest]); distance = norm(delta)
        if distance < 1e-7: continue
        p = add(vertices[nearest], mul(delta, min(1, cfg['step']/distance)))
        edge = connection(vertices[nearest], p)
        if edge is None: continue
        near = [i for i,v in enumerate(vertices) if norm(sub(v,p)) <= cfg['neighbor_radius']]
        parent, total = nearest, costs[nearest] + edge
        for i in near:
            edge = connection(vertices[i], p)
            if edge is not None and costs[i] + edge < total:
                parent, total = i, costs[i] + edge
        parent, total = visual(parent, p, total)
        index = len(vertices)
        vertices.append(p); parents.append(parent); costs.append(total); children.append(set())
        children[parent].add(index)
        ancestors=set(); k=parent
        while k >= 0: ancestors.add(k); k=parents[k]
        for i in near:
            if i in ancestors: continue
            edge = connection(p, vertices[i])
            if edge is None or total + edge >= costs[i]-1e-9: continue
            new_parent, new_cost = visual(index, vertices[i], total+edge)
            children[parents[i]].remove(i); parents[i]=new_parent; children[new_parent].add(i)
            propagate(i,new_cost-costs[i])
        if norm(sub(p,goal)) <= cfg['goal_radius'] and connection(p,goal) is not None:
            solutions.add(index)
    if not solutions: raise RuntimeError('Enhanced RRT*: no feasible route; increase iterations or revise planner settings')
    index = min(solutions,key=lambda i: costs[i]+connection(vertices[i],goal))
    path=[goal]
    while index >= 0:
        if norm(sub(path[-1],vertices[index])) > 1e-7: path.append(vertices[index])
        index=parents[index]
    path.reverse()
    return path, {'algorithm':'enhanced_rrt_star_visual', 'vertices':len(vertices),
                  'iterations':cfg['iterations'], 'mixed_cost':sum(edge_cost(env,a,b,cfg,re) for a,b in zip(path,path[1:])),
                  'waypoints':path}
