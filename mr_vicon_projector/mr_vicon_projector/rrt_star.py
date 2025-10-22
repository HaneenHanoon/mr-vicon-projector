
"""
rrt_star.py — RRT* (asymptotically optimal) with:
  • Edge collision checks (not just point checks)
  • Dynamic neighbor radius ~ sqrt(log n / n) (or fixed if you pass one)
  • Keep searching after first goal contact; track best goal-connected path
  • Cost propagation after rewires to keep descendants consistent
  • Optional path post-smoothing (shortcut)
  • Optional return of the built tree (return_tree=True)
  • Back-compat alias: RRTNode = Node

Typical use:
    from rrt_star import rrt_star

    path = rrt_star(
        start=(xs, ys),
        goal=(xg, yg),
        bounds={"x": (xmin, xmax), "y": (ymin, ymax)},
        obstacles=obstacles,
        max_iter=4000,
        step=0.4,
        rewire_radius=None,     # None => dynamic radius
        goal_tolerance=0.4,
        progress=True
    )
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional, Union
import math
import random

# -----------------------------
# Tunables / Defaults
# -----------------------------
DEFAULT_ROBOT_RADIUS = 0.35      # obstacle inflation to respect robot footprint
DEFAULT_STEP = 0.25              # extension step (meters)
DEFAULT_MAX_ITER = 4000
DEFAULT_SAMPLE_EDGE = 0.05       # edge sampling step (meters) for collision checks
DEFAULT_GAMMA = 40.0             # scale for dynamic neighbor radius
DEFAULT_GOAL_BIAS = 0.05         # 5% samples go straight to the goal
DEFAULT_GOAL_TOL = 0.40          # connect to goal when within this distance


# -----------------------------
# Node
# -----------------------------
@dataclass
class Node:
    x: float
    y: float
    parent: Optional["Node"] = None
    cost: float = 0.0

# Back-compat for code that imported/used RRTNode
RRTNode = Node


# -----------------------------
# Geometry helpers
# -----------------------------
def _seg_len(a: Node, b: Node) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def _point_in_bounds(x: float, y: float, bounds: Dict[str, Tuple[float, float]]) -> bool:
    return (bounds["x"][0] <= x <= bounds["x"][1]) and (bounds["y"][0] <= y <= bounds["y"][1])


# -----------------------------
# Collision checking
# -----------------------------
def point_collision_free(
    x: float,
    y: float,
    obstacles: List[dict],
    robot_r: float = DEFAULT_ROBOT_RADIUS,
) -> bool:
    """
    Returns True if point (x, y) is free given obstacles.

    Supported obstacle forms (adapt as needed to your schema):
      Circle-like:
        {"type": "circle", "cx": 0.0, "cy": 0.0, "r": 1.0}
        {"type": "circle", "center": [cx, cy], "radius": r}

      Axis-aligned or rotated rectangle:
        {"type": "rect", "x": x0, "y": y0, "w": width, "h": height}
        {"type": "rect", "center": [cx, cy], "w": width, "h": height}
        {"type": "rect", "center": [cx, cy], "w": width, "h": height, "theta": radians}

      Generic "box" (same as rect), optional per-obstacle "inflate".
    """
    for obs in obstacles:
        inflate = float(obs.get("inflate", 0.0)) + robot_r
        typ = (obs.get("type") or obs.get("shape") or "rect").lower()

        if typ == "circle":
            if "center" in obs:
                cx, cy = obs["center"]
                r = float(obs.get("radius", obs.get("r", 0.0)))
            else:
                cx = float(obs.get("cx", 0.0))
                cy = float(obs.get("cy", 0.0))
                r = float(obs.get("r", obs.get("radius", 0.0)))
            if math.hypot(x - cx, y - cy) <= (r + inflate):
                return False

        elif typ in ("rect", "box"):
            theta = float(obs.get("theta", 0.0))

            if "center" in obs:
                cx, cy = obs["center"]
                w = float(obs.get("w", obs.get("width", 0.0))) + 2 * inflate
                h = float(obs.get("h", obs.get("height", 0.0))) + 2 * inflate

                # Rotate (x, y) into rectangle local frame (rotate -theta about center)
                dx, dy = x - cx, y - cy
                c, s = math.cos(-theta), math.sin(-theta)
                lx = c * dx - s * dy
                ly = s * dx + c * dy
                if abs(lx) <= w / 2.0 and abs(ly) <= h / 2.0:
                    return False
            else:
                # Axis-aligned rectangle using (x0, y0, w, h)
                x0 = float(obs.get("x", 0.0)) - inflate
                y0 = float(obs.get("y", 0.0)) - inflate
                w = float(obs.get("w", obs.get("width", 0.0))) + 2 * inflate
                h = float(obs.get("h", obs.get("height", 0.0))) + 2 * inflate

                if theta != 0.0:
                    # Interpret (x0, y0) as center if theta provided
                    cx, cy = x0, y0
                    dx, dy = x - cx, y - cy
                    c, s = math.cos(-theta), math.sin(-theta)
                    lx = c * dx - s * dy
                    ly = s * dx + c * dy
                    if abs(lx) <= w / 2.0 and abs(ly) <= h / 2.0:
                        return False
                else:
                    if (x0 <= x <= x0 + w) and (y0 <= y <= y0 + h):
                        return False

        else:
            # Unknown type: treat as blocking AABB if we can
            x0 = float(obs.get("x", obs.get("cx", x)))
            y0 = float(obs.get("y", obs.get("cy", y)))
            w = float(obs.get("w", obs.get("r", 0.0))) + 2 * inflate
            h = float(obs.get("h", obs.get("r", 0.0))) + 2 * inflate
            if (x0 - w / 2.0 <= x <= x0 + w / 2.0) and (y0 - h / 2.0 <= y <= y0 + h / 2.0):
                return False

    return True


def _edge_collision_free(
    a: Node,
    b: Node,
    obstacles: List[dict],
    robot_r: float = DEFAULT_ROBOT_RADIUS,
    sample_step: float = DEFAULT_SAMPLE_EDGE,
) -> bool:
    """Sample along segment a→b (including b)."""
    dx, dy = b.x - a.x, b.y - a.y
    L = math.hypot(dx, dy)
    if L == 0.0:
        return point_collision_free(a.x, a.y, obstacles, robot_r)
    steps = max(1, int(L / sample_step))
    for i in range(1, steps + 1):
        t = i / steps
        px = a.x + t * dx
        py = a.y + t * dy
        if not point_collision_free(px, py, obstacles, robot_r):
            return False
    return True


# -----------------------------
# Tree utilities
# -----------------------------
def _children_of(tree: List[Node], parent: Node) -> List[Node]:
    return [n for n in tree if n.parent is parent]


def _propagate_costs_from(tree: List[Node], node: Node):
    """After rewiring a node, update costs of its descendants."""
    stack = [node]
    while stack:
        cur = stack.pop()
        for ch in _children_of(tree, cur):
            ch.cost = cur.cost + _seg_len(cur, ch)
            stack.append(ch)


def _extract_path(goal_node: Node) -> List[Tuple[float, float]]:
    path: List[Tuple[float, float]] = []
    n = goal_node
    while n is not None:
        path.append((n.x, n.y))
        n = n.parent
    path.reverse()
    return path


# -----------------------------
# Nearest / Near / Steer
# -----------------------------
def steer(from_nd: Node, to_px: Tuple[float, float], step: float) -> Node:
    tx, ty = to_px
    dx, dy = tx - from_nd.x, ty - from_nd.y
    L = math.hypot(dx, dy)
    if L == 0.0:
        return Node(from_nd.x, from_nd.y, from_nd.parent, from_nd.cost)
    s = min(step, L)
    return Node(from_nd.x + s * dx / L, from_nd.y + s * dy / L)


def nearest(tree: List[Node], pt: Tuple[float, float]) -> Node:
    px, py = pt
    best = tree[0]
    best_d2 = (best.x - px) ** 2 + (best.y - py) ** 2
    for n in tree[1:]:
        d2 = (n.x - px) ** 2 + (n.y - py) ** 2
        if d2 < best_d2:
            best_d2, best = d2, n
    return best


def near(tree: List[Node], center: Node, radius: float) -> List[Node]:
    r2 = radius * radius
    out: List[Node] = []
    cx, cy = center.x, center.y
    for n in tree:
        if n is center:
            continue
        d2 = (n.x - cx) ** 2 + (n.y - cy) ** 2
        if d2 <= r2:
            out.append(n)
    return out


# -----------------------------
# Public API: RRT*
# -----------------------------
def rrt_star(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    bounds: Dict[str, Tuple[float, float]],
    obstacles: List[dict],
    *,
    robot_radius: float = DEFAULT_ROBOT_RADIUS,
    step: float = DEFAULT_STEP,
    max_iter: int = DEFAULT_MAX_ITER,
    rewire_radius: Optional[float] = None,   # None => dynamic sqrt(log n / n)
    gamma: float = DEFAULT_GAMMA,            # scale for dynamic radius
    goal_tolerance: float = DEFAULT_GOAL_TOL,
    goal_sample_rate: float = DEFAULT_GOAL_BIAS,
    sample_edge: float = DEFAULT_SAMPLE_EDGE,
    seed: Optional[int] = None,
    progress: bool = True,
    return_tree: bool = False,
) -> Union[None, List[Tuple[float, float]], Tuple[List[Tuple[float, float]], List[Node]]]:
    """
    Core RRT* planner.

    Returns:
        - None if no solution found.
        - path as List[(x,y)] if found.
        - (path, tree) if return_tree=True.
    """
    if seed is not None:
        random.seed(seed)

    sx, sy = start
    gx, gy = goal

    if not _point_in_bounds(sx, sy, bounds):
        raise ValueError("Start is out of bounds")
    if not _point_in_bounds(gx, gy, bounds):
        raise ValueError("Goal is out of bounds")

    start_nd = Node(sx, sy, parent=None, cost=0.0)
    tree: List[Node] = [start_nd]

    best_goal_node: Optional[Node] = None
    best_cost: float = float("inf")

    def sample_free() -> Tuple[float, float]:
        if random.random() < goal_sample_rate:
            return gx, gy
        return (random.uniform(*bounds["x"]), random.uniform(*bounds["y"]))

    for it in range(max_iter):
        rx, ry = sample_free()
        nn = nearest(tree, (rx, ry))
        new = steer(nn, (rx, ry), step)

        # Bounds + edge collision from nearest
        if not _point_in_bounds(new.x, new.y, bounds):
            continue
        if not _edge_collision_free(nn, new, obstacles, robot_radius, sample_edge):
            continue

        # Neighbor radius
        n = len(tree)
        if rewire_radius is None:
            # 2D schedule; cap to avoid exploding early
            radius = min(step * 50.0, gamma * math.sqrt(max(1e-9, math.log(n + 1) / (n + 1))))
        else:
            radius = rewire_radius

        # ChooseParent: among neighbors that connect collision-free
        neighbors = near(tree, new, radius)
        best_parent = nn
        best_new_cost = nn.cost + _seg_len(nn, new)
        for nb in neighbors:
            if _edge_collision_free(nb, new, obstacles, robot_radius, sample_edge):
                c = nb.cost + _seg_len(nb, new)
                if c < best_new_cost:
                    best_new_cost = c
                    best_parent = nb

        new.parent = best_parent
        new.cost = best_new_cost
        tree.append(new)

        # Rewire neighbors if 'new' improves them
        for nb in neighbors:
            if nb is best_parent:
                continue
            cand = new.cost + _seg_len(new, nb)
            if cand + 1e-9 < nb.cost and _edge_collision_free(new, nb, obstacles, robot_radius, sample_edge):
                nb.parent = new
                nb.cost = cand
                _propagate_costs_from(tree, nb)

        # Tentative goal connection
        dg = math.hypot(new.x - gx, new.y - gy)
        if dg <= goal_tolerance:
            goal_nd = Node(gx, gy, parent=new, cost=new.cost + dg)
            if _edge_collision_free(new, goal_nd, obstacles, robot_radius, sample_edge):
                if goal_nd.cost < best_cost:
                    best_goal_node = goal_nd
                    best_cost = goal_nd.cost

        if progress and (it % 100 == 0) and best_cost < float("inf"):
            print(f"[it={it:5d}] best_cost={best_cost:.3f}  tree={len(tree):4d}  radius={radius:.3f}")

    if best_goal_node is None:
        return None

    path = _extract_path(best_goal_node)
    if return_tree:
        return path, tree
    return path


# -----------------------------
# Optional: post-smoothing
# -----------------------------
def shortcut(
    path: List[Tuple[float, float]],
    obstacles: List[dict],
    robot_radius: float = DEFAULT_ROBOT_RADIUS,
    tries: int = 200,
    sample_edge: float = DEFAULT_SAMPLE_EDGE,
) -> List[Tuple[float, float]]:
    """Simple 'shortcut' smoother that preserves feasibility."""
    if not path or len(path) <= 2:
        return path[:]

    # Build nodes for convenience
    nodes = [Node(x, y) for (x, y) in path]
    for _ in range(tries):
        i, j = sorted(random.sample(range(len(nodes)), 2))
        if j - i <= 1:
            continue
        a, b = nodes[i], nodes[j]
        if _edge_collision_free(a, b, obstacles, robot_radius, sample_edge):
            nodes = nodes[: i + 1] + nodes[j:]  # keep a, drop (i+1..j-1), keep b
    return [(n.x, n.y) for n in nodes]


__all__ = ["Node", "RRTNode", "rrt_star", "shortcut", "point_collision_free"]