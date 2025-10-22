import json
import importlib.resources
import random
import math
from typing import List, Tuple

# ---- Tunables ---------------------------------------------------------------

ROBOT_RADIUS = 0.35          # meters (create3 footprint ~35cm)
SAFETY_PAD   = 0.05          # extra margin (meters)
GOAL_BIAS    = 0.10          # 10% samples draw the goal to speed up planning
MAX_ITER     = 1200
STEP_SIZE    = 0.45          # tree extension step (meters)
EDGE_RES     = 0.05          # collision sampling resolution along an edge (meters)

# ----------------------------------------------------------------------------

class Node:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.parent: "Node | None" = None

def distance(a: Node, b: Node) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)

def steer(from_node: Node, to_node: Node, step: float) -> Node:
    d = distance(from_node, to_node)
    if d <= step:
        return Node(to_node.x, to_node.y)
    θ = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
    return Node(from_node.x + step * math.cos(θ),
                from_node.y + step * math.sin(θ))

# ---------- Obstacles IO -----------------------------------------------------

def load_obstacles() -> list:
    """Read rrt_planner/data/obstacles.json (your existing format)."""
    with importlib.resources.open_text('rrt_planner.data', 'obstacles.json') as f:
        return json.load(f)

# ---------- Collision helpers ------------------------------------------------

def _point_inside_box(px: float, py: float, cx: float, cy: float,
                      sx: float, sy: float, pad: float) -> bool:
    """Axis-aligned box centered at (cx,cy), size (sx,sy), grown by pad."""
    hx = sx / 2.0 + pad
    hy = sy / 2.0 + pad
    return (cx - hx) <= px <= (cx + hx) and (cy - hy) <= py <= (cy + hy)

def _point_inside_cylinder_or_sphere(px: float, py: float,
                                     cx: float, cy: float,
                                     radius: float, pad: float) -> bool:
    return math.hypot(px - cx, py - cy) <= (radius + pad)

def _point_collides(px: float, py: float, obstacles: list) -> bool:
    """Check a single 2D point against all obstacles with inflation."""
    pad = ROBOT_RADIUS + SAFETY_PAD
    for obs in obstacles:
        ox = obs["pose"]["x"]
        oy = obs["pose"]["y"]
        typ = obs["type"]
        if typ in ("sphere", "cylinder"):
            if _point_inside_cylinder_or_sphere(px, py, ox, oy, obs["radius"], pad):
                return True
        elif typ == "box":
            sx, sy = obs["size"][0], obs["size"][1]   # ignore Z
            if _point_inside_box(px, py, ox, oy, sx, sy, pad):
                return True
    return False

def _edge_collision_free(a: Node, b: Node, obstacles: list) -> bool:
    """Sample the segment a→b every EDGE_RES meters and check inflated collision."""
    seg_len = max(distance(a, b), EDGE_RES)
    steps = int(seg_len / EDGE_RES)
    for i in range(steps + 1):
        t = i / max(steps, 1)
        px = a.x + (b.x - a.x) * t
        py = a.y + (b.y - a.y) * t
        if _point_collides(px, py, obstacles):
            return False
    return True

# ---------- RRT core ---------------------------------------------------------

def _find_nearest(tree: List[Node], rnd: Node) -> Node:
    return min(tree, key=lambda n: distance(n, rnd))

def _extract_path(goal: Node) -> List[Tuple[float, float]]:
    out = []
    cur = goal
    while cur is not None:
        out.append((cur.x, cur.y))
        cur = cur.parent
    return out[::-1]

def rrt(start: Tuple[float, float],
        goal: Tuple[float, float],
        bounds: dict,
        obstacles: list,
        max_iter: int = MAX_ITER,
        step: float = STEP_SIZE) -> Tuple[List[Tuple[float, float]] | None, List[Node]]:
    """
    Return (path, tree). 'path' is a list of (x,y) or None when no solution.
    """
    start_node = Node(*start)
    goal_node  = Node(*goal)
    tree: List[Node] = [start_node]

    # make sure start itself is not inside an obstacle
    if _point_collides(start_node.x, start_node.y, obstacles):
        return None, tree

    for _ in range(max_iter):
        # goal-biased random sample
        if random.random() < GOAL_BIAS:
            rnd = Node(goal_node.x, goal_node.y)
        else:
            rnd = Node(random.uniform(*bounds["x"]),
                       random.uniform(*bounds["y"]))

        nearest = _find_nearest(tree, rnd)
        new = steer(nearest, rnd, step)

        # skip if new point is outside map bounds
        if not (bounds["x"][0] <= new.x <= bounds["x"][1] and
                bounds["y"][0] <= new.y <= bounds["y"][1]):
            continue

        # collision check along the entire edge
        if not _edge_collision_free(nearest, new, obstacles):
            continue

        new.parent = nearest
        tree.append(new)

        # try direct connection to goal when close enough
        if distance(new, goal_node) <= step:
            if _edge_collision_free(new, goal_node, obstacles):
                goal_node.parent = new
                tree.append(goal_node)
                return _extract_path(goal_node), tree

    # no solution
    return None, tree

