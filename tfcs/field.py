"""
TFCS 叠加场域势能模块
====================
实现三种场域的叠加计算：引力场（目标吸引）、斥力场（障碍排斥）、抑制场（已探索区域抑制）。
"""

import math
from typing import List, Optional, Tuple
from tfcs.core import Point, Chain


class Obstacle:
    """圆形障碍物。"""

    def __init__(self, center: Point, radius: float):
        self.center = center
        self.radius = radius


def _dist(a: Point, b: Point) -> float:
    """两点间的欧氏距离。"""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _point_to_segment_dist(p: Point, seg_start: Point, seg_end: Point) -> float:
    """点到线段的最短距离。"""
    dx = seg_end[0] - seg_start[0]
    dy = seg_end[1] - seg_start[1]
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        return _dist(p, seg_start)
    t = max(0.0, min(1.0, ((p[0] - seg_start[0]) * dx + (p[1] - seg_start[1]) * dy) / seg_len_sq))
    proj = (seg_start[0] + t * dx, seg_start[1] + t * dy)
    return _dist(p, proj)


class AttractionField:
    """引力场 —— 目标点产生吸引力，势能随距离增加而增加。

    势能函数: U(p) = min_i dist(p, target_i)
    离目标越近势能越低，梯度指向最近目标。
    """

    def __init__(self, targets: List[Point]):
        self.targets = targets

    def potential(self, p: Point) -> float:
        if not self.targets:
            return 0.0
        return min(_dist(p, t) for t in self.targets)

    def gradient(self, p: Point) -> Tuple[float, float]:
        if not self.targets:
            return (0.0, 0.0)
        # 找到最近目标
        nearest = min(self.targets, key=lambda t: _dist(p, t))
        d = _dist(p, nearest)
        if d < 1e-12:
            return (0.0, 0.0)
        # 梯度指向最近目标方向
        return ((nearest[0] - p[0]) / d, (nearest[1] - p[1]) / d)


class RepulsionField:
    """斥力场 —— 障碍物产生排斥力，势能在障碍物边界处急剧升高。

    势能函数: U(p) = sum_i 1 / max(dist(p, obs_i) - obs_i.radius, eps)
    靠近障碍物边界时势能急剧升高，超出影响范围后衰减。
    """

    def __init__(self, obstacles: List[Obstacle], influence_range: float = 50.0):
        self.obstacles = obstacles
        self.influence_range = influence_range
        self._eps = 0.01

    def potential(self, p: Point) -> float:
        total = 0.0
        for obs in self.obstacles:
            d_center = _dist(p, obs.center)
            d_boundary = d_center - obs.radius
            if d_boundary <= 0:
                return float('inf')  # 在障碍物内部
            if d_boundary < self.influence_range:
                total += 1.0 / max(d_boundary, self._eps)
        return total

    def gradient(self, p: Point) -> Tuple[float, float]:
        gx, gy = 0.0, 0.0
        for obs in self.obstacles:
            d_center = _dist(p, obs.center)
            d_boundary = d_center - obs.radius
            if d_boundary <= self._eps:
                # 非常靠近或进入障碍物，强排斥力推离中心
                if d_center < 1e-12:
                    continue
                strength = 100.0 / max(d_boundary, self._eps) ** 2
                dir_x = (p[0] - obs.center[0]) / d_center
                dir_y = (p[1] - obs.center[1]) / d_center
                gx += strength * dir_x
                gy += strength * dir_y
            elif d_boundary < self.influence_range:
                strength = 1.0 / d_boundary ** 2
                dir_x = (p[0] - obs.center[0]) / d_center
                dir_y = (p[1] - obs.center[1]) / d_center
                gx += strength * dir_x
                gy += strength * dir_y
        return (gx, gy)


class InhibitionField:
    """抑制场 —— 已探索区域产生抑制，降低重复探索的优先级。

    势能函数: U(p) = sum_{seg in explored} exp(-dist(p, seg)^2 / (2 * sigma^2))
    已探索区域势能升高，引导搜索走向未探索区域。

    使用滑动窗口限制存储的线段数量，保证 O(1) 性能。
    """

    def __init__(self, sigma: float = 10.0, max_segments: int = 500):
        self.sigma = sigma
        self.max_segments = max_segments
        self._explored_segments: List[Tuple[Point, Point]] = []

    def add_chain(self, chain: Chain):
        """将一条链的所有线段加入已探索集合（滑动窗口限制）。"""
        for seg in chain.segments:
            self._explored_segments.append((seg.start, seg.end))
        # 保持滑动窗口大小
        if len(self._explored_segments) > self.max_segments:
            self._explored_segments = self._explored_segments[-self.max_segments:]

    def potential(self, p: Point) -> float:
        if not self._explored_segments:
            return 0.0
        total = 0.0
        sigma2 = 2.0 * self.sigma * self.sigma
        for seg_start, seg_end in self._explored_segments:
            d = _point_to_segment_dist(p, seg_start, seg_end)
            total += math.exp(-d * d / sigma2)
        return total

    def gradient(self, p: Point) -> Tuple[float, float]:
        """数值差分计算抑制场梯度。"""
        if not self._explored_segments:
            return (0.0, 0.0)
        h = 0.1
        u0 = self.potential(p)
        ux = self.potential((p[0] + h, p[1]))
        uy = self.potential((p[0], p[1] + h))
        return ((ux - u0) / h, (uy - u0) / h)


class FieldPotential:
    """叠加场域势能 —— 加权合成引力场、斥力场和抑制场。

    势能: U(p) = w1 * U_att(p) + w2 * U_rep(p) + w3 * U_inh(p)
    梯度: ∇U(p) = w1 * ∇U_att(p) + w2 * ∇U_rep(p) + w3 * ∇U_inh(p)

    搜索方向为负梯度方向（向势能更低处移动）。
    """

    def __init__(
        self,
        attraction: AttractionField,
        repulsion: RepulsionField,
        inhibition: InhibitionField,
        w1: float = 1.0,
        w2: float = 10.0,
        w3: float = 2.0,
    ):
        self.attraction = attraction
        self.repulsion = repulsion
        self.inhibition = inhibition
        self.w1 = w1
        self.w2 = w2
        self.w3 = w3

    def compose(self, p: Point) -> float:
        """计算叠加势能值。"""
        return (
            self.w1 * self.attraction.potential(p) +
            self.w2 * self.repulsion.potential(p) +
            self.w3 * self.inhibition.potential(p)
        )

    def gradient(self, p: Point) -> Tuple[float, float]:
        """计算叠加势能梯度向量。"""
        g_att = self.attraction.gradient(p)
        g_rep = self.repulsion.gradient(p)
        if abs(self.w3) < 1e-12:
            g_inh = (0.0, 0.0)
        else:
            g_inh = self.inhibition.gradient(p)
        return (
            self.w1 * g_att[0] + self.w2 * g_rep[0] + self.w3 * g_inh[0],
            self.w1 * g_att[1] + self.w2 * g_rep[1] + self.w3 * g_inh[1],
        )

    def gradient_magnitude(self, p: Point) -> float:
        """梯度幅值。"""
        gx, gy = self.gradient(p)
        return math.hypot(gx, gy)

    def gradient_direction(self, p: Point) -> float:
        """梯度方向角（弧度），指向势能升高最快的方向。"""
        gx, gy = self.gradient(p)
        return math.atan2(gy, gx)

    def descent_direction(self, p: Point) -> float:
        """下降方向角（弧度），指向势能降低最快的方向，即搜索前进方向。"""
        return math.atan2(self.gradient(p)[1], self.gradient(p)[0])