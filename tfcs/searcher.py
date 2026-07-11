"""
TFCS 优先级队列搜索调度器
========================
使用优先级队列管理活跃链端点，按势能-代价比排序，优先扩展最有希望的方向。
实现混合终止策略：首达目标 → 冷却期择优 → 返回最优路径。
"""

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
from tfcs.core import Chain, DirectedSegment, ChainEndpoint, Point
from tfcs.field import FieldPotential, AttractionField, RepulsionField, InhibitionField, Obstacle
from tfcs.proliferate import Proliferator


@dataclass
class TFCSConfig:
    """TFCS 搜索器配置参数。

    Attributes:
        w1: 引力场权重
        w2: 斥力场权重
        w3: 抑制场权重
        alpha: 优先级中梯度幅值的权重
        beta: 优先级中代价的权重
        grad_threshold: 自适应阈值（梯度幅值）
        cooldown: 冷却期长度（连续无改进迭代次数）
        target_radius: 目标区域半径
        max_depth: 最大搜索深度
        max_iterations: 最大迭代次数硬限制
        angle_min: 最小角度步长（度）
        angle_max: 最大角度步长（度）
        length_min: 最小链长度
        length_max: 最大链长度
        repulsion_range: 斥力场影响范围
        inhibition_sigma: 抑制场 sigma 参数
    """
    w1: float = 1.0
    w2: float = 10.0
    w3: float = 2.0
    alpha: float = 1.0
    beta: float = 0.5
    grad_threshold: float = 0.5
    cooldown: int = 50
    target_radius: float = 3.0
    max_depth: int = 200
    max_iterations: int = 5000
    angle_min: float = 5.0       # 度
    angle_max: float = 45.0       # 度
    length_min: float = 2.0
    length_max: float = 20.0
    repulsion_range: float = 50.0
    inhibition_sigma: float = 10.0


class TFCS:
    """TFCS 搜索器 —— 趋向性场域锁链搜索算法主类。

    用法:
        config = TFCSConfig()
        searcher = TFCS(start, targets, obstacles, config)
        best_path = searcher.search()
    """

    def __init__(
        self,
        start: Point,
        targets: List[Point],
        obstacles: Optional[List[Obstacle]] = None,
        config: Optional[TFCSConfig] = None,
    ):
        self.start = start
        self.targets = targets
        self.obstacles = obstacles or []
        self.config = config or TFCSConfig()

        # 构建场域
        self.attraction = AttractionField(targets)
        self.repulsion = RepulsionField(self.obstacles, self.config.repulsion_range)
        self.inhibition = InhibitionField(sigma=self.config.inhibition_sigma)
        self.field = FieldPotential(
            self.attraction, self.repulsion, self.inhibition,
            w1=self.config.w1, w2=self.config.w2, w3=self.config.w3,
        )

        # 增殖器
        self.proliferator = Proliferator(
            angle_min=math.radians(self.config.angle_min),
            angle_max=math.radians(self.config.angle_max),
            length_min=self.config.length_min,
            length_max=self.config.length_max,
            grad_threshold=self.config.grad_threshold,
        )

        # 搜索状态
        self._all_chains: List[Chain] = []  # 所有生成的链（用于可视化和抑制场）
        self._best_chain: Optional[Chain] = None
        self._best_cost: float = float('inf')
        self._iterations: int = 0
        self._cooldown_counter: int = 0
        self._found_target: bool = False

    def _is_near_target(self, point: Point) -> bool:
        """检查点是否在目标区域内。"""
        return any(
            math.hypot(point[0] - t[0], point[1] - t[1]) < self.config.target_radius
            for t in self.targets
        )

    def _compute_priority(self, chain: Chain, endpoint: Point) -> float:
        """计算链端点的优先级。

        使用引力场和斥力场计算（跳过抑制场以提升性能）。
        势能越低优先级越高。
        """
        att_pot = self.attraction.potential(endpoint)
        rep_pot = self.repulsion.potential(endpoint)
        return -(self.config.w1 * att_pot + self.config.w2 * rep_pot)

    def _check_obstacle_collision(self, segment: DirectedSegment) -> bool:
        """检查线段是否与障碍物碰撞（简单端点检测）。"""
        for obs in self.obstacles:
            d_start = math.hypot(
                segment.start[0] - obs.center[0],
                segment.start[1] - obs.center[1],
            )
            d_end = math.hypot(
                segment.end[0] - obs.center[0],
                segment.end[1] - obs.center[1],
            )
            if d_start <= obs.radius or d_end <= obs.radius:
                return True
        return False

    def search(self) -> Optional[Chain]:
        """执行 TFCS 搜索，返回最优路径链。

        Returns:
            最优路径的 Chain 对象，如果未找到则返回 None。
        """
        # 初始化根链
        root_segment = DirectedSegment(
            start=self.start, angle=0.0, length=0.0, generation=0,
        )
        root_chain = Chain()
        root_chain.segments = [root_segment]
        self._all_chains.append(root_chain)

        # 如果起点就在目标区域内
        if self._is_near_target(self.start):
            return root_chain

        # 初始化优先级队列
        priority = self._compute_priority(root_chain, self.start)
        heap: List[Tuple[float, int, ChainEndpoint]] = []
        # 使用计数器作为 tiebreaker 保证堆稳定
        counter = 0
        heapq.heappush(heap, (-priority, counter, ChainEndpoint(root_chain, priority)))
        counter += 1

        while heap and self._iterations < self.config.max_iterations:
            self._iterations += 1

            # 弹出优先级最高的端点
            _, _, endpoint_wrapper = heapq.heappop(heap)
            chain = endpoint_wrapper.chain

            # 深度限制
            if chain.depth >= self.config.max_depth:
                continue

            # 检查是否到达目标区域
            if self._is_near_target(chain.endpoint):
                if not self._found_target:
                    self._found_target = True
                    self._best_chain = chain
                    self._best_cost = chain.total_cost
                    self._cooldown_counter = 0
                elif chain.total_cost < self._best_cost:
                    self._best_chain = chain
                    self._best_cost = chain.total_cost
                    self._cooldown_counter = 0  # 找到更优解，重置冷却
                else:
                    self._cooldown_counter += 1

                # 冷却期耗尽则终止
                if self._found_target and self._cooldown_counter >= self.config.cooldown:
                    break

            # 旋转增殖
            children = self.proliferator.proliferate(chain, self.field)

            for child in children:
                # 碰撞检测
                last_seg = child.segments[-1]
                if self._check_obstacle_collision(last_seg):
                    continue

                self._all_chains.append(child)

                # 更新抑制场
                self.inhibition.add_chain(child)

                # 计算优先级并入队
                child_priority = self._compute_priority(child, child.endpoint)
                heapq.heappush(
                    heap,
                    (-child_priority, counter, ChainEndpoint(child, child_priority)),
                )
                counter += 1

            # 如果还没找到目标，冷却计数
            if self._found_target and self._cooldown_counter < self.config.cooldown:
                self._cooldown_counter += 1

        # 返回结果
        if self._best_chain is not None:
            return self._best_chain

        # 如果没找到目标，返回距离目标最近的链
        if self._all_chains:
            best = min(
                self._all_chains[1:],  # 跳过根链
                key=lambda c: min(math.hypot(c.endpoint[0] - t[0], c.endpoint[1] - t[1])
                                  for t in self.targets),
            )
            return best

        return None

    @property
    def all_chains(self) -> List[Chain]:
        """返回搜索过程中生成的所有链（用于可视化）。"""
        return self._all_chains

    @property
    def iterations(self) -> int:
        """返回搜索迭代次数。"""
        return self._iterations