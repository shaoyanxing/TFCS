"""
TFCS (Tendency Field Chain Search) 核心数据结构
==============================================
以有向线段（锁链）为基本单元，以终点导向的场域势能为核心驱动力的搜索算法。
"""

import math
import itertools
from typing import Optional, Tuple, List

Point = Tuple[float, float]


class DirectedSegment:
    """有向线段 —— TFCS 的基本搜索单元。

    由起点、方向角和长度唯一定义，终点自动计算。
    """

    def __init__(self, start: Point, angle: float, length: float, generation: int = 0):
        self.start = start
        self.angle = angle  # 弧度
        self.length = length
        self.generation = generation
        self.end = (
            start[0] + length * math.cos(angle),
            start[1] + length * math.sin(angle),
        )

    def __repr__(self):
        return (f"DirectedSegment(start={self.start}, end={self.end}, "
                f"angle={self.angle:.3f}, length={self.length:.3f}, gen={self.generation})")


class Chain:
    """锁链 —— 由多个有向线段首尾相连组成的路径。

    每条链记录其完整路径、深度和累计代价。
    """

    _id_counter = itertools.count()

    def __init__(self, parent: Optional["Chain"] = None):
        self.id = next(Chain._id_counter)
        self.segments: List[DirectedSegment] = []
        self.parent_id: Optional[int] = parent.id if parent else None
        self.depth: int = parent.depth + 1 if parent else 0
        self.total_cost: float = parent.total_cost if parent else 0.0

    @property
    def endpoint(self) -> Point:
        """当前链的终点坐标（最后一个 segment 的 end）。"""
        if not self.segments:
            raise ValueError("Chain has no segments")
        return self.segments[-1].end

    @property
    def startpoint(self) -> Point:
        """当前链的起点坐标（第一个 segment 的 start）。"""
        if not self.segments:
            raise ValueError("Chain has no segments")
        return self.segments[0].start

    def append(self, segment: DirectedSegment) -> "Chain":
        """追加一个有向线段，返回新链。

        新链继承当前链的所有 segments 再加上新 segment。
        """
        new_chain = Chain(parent=self)
        new_chain.segments = self.segments.copy()
        new_chain.segments.append(segment)
        new_chain.total_cost = self.total_cost + segment.length
        new_chain.depth = self.depth + 1
        return new_chain

    def get_path_points(self) -> List[Point]:
        """返回路径上所有顶点的坐标序列。"""
        if not self.segments:
            return []
        points = [self.segments[0].start]
        for seg in self.segments:
            points.append(seg.end)
        return points

    def __repr__(self):
        return (f"Chain(id={self.id}, depth={self.depth}, "
                f"cost={self.total_cost:.3f}, segments={len(self.segments)})")


class ChainEndpoint:
    """链端点包装器，用于优先级队列排序。

    持有 Chain 引用，按优先级值支持堆排序（值越大优先级越高）。
    """

    def __init__(self, chain: Chain, priority: float):
        self.chain = chain
        self.priority = priority

    @property
    def point(self) -> Point:
        return self.chain.endpoint

    def __lt__(self, other: "ChainEndpoint") -> bool:
        # 优先级高的排在前面（大顶堆用负值实现，Python heapq 是小顶堆）
        return self.priority > other.priority

    def __repr__(self):
        return f"ChainEndpoint(chain_id={self.chain.id}, priority={self.priority:.4f})"