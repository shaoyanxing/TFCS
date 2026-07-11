"""TFCS (Tendency Field Chain Search) — 趋向性场域锁链搜索算法。

以有向线段（锁链）为基本单元，以终点导向的叠加场域势能为核心驱动力，
通过自适应旋转增殖实现高效探索的通用搜索框架。
"""

from tfcs.core import DirectedSegment, Chain, ChainEndpoint, Point
from tfcs.field import FieldPotential, AttractionField, RepulsionField, InhibitionField, Obstacle
from tfcs.proliferate import Proliferator
from tfcs.searcher import TFCS, TFCSConfig

__all__ = [
    "DirectedSegment", "Chain", "ChainEndpoint", "Point",
    "FieldPotential", "AttractionField", "RepulsionField", "InhibitionField", "Obstacle",
    "Proliferator",
    "TFCS", "TFCSConfig",
]