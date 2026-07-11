"""
TFCS 自适应旋转增殖引擎
======================
根据局部势能梯度自适应调整旋转角度步长和锁链步长，实现"粗探-细搜"两级策略。
"""

import math
from typing import List, Optional
from tfcs.core import Chain, DirectedSegment, Point
from tfcs.field import FieldPotential


class Proliferator:
    """自适应旋转增殖器。

    在链端点处，以梯度下降方向为中心，在 ±90° 扇形范围内
    按自适应角度步长生成多条子链。

    参数:
        angle_min: 最小角度步长（弧度），梯度大时使用
        angle_max: 最大角度步长（弧度），梯度小时使用
        length_min: 最小链长度，梯度大时使用
        length_max: 最大链长度，梯度小时使用
        grad_threshold: 梯度阈值，用于自适应映射
    """

    def __init__(
        self,
        angle_min: float = math.radians(5),
        angle_max: float = math.radians(45),
        length_min: float = 2.0,
        length_max: float = 20.0,
        grad_threshold: float = 0.5,
    ):
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.length_min = length_min
        self.length_max = length_max
        self.grad_threshold = grad_threshold

    def _adaptive_factor(self, grad_magnitude: float) -> float:
        """计算自适应因子，范围 [0, 1]。

        梯度越大，因子越小 → 细搜；梯度越小，因子越大 → 粗探。
        """
        return math.exp(-grad_magnitude / self.grad_threshold)

    def _adaptive_angle_step(self, grad_magnitude: float) -> float:
        """根据梯度幅值计算自适应角度步长。"""
        factor = self._adaptive_factor(grad_magnitude)
        return self.angle_min + (self.angle_max - self.angle_min) * factor

    def _adaptive_length(self, grad_magnitude: float) -> float:
        """根据梯度幅值计算自适应链长度（与角度步长联动）。"""
        factor = self._adaptive_factor(grad_magnitude)
        return self.length_min + (self.length_max - self.length_min) * factor

    def proliferate(
        self,
        chain: Chain,
        field: FieldPotential,
        max_branches: Optional[int] = None,
    ) -> List[Chain]:
        """在链端点处进行旋转增殖，生成子链列表。

        Args:
            chain: 当前链
            field: 叠加场域势能
            max_branches: 最大子链数量，None 表示不限制

        Returns:
            子链列表
        """
        endpoint = chain.endpoint
        grad_mag = field.gradient_magnitude(endpoint)
        descent_dir = field.descent_direction(endpoint)

        angle_step = self._adaptive_angle_step(grad_mag)
        chain_length = self._adaptive_length(grad_mag)

        # 采样方向：descent_dir ± 90° 扇形
        children = []
        angle = descent_dir - math.pi / 2  # 从 -90° 开始
        half_pi = math.pi / 2

        while angle <= descent_dir + half_pi + 1e-9:
            segment = DirectedSegment(
                start=endpoint,
                angle=angle,
                length=chain_length,
                generation=chain.depth + 1,
            )
            child = chain.append(segment)
            children.append(child)

            angle += angle_step
            if max_branches and len(children) >= max_branches:
                break

        return children