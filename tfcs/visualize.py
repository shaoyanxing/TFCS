"""
TFCS 可视化模块
===============
使用 matplotlib 实现势能场热力图、搜索链、最优路径、障碍物等可视化功能。
"""

import matplotlib.pyplot as plt
import numpy as np
from typing import List, Optional, Tuple
from tfcs.core import Chain, Point
from tfcs.field import FieldPotential, Obstacle


def plot_field(ax, field: FieldPotential, bounds: Tuple[float, float, float, float],
               resolution: int = 50):
    """绘制势能场热力图。

    Args:
        ax: matplotlib Axes 对象
        field: 叠加场域势能对象
        bounds: (xmin, xmax, ymin, ymax) 采样范围
        resolution: 每轴采样点数
    """
    xmin, xmax, ymin, ymax = bounds
    xs = np.linspace(xmin, xmax, resolution)
    ys = np.linspace(ymin, ymax, resolution)
    X, Y = np.meshgrid(xs, ys)
    Z = np.zeros_like(X)

    for i in range(resolution):
        for j in range(resolution):
            # 仅使用引力和斥力计算热力图（跳过抑制场以提升性能）
            p = (X[i, j], Y[i, j])
            val = (field.attraction.potential(p) * field.w1 +
                   field.repulsion.potential(p) * field.w2)
            Z[i, j] = val if val != float('inf') else np.nan

    im = ax.pcolormesh(X, Y, Z, cmap='viridis', shading='auto', alpha=0.7)
    plt.colorbar(im, ax=ax, label='Potential', shrink=0.8)


def plot_chains(ax, chains: List[Chain], alpha: float = 0.3, color: str = 'blue'):
    """绘制所有搜索链（半透明）。

    Args:
        ax: matplotlib Axes 对象
        chains: 搜索链列表
        alpha: 透明度
        color: 线条颜色
    """
    for chain in chains:
        pts = chain.get_path_points()
        if len(pts) < 2:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, color=color, alpha=alpha, linewidth=0.8)


def plot_best_path(ax, chain: Chain, color: str = 'red', linewidth: float = 2.5):
    """高亮绘制最优路径。

    Args:
        ax: matplotlib Axes 对象
        chain: 最优路径链
        color: 线条颜色
        linewidth: 线宽
    """
    pts = chain.get_path_points()
    if len(pts) < 2:
        return
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    ax.plot(xs, ys, color=color, linewidth=linewidth, zorder=5)


def plot_obstacles(ax, obstacles: List[Obstacle]):
    """绘制障碍物轮廓（填充灰色圆）。

    Args:
        ax: matplotlib Axes 对象
        obstacles: 障碍物列表
    """
    for obs in obstacles:
        circle = plt.Circle(obs.center, obs.radius, color='gray',
                            alpha=0.7, ec='black', linewidth=0.5)
        ax.add_patch(circle)


def plot_targets(ax, targets: List[Point]):
    """绘制目标点（红色星号）。

    Args:
        ax: matplotlib Axes 对象
        targets: 目标点列表
    """
    for t in targets:
        ax.plot(t[0], t[1], marker='*', color='red', markersize=15,
                markeredgecolor='darkred', markeredgewidth=0.5, zorder=6)


def plot_start(ax, start: Point):
    """绘制起点（绿色圆点）。

    Args:
        ax: matplotlib Axes 对象
        start: 起点坐标
    """
    ax.plot(start[0], start[1], marker='o', color='green', markersize=10,
            markeredgecolor='darkgreen', markeredgewidth=0.5, zorder=6)


def visualize(searcher, obstacles, targets, start, bounds, save_path=None):
    """综合可视化函数，在一个图中绘制所有元素。

    Args:
        searcher: TFCS 搜索器实例（已执行过 search()）
        obstacles: 障碍物列表
        targets: 目标点列表
        start: 起点坐标
        bounds: (xmin, xmax, ymin, ymax) 绘图范围
        save_path: 如果不为 None，保存图片到该路径
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    # 势能场热力图
    field = searcher.field
    plot_field(ax, field, bounds)

    # 障碍物
    plot_obstacles(ax, obstacles)

    # 所有搜索链
    plot_chains(ax, searcher.all_chains, alpha=0.3, color='blue')

    # 最优路径
    best_chain = searcher._best_chain
    if best_chain is not None:
        plot_best_path(ax, best_chain, color='red', linewidth=2.5)

    # 起点
    plot_start(ax, start)

    # 目标点
    plot_targets(ax, targets)

    # 图例（使用代理艺术家）
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='blue', alpha=0.3, linewidth=1.5, label='Search Chains'),
        Line2D([0], [0], color='red', linewidth=2.5, label='Best Path'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='green',
               markeredgecolor='darkgreen', markersize=8, label='Start'),
        Line2D([0], [0], marker='*', color='w', markerfacecolor='red',
               markeredgecolor='darkred', markersize=10, label='Target'),
        Line2D([0], [0], marker='s', color='w', markerfacecolor='gray',
               markeredgecolor='black', markersize=8, label='Obstacle'),
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=9)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('TFCS Path Search Visualization')
    ax.set_aspect('equal')
    ax.set_xlim(bounds[0], bounds[1])
    ax.set_ylim(bounds[2], bounds[3])

    plt.tight_layout()

    if save_path is not None:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')

    plt.show()