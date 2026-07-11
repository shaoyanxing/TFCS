"""TFCS 算法演示脚本"""

import matplotlib.pyplot as plt
import math

from tfcs import TFCS, TFCSConfig
from tfcs.field import Obstacle
from tfcs.visualize import visualize


def demo_simple():
    """简单场景：无障碍物，单一目标"""
    start = (0, 0)
    targets = [(50, 50)]
    obstacles = []
    config = TFCSConfig(max_iterations=2000, cooldown=30)
    searcher = TFCS(start, targets, obstacles, config)
    result = searcher.search()
    print(f"Simple: iterations={searcher.iterations}, cost={result.total_cost:.2f}")
    visualize(searcher, obstacles, targets, start, bounds=(-5, 55, -5, 55))
    return result


def demo_with_obstacles():
    """障碍物场景：多个圆形障碍物"""
    start = (0, 0)
    targets = [(60, 60)]
    obstacles = [
        Obstacle((20, 20), 8),
        Obstacle((40, 30), 10),
        Obstacle((30, 50), 7),
    ]
    config = TFCSConfig(max_iterations=3000, cooldown=50)
    searcher = TFCS(start, targets, obstacles, config)
    result = searcher.search()
    print(f"Obstacles: iterations={searcher.iterations}, cost={result.total_cost:.2f}")
    visualize(searcher, obstacles, targets, start, bounds=(-5, 65, -5, 65))
    return result


def demo_maze():
    """迷宫场景：密集障碍物形成通道"""
    start = (0, 5)
    targets = [(55, 55)]
    obstacles = [
        # 围墙
        Obstacle((10, 15), 5), Obstacle((20, 15), 5), Obstacle((30, 15), 5),
        Obstacle((40, 15), 5), Obstacle((50, 15), 5),
        Obstacle((10, 35), 5), Obstacle((20, 35), 5), Obstacle((30, 35), 5),
        Obstacle((40, 35), 5),
        Obstacle((20, 55), 5), Obstacle((30, 55), 5), Obstacle((40, 55), 5),
    ]
    config = TFCSConfig(max_iterations=5000, cooldown=80,
                        angle_min=3, angle_max=60, length_min=1.5, length_max=15)
    searcher = TFCS(start, targets, obstacles, config)
    result = searcher.search()
    print(f"Maze: iterations={searcher.iterations}, cost={result.total_cost:.2f}")
    visualize(searcher, obstacles, targets, start, bounds=(-5, 65, -5, 65))
    return result


if __name__ == "__main__":
    print("=== TFCS Demo ===")
    demo_simple()
    demo_with_obstacles()
    demo_maze()
    print("=== Done ===")
    plt.show()