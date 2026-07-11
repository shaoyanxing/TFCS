import math
import pytest
from tfcs.core import DirectedSegment, Chain, ChainEndpoint, Point
from tfcs.field import AttractionField, RepulsionField, InhibitionField, FieldPotential, Obstacle
from tfcs.proliferate import Proliferator
from tfcs.searcher import TFCS, TFCSConfig


class TestDirectedSegment:
    def test_create_from_angle_and_length(self):
        """测试从起点+角度+长度正确计算终点"""
        seg = DirectedSegment((0, 0), 0, 10)  # 向右
        assert abs(seg.end[0] - 10) < 0.001
        assert abs(seg.end[1] - 0) < 0.001

        seg = DirectedSegment((0, 0), math.pi / 2, 10)  # 向上
        assert abs(seg.end[0] - 0) < 0.001
        assert abs(seg.end[1] - 10) < 0.001

    def test_angle_45_degree(self):
        """45度方向"""
        seg = DirectedSegment((0, 0), math.pi / 4, math.sqrt(2))
        assert abs(seg.end[0] - 1) < 0.001
        assert abs(seg.end[1] - 1) < 0.001


class TestChain:
    def test_append(self):
        """测试追加 segment 后 depth 和 total_cost 正确累加"""
        root = Chain()
        root.segments = [DirectedSegment((0, 0), 0, 0)]

        seg1 = DirectedSegment((0, 0), 0, 5)
        child1 = root.append(seg1)
        assert child1.depth == 1
        assert abs(child1.total_cost - 5) < 0.001

        seg2 = DirectedSegment(child1.endpoint, math.pi / 4, 5)
        child2 = child1.append(seg2)
        assert child2.depth == 2
        assert abs(child2.total_cost - 10) < 0.001

    def test_get_path_points(self):
        """测试路径点序列"""
        chain = Chain()
        chain.segments = [
            DirectedSegment((0, 0), 0, 5),
            DirectedSegment((5, 0), math.pi / 2, 5),
        ]
        chain.total_cost = 10
        chain.depth = 2
        points = chain.get_path_points()
        assert len(points) == 3
        assert points[0] == (0, 0)
        assert abs(points[2][0] - 5) < 0.001
        assert abs(points[2][1] - 5) < 0.001


class TestChainEndpoint:
    def test_heap_ordering(self):
        """测试优先级堆排序"""
        import heapq
        c1 = Chain()
        c1.segments = [DirectedSegment((0, 0), 0, 0)]
        c2 = Chain()
        c2.segments = [DirectedSegment((0, 0), 0, 0)]

        e1 = ChainEndpoint(c1, 10.0)
        e2 = ChainEndpoint(c2, 5.0)

        heap = []
        heapq.heappush(heap, e1)
        heapq.heappush(heap, e2)
        top = heapq.heappop(heap)
        assert top.priority == 10.0  # 高优先级先出


class TestAttractionField:
    def test_potential_distance(self):
        """引力场势能随距离增加"""
        field = AttractionField([(10, 0)])
        p1 = field.potential((0, 0))  # 距离10
        p2 = field.potential((5, 0))  # 距离5
        assert p2 < p1  # 越近势能越低

    def test_gradient_points_to_target(self):
        """梯度指向目标"""
        field = AttractionField([(10, 0)])
        gx, gy = field.gradient((0, 0))
        assert gx > 0  # 指向正x方向
        assert abs(gy) < 0.001

    def test_multi_target_nearest(self):
        """多目标取最近"""
        field = AttractionField([(10, 0), (100, 100)])
        gx, gy = field.gradient((0, 0))
        assert gx > 0
        assert abs(gy) < 0.001


class TestRepulsionField:
    def test_far_from_obstacle(self):
        """远离障碍物时势能趋近0"""
        field = RepulsionField([Obstacle((0, 0), 5)], influence_range=50)
        p = field.potential((100, 100))
        assert p < 0.01

    def test_near_boundary(self):
        """靠近障碍物边界时势能升高"""
        field = RepulsionField([Obstacle((0, 0), 5)], influence_range=50)
        p_near = field.potential((6, 0))  # 距离边界1
        p_far = field.potential((20, 0))  # 距离边界15
        assert p_near > p_far

    def test_gradient_pushes_away(self):
        """梯度指向远离障碍物"""
        field = RepulsionField([Obstacle((0, 0), 5)], influence_range=50)
        gx, gy = field.gradient((10, 0))  # 障碍物在左侧
        assert gx > 0  # 向右推


class TestFieldPotential:
    def test_compose(self):
        """叠加场合成"""
        att = AttractionField([(10, 0)])
        rep = RepulsionField([], influence_range=50)
        inh = InhibitionField(sigma=10)
        field = FieldPotential(att, rep, inh, w1=1.0, w2=10.0, w3=2.0)

        p = field.compose((0, 0))
        assert isinstance(p, float)
        assert p > 0

    def test_gradient_nonzero(self):
        """梯度非零"""
        att = AttractionField([(10, 0)])
        rep = RepulsionField([], influence_range=50)
        inh = InhibitionField(sigma=10)
        field = FieldPotential(att, rep, inh)

        gx, gy = field.gradient((0, 0))
        assert gx != 0 or gy != 0


class TestProliferator:
    def test_adaptive_small_gradient(self):
        """小梯度时角度步长大、链长度长"""
        prolif = Proliferator(angle_min=0.1, angle_max=1.0, length_min=2, length_max=20)
        angle_step = prolif._adaptive_angle_step(0.01)  # 很小的梯度
        length = prolif._adaptive_length(0.01)
        assert angle_step > 0.5  # 接近 angle_max
        assert length > 10  # 接近 length_max

    def test_adaptive_large_gradient(self):
        """大梯度时角度步长小、链长度短"""
        prolif = Proliferator(angle_min=0.1, angle_max=1.0, length_min=2, length_max=20)
        angle_step = prolif._adaptive_angle_step(10.0)  # 很大的梯度
        length = prolif._adaptive_length(10.0)
        assert angle_step < 0.3  # 接近 angle_min
        assert length < 5  # 接近 length_min

    def test_proliferate_children_count(self):
        """增殖产生合理数量的子链"""
        from tfcs.field import FieldPotential, AttractionField, RepulsionField, InhibitionField

        att = AttractionField([(10, 0)])
        rep = RepulsionField([], influence_range=50)
        inh = InhibitionField(sigma=10)
        field = FieldPotential(att, rep, inh)

        chain = Chain()
        chain.segments = [DirectedSegment((0, 0), 0, 0)]

        prolif = Proliferator(angle_min=math.radians(10), angle_max=math.radians(30))
        children = prolif.proliferate(chain, field)
        assert len(children) > 0
        assert all(isinstance(c, Chain) for c in children)


class TestTFCSSearcher:
    def test_simple_search(self):
        """简单场景下能找到目标"""
        config = TFCSConfig(max_iterations=2000, cooldown=30, target_radius=5, w3=0.0)
        searcher = TFCS((0, 0), [(50, 50)], [], config)
        result = searcher.search()
        assert result is not None
        # 终点应该接近目标
        import math
        d = math.hypot(result.endpoint[0] - 50, result.endpoint[1] - 50)
        assert d < 5  # 在目标半径内

    def test_search_with_obstacle(self):
        """有障碍物时能绕过"""
        config = TFCSConfig(
            max_iterations=3000, cooldown=50, target_radius=5,
            w2=10.0, w3=0.0,
        )
        obstacle = Obstacle((25, 25), 10)
        searcher = TFCS((0, 0), [(50, 50)], [obstacle], config)
        result = searcher.search()
        assert result is not None
        # 路径不应穿过障碍物
        for seg in result.segments:
            for obs in [obstacle]:
                d = math.hypot(seg.end[0] - obs.center[0], seg.end[1] - obs.center[1])
                assert d > obs.radius