# TFCS (Tendency Field Chain Search) 规范

## Why
现有搜索算法（A*、RRT、PSO 等）以"点"为基本搜索单元，缺乏对"方向连续性"和"场域势能"的结构化利用。TFCS 提出以有向线段（锁链）为基本单元，以叠加场域势能为核心驱动力，通过自适应旋转增殖实现高效探索，填补"线段级搜索范式"的空白。

## What Changes
- 新增 TFCS 核心算法库，包含所有核心数据结构和搜索逻辑
- 新增叠加场域势能计算模块（引力场 + 斥力场 + 抑制场）
- 新增自适应旋转增殖引擎（角度步长 + 链长度的联动自适应）
- 新增优先级队列搜索调度器（混合终止策略）
- 新增可视化演示模块

## Impact
- Affected specs: 无（全新项目）
- Affected code: 全新目录 `tfcs/`

---

## ADDED Requirements

### Requirement: 核心数据结构
系统 SHALL 提供 DirectedSegment（有向线段）、Chain（锁链）、FieldPotential（叠加场）、PriorityQueue 四个核心数据结构。

#### Scenario: 创建有向线段
- **WHEN** 给定起点坐标、方向角 angle 和长度 length
- **THEN** 系统自动计算终点坐标，创建 DirectedSegment 实例

#### Scenario: 构建锁链
- **WHEN** 从父链继承并追加新的 DirectedSegment
- **THEN** 新 Chain 实例包含完整路径、深度 depth 递增、总代价 total_cost 累加

#### Scenario: 叠加场势能计算
- **WHEN** 给定目标点列表、障碍物列表、已探索链集合
- **THEN** compose(point) 返回该点的综合势能值，gradient(point) 返回势能梯度向量

---

### Requirement: 叠加场域势能
系统 SHALL 实现三种场域的叠加计算：引力场（目标吸引）、斥力场（障碍排斥）、抑制场（已探索区域抑制）。

#### Scenario: 引力场计算
- **WHEN** 空间中存在一个或多个目标点
- **THEN** 引力场在目标点处势能最大（或最小），随距离衰减，gradient 指向最近目标方向

#### Scenario: 斥力场计算
- **WHEN** 空间中存在障碍物
- **THEN** 斥力场在障碍物边界处势能急剧升高，超出影响范围后衰减为零

#### Scenario: 抑制场计算
- **WHEN** 某区域已被搜索链覆盖
- **THEN** 抑制场在该区域产生势能"凹槽"，降低重复探索的优先级

#### Scenario: 叠加场合成
- **WHEN** 三种场同时存在
- **THEN** compose() 返回加权叠加值：`w1 * attraction + w2 * repulsion + w3 * inhibition`

---

### Requirement: 自适应旋转增殖
系统 SHALL 根据局部势能梯度自适应调整旋转角度步长和锁链步长，实现"粗探-细搜"两级策略。

#### Scenario: 梯度大区域细采样
- **WHEN** 当前链端点处势能梯度幅值大于阈值
- **THEN** 旋转角度步长缩小（如 5°~15°），链长度缩短，实现精细搜索

#### Scenario: 梯度小区域粗探索
- **WHEN** 当前链端点处势能梯度幅值小于阈值
- **THEN** 旋转角度步长增大（如 30°~60°），链长度增长，实现快速跨越

#### Scenario: 角度与步长联动
- **WHEN** 计算自适应参数
- **THEN** 角度步长和链长度由同一梯度幅值函数决定，保证两者同步缩放

#### Scenario: 角度采样范围
- **WHEN** 进行旋转增殖
- **THEN** 采样范围限定在梯度方向 ±90° 的扇形区域内，避免反向探索

---

### Requirement: 优先级队列搜索调度
系统 SHALL 使用优先级队列管理活跃链端点，按势能-代价比排序，优先扩展最有希望的方向。

#### Scenario: 优先级计算
- **WHEN** 新链端点入队
- **THEN** 优先级 = `α * potential_gradient_magnitude + β * (1 / total_cost)`，其中 α、β 为可调权重

#### Scenario: 搜索主循环
- **WHEN** 队列非空且未触发终止条件
- **THEN** 弹出优先级最高的链端点，执行自适应旋转增殖，子链入队

#### Scenario: 首达目标
- **WHEN** 任意链端点首次进入目标区域（距离 < 阈值）
- **THEN** 记录该路径为当前最优，进入冷却期而非立即终止

#### Scenario: 冷却期择优
- **WHEN** 冷却期内发现更优路径（更低总代价）
- **THEN** 更新当前最优路径，冷却期计数器重置

#### Scenario: 搜索终止
- **WHEN** 冷却期耗尽（连续 N 次迭代无更优解）或队列为空
- **THEN** 返回最优路径，搜索结束

---

### Requirement: 算法可配置性
系统 SHALL 支持通过参数配置调整算法行为，适应不同应用场景。

#### Scenario: 参数配置
- **WHEN** 初始化 TFCS 搜索器
- **THEN** 可配置参数包括：场权重 (w1, w2, w3)、优先级权重 (α, β)、自适应阈值、冷却期长度、目标区域半径、最大搜索深度

---

### Requirement: 可视化演示
系统 SHALL 提供可视化模块，展示搜索过程中的场域势能分布、锁链增殖过程、最终路径。

#### Scenario: 2D 可视化
- **WHEN** 运行演示脚本
- **THEN** 渲染势能场热力图、搜索链树状图、最优路径高亮、障碍物轮廓