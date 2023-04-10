# HUAWEIRobot

- **概要**：机器人在各大领域显现出巨大的商业价值，本次赛题抽象自华为5G应用场景，在一个地图中对每个机器人做路径规划，进行买卖物品操作，使得最后的总收益最大。
- **成果：**2023年华为软件精英挑战赛上合赛区 **初赛正式赛 rk7** 、**复赛正式赛 rk11**

## 项目结构

```
_________
 |____Demo
 |____Img
 |____SDK
 |____test
 	     |___PID.py
 	     |___calcation.py
 	     |___config.py
 	     |___contraception.py
 	     |___halfoptimizie.py
 	     |___main.py
 	     |___policy.py
 	     |___robot.py
 	     |___wall.py
 	     |___workbench.py
 |____log.txt
```

- Demo、Img、SDK为华为封装好的接口文件；log.txt为调试中间信息。

- PID.py为机器人移动控制函数，calcation.py为项目常用函数集合，config.py为项目所用超参数，main.py为项目主函数，policy.py为过往的机器人任务分配函数集合，wall.py为墙类，workbench.py为工作台类，robot.py为机器人类，contraception.py和halfoptimize.py为实现orca的防碰撞函数。
- 整个项目仅需numpy依赖。

## 项目流程

- 通过指令 `./Robot_gui -m maps "python test/main.py"`与判题器交互来运行代码。

- 在首次读入地图之后，有5s的初始化时间；判题器每帧会返回当前帧信息，即所有工作台的位置与工作状态和所有机器人的位置与工作台状态，每帧处理并返回指令的时间0.02s；其余具体信息可以查看项目报告书。

- ```
  算法流程：
  	初始化地图信息
  		保存所有的工作台信息和机器人信息
  		对所有机器人做BFS求得其到所有工作台的路径
  		对所有工作台做BFS求得工作台到工作台之间的路径
  	针对每一帧
  		对空闲机器人和可接的任务做分配
  			对没有涉及过的路径做路径优化
  		调整机器人所处状态
  			根据PID控制机器人移动
  			根据orca防碰撞
  ```

## 项目核心

整个项目共可分为三大部分：初始化路径规划（复赛添加）、物品分配策略、实际移动策略。下面开始逐个介绍三个部分的创新点。

### 路径规划

路径规划部分使用两个算法来控制，分别是规划算法和优化算法。

- **规划算法**：机器人在计算机存储时状态是离散的，但在实际移动时状态是连续的。为了防止在实际移动中由离散到连续中所忽略的部分状态，提出一种“走半步”的想法，在地图上每次不是移动1的单位距离，而是移动0.5的单位距离。

  具体实现使用了BFS和ASTAR两种算法。

- **优化算法**：根据规划算法得到的路径是一连串的点，其中很多点都是没有意义的。为了只保留关键点，使用了基于二分算法的路径优化算法。对于答案数组，选定`st`和`ed`，如果两者不能优化那么选择两者的`mid`为另一个关键点；之后判断优化`[st, mid], [mid, ed]`这两段能否优化，如果`[st, mid]`可以优化，那么需要优化的路径仅剩`[mid, ed]`，如果`[st, mid]`不能优化，那么需要优化的路径为`[st, mid2], [mid2, mid], [mid, ed]`，每一段都同理。

  针对优化出来的路径，还需要遍历一遍，只保留每条直线的起点和终点。

### 分配策略

分配策略部分有两种不同的策略，分别是以机器人为视角的自底向上的贪心和以工作台为视角的自顶向下的分配。

- **自底向上**：考虑每一个机器人的任务分配，根据价值及潜在价值选择单位时间内价值最高的任务。同时为了避免陷入局部最优，通过限制基础任务之间的任务次数来保证最高级任务能被完成。

- **自顶向下**：从工作台的角度来考虑离工作台最近的机器人，需要先看最高级任务所需要的物品，再看次高级任务所需要的物品，再选择同一层次物品中价值最高且单位时间内价值最高的任务。该策略保证了最高级任务一定是最优先完成的。

  具体实现：使用了多级队列。

- **合并策略**：在任务刚开始时，使用自顶向下的策略来保证有一个或多个最高级任务肯定可以被生产，之后切换回自底向上的任务来保证部分时间最优。

### 移动策略

这部分主要由两个算法组成：PID算法和ORCA算法。整个项目通过该模块来控制机器人的移动。

- **PID算法**：根据实际输出值与目标输出值之间的误差来调整控制器的控制量，以使得输出值能够稳定地达到目标值。基于三个参数：比例增益（P）、积分时间（I）和微分时间（D）。其中，比例增益通过将误差乘以一个常数来产生控制量；积分时间通过对误差进行积分，以减小稳态误差并提高系统鲁棒性；微分时间则通过对误差变化率进行测量，用于抑制系统的超调和振荡。
- **ORCA算法**：通过对智能体运动的速度和方向进行限制来避免碰撞。每个智能体都会计算出一个“优先速度”，该速度既可以保证其朝着目标点移动，又可以避免与周围其他智能体发生碰撞。为了实现这一目标，ORCA算法引入了一个“半平面交”概念，即根据当前智能体的速度和其它所有智能体的速度，计算出限制其运动的半平面区域，然后在该区域内选择一个最优速度作为其优先速度。

## 改进方向

- **窄道碰撞问题**：在狭窄通道不能容忍两个机器人并行，且无法进行防碰撞。考虑实时分级加锁的方法，来保证分配到高等级任务的机器人会优先完成任务，而低等级任务的机器人会在和高等级任务机器人重复路径前停住脚步。
- **初始化超时问题**：为了避免初始化超时，限制全局BFS的工作台数量，只对工作台种类集合`[4, 5, 6, 7, 9]`或`[1, 2, 3, 4, 5, 6, (7 or 8)]`的工作台种类集合进行初始化。
- **针对每张图调参**
