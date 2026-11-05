# diff_RDW

一个面向重定向行走研究的最小实验框架。当前包含统一控制器接口、二维仿真器、S2C / S2O / APF 三个 baseline、对比实验、CSV 指标和轨迹图。

## Baseline

| 名称 | 文件 | 引导目标 |
|---|---|---|
| `s2c` | `rdw/controllers/s2c.py` | 房间中心；中心几乎在正后方时改用侧向临时目标 |
| `s2o` | `rdw/controllers/s2o.py` | 绕中心的圆轨道（圆外取切点，圆内取 ±60° 处的点） |
| `apf` | `rdw/controllers/apf.py` | 边界线段排斥力合力的方向 |

三者共用 `rdw/controllers/_steering.py` 中的转向规则（参考 Redirected Walking
Toolkit 的 SteerToRedirector）：行走时用曲率增益（默认半径 7.5 m），转身时用旋转增益，
两者取注入旋转更大的一个；朝向已接近目标或离目标很近时减弱，并做时间平滑。
所有控制器从 `Observation.limits` 读取与仿真器相同的增益上限。

## 环境

项目使用 Conda 管理 Python 环境，所有 Python 包统一维护在 `requirements.txt`。

```powershell
conda env create -f environment.yml
conda activate rdw
```

已有环境需要同步依赖时：

```powershell
python -m pip install -r requirements.txt
```

## 运行

```powershell
python -m pytest
python -m rdw.experiments --output results
```

实验输出：

- `results/summary.csv`：各场景和控制器的汇总指标
- `results/trajectories/*.csv`：逐帧轨迹与控制量
- `results/comparison.png`：物理空间轨迹对比

### 多 seed 随机路径 benchmark

两条固定路径的结果噪声很大，对比控制器时建议用随机路径：

```powershell
python -m rdw.experiments --trials 30 --duration 120 --output results
```

在 6×6、4×8、10×10 三个房间里，每个控制器都在同一组随机虚拟路径上运行
（seed 为 `--seed` 起的连续整数，默认 0；路径由 2–8 m 的直线段和随机转角组成）。

- `results/random_runs.csv`：每个房间 × 控制器 × seed 一行
- `results/random_aggregate.csv`：每个房间 × 控制器的均值、标准差、标准误、
  每 100 m 虚拟距离的 reset 次数，以及相对 `none` 的 reset 降幅（同 seed 配对）
- `results/random_examples.png`：每个房间第一个 seed 的轨迹

## 结构

```text
rdw/
  controllers/       # 每个算法一个文件，自动发现
  models.py          # 不可变输入、输出和场景模型
  simulation.py      # 与算法无关的仿真器
  scenarios.py       # 标准实验场景
  experiments.py     # 对比实验、CSV 和绘图
tests/               # 接口、仿真和实验测试
```

每个 RDW 算法使用单独文件。新增算法时，只需在 `rdw/controllers/`
中加入一个模块，无需修改实验入口或中央注册表：

```python
class MyRedirector(Redirector):
    @property
    def name(self) -> str: ...
    def reset(self) -> None: ...
    def compute(self, observation: Observation) -> RedirectionCommand: ...


CONTROLLER_FACTORY = MyRedirector
```

算法类必须继承 `Redirector`。框架会调用每个模块中必须存在的
`CONTROLLER_FACTORY`，并使用控制器的 `name` 作为命令行名称。
缺少该变量、构造类型错误或名称重复时会直接报错，不会跳过或使用默认算法。
`Simulator` 不依赖具体算法；后续 S2O、APF、Warp-MPC 和可微控制器
都通过同一输入输出接口接入。
