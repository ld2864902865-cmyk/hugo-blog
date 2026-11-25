+++
title = '08 | LQR 状态反馈与状态观测器'
date = 2026-11-25T14:01:07+08:00
draft = false
tags = ['自动控制', '学习笔记']
+++

> **本章核心目标**：掌握现代控制里最实用的控制器设计范式——**全状态反馈 + LQR**，知道怎么调 $Q/R$ 权重，理解**观测器（以及卡尔曼滤波的入口）**，并明确二连杆控制的最终落地架构。

---

## 1. 能控性与能观性：动手设计前先问两个问题

拿到线性化后的 $(A, B, C)$，不要急着算控制器，先用两行 MATLAB 检查系统「有没有救」：

| 性质 | 物理直觉 | MATLAB 判别 | 工程含义 |
| --- | --- | --- | --- |
| **能控性**（Controllability） | 靠你的执行器 $\mathbf{u}$，能不能在有限时间内把所有状态 $\mathbf{x}$ 搬到任意想去的地方？ | `rank(ctrb(A, B)) == size(A, 1)` | 满秩就能任意配置闭环极点；不满秩则存在「电机管不到的死角模态」 |
| **能观性**（Observability） | 只看传感器输出 $\mathbf{y}$，能不能反推出系统内部所有未知状态 $\mathbf{x}$？ | `rank(obsv(A, C)) == size(A, 1)` | 满秩就能设计状态观测器（比如只装角度编码器，就能推算出角速度） |

> **工程提醒**：能控/能观是「是或否」的二值判据；在现实中即使勉强满秩，如果某些模态非常弱（条件数极大），也意味着需要消耗极大能量才能控制。

---

## 2. 状态反馈与极点配置：把闭环极点「钉」在任意位置

全状态反馈控制律非常简洁：把所有状态偏差乘以一个增益矩阵 $K$ 反馈回来：

$$\mathbf{u} = \mathbf{u}_e - K(\mathbf{x} - \mathbf{x}_e) = \mathbf{u}_e - K\Delta\mathbf{x}$$

代入状态方程 $\Delta\dot{\mathbf{x}} = A\Delta\mathbf{x} + B\Delta\mathbf{u}$，闭环方程变成：

$$\Delta\dot{\mathbf{x}} = (A - BK)\Delta\mathbf{x}$$

- **闭环极点**就是矩阵 $(A - BK)$ 的特征值！
- **极点配置法（`place`）**：只要系统能控，你直接指定期望闭环极点 `p = [...]`，`K = place(A, B, p)` 就能自动算出 $K$。
- **极点配置的痛点**：对于 2 阶单连杆，你很容易拍出 2 个极点；但对于 4 阶二连杆（或 6 阶轮腿），**4 个、6 个极点该放在哪？放太左电机力矩会爆炸，放太右响应太慢**——这就引出了工程上最强大的工具：**LQR**！

---

## 3. LQR（线性二次型调节器）：用「打分思维」设计最优控制器

LQR 不让你去猜极点位置，而是让你定义一个**代价函数（Cost Function）**，让数学自动帮你找到总代价最小的增益矩阵 $K$：

$$J = \int_{0}^{\infty} \Big(\Delta\mathbf{x}^T Q \Delta\mathbf{x} + \Delta\mathbf{u}^T R \Delta\mathbf{u}\Big)\,dt$$

- **第一项 $\Delta\mathbf{x}^T Q \Delta\mathbf{x}$**：衡量**状态误差的惩罚**（偏离目标越久、越远，扣分越多）。
- **第二项 $\Delta\mathbf{u}^T R \Delta\mathbf{u}$**：衡量**控制能量的惩罚**（电机输出力矩越大，扣分越多）。
- **LQR 的绝妙之处**：在平衡点线性化模型上，只要系统能控且 $Q \ge 0, R > 0$，LQR 求出的控制器**天生保证闭环稳定**，且天然拥有极佳的鲁棒性（至少 **$60^\circ$ 相位裕度** 和 **无穷大上幅值裕度**！）。

### 3.1 $Q$ 和 $R$ 权重矩阵怎么调？（实战必背：Bryson 法则）

$Q$ 和 $R$ 通常取**对角矩阵**：
- 对于二连杆 $\mathbf{x} = [\theta_1, \theta_2, \dot{\theta}_1, \dot{\theta}_2]^T$，$\mathbf{u} = [\tau_1, \tau_2]^T$：
  $$Q = \text{diag}(q_{\theta_1},\ q_{\theta_2},\ q_{\dot{\theta}_1},\ q_{\dot{\theta}_2}), \qquad R = \text{diag}(r_1,\ r_2)$$

**第一步起步（Bryson 归一化法则）**：不同状态的物理量纲不同（角度是 rad，力矩是 N·m），先按各自允许的最大值倒数平方做初始化：

$$q_i = \frac{1}{(\text{状态 } x_i \text{ 允许的最大偏差})^2}, \qquad r_j = \frac{1}{(\text{力矩 } u_j \text{ 允许的最大输出})^2}$$

**第二步微调（直觉口诀）**：
- **想让某个状态收敛更快、盯得更死**（比如关节 1 角度） → **调大对应的 $q_i$**。
- **发现某个关节震荡、超调大** → **调大对应的角速度权重 $q_{\dot{\theta}}$**（相当于加阻尼！）。
- **发现电机力矩超限/抖动太狠** → **调大对应的 $r_j$**（告诉控制器「省着点使劲」）。
- 注意：$Q$ 和 $R$ 只有**相对比例**有意义，把 $Q$ 和 $R$ 同时乘以 10，算出来的 $K$ 完全不变。

---

## 4. 状态观测器与分离原理：测不到全状态怎么办？

LQR 需要用到全状态 $\mathbf{x} = [\theta_1, \theta_2, \dot{\theta}_1, \dot{\theta}_2]^T$，但真实硬件往往只有编码器测角度 $\mathbf{y} = [\theta_1, \theta_2]^T$，直接对角度差分求速度又会有极大噪声。

**龙伯格观测器（Luenberger Observer）的核心思想**：在控制器内部并行跑一个系统的「虚拟数字孪生模型」，并用「真实传感器测量值 $\mathbf{y}$ 与模型预测值 $\hat{\mathbf{y}}$ 的偏差」不断修正它：

$$\dot{\hat{\mathbf{x}}} = \underbrace{A\hat{\mathbf{x}} + B\mathbf{u}}_{\text{用模型往前推演}} + \underbrace{L(\mathbf{y} - C\hat{\mathbf{x}})}_{\text{用传感器误差纠偏}}$$

- **分离原理（Separation Principle）**：在线性系统里，你可以**完全独立地**设计状态反馈增益 $K$（用 `lqr`）和观测器增益 $L$（用 `place(A', C', p_obs)'`），合在一起依然保证整体稳定！通常把观测器极点放得比控制器极点**左移 $3 \sim 5$ 倍**（让估计值比系统动作追得更快）。
- **自然衔接卡尔曼滤波（Kalman Filter）**：如果系统既有过程扰动噪声，又有传感器测量噪声，把观测器增益 $L$ 按「最小化估计方差」最优求解出来，**它就是大名鼎鼎的卡尔曼滤波器（LQE）**！而 **LQR + 卡尔曼滤波 = LQG**。

---

## 5. 二连杆的终极控制架构（从 LQR 到非线性进阶）

在 Simulink 里控制非线性二连杆时，控制器与对象的闭环架构如下：

```mermaid
flowchart LR
    Ref["目标平衡姿态 x_e<br/>与平衡重力矩 u_e"] --> SumX(("- / +"))
    Ref --> SumU(("+ / -"))
    SumX -->|"状态偏差 dx = x - x_e"| GainK["LQR 增益矩阵 K<br/>(2x4 矩阵)"]
    GainK -->|"反馈修正 du = -K*dx"| SumU
    SumU -->|"总力矩 tau = u_e - K*dx"| Sat["力矩限幅<br/>Saturation"]
    Sat --> Plant["非线性二连杆动力学<br/>M(q)q'' + C(q,q')q' + G(q) = tau"]
    Plant -->|"全状态 x = [q; q']"| Scope["Scope / 动画可视化"]
    Plant --> SumX
```

> **后续进阶融合预告（等你跑通 LQR 后自然扩展）**：
> 1. **LQR 擅长「局部稳态与抗扰」**：在平衡点附近（哪怕是**倒立双摆**这种开环极度不稳定的姿态！）表现极佳，但离平衡点太远时非线性误差会变大。
> 2. **大范围轨迹跟踪怎么搞？** → 引入机器人学里的 **计算力矩法（Computed Torque Control, CTC）**：直接用非线性模型把 $C(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}} + G(\mathbf{q})$ 精确抵消，并乘以 $M(\mathbf{q})$ 解耦成两个完全独立的线性双积分器，再套 PD/PID！
> 3. **带力矩硬约束的多步预测控制** → 后续学到 **MPC（模型预测控制）** 时，你会发现它本质上就是「每一步在线滚动求解带约束的有限时域 LQR」！

---

## 6. MATLAB 实操：一行 `lqr` 稳住倒立单连杆（二连杆完全同款模板）

```matlab
%% 1. 承接第 07 章：倒立点 (theta_e = pi) 线性化模型
J = 0.05; b = 0.1; m = 1.0; l = 0.25; g = 9.81;
A = [0,       1;
     m*g*l/J, -b/J];
B = [0; 1/J];

% 检查能控性
assert(rank(ctrb(A, B)) == size(A, 1), '系统不可控！');

%% 2. 用 Bryson 法则初始化 Q 和 R
max_theta  = 0.2;   % 允许最大角度偏差 0.2 rad (~11.5°)
max_dtheta = 1.0;   % 允许最大角速度 1.0 rad/s
max_u      = 5.0;   % 允许最大力矩 5 N*m

Q = diag([1/max_theta^2, 1/max_dtheta^2]);
R = 1/max_u^2;

%% 3. 一行代码求最优 LQR 反馈矩阵 K
[K, ~, poles_cl] = lqr(A, B, Q, R);
disp('LQR 增益矩阵 K ='); disp(K);
disp('闭环极点（应全在左半平面）：'); disp(poles_cl);

%% 4. 用真实非线性 ODE 验证 LQR 能否稳住偏离 20° 的倒立单连杆！
theta_e = pi;
x0 = [pi - deg2rad(20); 0];  % 初始歪了 20°

ode_fun = @(t, x) [
    x(2);
    (max(-max_u, min(max_u, -K*(x - [theta_e; 0]))) - b*x(2) - m*g*l*sin(x(1))) / J
];

[t, x] = ode45(ode_fun, [0 3], x0);

figure;
subplot(2,1,1); plot(t, rad2deg(x(:,1)), 'b-', 'LineWidth', 1.5);
yline(180, 'r--'); grid on; ylabel('角度 (deg)'); title('LQR 稳住非线性倒立单连杆');
subplot(2,1,2); plot(t, x(:,2), 'm-', 'LineWidth', 1.5);
grid on; xlabel('时间 (s)'); ylabel('角速度 (rad/s)');
```
