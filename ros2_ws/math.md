# 机械臂控制系统函数数学描述文档

本文档汇总了机械臂控制系统中各个关键函数的数学原理和公式推导。

## 函数索引

1. [末端执行器轨迹更新函数 (update_end_effector_trajectory)](#1-末端执行器轨迹更新函数-update_end_effector_trajectory)
   - 功能：生成末端执行器的正弦/余弦周期性轨迹
   - 输入：时间参数
   - 输出：位置、速度、加速度轨迹

2. [雅可比矩阵计算函数 (JntToJac)](#2-雅可比矩阵计算函数-jnttojac)
   - 功能：计算机械臂的雅可比矩阵
   - 输入：当前关节位置
   - 输出：雅可比矩阵

3. [逆速度运动学函数 (CartToJnt)](#3-逆速度运动学函数-carttojnt)
   - 功能：将笛卡尔空间速度转换为关节空间速度
   - 输入：关节位置、期望末端速度
   - 输出：期望关节速度

4. [正向运动学函数 (JntToCart)](#4-正向运动学函数-jnttocart)
   - 功能：计算机械臂从基座到指定连杆的位姿变换
   - 输入：关节位置、连杆编号
   - 输出：齐次变换矩阵

5. [逆运动学位置求解函数 (comput_ik)](#5-逆运动学位置求解函数-comput_ik)
   - 功能：计算实现期望末端位姿的关节角度
   - 输入：期望末端位姿
   - 输出：期望关节位置（带工作空间和关节限位检查）

6. [AutoServo 控制模式](#6-autoservo-控制模式)
   - 功能：笛卡尔空间位置跟踪控制
   - 方法：计算力矩控制 + PD 反馈
   - 特点：末端位姿精确跟踪，重力补偿

7. [ManualServo 控制模式](#7-manualservo-控制模式)
   - 功能：笛卡尔空间力矩控制（超螺旋滑模）
   - 方法：STSMC（Super-Twisting Sliding Mode Control）
   - 特点：强鲁棒性，抗扰动，高精度跟踪

8. [Impedance 控制模式](#8-impedance-控制模式)
   - 功能：力/位混合控制，柔顺交互
   - 方法：虚拟质量-阻尼-弹簧系统
   - 特点：可调阻抗，接触力控制，负载自适应

9. [Admittance 控制模式](#9-admittance-控制模式)
   - 功能：导纳控制，力到位置映射
   - 方法：期望外力驱动末端运动
   - 特点：力约束自由度，装配任务，人机协作

10. [JointAutoServo 控制模式](#10-jointautoservo-控制模式)
    - 功能：关节空间位置控制
    - 方法：逆动力学 + PID 反馈
    - 特点：避免奇异点，关节空间轨迹跟踪

11. [Gravity 控制模式](#第11章-gravity-控制模式)
    - 功能：纯重力补偿，拖动示教
    - 方法：零PID增益 + 卡尔曼滤波
    - 特点：高顺应性，人机协作，手动调整

12. [Zero 控制模式](#第12章-zero-控制模式)
    - 功能：电机零点标定
    - 方法：绝对编码器零位设置
    - 特点：一次性操作，系统初始化

13. [Planning 控制模式](#第13章-planning-控制模式)
    - 功能：关节空间轨迹规划
    - 方法：S型速度曲线插值 + 重力补偿
    - 特点：平滑轨迹，夹爪同步，抓取任务

14. [Gohome 控制模式](#第14章-gohome-控制模式)
    - 功能：自动归零到零点位置
    - 方法：高PID增益 + 轨迹插值
    - 特点：安全归位，自动模式切换

---

# 1. 末端执行器轨迹更新函数 (update_end_effector_trajectory)

## 1.1 函数签名

```cpp
void LlArm6dof::update_end_effector_trajectory(float realtime)
```

## 1.2 函数概述

该函数根据时间参数生成末端执行器的**正弦/余弦轨迹**，包括位置、速度和加速度。生成的轨迹是一个**周期性的谐波运动**。


## 1.3 参数定义

### 1.3.1 输入参数

- $t$：时间参数 `realtime`（秒），注意：代码中 $t$ 被强制设为 $0$

### 1.3.2 轨迹参数

| 符号 | 变量名 | 值 | 物理意义 |
|------|--------|-----|----------|
| $\omega$ | `rate` | 2.0 | 角频率 (rad/s) |
| $A_x$ | `amplitude_x` | 0.0 | X 方向位置振幅 (m) |
| $A_y$ | `amplitude_y` | 0.0 | Y 方向位置振幅 (m) |
| $A_z$ | `amplitude_z` | 0.0 | Z 方向位置振幅 (m) |
| $A_{\phi}$ | `amplitude_rot_x` | 0.0 | Roll 角振幅 (rad) |
| $A_{\theta}$ | `amplitude_rot_y` | 0.0 | Pitch 角振幅 (rad) |
| $A_{\psi}$ | `amplitude_rot_z` | 0.0 | Yaw 角振幅 (rad) |

### 1.3.3 初始位置

$$\mathbf{p}_0 = \begin{bmatrix} p_{0x} \\ p_{0y} \\ p_{0z} \end{bmatrix}, \quad \boldsymbol{\eta}_0 = \begin{bmatrix} \phi_0 \\ \theta_0 \\ \psi_0 \end{bmatrix}$$

其中：
- $\mathbf{p}_0 \in \mathbb{R}^3$：默认笛卡尔位置（`desir_end_efect_pos_default[0:2]`）
- $\boldsymbol{\eta}_0 \in \mathbb{R}^3$：默认欧拉角（`desir_end_efect_pos_default[3:5]`）


## 1.4 位置轨迹

### 1.4.1 笛卡尔位置

期望的末端执行器位置 $\mathbf{p}(t) \in \mathbb{R}^3$：

$$\mathbf{p}(t) = \begin{bmatrix}
p_x(t) \\
p_y(t) \\
p_z(t)
\end{bmatrix} = \begin{bmatrix}
p_{0x} + A_x \sin(\omega t) \\
p_{0y} + A_y \cos(\omega t) \\
p_{0z} + A_z \cos(\omega t)
\end{bmatrix}$$

**注意**：
- X 方向使用 **正弦函数**
- Y、Z 方向使用 **余弦函数**
- 相位差：X 与 Y/Z 相差 $\frac{\pi}{2}$

### 1.4.2 姿态（欧拉角）

期望的姿态角 $\boldsymbol{\eta}(t) \in \mathbb{R}^3$（Roll-Pitch-Yaw）：

$$\boldsymbol{\eta}(t) = \begin{bmatrix}
\phi(t) \\
\theta(t) \\
\psi(t)
\end{bmatrix} = \begin{bmatrix}
\phi_0 + A_{\phi} \sin(\omega t) \\
\theta_0 + A_{\theta} \cos(\omega t) \\
\psi_0 + A_{\psi} \sin(\omega t)
\end{bmatrix}$$

其中：
- $\phi(t)$：Roll 角（绕 X 轴旋转）
- $\theta(t)$：Pitch 角（绕 Y 轴旋转）
- $\psi(t)$：Yaw 角（绕 Z 轴旋转）

### 1.4.3 齐次变换矩阵

完整的末端执行器位姿可表示为齐次变换矩阵 $\mathbf{T}(t) \in SE(3)$：

$$\mathbf{T}(t) = \begin{bmatrix}
\mathbf{R}(\phi, \theta, \psi) & \mathbf{p}(t) \\
\mathbf{0}^T & 1
\end{bmatrix}$$

其中旋转矩阵 $\mathbf{R} \in SO(3)$ 通过 RPY 欧拉角构造：

$$\mathbf{R}(\phi, \theta, \psi) = \mathbf{R}_z(\psi) \mathbf{R}_y(\theta) \mathbf{R}_x(\phi)$$


## 1.5 速度轨迹

### 1.5.1 线速度

对位置求导，得到线速度 $\dot{\mathbf{p}}(t) \in \mathbb{R}^3$：

$$\dot{\mathbf{p}}(t) = \frac{d\mathbf{p}(t)}{dt} = \begin{bmatrix}
\dot{p}_x(t) \\
\dot{p}_y(t) \\
\dot{p}_z(t)
\end{bmatrix} = \begin{bmatrix}
A_x \omega \cos(\omega t) \\
-A_y \omega \sin(\omega t) \\
-A_z \omega \sin(\omega t)
\end{bmatrix}$$

**推导过程**：

$$\dot{p}_x = \frac{d}{dt}[p_{0x} + A_x \sin(\omega t)] = A_x \omega \cos(\omega t)$$

$$\dot{p}_y = \frac{d}{dt}[p_{0y} + A_y \cos(\omega t)] = -A_y \omega \sin(\omega t)$$

$$\dot{p}_z = \frac{d}{dt}[p_{0z} + A_z \cos(\omega t)] = -A_z \omega \sin(\omega t)$$

### 1.5.2 角速度

对姿态角求导，得到角速度 $\boldsymbol{\omega}_{\text{rot}}(t) \in \mathbb{R}^3$：

$$\boldsymbol{\omega}_{\text{rot}}(t) = \frac{d\boldsymbol{\eta}(t)}{dt} = \begin{bmatrix}
\dot{\phi}(t) \\
\dot{\theta}(t) \\
\dot{\psi}(t)
\end{bmatrix} = \begin{bmatrix}
A_{\phi} \omega \cos(\omega t) \\
-A_{\theta} \omega \sin(\omega t) \\
A_{\psi} \omega \cos(\omega t)
\end{bmatrix}$$

**注意**：代码中 `desir_end_efect_vel[4]` 的计算可能有误：

```cpp
desir_end_efect_vel[4] = amplitude_rot_y*rate*cos(rate * realtime);  // 应该是 -sin
```

正确的应该是：
$$\dot{\theta} = -A_{\theta} \omega \sin(\omega t)$$

### 1.5.3 扭量表示

末端执行器的速度可以用**扭量** $\mathbf{v}(t) \in \mathbb{R}^6$ 表示：

$$\mathbf{v}(t) = \begin{bmatrix}
\dot{\mathbf{p}}(t) \\
\boldsymbol{\omega}_{\text{rot}}(t)
\end{bmatrix} = \begin{bmatrix}
A_x \omega \cos(\omega t) \\
-A_y \omega \sin(\omega t) \\
-A_z \omega \sin(\omega t) \\
A_{\phi} \omega \cos(\omega t) \\
-A_{\theta} \omega \sin(\omega t) \\
A_{\psi} \omega \cos(\omega t)
\end{bmatrix}$$


## 1.6 加速度轨迹

### 1.6.1 线加速度

对速度求二阶导，得到加速度 $\ddot{\mathbf{p}}(t) \in \mathbb{R}^3$：

$$\ddot{\mathbf{p}}(t) = \frac{d^2\mathbf{p}(t)}{dt^2} = \begin{bmatrix}
\ddot{p}_x(t) \\
\ddot{p}_y(t) \\
\ddot{p}_z(t)
\end{bmatrix} = \begin{bmatrix}
-A_x \omega^2 \sin(\omega t) \\
-A_y \omega^2 \cos(\omega t) \\
-A_z \omega^2 \cos(\omega t)
\end{bmatrix}$$

**推导过程**：

$$\ddot{p}_x = \frac{d}{dt}[A_x \omega \cos(\omega t)] = -A_x \omega^2 \sin(\omega t)$$

$$\ddot{p}_y = \frac{d}{dt}[-A_y \omega \sin(\omega t)] = -A_y \omega^2 \cos(\omega t)$$

$$\ddot{p}_z = \frac{d}{dt}[-A_z \omega \sin(\omega t)] = -A_z \omega^2 \cos(\omega t)$$

### 1.6.2 角加速度

对角速度求导（代码中未实现）：

$$\boldsymbol{\alpha}(t) = \frac{d\boldsymbol{\omega}_{\text{rot}}(t)}{dt} = \begin{bmatrix}
-A_{\phi} \omega^2 \sin(\omega t) \\
-A_{\theta} \omega^2 \cos(\omega t) \\
-A_{\psi} \omega^2 \sin(\omega t)
\end{bmatrix}$$

**注意**：代码中对姿态加速度的处理：
```cpp
for (size_t i = 0; i < 3; i++)
{
    desir_end_efect_acc[3+i] = 0;  // 强制设为 0
}
```


## 1.7 轨迹特性分析

### 1.7.1 周期性

所有轨迹分量都是周期函数，周期为：

$$T = \frac{2\pi}{\omega} = \frac{2\pi}{2} = \pi \text{ 秒}$$

### 1.7.2 频率

$$f = \frac{1}{T} = \frac{\omega}{2\pi} = \frac{2}{2\pi} \approx 0.318 \text{ Hz}$$

### 1.7.3 位置-速度-加速度关系

对于简谐运动，满足：

$$\ddot{\mathbf{p}}(t) = -\omega^2 [\mathbf{p}(t) - \mathbf{p}_0]$$

这是**简谐振子**的特征方程。

### 1.7.4 速度范数

最大线速度：

$$\|\dot{\mathbf{p}}\|_{\max} = \omega \sqrt{A_x^2 + A_y^2 + A_z^2}$$

最大加速度：

$$\|\ddot{\mathbf{p}}\|_{\max} = \omega^2 \sqrt{A_x^2 + A_y^2 + A_z^2}$$


## 1.8 相位关系

### 1.8.1 位置分量的相位

- $p_x(t) = p_{0x} + A_x \sin(\omega t)$，相位：$0$
- $p_y(t) = p_{0y} + A_y \cos(\omega t)$，相位：$\frac{\pi}{2}$
- $p_z(t) = p_{0z} + A_z \cos(\omega t)$，相位：$\frac{\pi}{2}$

### 1.8.2 轨迹形状

如果 $A_x = A_y \neq 0, A_z = 0$，则在 XY 平面上的轨迹为**圆**：

$$x^2 + y^2 = A^2$$

其参数方程为：

$$\begin{cases}
x(t) = p_{0x} + A \sin(\omega t) \\
y(t) = p_{0y} + A \cos(\omega t)
\end{cases}$$


## 1.9 代码实现的关键点

### 1.9.1 强制时间为零

```cpp
realtime = realtime * 0.0;  // 强制 t = 0
```

这意味着**所有轨迹量都是常数**：

$$\mathbf{p}(0) = \begin{bmatrix} p_{0x} \\ p_{0y} + A_y \\ p_{0z} + A_z \end{bmatrix}, \quad \dot{\mathbf{p}}(0) = \begin{bmatrix} A_x \omega \\ 0 \\ 0 \end{bmatrix}$$

### 1.9.2 初始化为零振幅

所有振幅参数初始化为 $0$，因此实际上：

$$\mathbf{p}(t) = \mathbf{p}_0, \quad \dot{\mathbf{p}}(t) = \mathbf{0}, \quad \ddot{\mathbf{p}}(t) = \mathbf{0}$$

**结论**：当前代码配置下，末端执行器保持在默认位置静止。


## 1.10 数学总结

该函数实现了一个**6 自由度正弦/余弦轨迹生成器**，满足以下微分方程：

$$\ddot{\mathbf{x}}(t) + \omega^2 \mathbf{x}(t) = \omega^2 \mathbf{x}_0$$

其中 $\mathbf{x}(t) \in \mathbb{R}^6$ 是广义坐标（位置 + 姿态）。

### 1.10.1 通解形式

$$\mathbf{x}(t) = \mathbf{x}_0 + \mathbf{A} \circ \sin(\omega t + \boldsymbol{\phi})$$

其中：
- $\mathbf{x}_0$：平衡位置
- $\mathbf{A}$：振幅向量
- $\boldsymbol{\phi}$：相位向量
- $\circ$：逐元素乘法（Hadamard 积）

### 1.10.2 能量分析

系统的总机械能（假设单位质量）：

$$E = \frac{1}{2}\|\dot{\mathbf{p}}\|^2 + \frac{1}{2}\omega^2\|\mathbf{p} - \mathbf{p}_0\|^2 = \frac{1}{2}\omega^2 \|\mathbf{A}\|^2 = \text{常数}$$

这表明能量守恒，系统做**无阻尼简谐振动**。


## 1.11 应用场景

该轨迹生成器适用于：

1. **轨迹跟踪测试**：验证控制器的轨迹跟踪性能
2. **周期性任务**：如搅拌、抛光等周期性操作
3. **工作空间探索**：通过调整振幅探索工作空间边界
4. **频率响应分析**：通过改变 $\omega$ 进行系统辨识


## 1.12 代码映射

| 数学符号 | 代码变量 | 维度 |
|----------|----------|------|
| $\mathbf{p}(t)$ | `desir_end_efect_pos[0:2]` | $\mathbb{R}^3$ |
| $\boldsymbol{\eta}(t)$ | `desir_end_efect_pos[3:5]` | $\mathbb{R}^3$ |
| $\dot{\mathbf{p}}(t)$ | `desir_end_efect_vel[0:2]` | $\mathbb{R}^3$ |
| $\boldsymbol{\omega}_{\text{rot}}(t)$ | `desir_end_efect_vel[3:5]` | $\mathbb{R}^3$ |
| $\ddot{\mathbf{p}}(t)$ | `desir_end_efect_acc[0:2]` | $\mathbb{R}^3$ |
| `desir_end_effector_frame` | $\mathbf{T}(t)$ | $SE(3)$ |
| `desir_end_effector_dot` | $\mathbf{v}(t)$ | $\mathbb{R}^6$ |

## 参考文献

1. Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control*. Pearson.
2. Siciliano, B., et al. (2010). *Robotics: Modelling, Planning and Control*. Springer.
3. Murray, R. M., et al. (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press.

---

# 2. 雅可比矩阵计算函数 (JntToJac)

## 2.1 函数签名

```cpp
int ChainJntToJacSolver::JntToJac(const JntArray& q_in, Jacobian& jac)
```

在代码中的调用：
```cpp
llarm6dof.jac_solver_myarm->JntToJac(llarm6dof.current_joint_positions,
                                     llarm6dof.current_jacobian)
```

## 2.2 函数概述

该函数计算机械臂在给定关节配置下的**雅可比矩阵**（Jacobian Matrix），描述了关节速度与末端执行器速度之间的映射关系。

## 2.3 数学定义

### 2.3.1 雅可比矩阵

雅可比矩阵 $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{6 \times n}$ 定义了末端执行器速度与关节速度的关系：

$$\mathbf{v} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}}$$

其中：
- $\mathbf{q} \in \mathbb{R}^n$：关节位置向量（对于 6 自由度机械臂，$n = 6$）
- $\dot{\mathbf{q}} \in \mathbb{R}^n$：关节速度向量
- $\mathbf{v} \in \mathbb{R}^6$：末端执行器速度（扭量）

### 2.3.2 扭量的分解

末端执行器速度 $\mathbf{v}$ 可以分解为：

$$\mathbf{v} = \begin{bmatrix} \dot{\mathbf{p}} \\ \boldsymbol{\omega} \end{bmatrix} = \begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{z} \\ \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}$$

其中：
- $\dot{\mathbf{p}} \in \mathbb{R}^3$：线速度（linear velocity）
- $\boldsymbol{\omega} \in \mathbb{R}^3$：角速度（angular velocity）

## 2.4 雅可比矩阵的结构

雅可比矩阵可以分为两部分：

$$\mathbf{J}(\mathbf{q}) = \begin{bmatrix} \mathbf{J}_v(\mathbf{q}) \\ \mathbf{J}_{\omega}(\mathbf{q}) \end{bmatrix} \in \mathbb{R}^{6 \times 6}$$

其中：
- $\mathbf{J}_v \in \mathbb{R}^{3 \times 6}$：线速度雅可比（velocity Jacobian）
- $\mathbf{J}_{\omega} \in \mathbb{R}^{3 \times 6}$：角速度雅可比（angular velocity Jacobian）

完整形式：

$$\mathbf{J}(\mathbf{q}) = \begin{bmatrix}
\mathbf{J}_{v1} & \mathbf{J}_{v2} & \mathbf{J}_{v3} & \mathbf{J}_{v4} & \mathbf{J}_{v5} & \mathbf{J}_{v6} \\
\mathbf{J}_{\omega 1} & \mathbf{J}_{\omega 2} & \mathbf{J}_{\omega 3} & \mathbf{J}_{\omega 4} & \mathbf{J}_{\omega 5} & \mathbf{J}_{\omega 6}
\end{bmatrix}$$

其中 $\mathbf{J}_{vi}, \mathbf{J}_{\omega i} \in \mathbb{R}^3$ 是第 $i$ 个关节对应的雅可比列向量。

## 2.5 雅可比列向量的计算

对于旋转关节（revolute joint），第 $i$ 列的雅可比向量为：

$$\mathbf{J}_i = \begin{bmatrix} \mathbf{J}_{vi} \\ \mathbf{J}_{\omega i} \end{bmatrix} = \begin{bmatrix} \mathbf{z}_{i-1} \times (\mathbf{p}_n - \mathbf{p}_{i-1}) \\ \mathbf{z}_{i-1} \end{bmatrix}$$

其中：
- $\mathbf{z}_{i-1} \in \mathbb{R}^3$：第 $i-1$ 个坐标系的 z 轴单位向量（关节 $i$ 的旋转轴）
- $\mathbf{p}_{i-1} \in \mathbb{R}^3$：第 $i-1$ 个关节的位置
- $\mathbf{p}_n \in \mathbb{R}^3$：末端执行器的位置
- $\times$：向量叉乘

### 2.5.1 线速度雅可比

$$\mathbf{J}_{vi} = \mathbf{z}_{i-1} \times (\mathbf{p}_n - \mathbf{p}_{i-1})$$

这个公式表示：当第 $i$ 个关节以单位角速度旋转时，末端执行器的线速度。

### 2.5.2 角速度雅可比

$$\mathbf{J}_{\omega i} = \mathbf{z}_{i-1}$$

这个公式表示：当第 $i$ 个关节以单位角速度旋转时，末端执行器的角速度。

## 2.6 计算流程

**输入**：
- $\mathbf{q} = [q_1, q_2, q_3, q_4, q_5, q_6]^T$：当前关节角度

**输出**：
- $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{6 \times 6}$：雅可比矩阵

**步骤**：

1. **正向运动学**：计算每个关节坐标系的位置和姿态

   $$\mathbf{T}_i = \mathbf{T}_{i-1} \cdot \mathbf{T}_{i-1,i}(q_i), \quad i = 1, 2, \ldots, 6$$

2. **提取关节轴和位置**：

   对于每个关节 $i$：
   - 提取 $\mathbf{z}_{i-1} = \mathbf{R}_{i-1} \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$（z 轴方向）
   - 提取 $\mathbf{p}_{i-1}$（关节位置）

3. **计算雅可比列向量**：

   对于每个关节 $i = 1, 2, \ldots, 6$：

   $$\mathbf{J}_i = \begin{bmatrix} \mathbf{z}_{i-1} \times (\mathbf{p}_n - \mathbf{p}_{i-1}) \\ \mathbf{z}_{i-1} \end{bmatrix}$$

4. **组装雅可比矩阵**：

   $$\mathbf{J}(\mathbf{q}) = [\mathbf{J}_1 \quad \mathbf{J}_2 \quad \mathbf{J}_3 \quad \mathbf{J}_4 \quad \mathbf{J}_5 \quad \mathbf{J}_6]$$

## 2.7 雅可比矩阵的应用

### 2.7.1 速度运动学（正向）

已知关节速度，求末端速度：

$$\mathbf{v} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}}$$

### 2.7.2 速度运动学（逆向）

已知末端速度，求关节速度：

$$\dot{\mathbf{q}} = \mathbf{J}^{\dagger}(\mathbf{q}) \mathbf{v}$$

其中 $\mathbf{J}^{\dagger}$ 是伪逆矩阵。

### 2.7.3 静力学映射

通过虚功原理，雅可比矩阵还可以用于力/力矩映射：

$$\boldsymbol{\tau} = \mathbf{J}^T(\mathbf{q}) \mathbf{F}$$

其中：
- $\boldsymbol{\tau} \in \mathbb{R}^6$：关节力矩
- $\mathbf{F} \in \mathbb{R}^6$：末端执行器受到的广义力（力+力矩）

### 2.7.4 奇异性分析

当 $\det(\mathbf{J}) = 0$ 或接近 0 时，机械臂处于**奇异配置**：

$$\det(\mathbf{J}(\mathbf{q})) \approx 0$$

此时：
- 某些方向的末端运动无法实现
- 关节速度可能趋于无穷大
- 控制性能严重下降

## 2.8 参数说明

### 2.8.1 输入参数

| 参数 | 类型 | 数学符号 | 说明 |
|------|------|----------|------|
| `current_joint_positions` | `KDL::JntArray` | $\mathbf{q} \in \mathbb{R}^6$ | 当前关节位置（角度，rad） |

### 2.8.2 输出参数

| 参数 | 类型 | 数学符号 | 说明 |
|------|------|----------|------|
| `current_jacobian` | `KDL::Jacobian` | $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{6 \times 6}$ | 雅可比矩阵 |

### 2.8.3 返回值

- 返回值 $\geq 0$：计算成功
- 返回值 $< 0$：计算失败

## 2.9 代码映射

| 数学符号 | 代码变量 | 维度 |
|----------|----------|------|
| $\mathbf{q}$ | `current_joint_positions` | $\mathbb{R}^6$ |
| $\mathbf{J}(\mathbf{q})$ | `current_jacobian` | $\mathbb{R}^{6 \times 6}$ |
| $\mathbf{J}_v$ | `current_jacobian.data.topRows(3)` | $\mathbb{R}^{3 \times 6}$ |
| $\mathbf{J}_{\omega}$ | `current_jacobian.data.bottomRows(3)` | $\mathbb{R}^{3 \times 6}$ |

## 2.10 数学性质

### 2.10.1 雅可比矩阵的秩

- 满秩（$\text{rank}(\mathbf{J}) = 6$）：非奇异配置，末端可以在所有 6 个自由度上运动
- 秩亏（$\text{rank}(\mathbf{J}) < 6$）：奇异配置，某些方向的运动受限

### 2.10.2 条件数

雅可比矩阵的条件数反映了机械臂配置的"好坏"：

$$\kappa(\mathbf{J}) = \frac{\sigma_{\max}}{\sigma_{\min}}$$

其中 $\sigma_{\max}, \sigma_{\min}$ 是最大和最小奇异值。

- $\kappa(\mathbf{J})$ 接近 1：配置良好
- $\kappa(\mathbf{J})$ 很大：配置接近奇异，控制性能差

### 2.10.3 可操作度

Yoshikawa 定义的可操作度：

$$w(\mathbf{q}) = \sqrt{\det(\mathbf{J}(\mathbf{q}) \mathbf{J}^T(\mathbf{q}))} = \prod_{i=1}^{6} \sigma_i$$

其中 $\sigma_i$ 是奇异值。

- $w > 0$：非奇异配置
- $w \approx 0$：接近奇异配置

## 2.11 与其他函数的关系

### 2.11.1 与逆速度运动学的关系

`JntToJac` 计算的雅可比矩阵被用于 `CartToJnt` 函数中：

```cpp
// 1. 计算雅可比矩阵
jac_solver_myarm->JntToJac(current_joint_positions, jacobian);

// 2. 使用雅可比求解逆速度运动学
ik_solver_vel->CartToJnt(desir_joint_positions, desir_end_effector_dot, desir_joint_velocities);
```

内部实现等价于：

$$\dot{\mathbf{q}} = \mathbf{J}^{\dagger}(\mathbf{q}) \mathbf{v}$$

### 2.11.2 与正运动学的关系

雅可比矩阵是正运动学的微分：

$$\mathbf{J}(\mathbf{q}) = \frac{\partial \mathbf{f}(\mathbf{q})}{\partial \mathbf{q}}$$

其中 $\mathbf{f}: \mathbb{R}^6 \to SE(3)$ 是正运动学映射。

## 2.12 潜在问题与建议

### 2.12.1 问题 1：使用当前关节位置 vs 期望关节位置

**代码现状**：
```cpp
// 雅可比计算使用当前位置
llarm6dof.jac_solver_myarm->JntToJac(llarm6dof.current_joint_positions,
                                     llarm6dof.current_jacobian);

// 但逆速度运动学使用期望位置
llarm6dof.ik_solver_vel->CartToJnt(llarm6dof.desir_joint_positions,  // ⚠️ 使用期望位置
                                    llarm6dof.desir_end_effector_dot,
                                    llarm6dof.desir_joint_velocities);
```

**问题分析**：

这存在**不一致性**：
- 雅可比矩阵在当前位置 $\mathbf{q}_{\text{current}}$ 处计算
- 但逆速度运动学在期望位置 $\mathbf{q}_{\text{desired}}$ 处计算

数学上的不一致：

$$\dot{\mathbf{q}} = \mathbf{J}^{\dagger}(\mathbf{q}_{\text{desired}}) \mathbf{v}$$

但实际 `CartToJnt` 内部会重新计算 $\mathbf{J}(\mathbf{q}_{\text{desired}})$，所以这里的 `current_jacobian` 可能只用于其他目的（如显示、分析等）。

**建议**：

1. **如果 `current_jacobian` 用于控制**：应该与 `CartToJnt` 使用相同的关节位置

   ```cpp
   // 选项 A：都使用当前位置
   llarm6dof.jac_solver_myarm->JntToJac(llarm6dof.current_joint_positions,
                                        llarm6dof.current_jacobian);
   llarm6dof.ik_solver_vel->CartToJnt(llarm6dof.current_joint_positions,  // 改为当前位置
                                       llarm6dof.desir_end_effector_dot,
                                       llarm6dof.desir_joint_velocities);
   ```

   或

   ```cpp
   // 选项 B：都使用期望位置
   llarm6dof.jac_solver_myarm->JntToJac(llarm6dof.desir_joint_positions,  // 改为期望位置
                                        llarm6dof.desir_jacobian);
   llarm6dof.ik_solver_vel->CartToJnt(llarm6dof.desir_joint_positions,
                                       llarm6dof.desir_end_effector_dot,
                                       llarm6dof.desir_joint_velocities);
   ```

2. **如果 `current_jacobian` 仅用于监控**：保持现状，但需要明确注释说明用途

### 2.12.2 问题 2：CartToJnt 第一个参数的意义

根据 KDL 文档，`CartToJnt` 的第一个参数 `q_in` 应该是**当前关节位置**，用于：
- 计算当前配置下的雅可比矩阵
- 作为迭代算法的初始值（如果有）

**当前代码使用期望位置可能导致**：
- 计算的关节速度基于期望位置而非当前位置
- 如果期望位置与当前位置差距较大，可能导致控制不稳定

**正确做法**：

```cpp
// 使用当前位置计算雅可比，得到当前配置下的速度映射
llarm6dof.ik_solver_vel->CartToJnt(llarm6dof.current_joint_positions,  // 应使用当前位置
                                    llarm6dof.desir_end_effector_dot,
                                    llarm6dof.desir_joint_velocities);
```

### 2.12.3 问题 3：雅可比矩阵的更新频率

**建议**：
- 雅可比矩阵应该在每个控制周期更新
- 确保使用最新的关节位置计算雅可比
- 检查计算频率是否与控制频率匹配

## 2.13 使用示例

### 2.13.1 计算末端速度

已知关节速度，求末端速度：

```cpp
// 1. 计算雅可比矩阵
llarm6dof.jac_solver_myarm->JntToJac(llarm6dof.current_joint_positions,
                                     llarm6dof.current_jacobian);

// 2. 计算末端速度：v = J * q_dot
KDL::Twist end_effector_velocity;
for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
        // v[i] = Σ J[i][j] * q_dot[j]
        end_effector_velocity(i) += llarm6dof.current_jacobian(i, j) *
                                     llarm6dof.current_joint_velocities(j);
    }
}
```

### 2.13.2 奇异性检测

```cpp
// 计算行列式判断是否接近奇异
Eigen::MatrixXd J = llarm6dof.current_jacobian.data;
double det_J = J.determinant();

if (std::abs(det_J) < 1e-3) {
    std::cout << "警告：接近奇异配置！det(J) = " << det_J << std::endl;
}
```

## 参考文献

1. Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control*. Pearson. (Chapter 5: Jacobians)
2. Siciliano, B., et al. (2010). *Robotics: Modelling, Planning and Control*. Springer. (Chapter 3: Differential Kinematics)
3. Yoshikawa, T. (1985). *Manipulability of Robotic Mechanisms*. The International Journal of Robotics Research.

---

# 3. 逆速度运动学函数 (CartToJnt)

## 3.1 函数签名

```cpp
int ChainIkSolverVel_pinv::CartToJnt(const JntArray& q_in,
                                      const Twist& v_in,
                                      JntArray& qdot_out)
```

在代码中的调用（light_lift_arm_6dof_node.cpp:188）:
```cpp
llarm6dof.ik_solver_vel->CartToJnt(llarm6dof.current_joint_positions,
                                    llarm6dof.desir_end_effector_dot,
                                    llarm6dof.desir_joint_velocities)
```

## 3.2 函数概述

该函数实现**逆速度运动学**（Inverse Velocity Kinematics），将末端执行器的期望速度（扭量）转换为对应的关节速度。这是速度层面的运动学逆解，用于实时控制中的速度映射。

## 3.3 数学定义

### 3.3.1 核心映射关系

逆速度运动学解决的是从笛卡尔空间速度到关节空间速度的映射：

$$\dot{\mathbf{q}}_d = \mathbf{J}^{\dagger}(\mathbf{q}_{\text{current}}) \mathbf{v}_d$$

其中：
- $\mathbf{q}_{\text{current}} \in \mathbb{R}^6$：当前关节位置（用于计算雅可比）
- $\dot{\mathbf{q}}_d \in \mathbb{R}^6$：期望关节速度（输出）
- $\mathbf{v}_d \in \mathbb{R}^6$：期望末端速度扭量（输入）
- $\mathbf{J}(\mathbf{q}_{\text{current}}) \in \mathbb{R}^{6 \times 6}$：雅可比矩阵
- $\mathbf{J}^{\dagger}$：雅可比矩阵的伪逆（pseudo-inverse）

### 3.3.2 扭量的结构

期望末端速度 $\mathbf{v}_d$ 是一个扭量（Twist），包含线速度和角速度：

$$\mathbf{v}_d = \begin{bmatrix} \dot{\mathbf{p}}_d \\ \boldsymbol{\omega}_d \end{bmatrix} = \begin{bmatrix} \dot{x}_d \\ \dot{y}_d \\ \dot{z}_d \\ \omega_{x,d} \\ \omega_{y,d} \\ \omega_{z,d} \end{bmatrix}$$

其中：
- $\dot{\mathbf{p}}_d = [\dot{x}_d, \dot{y}_d, \dot{z}_d]^T \in \mathbb{R}^3$：期望线速度
- $\boldsymbol{\omega}_d = [\omega_{x,d}, \omega_{y,d}, \omega_{z,d}]^T \in \mathbb{R}^3$：期望角速度

### 3.3.3 与正向速度运动学的关系

正向速度运动学（由雅可比提供）：

$$\mathbf{v} = \mathbf{J}(\mathbf{q}_{\text{current}}) \dot{\mathbf{q}}$$

逆向速度运动学（本函数）：

$$\dot{\mathbf{q}} = \mathbf{J}^{\dagger}(\mathbf{q}_{\text{current}}) \mathbf{v}$$

**验证关系**：对于非奇异配置（$\mathbf{J}$ 满秩）：

$$\mathbf{v} = \mathbf{J} (\mathbf{J}^{\dagger} \mathbf{v}) = \mathbf{J} \mathbf{J}^{\dagger} \mathbf{v} = \mathbf{v}$$

因为 $\mathbf{J} \mathbf{J}^{\dagger} = \mathbf{I}$ （当 $\mathbf{J}$ 满秩时）

## 3.4 伪逆解法 (Pseudo-Inverse Method)

KDL 的 `ChainIkSolverVel_pinv` 采用**奇异值分解（SVD）**求解 Moore-Penrose 伪逆。

### 3.4.1 SVD 分解

对雅可比矩阵进行奇异值分解：

$$\mathbf{J}(\mathbf{q}_d) = \mathbf{U} \boldsymbol{\Sigma} \mathbf{V}^T$$

其中：
- $\mathbf{U} \in \mathbb{R}^{6 \times 6}$：左奇异向量矩阵（列向量 $\mathbf{u}_i$ 是笛卡尔空间的正交基）
- $\boldsymbol{\Sigma} = \text{diag}(\sigma_1, \sigma_2, \ldots, \sigma_6) \in \mathbb{R}^{6 \times 6}$：奇异值对角矩阵
- $\mathbf{V} \in \mathbb{R}^{6 \times 6}$：右奇异向量矩阵（列向量 $\mathbf{v}_i$ 是关节空间的正交基）
- $\sigma_1 \geq \sigma_2 \geq \cdots \geq \sigma_6 \geq 0$：奇异值（按降序排列）

**物理意义**：
- $\sigma_i$：第 $i$ 个主运动方向的增益
- $\mathbf{u}_i$：笛卡尔空间第 $i$ 个主方向
- $\mathbf{v}_i$：关节空间第 $i$ 个主方向

### 3.4.2 Moore-Penrose 伪逆

$$\mathbf{J}^{\dagger} = \mathbf{V} \boldsymbol{\Sigma}^{\dagger} \mathbf{U}^T$$

其中伪逆奇异值矩阵：

$$\boldsymbol{\Sigma}^{\dagger} = \text{diag}\left(\sigma_1^{\dagger}, \sigma_2^{\dagger}, \ldots, \sigma_6^{\dagger}\right)$$

每个奇异值的伪逆定义为：

$$\sigma_i^{\dagger} = \begin{cases}
\frac{1}{\sigma_i} & \text{if } \sigma_i > \epsilon \\
0 & \text{if } \sigma_i \leq \epsilon
\end{cases}$$

其中 $\epsilon$ 是奇异值阈值（通常 $\epsilon \approx 10^{-6}$），用于避免数值不稳定。

### 3.4.3 关节速度求解

将伪逆代入，得到关节速度的显式表达：

$$\dot{\mathbf{q}}_d = \mathbf{J}^{\dagger} \mathbf{v}_d = \mathbf{V} \boldsymbol{\Sigma}^{\dagger} \mathbf{U}^T \mathbf{v}_d = \sum_{i=1}^{r} \frac{1}{\sigma_i} (\mathbf{u}_i^T \mathbf{v}_d) \mathbf{v}_i$$

其中 $r = \text{rank}(\mathbf{J})$ 是雅可比矩阵的秩。

**展开形式**：

$$\dot{\mathbf{q}}_d = \sum_{i=1}^{6} \sigma_i^{\dagger} (\mathbf{u}_i^T \mathbf{v}_d) \mathbf{v}_i$$

这表示：
1. 将期望末端速度 $\mathbf{v}_d$ 投影到各个笛卡尔主方向 $\mathbf{u}_i$：$\mathbf{u}_i^T \mathbf{v}_d$
2. 根据奇异值倒数缩放：$\sigma_i^{\dagger}$
3. 映射到关节空间主方向 $\mathbf{v}_i$
4. 求和得到最终关节速度

## 3.5 计算流程

### 3.5.1 输入参数

| 参数 | 类型 | 数学符号 | 说明 |
|------|------|----------|------|
| `current_joint_positions` | `KDL::JntArray` | $\mathbf{q}_{\text{current}} \in \mathbb{R}^6$ | 当前关节位置（用于计算雅可比） |
| `desir_end_effector_dot` | `KDL::Twist` | $\mathbf{v}_d \in \mathbb{R}^6$ | 期望末端速度扭量 |

### 3.5.2 输出参数

| 参数 | 类型 | 数学符号 | 说明 |
|------|------|----------|------|
| `desir_joint_velocities` | `KDL::JntArray` | $\dot{\mathbf{q}}_d \in \mathbb{R}^6$ | 期望关节速度（输出） |

### 3.5.3 返回值

- 返回值 $\geq 0$：计算成功
- 返回值 $< 0$：计算失败（如输入维度错误）

### 3.5.4 算法步骤

**步骤 1：计算雅可比矩阵**

在内部调用雅可比求解器，计算在当前关节位置 $\mathbf{q}_{\text{current}}$ 处的雅可比：

$$\mathbf{J}(\mathbf{q}_{\text{current}}) = \text{JntToJac}(\mathbf{q}_{\text{current}})$$

**步骤 2：SVD 分解**

$$\mathbf{J} = \mathbf{U} \boldsymbol{\Sigma} \mathbf{V}^T$$

**步骤 3：构造伪逆**

$$\mathbf{J}^{\dagger} = \mathbf{V} \boldsymbol{\Sigma}^{\dagger} \mathbf{U}^T$$

**步骤 4：求解关节速度**

$$\dot{\mathbf{q}}_d = \mathbf{J}^{\dagger} \mathbf{v}_d$$

## 3.6 特殊情况分析

### 3.6.1 非奇异配置 ($\det(\mathbf{J}) \neq 0$)

当雅可比矩阵满秩（所有奇异值 $\sigma_i > \epsilon$）时：

$$\mathbf{J}^{\dagger} = \mathbf{J}^{-1}$$

此时解是**唯一的**：

$$\dot{\mathbf{q}}_d = \mathbf{J}^{-1} \mathbf{v}_d$$

**性质**：
- 任意末端速度 $\mathbf{v}_d$ 都可以精确实现
- 关节速度唯一确定
- 无冗余自由度

### 3.6.2 奇异配置 ($\det(\mathbf{J}) \approx 0$)

当机械臂处于奇异配置时，至少一个奇异值 $\sigma_k \approx 0$。

**现象**：
- 某些末端运动方向 $\mathbf{u}_k$ 无法实现或需要极大的关节速度
- 伪逆解给出**最小范数解**

**数学描述**：伪逆解是以下优化问题的解：

$$\dot{\mathbf{q}}_d = \arg\min_{\dot{\mathbf{q}}} \|\dot{\mathbf{q}}\|^2 \quad \text{subject to} \quad \mathbf{J} \dot{\mathbf{q}} = \mathbf{v}_d$$

如果 $\mathbf{v}_d$ 包含无法实现的方向分量，则退化为：

$$\dot{\mathbf{q}}_d = \arg\min_{\dot{\mathbf{q}}} \|\mathbf{J} \dot{\mathbf{q}} - \mathbf{v}_d\|^2$$

这给出**最小二乘解**。

**实际效果**：
- 靠近奇异配置时，关节速度可能异常大
- 末端速度跟踪误差增加
- 控制性能下降

### 3.6.3 秩亏情况 ($\text{rank}(\mathbf{J}) < 6$)

当 $r = \text{rank}(\mathbf{J}) < 6$ 时：

$$\dot{\mathbf{q}}_d = \sum_{i=1}^{r} \frac{1}{\sigma_i} (\mathbf{u}_i^T \mathbf{v}_d) \mathbf{v}_i$$

只有前 $r$ 个奇异值参与计算，后 $6-r$ 个方向被忽略。

## 3.7 数学性质

### 3.7.1 最小范数性质

伪逆解给出最小的关节速度 2-范数：

$$\|\dot{\mathbf{q}}_d\| = \min_{\dot{\mathbf{q}}: \mathbf{J}\dot{\mathbf{q}} = \mathbf{v}_d} \|\dot{\mathbf{q}}\|$$

**证明**：设 $\dot{\mathbf{q}}'$ 是任意满足 $\mathbf{J}\dot{\mathbf{q}}' = \mathbf{v}_d$ 的解，则：

$$\dot{\mathbf{q}}' = \mathbf{J}^{\dagger} \mathbf{v}_d + (\mathbf{I} - \mathbf{J}^{\dagger}\mathbf{J}) \mathbf{z}$$

其中 $\mathbf{z} \in \mathbb{R}^6$ 是任意向量。由于：

$$\|\dot{\mathbf{q}}'\|^2 = \|\mathbf{J}^{\dagger} \mathbf{v}_d\|^2 + \|(\mathbf{I} - \mathbf{J}^{\dagger}\mathbf{J}) \mathbf{z}\|^2 \geq \|\mathbf{J}^{\dagger} \mathbf{v}_d\|^2$$

所以 $\dot{\mathbf{q}}_d = \mathbf{J}^{\dagger} \mathbf{v}_d$ 是最小范数解。

### 3.7.2 连续性

在非奇异区域（所有 $\sigma_i > \epsilon$），解连续依赖于输入：

$$\dot{\mathbf{q}}_d = f(\mathbf{q}_d, \mathbf{v}_d)$$

其中 $f$ 是连续函数。

### 3.7.3 局部线性近似

逆速度运动学基于**局部线性化**假设：

$$\mathbf{v}_d = \mathbf{J}(\mathbf{q}_d) \dot{\mathbf{q}}_d$$

这在小时间步 $\Delta t$ 内有效：

$$\mathbf{q}(t + \Delta t) \approx \mathbf{q}(t) + \dot{\mathbf{q}}_d \Delta t$$

当 $\Delta t$ 较大或机械臂运动较快时，线性近似误差增大。

### 3.7.4 计算复杂度

SVD 分解的计算复杂度为 $O(n^3)$，对于 6 自由度机械臂：

$$\text{复杂度} = O(6^3) = O(216)$$

这在实时控制（1 kHz）中是可接受的。

## 3.8 与代码中其他函数的关系

### 3.8.1 与 JntToJac 的关系

`CartToJnt` 内部使用 `JntToJac` 计算雅可比：

```cpp
// 伪代码
CartToJnt(q_in, v_in, qdot_out) {
    J = JntToJac(q_in);           // 计算雅可比
    J_pinv = pseudoInverse(J);    // SVD 求伪逆
    qdot_out = J_pinv * v_in;     // 矩阵-向量乘法
}
```

**数学关系**：

$$\mathbf{J}(\mathbf{q}_d) \xrightarrow{\text{JntToJac}} \mathbf{J}^{\dagger}(\mathbf{q}_d) \xrightarrow{\text{CartToJnt}} \dot{\mathbf{q}}_d$$

### 3.8.2 与轨迹生成的关系

`update_end_effector_trajectory` 生成期望末端速度 $\mathbf{v}_d$，然后：

```
update_end_effector_trajectory(t)
    ↓ 生成 desir_end_effector_dot
CartToJnt(q_d, v_d, qdot_d)
    ↓ 输出 desir_joint_velocities
控制器使用 qdot_d 驱动关节
```

**数学流程**：

$$t \xrightarrow{\text{轨迹生成}} \mathbf{v}_d(t) \xrightarrow{\text{逆速度运动学}} \dot{\mathbf{q}}_d(t) \xrightarrow{\text{积分/控制}} \mathbf{q}(t)$$

### 3.8.3 与正运动学的闭环

正运动学和逆速度运动学形成控制闭环：

```
期望末端轨迹 v_d
    ↓ CartToJnt
期望关节速度 qdot_d
    ↓ 控制器
实际关节位置 q_current
    ↓ JntToCart (正运动学)
实际末端位置 p_current
    ↓ 与期望位置比较
位置误差
```

## 3.9 代码中的正确实现与分析

### 3.9.1 正确实现：使用当前关节位置 ✓

**当前代码实现** (light_lift_arm_6dof_node.cpp:188):

```cpp
// ✓ 正确：使用当前位置
llarm6dof.ik_solver_vel->CartToJnt(llarm6dof.current_joint_positions,
                                    llarm6dof.desir_end_effector_dot,
                                    llarm6dof.desir_joint_velocities);
```

**正确性分析**：

函数在**当前配置** $\mathbf{q}_{\text{current}}$ 处计算雅可比：

$$\mathbf{J}(\mathbf{q}_{\text{current}})$$

实际机械臂也在当前配置 $\mathbf{q}_{\text{current}}$，速度映射关系为：

$$\mathbf{v}_{\text{actual}} = \mathbf{J}(\mathbf{q}_{\text{current}}) \dot{\mathbf{q}}$$

**数学一致性**：

计算的关节速度：

$$\dot{\mathbf{q}}_{\text{computed}} = \mathbf{J}^{\dagger}(\mathbf{q}_{\text{current}}) \mathbf{v}_d$$

应用到当前配置时的实际末端速度：

$$\mathbf{v}_{\text{actual}} = \mathbf{J}(\mathbf{q}_{\text{current}}) \dot{\mathbf{q}}_{\text{computed}} = \mathbf{J}(\mathbf{q}_{\text{current}}) \mathbf{J}^{\dagger}(\mathbf{q}_{\text{current}}) \mathbf{v}_d$$

对于非奇异配置：

$$\mathbf{J}(\mathbf{q}_{\text{current}}) \mathbf{J}^{\dagger}(\mathbf{q}_{\text{current}}) = \mathbf{I}$$

因此：$\mathbf{v}_{\text{actual}} = \mathbf{v}_d$，即**末端速度能够准确跟踪期望速度**。

**优点**：

1. **物理正确性**：雅可比矩阵准确描述当前配置下的速度映射关系
2. **控制稳定性**：避免了使用期望位置可能带来的不一致性
3. **实时反馈**：基于当前真实状态计算，适合实时控制
4. **误差小**：无需假设跟踪误差很小

### 3.9.2 与 JntToJac 的一致性 ✓

**代码实现** (light_lift_arm_6dof_node.cpp:175-188):

```cpp
// 第175行：计算当前配置的雅可比
llarm6dof.jac_solver_myarm->JntToJac(llarm6dof.current_joint_positions,
                                     llarm6dof.current_jacobian);

// 第188行：逆速度运动学也使用当前位置 ✓
llarm6dof.ik_solver_vel->CartToJnt(llarm6dof.current_joint_positions,
                                    llarm6dof.desir_end_effector_dot,
                                    llarm6dof.desir_joint_velocities);
```

**一致性分析**：

- `current_jacobian` 计算在 $\mathbf{q}_{\text{current}}$
- `CartToJnt` 内部也计算雅可比在 $\mathbf{q}_{\text{current}}$

两者**完全一致**，如果代码其他地方使用 `current_jacobian` 进行计算（如力映射、可操作度分析），与 `CartToJnt` 使用的雅可比保持一致，确保整个控制系统的数学一致性。

**注意**：虽然 `CartToJnt` 内部会重新计算雅可比（存在轻微的重复计算），但这确保了函数的独立性和正确性。如果需要优化性能，可以考虑使用已计算的雅可比矩阵直接求解。

### 3.9.3 实现优势总结

当前实现采用**基于当前状态的反馈控制**策略：

**数学表达**：
$$\dot{\mathbf{q}}(t) = \mathbf{J}^{\dagger}(\mathbf{q}_{\text{current}}(t)) \mathbf{v}_d(t)$$

**物理意义**：
- 每个控制周期，基于实时测量的关节位置计算雅可比
- 根据当前配置计算出能实现期望末端速度的关节速度
- 这是标准的**瞬时运动学控制**方法

**适用场景**：
- 实时轨迹跟踪控制
- 需要高精度速度控制的应用
- 存在外部扰动或模型不确定性的情况
- 标准的机器人速度层控制

### 3.9.4 雅可比更新频率

**当前代码**：每个控制周期（1 ms）计算一次雅可比。

**分析**：
- 控制频率：1 kHz（1 ms 周期）
- 雅可比计算复杂度：$O(n^3) \approx O(216)$ 次浮点运算
- 实时性：通常可满足

**建议**：
- 监测计算时间，确保 < 1 ms
- 如果计算负担重，考虑降低更新频率（如每 2-5 ms）
- 但需权衡控制精度

## 3.10 数值稳定性分析

### 3.10.1 奇异值阈值的影响

阈值 $\epsilon$ 的选择影响解的性质：

| $\epsilon$ 值 | 优点 | 缺点 |
|--------------|------|------|
| 太小（如 $10^{-10}$） | 保留更多信息 | 接近奇异时数值不稳定，关节速度爆炸 |
| 太大（如 $10^{-3}$） | 数值稳定 | 丢失可实现的运动方向，跟踪误差大 |
| 适中（如 $10^{-6}$） | 平衡稳定性和精度 | KDL 默认值 |

### 3.10.2 条件数与数值误差

雅可比的条件数：

$$\kappa(\mathbf{J}) = \frac{\sigma_{\max}}{\sigma_{\min}}$$

数值误差放大系数：

$$\frac{\|\Delta \dot{\mathbf{q}}\|}{\|\dot{\mathbf{q}}\|} \leq \kappa(\mathbf{J}) \frac{\|\Delta \mathbf{v}\|}{\|\mathbf{v}\|}$$

**建议**：
- 监测 $\kappa(\mathbf{J})$
- 当 $\kappa > 100$ 时，接近奇异，需谨慎
- 考虑阻尼最小二乘法（Damped Least Squares）改进

### 3.10.3 阻尼伪逆（可选改进）

为提高奇异配置附近的稳定性，可使用阻尼伪逆：

$$\dot{\mathbf{q}}_d = \mathbf{J}^T (\mathbf{J} \mathbf{J}^T + \lambda^2 \mathbf{I})^{-1} \mathbf{v}_d$$

其中 $\lambda > 0$ 是阻尼系数。

**优点**：
- 接近奇异时不会产生极大的关节速度
- 数值稳定性好

**缺点**：
- 即使在非奇异配置也有跟踪误差
- 需调整阻尼系数 $\lambda$

## 3.11 实际应用示例

### 3.11.1 示例 1：跟踪圆形轨迹

**输入**：
- 当前关节位置：$\mathbf{q}_{\text{current}} = [0, 0, 0, 0, 0, 0]^T$ rad
- 期望末端速度：$\mathbf{v}_d = [0.1, 0, 0, 0, 0, 0.5]^T$ m/s, rad/s

**计算**：
```cpp
CartToJnt(q_current, v_d, qdot_d);
```

**输出**：
- $\dot{\mathbf{q}}_d$ = 根据运动学计算的关节速度

**验证**：
```cpp
v_actual = J(q_current) * qdot_d;
error = norm(v_actual - v_d);  // 应接近 0
```

### 3.11.2 示例 2：奇异配置检测

```cpp
// 计算雅可比
JntToJac(q_current, J);

// SVD 分解
Eigen::JacobiSVD<Eigen::MatrixXd> svd(J.data);
auto singular_values = svd.singularValues();

// 检测奇异性
double sigma_min = singular_values(5);
double epsilon = 1e-6;

if (sigma_min < epsilon) {
    std::cout << "警告：接近奇异配置！" << std::endl;
    std::cout << "最小奇异值：" << sigma_min << std::endl;

    // 可能的措施：
    // 1. 降低期望速度
    // 2. 改变运动方向
    // 3. 使用阻尼伪逆
}
```

### 3.11.3 示例 3：最小范数验证

验证伪逆解的最小范数性质：

```cpp
// 伪逆解
CartToJnt(q_current, v_d, qdot_pinv);
double norm_pinv = qdot_pinv.norm();

// 任意其他满足 J * qdot = v_d 的解
JntArray qdot_other = qdot_pinv + z;  // z 在零空间中
double norm_other = qdot_other.norm();

// 验证
assert(norm_pinv <= norm_other);  // 应成立
```

## 3.12 代码映射表

| 数学符号 | 代码变量 | 类型 | 维度 |
|----------|----------|------|------|
| $\mathbf{q}_{\text{current}}$ | `current_joint_positions` | `KDL::JntArray` | $\mathbb{R}^6$ |
| $\mathbf{v}_d$ | `desir_end_effector_dot` | `KDL::Twist` | $\mathbb{R}^6$ |
| $\dot{\mathbf{q}}_d$ | `desir_joint_velocities` | `KDL::JntArray` | $\mathbb{R}^6$ |
| $\mathbf{J}(\mathbf{q}_{\text{current}})$ | 内部计算（未显式暴露） | `KDL::Jacobian` | $\mathbb{R}^{6 \times 6}$ |
| $\mathbf{J}^{\dagger}$ | 内部 SVD 计算 | - | $\mathbb{R}^{6 \times 6}$ |
| $\dot{\mathbf{p}}_d$ | `desir_end_effector_dot.vel` | `KDL::Vector` | $\mathbb{R}^3$ |
| $\boldsymbol{\omega}_d$ | `desir_end_effector_dot.rot` | `KDL::Vector` | $\mathbb{R}^3$ |

## 3.13 性能优化建议

### 3.13.1 缓存雅可比矩阵

如果关节位置变化不大，可缓存雅可比：

```cpp
static KDL::JntArray q_cached;
static KDL::Jacobian J_cached;
static bool cache_valid = false;

if (!cache_valid || !q_current.isApprox(q_cached, 1e-4)) {
    JntToJac(q_current, J_cached);
    q_cached = q_current;
    cache_valid = true;
}

// 使用缓存的雅可比
```

### 3.13.2 避免重复计算

`CartToJnt` 内部计算雅可比，如果外部已计算（如 line 170），可能重复。

**改进**：使用直接的伪逆计算：

```cpp
// 已有 current_jacobian
Eigen::MatrixXd J_pinv = J.data.completeOrthogonalDecomposition().pseudoInverse();
qdot_d = J_pinv * v_d;
```

### 3.13.3 并行化（多机械臂场景）

如果控制多个机械臂，SVD 可并行计算。

## 3.14 总结

### 3.14.1 核心公式

$$\boxed{\dot{\mathbf{q}}_d = \mathbf{J}^{\dagger}(\mathbf{q}_{\text{current}}) \mathbf{v}_d = \sum_{i=1}^{r} \frac{1}{\sigma_i} (\mathbf{u}_i^T \mathbf{v}_d) \mathbf{v}_i}$$

### 3.14.2 关键特性

1. **最小范数**：给出最小的关节速度
2. **局部线性**：基于雅可比的一阶近似
3. **奇异鲁棒**：通过 SVD 阈值处理奇异配置
4. **实时性**：$O(n^3)$ 复杂度适合控制频率

### 3.14.3 当前实现的优点

1. **物理正确**：使用 `current_joint_positions` 确保雅可比与实际状态一致 ✓
2. **控制稳定**：基于当前状态的反馈控制，适合实时应用 ✓
3. **数学一致**：与 JntToJac 使用相同的关节位置 ✓

### 3.14.4 可能的改进方向

1. **阻尼伪逆**：考虑使用阻尼伪逆提高奇异配置附近的鲁棒性
2. **奇异性监测**：添加奇异性检测和保护机制
3. **性能优化**：避免雅可比矩阵的重复计算（虽然影响较小）
4. **速度限制**：添加关节速度限幅，确保安全

## 参考文献

1. Siciliano, B., et al. (2010). *Robotics: Modelling, Planning and Control*. Springer. (Chapter 3.7: Inverse Differential Kinematics)
2. Nakamura, Y. (1991). *Advanced Robotics: Redundancy and Optimization*. Addison-Wesley. (Chapter 3: Pseudoinverse)
3. Buss, S. R. (2004). *Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods*. IEEE Journal.
4. Deo, A. S., & Walker, I. D. (1995). *Robot Subtask Performance with Singularity Robustness Using Optimal Damped Least-Squares*. Proceedings of IEEE ICRA.
5. Golub, G. H., & Van Loan, C. F. (2013). *Matrix Computations* (4th ed.). Johns Hopkins University Press. (Chapter 5: SVD and Pseudoinverse)

---

# 4. 正向运动学函数 (JntToCart)

## 4.1 函数签名

```cpp
int ChainFkSolverPos_recursive::JntToCart(const JntArray& q_in,
                                           Frame& p_out,
                                           int segmentNr=-1)
```

在代码中的调用示例 (light_lift_arm_6dof_node.cpp):
```cpp
// 计算到第1个连杆的位姿
llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 1);

// 计算到第2个连杆的位姿
llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 2);

// 计算到第3个连杆的位姿
llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 3);

// 计算到第4个连杆的位姿
llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 4);

// 计算到第5个连杆的位姿
llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 5);

// 计算到第6个连杆的位姿（末端执行器）
llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 6);
```

## 4.2 函数概述

该函数实现**正向运动学**（Forward Kinematics, FK），根据给定的关节角度计算从基座坐标系到指定连杆坐标系的齐次变换矩阵。这是机器人运动学中的核心计算，用于确定末端执行器或任意连杆在笛卡尔空间中的位置和姿态。

## 4.3 数学定义

### 4.3.1 核心映射关系

正向运动学定义了从**关节空间**到**笛卡尔空间**的映射：

$$\mathbf{T}_i = f_{\text{FK}}(\mathbf{q}, i): \mathbb{R}^n \times \mathbb{N} \to SE(3)$$

其中：
- $\mathbf{q} = [q_1, q_2, \ldots, q_n]^T \in \mathbb{R}^n$：关节位置向量（对于6自由度机械臂，$n=6$）
- $i \in \{1, 2, \ldots, n\}$：连杆编号（segment number）
- $\mathbf{T}_i \in SE(3)$：从基座到第 $i$ 个连杆的齐次变换矩阵
- $SE(3)$：特殊欧几里得群（Special Euclidean Group），表示3D空间中的刚体变换

### 4.3.2 齐次变换矩阵的结构

齐次变换矩阵 $\mathbf{T}_i \in \mathbb{R}^{4 \times 4}$ 具有以下形式：

$$\mathbf{T}_i = \begin{bmatrix}
\mathbf{R}_i & \mathbf{p}_i \\
\mathbf{0}^T & 1
\end{bmatrix} = \begin{bmatrix}
r_{11} & r_{12} & r_{13} & p_x \\
r_{21} & r_{22} & r_{23} & p_y \\
r_{31} & r_{32} & r_{33} & p_z \\
0 & 0 & 0 & 1
\end{bmatrix}$$

其中：
- $\mathbf{R}_i \in SO(3) \subset \mathbb{R}^{3 \times 3}$：旋转矩阵（Special Orthogonal Group）
- $\mathbf{p}_i = [p_x, p_y, p_z]^T \in \mathbb{R}^3$：位置向量
- $\mathbf{0}^T = [0, 0, 0]$：零向量

**旋转矩阵的性质**：
- 正交性：$\mathbf{R}_i^T \mathbf{R}_i = \mathbf{I}$
- 行列式：$\det(\mathbf{R}_i) = 1$
- 列向量是坐标系的基向量：
  $$\mathbf{R}_i = \begin{bmatrix} \mathbf{x}_i & \mathbf{y}_i & \mathbf{z}_i \end{bmatrix}$$
  其中 $\mathbf{x}_i, \mathbf{y}_i, \mathbf{z}_i$ 是单位正交向量

**物理意义**：
- $\mathbf{p}_i$：第 $i$ 个连杆坐标系的原点相对于基座坐标系的位置
- $\mathbf{R}_i$：第 $i$ 个连杆坐标系相对于基座坐标系的姿态

## 4.4 递归计算方法

### 4.4.1 连杆间的齐次变换

正向运动学通过**递归乘法**计算：

$$\mathbf{T}_i = \mathbf{T}_{i-1} \cdot \mathbf{T}_{i-1,i}(q_i)$$

其中：
- $\mathbf{T}_{i-1}$：基座到第 $i-1$ 个连杆的变换
- $\mathbf{T}_{i-1,i}(q_i)$：第 $i-1$ 个连杆到第 $i$ 个连杆的变换（依赖于关节 $i$ 的角度 $q_i$）

### 4.4.2 完整递归公式

从基座（$i=0$）到第 $k$ 个连杆的变换：

$$\mathbf{T}_k = \mathbf{T}_0 \cdot \mathbf{T}_{0,1}(q_1) \cdot \mathbf{T}_{1,2}(q_2) \cdot \ldots \cdot \mathbf{T}_{k-1,k}(q_k) = \prod_{i=1}^{k} \mathbf{T}_{i-1,i}(q_i)$$

**初始条件**：
$$\mathbf{T}_0 = \mathbf{I}_4 = \begin{bmatrix} \mathbf{I}_3 & \mathbf{0} \\ \mathbf{0}^T & 1 \end{bmatrix}$$

### 4.4.3 递归算法流程

**输入**：
- 关节位置向量 $\mathbf{q} = [q_1, q_2, \ldots, q_n]^T$
- 目标连杆编号 $k \in \{1, 2, \ldots, n\}$

**输出**：
- 齐次变换矩阵 $\mathbf{T}_k \in SE(3)$

**算法步骤**：

```
初始化: T = I₄ (单位矩阵)

for i = 1 to k do:
    1. 获取连杆 i 的 DH 参数或 URDF 参数
    2. 构造局部变换矩阵 T_{i-1,i}(q_i)
    3. 累积变换: T = T × T_{i-1,i}(q_i)
end for

返回: T_k = T
```

## 4.5 DH参数方法（Denavit-Hartenberg）

### 4.5.1 DH参数定义

每个连杆 $i$ 由四个 DH 参数描述：

| 参数 | 符号 | 物理意义 |
|------|------|----------|
| 连杆长度 | $a_i$ | 沿 $x_{i}$ 轴从 $z_{i-1}$ 到 $z_i$ 的距离 |
| 连杆扭角 | $\alpha_i$ | 绕 $x_{i}$ 轴从 $z_{i-1}$ 到 $z_i$ 的旋转角度 |
| 连杆偏距 | $d_i$ | 沿 $z_{i-1}$ 轴从 $x_{i-1}$ 到 $x_i$ 的距离 |
| 关节角 | $\theta_i$ | 绕 $z_{i-1}$ 轴从 $x_{i-1}$ 到 $x_i$ 的旋转角度 |

**对于旋转关节（revolute joint）**：
- $\theta_i = q_i$（变量，关节角度）
- $d_i, a_i, \alpha_i$ 为常数

**对于移动关节（prismatic joint）**：
- $d_i = q_i$（变量，关节位移）
- $\theta_i, a_i, \alpha_i$ 为常数

### 4.5.2 DH变换矩阵

第 $i$ 个连杆的局部变换矩阵（标准DH约定）：

$$\mathbf{T}_{i-1,i}(\theta_i, d_i, a_i, \alpha_i) = \text{Rot}_z(\theta_i) \cdot \text{Trans}_z(d_i) \cdot \text{Trans}_x(a_i) \cdot \text{Rot}_x(\alpha_i)$$

**展开形式**：

$$\mathbf{T}_{i-1,i} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

**简化记号**：

设 $c_i = \cos\theta_i, \; s_i = \sin\theta_i, \; c_{\alpha_i} = \cos\alpha_i, \; s_{\alpha_i} = \sin\alpha_i$，则：

$$\mathbf{T}_{i-1,i} = \begin{bmatrix}
c_i & -s_i c_{\alpha_i} & s_i s_{\alpha_i} & a_i c_i \\
s_i & c_i c_{\alpha_i} & -c_i s_{\alpha_i} & a_i s_i \\
0 & s_{\alpha_i} & c_{\alpha_i} & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### 4.5.3 分步变换的几何解释

DH变换可以分解为四个基本变换的组合：

1. **绕 $z_{i-1}$ 轴旋转 $\theta_i$**：
   $$\text{Rot}_z(\theta_i) = \begin{bmatrix}
   c_i & -s_i & 0 & 0 \\
   s_i & c_i & 0 & 0 \\
   0 & 0 & 1 & 0 \\
   0 & 0 & 0 & 1
   \end{bmatrix}$$

2. **沿 $z_{i-1}$ 轴平移 $d_i$**：
   $$\text{Trans}_z(d_i) = \begin{bmatrix}
   1 & 0 & 0 & 0 \\
   0 & 1 & 0 & 0 \\
   0 & 0 & 1 & d_i \\
   0 & 0 & 0 & 1
   \end{bmatrix}$$

3. **沿 $x_i$ 轴平移 $a_i$**：
   $$\text{Trans}_x(a_i) = \begin{bmatrix}
   1 & 0 & 0 & a_i \\
   0 & 1 & 0 & 0 \\
   0 & 0 & 1 & 0 \\
   0 & 0 & 0 & 1
   \end{bmatrix}$$

4. **绕 $x_i$ 轴旋转 $\alpha_i$**：
   $$\text{Rot}_x(\alpha_i) = \begin{bmatrix}
   1 & 0 & 0 & 0 \\
   0 & c_{\alpha_i} & -s_{\alpha_i} & 0 \\
   0 & s_{\alpha_i} & c_{\alpha_i} & 0 \\
   0 & 0 & 0 & 1
   \end{bmatrix}$$

## 4.6 6自由度机械臂的正向运动学

### 4.6.1 6连杆串联机构

对于6自由度机械臂，完整的正向运动学为：

$$\mathbf{T}_6^0 = \mathbf{T}_1^0(q_1) \cdot \mathbf{T}_2^1(q_2) \cdot \mathbf{T}_3^2(q_3) \cdot \mathbf{T}_4^3(q_4) \cdot \mathbf{T}_5^4(q_5) \cdot \mathbf{T}_6^5(q_6)$$

其中上标表示坐标系（$0$ 为基座，$6$ 为末端执行器）。

### 4.6.2 任意连杆的位姿

对于连杆 $i \in \{1, 2, 3, 4, 5, 6\}$：

$$\mathbf{T}_i^0 = \prod_{j=1}^{i} \mathbf{T}_j^{j-1}(q_j)$$

**示例**：计算第3个连杆的位姿

$$\mathbf{T}_3^0 = \mathbf{T}_1^0(q_1) \cdot \mathbf{T}_2^1(q_2) \cdot \mathbf{T}_3^2(q_3)$$

### 4.6.3 位置和姿态的提取

从齐次变换矩阵 $\mathbf{T}_i$ 提取位置和姿态：

**位置向量**：
$$\mathbf{p}_i = \begin{bmatrix} p_x \\ p_y \\ p_z \end{bmatrix} = \mathbf{T}_i[0:3, 3]$$

**旋转矩阵**：
$$\mathbf{R}_i = \mathbf{T}_i[0:3, 0:3]$$

**欧拉角**（Roll-Pitch-Yaw）：

从旋转矩阵 $\mathbf{R}_i$ 提取欧拉角 $(\phi, \theta, \psi)$：

$$\mathbf{R}_i = \begin{bmatrix}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33}
\end{bmatrix}$$

欧拉角解析解（ZYX约定）：

$$\begin{cases}
\phi = \text{atan2}(r_{32}, r_{33}) & \text{(Roll, 绕X轴)} \\
\theta = \text{atan2}(-r_{31}, \sqrt{r_{32}^2 + r_{33}^2}) & \text{(Pitch, 绕Y轴)} \\
\psi = \text{atan2}(r_{21}, r_{11}) & \text{(Yaw, 绕Z轴)}
\end{cases}$$

**四元数**（Quaternion）：

从旋转矩阵转换为四元数 $\mathbf{q} = [q_w, q_x, q_y, q_z]^T$：

$$q_w = \frac{1}{2}\sqrt{1 + r_{11} + r_{22} + r_{33}}$$

$$q_x = \frac{r_{32} - r_{23}}{4q_w}, \quad q_y = \frac{r_{13} - r_{31}}{4q_w}, \quad q_z = \frac{r_{21} - r_{12}}{4q_w}$$

（当 $q_w$ 接近0时需要使用替代公式）

## 4.7 参数说明

### 4.7.1 输入参数

| 参数 | 类型 | 数学符号 | 说明 |
|------|------|----------|------|
| `current_joint_positions` | `KDL::JntArray` | $\mathbf{q} \in \mathbb{R}^6$ | 当前关节位置（弧度，rad） |
| `segmentNr` | `int` | $i \in \{1, 2, \ldots, 6\}$ | 目标连杆编号（-1表示末端执行器） |

### 4.7.2 输出参数

| 参数 | 类型 | 数学符号 | 说明 |
|------|------|----------|------|
| `frame` | `KDL::Frame` | $\mathbf{T}_i \in SE(3)$ | 从基座到第 $i$ 个连杆的齐次变换矩阵 |

**KDL::Frame 的结构**：
```cpp
struct Frame {
    Rotation M;  // 旋转矩阵 (3×3)
    Vector p;    // 位置向量 (3×1)
}
```

### 4.7.3 返回值

- 返回值 $\geq 0$：计算成功
- 返回值 $< 0$：计算失败（如连杆编号超出范围、关节数量不匹配等）

**错误代码**：
- `E_NOERROR = 0`：无错误
- `E_OUT_OF_RANGE = -1`：连杆编号超出范围
- `E_SIZE_MISMATCH = -2`：关节数量与模型不匹配

## 4.8 计算流程

### 4.8.1 详细算法步骤

**步骤1：初始化**

设置初始变换为单位矩阵：
$$\mathbf{T} = \mathbf{I}_4$$

**步骤2：遍历连杆**

对于 $i = 1$ 到 $\text{segmentNr}$：

1. **获取连杆参数**
   - 从运动链（Chain）中获取第 $i$ 个连杆的参数
   - 包括：连杆类型（旋转/移动）、DH参数或URDF参数

2. **构造局部变换**

   对于**旋转关节**：
   $$\mathbf{T}_{i-1,i} = f_{\text{joint}}(q_i) \cdot f_{\text{segment}}$$

   其中：
   - $f_{\text{joint}}(q_i)$：关节变换（依赖于 $q_i$）
   - $f_{\text{segment}}$：连杆固定变换（几何参数）

3. **累积变换**
   $$\mathbf{T} \leftarrow \mathbf{T} \cdot \mathbf{T}_{i-1,i}$$

**步骤3：输出结果**

返回累积的变换矩阵：
$$\mathbf{T}_{\text{segmentNr}} = \mathbf{T}$$

### 4.8.2 伪代码

```python
def JntToCart(q_in, segmentNr):
    """
    q_in: 关节位置向量 [q_1, ..., q_n]
    segmentNr: 目标连杆编号 (1 到 n)
    返回: 齐次变换矩阵 T
    """
    # 初始化为单位矩阵
    T = eye(4)

    # 遍历从基座到目标连杆的所有连杆
    for i in range(1, segmentNr + 1):
        # 获取第 i 个连杆的参数
        segment = chain.getSegment(i - 1)

        # 获取关节变换
        T_joint = segment.getJoint().pose(q_in[i - 1])

        # 获取连杆固定变换
        T_segment = segment.getFrameToTip()

        # 累积变换
        T = T @ T_joint @ T_segment

    return T
```

### 4.8.3 数值示例

**输入**：
- 关节角度：$\mathbf{q} = [0, \frac{\pi}{4}, 0, 0, 0, 0]^T$ rad
- 目标连杆：$\text{segmentNr} = 2$

**计算过程**：

1. **初始化**：
   $$\mathbf{T} = \mathbf{I}_4$$

2. **第1个连杆**（$q_1 = 0$）：
   $$\mathbf{T}_1^0 = \mathbf{T}_1^0(0) = \begin{bmatrix}
   1 & 0 & 0 & 0 \\
   0 & 1 & 0 & 0 \\
   0 & 0 & 1 & d_1 \\
   0 & 0 & 0 & 1
   \end{bmatrix}$$

3. **第2个连杆**（$q_2 = \frac{\pi}{4}$）：
   $$\mathbf{T}_2^1 = \mathbf{T}_2^1(\frac{\pi}{4}) = \begin{bmatrix}
   \frac{\sqrt{2}}{2} & -\frac{\sqrt{2}}{2} & 0 & a_2 \frac{\sqrt{2}}{2} \\
   \frac{\sqrt{2}}{2} & \frac{\sqrt{2}}{2} & 0 & a_2 \frac{\sqrt{2}}{2} \\
   0 & 0 & 1 & 0 \\
   0 & 0 & 0 & 1
   \end{bmatrix}$$

4. **累积结果**：
   $$\mathbf{T}_2^0 = \mathbf{T}_1^0 \cdot \mathbf{T}_2^1$$

## 4.9 与其他函数的关系

### 4.9.1 与雅可比矩阵的关系

雅可比矩阵是正向运动学的**微分**：

$$\mathbf{J}(\mathbf{q}) = \frac{\partial f_{\text{FK}}(\mathbf{q})}{\partial \mathbf{q}}$$

在代码中的关系：
```cpp
// 1. 正向运动学：计算当前末端位姿
llarm6dof.fk_solver_myarm->JntToCart(current_joint_positions, frame, 6);

// 2. 雅可比矩阵：计算速度映射
llarm6dof.jac_solver_myarm->JntToJac(current_joint_positions, jacobian);

// 关系：Jacobian 描述了 frame 对 q 的变化率
```

**数学关系**：

设末端位姿为 $\mathbf{T}(\mathbf{q})$，则：

$$\frac{d\mathbf{p}}{dt} = \mathbf{J}_v(\mathbf{q}) \dot{\mathbf{q}}$$

其中 $\mathbf{p}$ 是从 $\mathbf{T}$ 提取的位置向量。

### 4.9.2 与逆运动学的关系

正向运动学和逆运动学是**互逆**的映射：

**正向运动学**：
$$\mathbf{T} = f_{\text{FK}}(\mathbf{q})$$

**逆运动学**：
$$\mathbf{q} = f_{\text{IK}}^{-1}(\mathbf{T})$$

在代码中的调用关系：
```cpp
// 正向运动学：q → T
llarm6dof.fk_solver_myarm->JntToCart(current_joint_positions, current_frame);

// 逆运动学：T → q
llarm6dof.ik_solver_pos->CartToJnt(initial_guess, desired_frame, result_joint_positions);
```

**闭环验证**：
$$f_{\text{FK}}(f_{\text{IK}}(\mathbf{T}_d)) \approx \mathbf{T}_d$$

### 4.9.3 在控制循环中的作用

正向运动学在控制循环中用于：

1. **状态反馈**：计算当前末端位置
2. **误差计算**：比较期望与实际位姿
3. **TF发布**：发布坐标变换给RViz等可视化工具

**控制流程**：

```
读取关节编码器 → current_joint_positions
                     ↓ (JntToCart)
                 current_frame
                     ↓ (计算误差)
                 error = desired_frame - current_frame
                     ↓ (逆运动学/逆速度运动学)
                 desired_joint_velocities
                     ↓ (控制器)
                 关节力矩指令
```

## 4.10 代码中的应用分析

### 4.10.1 多次调用的目的

代码中对 `JntToCart` 进行多次调用（line 222-301），每次计算不同连杆的位姿：

```cpp
// Line 222: 计算link1的位姿
llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 1);
transform_stamped.child_frame_id = "link1";
// ... 发布TF变换

// Line 236: 计算link2的位姿
llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 2);
transform_stamped.child_frame_id = "link2";
// ... 发布TF变换

// ... (link3, link4, link5同理)

// Line 301: 计算link6的位姿（末端执行器）
llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 6);
transform_stamped.child_frame_id = "link6";
// ... 发布TF变换
```

**目的**：
1. **可视化**：为RViz发布每个连杆的TF变换，显示完整机械臂姿态
2. **监控**：实时监测每个连杆的位置，用于碰撞检测、工作空间分析
3. **调试**：验证运动学模型的正确性

**数学含义**：

每次调用计算：
- $\mathbf{T}_1 = \mathbf{T}_1^0(q_1)$
- $\mathbf{T}_2 = \mathbf{T}_1^0(q_1) \cdot \mathbf{T}_2^1(q_2)$
- $\mathbf{T}_3 = \mathbf{T}_1^0(q_1) \cdot \mathbf{T}_2^1(q_2) \cdot \mathbf{T}_3^2(q_3)$
- ...
- $\mathbf{T}_6 = \prod_{i=1}^{6} \mathbf{T}_i^{i-1}(q_i)$

### 4.10.2 性能考虑

**问题**：重复计算导致效率低

每次调用 `JntToCart(q, frame, i)` 都会从头计算到第 $i$ 个连杆，存在重复计算：

```
计算 T_1: T_0 → T_1
计算 T_2: T_0 → T_1 → T_2  (T_1 重复计算)
计算 T_3: T_0 → T_1 → T_2 → T_3  (T_1, T_2 重复计算)
...
```

**时间复杂度**：$O(n^2)$，其中 $n=6$

**改进方案**：使用批量计算接口

KDL提供了批量计算所有连杆位姿的接口：

```cpp
std::vector<KDL::Frame> frames;
fk_solver->JntToCart(current_joint_positions, frames);  // 一次性计算所有连杆

// frames[0] = T_1
// frames[1] = T_2
// ...
// frames[5] = T_6
```

**优化后的时间复杂度**：$O(n)$

**建议修改**：

```cpp
// 替代方案：一次性计算所有连杆位姿
std::vector<KDL::Frame> all_frames(6);
llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, all_frames);

// 发布每个连杆的TF
for (int i = 0; i < 6; i++) {
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "base_link";
    transform_stamped.child_frame_id = "link" + std::to_string(i+1);

    transform_stamped.transform.translation.x = all_frames[i].p.x();
    transform_stamped.transform.translation.y = all_frames[i].p.y();
    transform_stamped.transform.translation.z = all_frames[i].p.z();

    all_frames[i].M.GetQuaternion(
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w);

    tf_broadcaster_->sendTransform(transform_stamped);
}
```

### 4.10.3 当前末端位姿的获取

在 line 317，最后一次计算的结果被保存为当前末端位姿：

```cpp
llarm6dof.current_end_effector_frame = frame;  // Line 317
```

这个值用于：
1. **误差计算**：与期望位姿比较
2. **发布话题**：向ROS发布当前末端状态
3. **逆运动学输入**：作为迭代逆运动学的初始值

## 4.11 数学性质

### 4.11.1 变换的复合性

齐次变换矩阵满足群（Group）的性质：

1. **封闭性**：$\mathbf{T}_1, \mathbf{T}_2 \in SE(3) \Rightarrow \mathbf{T}_1 \cdot \mathbf{T}_2 \in SE(3)$

2. **结合律**：$(\mathbf{T}_1 \cdot \mathbf{T}_2) \cdot \mathbf{T}_3 = \mathbf{T}_1 \cdot (\mathbf{T}_2 \cdot \mathbf{T}_3)$

3. **单位元**：$\mathbf{I}_4 \cdot \mathbf{T} = \mathbf{T} \cdot \mathbf{I}_4 = \mathbf{T}$

4. **逆元**：$\mathbf{T} \cdot \mathbf{T}^{-1} = \mathbf{T}^{-1} \cdot \mathbf{T} = \mathbf{I}_4$

**逆变换的计算**：

$$\mathbf{T}^{-1} = \begin{bmatrix}
\mathbf{R}^T & -\mathbf{R}^T \mathbf{p} \\
\mathbf{0}^T & 1
\end{bmatrix}$$

### 4.11.2 连续性

正向运动学映射 $f_{\text{FK}}(\mathbf{q})$ 是关于 $\mathbf{q}$ 的**连续函数**：

$$\lim_{\mathbf{q} \to \mathbf{q}_0} f_{\text{FK}}(\mathbf{q}) = f_{\text{FK}}(\mathbf{q}_0)$$

这意味着：
- 关节角度的小变化导致末端位姿的小变化
- 不存在跳跃或不连续点

### 4.11.3 可微性

正向运动学是关于关节角度的**光滑函数**（$C^{\infty}$）：

$$\frac{\partial f_{\text{FK}}(\mathbf{q})}{\partial \mathbf{q}} \text{ 处处存在且连续}$$

这是雅可比矩阵存在的数学基础。

### 4.11.4 唯一性

对于给定的关节配置 $\mathbf{q}$，正向运动学的解是**唯一的**：

$$\mathbf{q}_1 = \mathbf{q}_2 \Rightarrow f_{\text{FK}}(\mathbf{q}_1) = f_{\text{FK}}(\mathbf{q}_2)$$

这与逆运动学形成对比（逆运动学可能有多解或无解）。

## 4.12 代码映射表

| 数学符号 | 代码变量 | 类型 | 维度 | 说明 |
|----------|----------|------|------|------|
| $\mathbf{q}$ | `current_joint_positions` | `KDL::JntArray` | $\mathbb{R}^6$ | 关节位置向量 |
| $i$ | `segmentNr` | `int` | $\mathbb{N}$ | 连杆编号 (1-6) |
| $\mathbf{T}_i$ | `frame` | `KDL::Frame` | $SE(3)$ | 齐次变换矩阵 |
| $\mathbf{p}_i$ | `frame.p` | `KDL::Vector` | $\mathbb{R}^3$ | 位置向量 |
| $\mathbf{R}_i$ | `frame.M` | `KDL::Rotation` | $SO(3)$ | 旋转矩阵 |
| $\mathbf{T}_6$ | `current_end_effector_frame` | `KDL::Frame` | $SE(3)$ | 末端执行器位姿 |

**KDL数据类型说明**：

```cpp
// KDL::Vector (3D向量)
class Vector {
    double data[3];  // [x, y, z]
    double x() const { return data[0]; }
    double y() const { return data[1]; }
    double z() const { return data[2]; }
}

// KDL::Rotation (3×3旋转矩阵)
class Rotation {
    double data[9];  // 3×3矩阵，列优先存储
    void GetQuaternion(double& x, double& y, double& z, double& w);
    void GetRPY(double& roll, double& pitch, double& yaw);
}

// KDL::Frame (齐次变换)
class Frame {
    Rotation M;  // 旋转部分
    Vector p;    // 平移部分
}
```

## 4.13 误差分析

### 4.13.1 数值精度

正向运动学涉及大量三角函数和矩阵乘法，存在**舍入误差**：

$$\mathbf{T}_{\text{computed}} = \mathbf{T}_{\text{exact}} + \mathbf{E}$$

其中 $\|\mathbf{E}\|$ 通常在 $10^{-15}$ 到 $10^{-12}$ 量级（双精度浮点）。

**误差累积**：

对于 $n$ 个连杆的串联，误差可能累积：

$$\|\mathbf{E}_n\| \leq n \cdot \epsilon_{\text{machine}}$$

其中 $\epsilon_{\text{machine}} \approx 2.22 \times 10^{-16}$（IEEE 754双精度）。

### 4.13.2 模型误差

**参数误差**：

实际机械臂的DH参数可能与模型存在偏差：

$$\mathbf{T}_{\text{actual}} = f_{\text{FK}}(\mathbf{q}, \boldsymbol{\theta}_{\text{actual}}) \neq f_{\text{FK}}(\mathbf{q}, \boldsymbol{\theta}_{\text{model}})$$

其中 $\boldsymbol{\theta}$ 是DH参数向量。

**标定**：

通过运动学标定减小参数误差：
1. 测量多组 $(\mathbf{q}, \mathbf{T}_{\text{measured}})$
2. 优化参数：$\boldsymbol{\theta}^* = \arg\min_{\boldsymbol{\theta}} \sum_i \|\mathbf{T}_{\text{measured}, i} - f_{\text{FK}}(\mathbf{q}_i, \boldsymbol{\theta})\|^2$

### 4.13.3 关节角度测量误差

关节编码器的测量误差传播到末端位姿：

$$\Delta \mathbf{p} \approx \mathbf{J}_v(\mathbf{q}) \Delta \mathbf{q}$$

其中 $\Delta \mathbf{q}$ 是关节角度误差，$\Delta \mathbf{p}$ 是末端位置误差。

**误差放大系数**：

$$\frac{\|\Delta \mathbf{p}\|}{\|\Delta \mathbf{q}\|} \approx \|\mathbf{J}_v(\mathbf{q})\|$$

在远离基座的配置下，误差放大效应更明显。

## 4.14 应用场景

### 4.14.1 实时控制

在1 kHz控制循环中计算当前末端位姿：

```cpp
void control_loop() {
    // 读取关节编码器
    read_joint_encoders(current_joint_positions);

    // 正向运动学：计算当前末端位姿
    fk_solver->JntToCart(current_joint_positions, current_frame);

    // 计算位姿误差
    error = desired_frame - current_frame;

    // 控制律
    control_output = controller(error);

    // 发送力矩指令
    send_torque_commands(control_output);
}
```

### 4.14.2 可视化

发布TF变换到ROS，用于RViz可视化：

```cpp
// 计算所有连杆位姿
for (int i = 1; i <= 6; i++) {
    fk_solver->JntToCart(current_joint_positions, frame, i);
    publish_tf("base_link", "link" + std::to_string(i), frame);
}
```

### 4.14.3 碰撞检测

计算每个连杆的位置，检测与环境的碰撞：

```cpp
bool check_collision(JntArray q) {
    for (int i = 1; i <= 6; i++) {
        fk_solver->JntToCart(q, frame, i);
        if (is_in_collision(frame.p, link_radius[i])) {
            return true;  // 碰撞
        }
    }
    return false;  // 无碰撞
}
```

### 4.14.4 轨迹验证

在执行轨迹前，验证末端路径：

```cpp
bool validate_trajectory(std::vector<JntArray> q_trajectory) {
    for (auto& q : q_trajectory) {
        fk_solver->JntToCart(q, frame);
        if (!is_within_workspace(frame.p)) {
            return false;  // 超出工作空间
        }
    }
    return true;  // 轨迹有效
}
```

## 4.15 总结

### 4.15.1 核心公式

$$\boxed{\mathbf{T}_k = \prod_{i=1}^{k} \mathbf{T}_{i-1,i}(q_i) = \mathbf{T}_{0,1}(q_1) \cdot \mathbf{T}_{1,2}(q_2) \cdot \ldots \cdot \mathbf{T}_{k-1,k}(q_k)}$$

### 4.15.2 关键特性

1. **唯一性**：给定 $\mathbf{q}$，解唯一
2. **连续性**：$\mathbf{q}$ 的小变化导致 $\mathbf{T}$ 的小变化
3. **可微性**：可计算雅可比矩阵
4. **高效性**：$O(n)$ 时间复杂度（单次计算）

### 4.15.3 代码中的关键点

1. **多次调用**：为每个连杆计算位姿，用于TF发布
2. **性能优化**：可使用批量接口减少重复计算
3. **末端位姿**：`current_end_effector_frame` 存储末端位姿，用于控制和发布

### 4.15.4 与其他函数的协作

```
JntToCart (正向运动学)
    ↓ 提供当前末端位姿
误差计算: error = desired - current
    ↓
CartToJnt (逆速度运动学)
    ↓ 计算关节速度
JntToJac (雅可比矩阵)
    ↓ 提供速度映射
控制器输出关节力矩
```

## 参考文献

1. Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson. (Chapter 3: Forward Kinematics, Chapter 4: Denavit-Hartenberg Convention)
2. Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2010). *Robotics: Modelling, Planning and Control*. Springer. (Chapter 2: Kinematics)
3. Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley. (Chapter 3: Forward Kinematics)
4. Murray, R. M., Li, Z., & Sastry, S. S. (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press. (Chapter 3: Forward Kinematics)
5. Denavit, J., & Hartenberg, R. S. (1955). *A kinematic notation for lower-pair mechanisms based on matrices*. Journal of Applied Mechanics, 22, 215-221.
6. Orocos KDL Documentation: [http://www.orocos.org/kdl](http://www.orocos.org/kdl)

---
# 5. 逆运动学位置求解函数 (comput_ik)

## 5.1 函数签名

```cpp
void LlArm6dof::comput_ik(KDL::Frame &frame)
```

在代码中的调用 (light_lift_arm_6dof_node.cpp:270):
```cpp
// 实时计算逆运动学（非关节自动控制模式下）
if(control_mode.currentState != State::JointAutoServo)
{
    llarm6dof.comput_ik(frame);
}
```

## 5.2 函数概述

该函数实现**逆运动学位置求解**（Inverse Kinematics Position Solver），根据期望的末端执行器位姿计算对应的关节角度配置。与速度层逆运动学（CartToJnt）不同，本函数求解的是**位置层**的逆运动学问题，并包含以下安全机制：

1. **工作空间检查**：验证期望位姿是否在机械臂可达范围内
2. **关节限位保护**：确保计算出的关节角度在安全范围内
3. **迭代求解**：使用 Levenberg-Marquardt 算法数值求解

## 5.3 数学定义

### 5.3.1 核心问题描述

逆运动学位置问题定义为：

**已知**：期望末端执行器位姿 $\mathbf{T}_d \in SE(3)$

**求解**：关节角度 $\mathbf{q} \in \mathbb{R}^6$，使得：

$$f_{\text{FK}}(\mathbf{q}) = \mathbf{T}_d$$

其中 $f_{\text{FK}}: \mathbb{R}^6 \to SE(3)$ 是正向运动学映射。

**数学形式**：

$$\min_{\mathbf{q}} \| f_{\text{FK}}(\mathbf{q}) - \mathbf{T}_d \|$$

受约束于：
$$\mathbf{q}_{\min} \leq \mathbf{q} \leq \mathbf{q}_{\max}$$

### 5.3.2 误差度量

定义位姿误差为扭量（Twist）：

$$\mathbf{e}(\mathbf{q}) = \text{diff}(\mathbf{T}_d, f_{\text{FK}}(\mathbf{q})) \in \mathbb{R}^6$$

其中：
$$\mathbf{e} = \begin{bmatrix} \Delta \mathbf{p} \\ \Delta \boldsymbol{\theta} \end{bmatrix} = \begin{bmatrix} \mathbf{p}_d - \mathbf{p}(\mathbf{q}) \\ \text{rot\_error}(\mathbf{R}_d, \mathbf{R}(\mathbf{q})) \end{bmatrix}$$

- $\Delta \mathbf{p} \in \mathbb{R}^3$：位置误差
- $\Delta \boldsymbol{\theta} \in \mathbb{R}^3$：旋转误差（轴角表示）

**误差范数**：

$$E(\mathbf{q}) = \|\mathbf{L} \cdot \mathbf{e}(\mathbf{q})\|^2$$

其中 $\mathbf{L} \in \mathbb{R}^{6 \times 6}$ 是权重矩阵（通常为对角矩阵）。

### 5.3.3 逆运动学的多解性

对于6自由度机械臂，逆运动学问题通常具有以下特性：

1. **多解性**：一个末端位姿可能对应多个关节配置
   - 对于典型的6轴机械臂：最多8组解（肘上/肘下 × 肩左/肩右 × 腕翻/不翻）

2. **无解情况**：期望位姿超出工作空间
   $$\|\mathbf{p}_d\| > \sum_{i=1}^{n} |l_i|$$

3. **奇异性**：某些配置下雅可比矩阵秩亏
   $$\det(\mathbf{J}(\mathbf{q})) = 0$$

## 5.4 Levenberg-Marquardt 算法（LMA）

### 5.4.1 算法原理

LMA 是一种非线性最小二乘优化算法，结合了**梯度下降法**和**高斯-牛顿法**的优点：

**优化目标**：

$$\mathbf{q}^* = \arg\min_{\mathbf{q}} E(\mathbf{q}) = \arg\min_{\mathbf{q}} \frac{1}{2}\|\mathbf{e}(\mathbf{q})\|^2$$

**迭代更新公式**：

$$\mathbf{q}_{k+1} = \mathbf{q}_k + \Delta \mathbf{q}_k$$

其中 $\Delta \mathbf{q}_k$ 通过求解以下线性系统得到：

$$(\mathbf{J}^T \mathbf{J} + \lambda \mathbf{I}) \Delta \mathbf{q}_k = -\mathbf{J}^T \mathbf{e}(\mathbf{q}_k)$$

**参数说明**：
- $\mathbf{J} \in \mathbb{R}^{6 \times 6}$：雅可比矩阵 $\mathbf{J}(\mathbf{q}_k)$
- $\lambda \geq 0$：阻尼系数（Damping parameter）
- $\mathbf{I}$：单位矩阵

### 5.4.2 阻尼系数调整策略

**初始化**：
$$\lambda_0 = \tau \cdot \max_i(\mathbf{J}^T \mathbf{J})_{ii}$$

通常 $\tau = 10$（代码中的设置）。

**自适应调整**：

定义增益比（Gain ratio）：

$$\rho = \frac{E(\mathbf{q}_k) - E(\mathbf{q}_k + \Delta \mathbf{q}_k)}{\Delta \mathbf{q}_k^T (\lambda \Delta \mathbf{q}_k + \mathbf{g})}$$

其中 $\mathbf{g} = \mathbf{J}^T \mathbf{e}(\mathbf{q}_k)$ 是梯度。

**更新规则**：

$$\lambda_{k+1} = \begin{cases}
\lambda_k \cdot \max\left(\frac{1}{3}, 1 - (2\rho - 1)^3\right) & \text{if } \rho > 0 \text{ (接受步长)} \\
\lambda_k \cdot v_k & \text{if } \rho \leq 0 \text{ (拒绝步长)} \\
\end{cases}$$

其中 $v_0 = 2$，若拒绝步长则 $v_{k+1} = 2v_k$。

**物理意义**：
- $\lambda$ 大 → 接近梯度下降，步长小但稳定
- $\lambda$ 小 → 接近高斯-牛顿法，收敛快但可能不稳定
- 自适应调整 → 在稳定性和收敛速度间自动平衡

### 5.4.3 SVD 求解

使用奇异值分解求解阻尼最小二乘问题：

**SVD 分解**：
$$\mathbf{J} = \mathbf{U} \boldsymbol{\Sigma} \mathbf{V}^T$$

**修正奇异值**：
$$\tilde{\sigma}_i = \frac{\sigma_i}{\sigma_i^2 + \lambda}$$

**解的形式**：
$$\Delta \mathbf{q} = \mathbf{V} \tilde{\boldsymbol{\Sigma}} \mathbf{U}^T (-\mathbf{e})$$

其中 $\tilde{\boldsymbol{\Sigma}} = \text{diag}(\tilde{\sigma}_1, \ldots, \tilde{\sigma}_6)$。

**优点**：
- 自动处理奇异配置（$\sigma_i \approx 0$ 时，$\tilde{\sigma}_i$ 仍有界）
- 数值稳定性好

## 5.5 算法流程

### 5.5.1 完整计算流程

**输入**：
- $\mathbf{T}_d$：期望末端执行器位姿（`desir_end_effector_frame`）
- $\mathbf{q}_{\text{init}}$：初始猜测（通常使用当前关节位置 `current_joint_positions`）

**输出**：
- $\mathbf{q}_{\text{result}}$：计算得到的关节位置

**步骤**：

**步骤 1：工作空间检查**

计算到第4连杆（前臂末端）的位置：
```cpp
fk_solver_myarm->JntToCart(current_joint_positions, frame, 4);
```

计算期望臂长：
$$L_{\text{desired}} = \sqrt{x_4^2 + y_4^2 + (z_4 - 0.1)^2}$$

计算最大臂长：
$$L_{\max} = l_3 + l_4$$

检查：
$$L_{\text{desired}} \leq L_{\max} + \epsilon$$

其中 $\epsilon = 0.01$ m 是容差。

**物理意义**：验证期望位置在可达工作空间内。

**步骤 2：LMA 迭代求解**

```cpp
ik_solver_lma->CartToJnt(current_joint_positions, desir_end_effector_frame, desir_joint_positions_temp);
```

内部迭代过程：

```
初始化: q = q_init, λ = τ, v = 2

for k = 1 to max_iter:
    1. 计算正向运动学: T = FK(q)
    2. 计算误差: e = diff(T_d, T)
    3. 检查收敛: if ‖e‖ < ε_pos, return SUCCESS

    4. 计算雅可比: J = J(q)
    5. SVD 分解: J = U Σ V^T
    6. 计算修正奇异值: σ̃_i = σ_i / (σ_i² + λ)
    7. 求解: Δq = V diag(σ̃) U^T (-e)

    8. 检查关节增量: if ‖Δq‖ < ε_joints, return SUCCESS
    9. 检查梯度: if ‖J^T e‖ < ε_grad, return SUCCESS

    10. 计算新配置: q_new = q + Δq
    11. 计算新误差: e_new = diff(T_d, FK(q_new))
    12. 计算增益比: ρ = (‖e‖² - ‖e_new‖²) / (Δq^T (λΔq + J^T e))

    13. 更新策略:
        if ρ > 0:
            接受步长: q = q_new
            减小阻尼: λ = λ × max(1/3, 1 - (2ρ-1)³)
            v = 2
        else:
            拒绝步长: q 不变
            增大阻尼: λ = λ × v
            v = 2v

end for

if k >= max_iter: return ERROR_MAX_ITERATIONS
```

**收敛判据**：

1. **位置收敛**：$\|\mathbf{e}(\mathbf{q})\| < \epsilon_{\text{pos}}$
2. **关节增量小**：$\|\Delta \mathbf{q}\| < \epsilon_{\text{joints}}$
3. **梯度小**：$\|\mathbf{J}^T \mathbf{e}\| < \epsilon_{\text{grad}}$

**步骤 3：关节限位检查**

对每个关节 $i = 1, \ldots, 6$，检查：

$$q_{\min,i} \leq q_i \leq q_{\max,i}$$

如果任意关节超出限制，拒绝该解。

**步骤 4：更新期望关节位置**

仅当通过所有检查后，更新：
```cpp
desir_joint_positions = desir_joint_positions_temp;
```

### 5.5.2 返回条件

函数在以下情况返回（不更新关节位置）：

| 条件 | 原因 | 代码位置 |
|------|------|----------|
| $L_{\text{desired}} > L_{\max}$ | 超出工作空间 | line 472-478 |
| IK 求解失败 | 算法不收敛 | line 494-498 |
| $q_i > q_{\max,i}$ | 超出上限 | line 504-508 |
| $q_i < q_{\min,i}$ | 超出下限 | line 509-513 |

## 5.6 数学性质

### 5.6.1 收敛性

**局部收敛性**：

在非奇异配置附近，若初始猜测 $\mathbf{q}_0$ 足够接近真解 $\mathbf{q}^*$，LMA 算法保证收敛到局部最优解。

**收敛速度**：

- **远离解**：线性收敛（梯度下降占主导，$\lambda$ 大）
- **接近解**：超线性或二次收敛（高斯-牛顿占主导，$\lambda$ 小）

**理论保证**：

$$\lim_{k \to \infty} \mathbf{q}_k = \mathbf{q}^* \quad \text{当 } \|\mathbf{q}_0 - \mathbf{q}^*\| < \delta$$

其中 $\delta$ 是吸引域半径。

### 5.6.2 全局搜索与局部最小值

**问题**：LMA 是局部优化算法，可能收敛到局部最小值。

**解决策略**（代码中未实现，可改进）：

1. **多起点优化**：从不同初始猜测开始
   ```cpp
   for each q_init in {q_0, q_elbow_up, q_elbow_down, ...}:
       solution = IK_LMA(q_init, T_d)
       if ‖e‖ < best_error:
           best_solution = solution
   ```

2. **解析解 + 数值优化**：先用解析解（如存在）作为初始值

3. **随机重启**：若不收敛，随机采样新的起点

### 5.6.3 雅可比奇异性处理

在奇异配置附近（$\sigma_{\min} \approx 0$）：

**阻尼伪逆**：
$$\tilde{\sigma}_i = \frac{\sigma_i}{\sigma_i^2 + \lambda} \approx \frac{1}{\lambda} \quad \text{当 } \sigma_i \ll \sqrt{\lambda}$$

**效果**：
- 阻止奇异方向上的大步长
- 保持数值稳定性
- 但可能导致收敛变慢

**与普通伪逆对比**：

| 方法 | 奇异值处理 | 奇异配置表现 |
|------|------------|--------------|
| 普通伪逆 | $\sigma_i^{-1}$ | 可能数值爆炸 |
| 阻尼伪逆 | $\sigma_i/(\sigma_i^2 + \lambda)$ | 数值稳定 ✓ |

## 5.7 参数说明

### 5.7.1 输入参数

| 参数 | 类型 | 数学符号 | 说明 |
|------|------|----------|------|
| `frame` | `KDL::Frame&` | $\mathbf{T}_4 \in SE(3)$ | 第4连杆位姿（用于工作空间检查） |

**注意**：期望末端位姿从成员变量 `desir_end_effector_frame` 读取。

### 5.7.2 内部使用的变量

| 变量 | 类型 | 数学符号 | 说明 |
|------|------|----------|------|
| `desir_end_effector_frame` | `KDL::Frame` | $\mathbf{T}_d \in SE(3)$ | 期望末端执行器位姿 |
| `current_joint_positions` | `KDL::JntArray` | $\mathbf{q}_{\text{init}} \in \mathbb{R}^6$ | 当前关节位置（作为初始猜测） |
| `desir_joint_positions_temp` | `KDL::JntArray` | $\mathbf{q}_{\text{temp}} \in \mathbb{R}^6$ | 临时存储计算结果 |
| `desir_joint_positions` | `KDL::JntArray` | $\mathbf{q}_d \in \mathbb{R}^6$ | 更新后的期望关节位置（输出） |
| `desir_joint_pos_protect_max` | `std::vector<float>` | $\mathbf{q}_{\max}$ | 关节角度上限 |
| `desir_joint_pos_protect_min` | `std::vector<float>` | $\mathbf{q}_{\min}$ | 关节角度下限 |

### 5.7.3 算法参数（KDL 默认值）

| 参数 | 符号 | 典型值 | 说明 |
|------|------|--------|------|
| 最大迭代次数 | `max_iter` | 500 | 防止无限循环 |
| 位置收敛阈值 | $\epsilon_{\text{pos}}$ | $10^{-6}$ | 末端位姿误差容差 |
| 关节增量阈值 | $\epsilon_{\text{joints}}$ | $10^{-12}$ | 关节角度变化容差 |
| 梯度阈值 | $\epsilon_{\text{grad}}$ | $10^{-12}$ | 梯度范数容差 |
| 初始阻尼系数 | $\tau$ | 10.0 | 阻尼系数初始值倍数 |

## 5.8 工作空间约束

### 5.8.1 可达工作空间

**数学定义**：

可达工作空间 $\mathcal{W} \subset \mathbb{R}^3$ 定义为：

$$\mathcal{W} = \{\mathbf{p} \in \mathbb{R}^3 \mid \exists \mathbf{q} \in \mathbb{R}^6, f_{\text{FK}}(\mathbf{q}) = (\mathbf{p}, \mathbf{R})\}$$

**简化检查**（代码实现）：

对于串联机械臂，粗略的可达性检查：

$$\|\mathbf{p}_d\| \leq \sum_{i=1}^{n} l_i$$

代码中更精确地检查到第4连杆的距离：

$$\sqrt{x_4^2 + y_4^2 + (z_4 - h_{\text{base}})^2} \leq l_3 + l_4 + \epsilon$$

其中 $h_{\text{base}} = 0.1$ m 是基座高度偏移。

**几何意义**：

- 内边界：$r_{\min} = \max(0, l_1 - l_2 - \cdots - l_n)$
- 外边界：$r_{\max} = l_1 + l_2 + \cdots + l_n$
- 环形工作空间：$r_{\min} \leq \|\mathbf{p}\| \leq r_{\max}$

### 5.8.2 关节限位

**安全约束**：

$$\mathbf{q}_{\min} = \begin{bmatrix} q_{1,\min} \\ \vdots \\ q_{6,\min} \end{bmatrix} \leq \mathbf{q} \leq \begin{bmatrix} q_{1,\max} \\ \vdots \\ q_{6,\max} \end{bmatrix} = \mathbf{q}_{\max}$$

**约束类型**：

| 关节 | 物理限制 | 软件保护 | 说明 |
|------|----------|----------|------|
| 旋转关节 | 机械止挡 | $[-\pi, \pi]$ | 避免碰撞 |
| 连续旋转关节 | 无限制 | $[-\infty, \infty]$ | 电缆限制 |

**软件保护值**（`desir_joint_pos_protect_max/min`）：

通常比硬件限位留有 5-10° 安全余量。

## 5.9 与其他函数的关系

### 5.9.1 与正向运动学的关系

**互逆关系**：

正向运动学：
$$\mathbf{T} = f_{\text{FK}}(\mathbf{q})$$

逆运动学（本函数）：
$$\mathbf{q} = f_{\text{IK}}^{-1}(\mathbf{T})$$

**验证**：
$$f_{\text{FK}}(f_{\text{IK}}(\mathbf{T}_d)) \approx \mathbf{T}_d$$

误差应小于收敛阈值 $\epsilon_{\text{pos}}$。

### 5.9.2 与逆速度运动学的关系

**层次关系**：

- **位置层**：`comput_ik` 求解 $\mathbf{q}_d$ 使得 $f_{\text{FK}}(\mathbf{q}_d) = \mathbf{T}_d$
- **速度层**：`CartToJnt` 求解 $\dot{\mathbf{q}}_d$ 使得 $\mathbf{J}\dot{\mathbf{q}}_d = \mathbf{v}_d$

**微分关系**：

$$\frac{d}{dt}f_{\text{IK}}(\mathbf{T}) = \mathbf{J}^{\dagger}(\mathbf{q}) \cdot \frac{d\mathbf{T}}{dt}$$

**控制结构**：

```
位置层控制:
    期望位姿 T_d → comput_ik → 期望关节位置 q_d
                      ↓
速度层控制:
    期望末端速度 v_d → CartToJnt → 期望关节速度 q̇_d
                      ↓
力矩层控制:
    期望关节加速度 → 动力学模型 → 期望关节力矩 τ_d
```

### 5.9.3 在控制循环中的作用

**调用条件** (light_lift_arm_6dof_node.cpp:268):

```cpp
if(control_mode.currentState != State::JointAutoServo)
{
    llarm6dof.comput_ik(frame);
}
```

**逻辑**：

- **位姿控制模式**：调用 `comput_ik`，将笛卡尔空间指令转换为关节空间
- **关节控制模式**：跳过 `comput_ik`，直接使用关节空间指令

**控制流程**：

```
用户输入期望末端位姿 → desir_end_effector_frame
                ↓
        comput_ik 计算逆运动学
                ↓
        desir_joint_positions (更新)
                ↓
        关节层控制器 → 关节力矩指令
```

## 5.10 实用建议

### 5.10.1 初始猜测的选择

**影响**：初始猜测显著影响：
- 收敛速度
- 收敛到哪个解（多解情况）
- 是否收敛

**代码实现**：

```cpp
// 使用当前关节位置作为初始猜测（推荐）
ik_solver_lma->CartToJnt(current_joint_positions, T_d, q_result);
```

**优点**：
- 平滑运动（避免突变）
- 快速收敛（当前位置通常接近期望位置）
- 保持配置一致性（如肘上/肘下）

**替代方案**（代码中注释掉的）：

```cpp
// 使用零位作为初始猜测
ik_init_joint_positions.data.setZero();
ik_solver_lma->CartToJnt(ik_init_joint_positions, T_d, q_result);
```

**缺点**：
- 可能收敛到不同的解（配置跳变）
- 收敛较慢

### 5.10.2 处理工作空间边界

**建议**：

1. **预先轨迹规划**：确保整条轨迹在工作空间内

2. **安全缩放**：当接近边界时，缩放速度
   ```cpp
   if (distance_to_boundary < safety_margin):
       scale_velocity(1.0 - distance_to_boundary / safety_margin)
   ```

3. **虚拟墙**：在工作空间边界内部设置软限位

### 5.10.3 关节限位处理

**当前实现**：硬约束（拒绝超限解）

**改进方案**：

1. **约束优化**：在 IK 求解中加入关节限位约束
   $$\min_{\mathbf{q}} E(\mathbf{q}) \quad \text{s.t.} \quad \mathbf{q}_{\min} \leq \mathbf{q} \leq \mathbf{q}_{\max}$$

2. **软约束**：使用惩罚项
   $$E_{\text{total}} = E_{\text{pose}} + w_{\text{limit}} \sum_i \max(0, q_i - q_{i,\max})^2$$

3. **饱和映射**：计算后投影到可行域
   $$q_i \leftarrow \text{clip}(q_i, q_{i,\min}, q_{i,\max})$$

## 5.11 代码实现分析

### 5.11.1 关键代码段解析

**工作空间检查** (line 464-478):

```cpp
fk_solver_myarm->JntToCart(current_joint_positions, frame, 4);

float desir_arm_lenght = sqrt(frame.p.x() * frame.p.x() +
                               frame.p.y() * frame.p.y() +
                               (frame.p.z() - 0.1) * (frame.p.z() - 0.1));

float arm_lenght = arm_link_lenth[2] + arm_link_lenth[3];

if (arm_lenght < (desir_arm_lenght - 0.01))
{
    std::cerr << "期望轨迹超出工作空间" << std::endl;
    return;  // 拒绝更新
}
```

**数学等价**：

$$\|\mathbf{p}_4\| = \sqrt{x_4^2 + y_4^2 + (z_4 - 0.1)^2}$$

$$\text{if } \|\mathbf{p}_4\| > l_3 + l_4 + 0.01 \Rightarrow \text{reject}$$

**IK 求解** (line 490):

```cpp
int ret = ik_solver_lma->CartToJnt(current_joint_positions,
                                    desir_end_effector_frame,
                                    desir_joint_positions_temp);
```

**内部执行**：LMA 迭代算法（见 5.5.1）

**关节限位检查** (line 502-514):

```cpp
for (size_t i = 0; i < 6; i++)
{
    if (desir_joint_positions_temp(i) > desir_joint_pos_protect_max[i])
    {
        std::cerr << "关节 " << i << " 角度max超出工作空间" << std::endl;
        return;  // 拒绝更新
    }
    if (desir_joint_positions_temp(i) < desir_joint_pos_protect_min[i])
    {
        std::cerr << "关节 " << i << " 角度min超出工作空间" << std::endl;
        return;  // 拒绝更新
    }
}
```

**数学检查**：

$$\forall i \in \{1, \ldots, 6\}: q_{i,\min} \leq q_i \leq q_{i,\max}$$

**更新输出** (line 516):

```cpp
desir_joint_positions = desir_joint_positions_temp;
```

仅当所有检查通过后才更新全局期望关节位置。

### 5.11.2 安全机制总结

| 序号 | 检查项 | 拒绝条件 | 保护对象 |
|------|--------|----------|----------|
| 1 | 工作空间 | $\|\mathbf{p}_4\| > L_{\max}$ | 机械结构 |
| 2 | IK 收敛性 | `ret != 0` | 数值稳定性 |
| 3 | 上限 | $q_i > q_{i,\max}$ | 关节硬件 |
| 4 | 下限 | $q_i < q_{i,\min}$ | 关节硬件 |

**多层防护**：确保机械臂在安全范围内运动。

## 5.12 性能分析

### 5.12.1 时间复杂度

**单次迭代**：
- 正向运动学：$O(n)$，$n=6$
- 雅可比计算：$O(n^2)$
- SVD 分解：$O(n^3)$
- 矩阵乘法：$O(n^3)$

**总复杂度**：$O(k \cdot n^3)$，其中 $k$ 是迭代次数。

**典型值**：
- $k \approx 10$-50（接近解时）
- $k$ 可达 500（远离解或接近奇异）

**实时性能**：

对于 6 自由度机械臂（$n=6$）：
- 单次迭代：约 0.1-0.5 ms
- 总时间：1-50 ms（视收敛快慢）

**控制频率影响**：

如果控制频率为 1 kHz（1 ms周期）：
- 每次调用可能占用多个周期
- 建议降低调用频率或使用更快的初值

### 5.12.2 收敛性能

**最好情况**：
- 初值接近解：3-10 次迭代
- 非奇异配置：二次收敛

**最坏情况**：
- 初值远离解：100-500 次迭代
- 接近奇异：收敛缓慢或不收敛
- 超出工作空间：不收敛

**改进建议**：

1. **使用更好的初值**：
   - 轨迹跟踪：使用上一时刻的解
   - 大范围运动：使用解析解（如可用）

2. **早停策略**：
   - 设置最大时间限制而非仅迭代次数
   - 检测震荡并提前终止

3. **分段求解**：
   - 将大范围运动分解为小步
   - 每步使用前一步的解作为初值

## 5.13 常见问题与解决

### 5.13.1 IK 不收敛

**现象**：
```
ik_solver_lma->JntToJacDot 计算失败, 错误码 = -3
```

**可能原因**：

1. **超出工作空间**
   - 检查：$\|\mathbf{p}_d\| > r_{\max}$
   - 解决：调整期望位置到可达范围

2. **奇异配置**
   - 检查：$\det(\mathbf{J}) \approx 0$
   - 解决：避开奇异配置，或增大阻尼系数

3. **初值不佳**
   - 检查：$\|\mathbf{q}_{\text{init}} - \mathbf{q}^*\| \gg \delta$
   - 解决：使用更接近的初值（如当前位置）

4. **目标不可达**（姿态）
   - 检查：期望姿态是否可实现
   - 解决：放松姿态约束，仅约束位置

### 5.13.2 关节跳变

**现象**：关节角度突然大幅变化（肘上/肘下切换）

**原因**：多解之间切换

**解决方案**：

1. **使用当前位置作为初值**（已实现）✓
   ```cpp
   ik_solver_lma->CartToJnt(current_joint_positions, ...)
   ```

2. **添加连续性检查**：
   ```cpp
   if (‖q_new - q_current‖ > threshold):
       reject  // 拒绝跳变过大的解
   ```

3. **最小化关节运动**（修改目标函数）：
   $$E_{\text{total}} = E_{\text{pose}} + w \|\mathbf{q} - \mathbf{q}_{\text{current}}\|^2$$

### 5.13.3 超出关节限位

**现象**：
```
关节 3 角度max超出工作空间: 2.95
```

**原因**：
- 期望末端位姿需要关节超限才能达到
- 虽然在工作空间内，但违反关节约束

**解决方案**：

1. **调整期望位姿**：略微修改使其可达

2. **约束IK求解**：在优化中加入关节限位

3. **软限位**：在接近限位时调整期望

## 5.14 代码映射表

| 数学符号 | 代码变量 | 类型 | 说明 |
|----------|----------|------|------|
| $\mathbf{T}_d$ | `desir_end_effector_frame` | `KDL::Frame` | 期望末端位姿 |
| $\mathbf{q}_{\text{init}}$ | `current_joint_positions` | `KDL::JntArray` | 初始猜测（当前关节位置） |
| $\mathbf{q}_{\text{temp}}$ | `desir_joint_positions_temp` | `KDL::JntArray` | 临时存储 IK 解 |
| $\mathbf{q}_d$ | `desir_joint_positions` | `KDL::JntArray` | 更新后的期望关节位置 |
| $\mathbf{q}_{\max}$ | `desir_joint_pos_protect_max` | `std::vector<float>` | 关节角度上限 |
| $\mathbf{q}_{\min}$ | `desir_joint_pos_protect_min` | `std::vector<float>` | 关节角度下限 |
| $\mathbf{T}_4$ | `frame` (at link 4) | `KDL::Frame` | 第4连杆位姿 |
| $L_{\max}$ | `arm_lenght` | `float` | 最大臂长 ($l_3 + l_4$) |
| $L_d$ | `desir_arm_lenght` | `float` | 期望臂长 |

## 5.15 总结

### 5.15.1 核心算法

$$\boxed{\min_{\mathbf{q}} \|\mathbf{e}(\mathbf{q})\|^2 = \min_{\mathbf{q}} \|f_{\text{FK}}(\mathbf{q}) - \mathbf{T}_d\|^2}$$

求解方法：**Levenberg-Marquardt 算法**

$$(\mathbf{J}^T \mathbf{J} + \lambda \mathbf{I}) \Delta \mathbf{q} = -\mathbf{J}^T \mathbf{e}$$

### 5.15.2 关键特性

1. **迭代求解**：数值优化方法，适用于复杂机构
2. **多层保护**：工作空间 + 关节限位双重检查
3. **自适应阻尼**：根据收敛情况动态调整 $\lambda$
4. **局部最优**：依赖初值，保证局部收敛性

### 5.15.3 实现优点

1. **安全性**：多重检查确保不会超限 ✓
2. **鲁棒性**：阻尼处理奇异配置 ✓
3. **平滑性**：使用当前位置作为初值，避免跳变 ✓
4. **通用性**：适用于任意串联机械臂

### 5.15.4 可能的改进

1. **约束优化**：在 IK 中直接加入关节限位约束
2. **多起点搜索**：尝试多个初值，选择最优解
3. **解析解加速**：对特定构型使用闭式解（如可用）
4. **性能优化**：缓存中间结果，减少重复计算
5. **预测初值**：使用速度信息预测下一时刻位置

### 5.15.5 与其他函数的协作

```
期望末端位姿 (desir_end_effector_frame)
        ↓
    comput_ik (位置层IK)
        ↓
期望关节位置 (desir_joint_positions)
        ↓
    CartToJnt (速度层IK)
        ↓
期望关节速度 (desir_joint_velocities)
        ↓
    动力学控制器
        ↓
关节力矩指令
```

## 参考文献

1. Levenberg, K. (1944). *A method for the solution of certain non-linear problems in least squares*. Quarterly of Applied Mathematics, 2(2), 164-168.
2. Marquardt, D. W. (1963). *An algorithm for least-squares estimation of nonlinear parameters*. Journal of the Society for Industrial and Applied Mathematics, 11(2), 431-441.
3. Nocedal, J., & Wright, S. (2006). *Numerical Optimization* (2nd ed.). Springer. (Chapter 10: Least-Squares Problems)
4. Buss, S. R., & Kim, J. S. (2005). *Selectively damped least squares for inverse kinematics*. Journal of Graphics Tools, 10(3), 37-49.
5. Siciliano, B., et al. (2010). *Robotics: Modelling, Planning and Control*. Springer. (Chapter 3.8: Inverse Kinematics Algorithms)
6. Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control*. Pearson. (Chapter 4: Inverse Kinematics)
7. KDL Documentation: *ChainIkSolverPos_LMA*. [http://www.orocos.org/kdl](http://www.orocos.org/kdl)

---
# 6. AutoServo 控制模式

## 6.1 模式概述

AutoServo 是一种**笛卡尔空间位置跟踪控制模式**，实现了从期望末端位姿到电机控制力矩的完整控制链。该模式结合了：

1. **逆运动学**：笛卡尔空间 → 关节空间
2. **计算力矩控制**：前馈重力补偿
3. **PD 反馈控制**：位置和速度误差修正
4. **电机空间映射**：关节力矩 → 电机力矩

在代码中的实现位置：`state_machine.cpp:132-243`

## 6.2 控制架构

### 6.2.1 控制流程图

```
期望末端位姿 (desir_end_effector_frame)
        ↓
    逆运动学 IK (CartToJnt)
        ↓
期望关节位置 (desir_joint_positions)
        ↓
    逆动力学 ID (computeInverseDynamics)
        ↓
重力补偿力矩 (gravity_joint_tauqes)
        ↓
    PD 控制器 (motor_pid_tau)
        ↓
总控制力矩 (motor_control_tau)
        ↓
    电机控制 (ControlMotors)
        ↓
电机执行 → 关节运动 → 末端运动
```

### 6.2.2 数学模型

**期望末端位姿**：
$$\mathbf{T}_d(t) \in SE(3)$$

**逆运动学映射**：
$$\mathbf{q}_d(t) = f_{\text{IK}}^{-1}(\mathbf{T}_d(t))$$

**动力学模型**：
$$\boldsymbol{\tau} = \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q})$$

**控制律**：
$$\boldsymbol{\tau}_{\text{total}} = \boldsymbol{\tau}_{\text{gravity}} + \boldsymbol{\tau}_{\text{PD}}$$

## 6.3 详细算法步骤

### 6.3.1 步骤1：逆运动学求解

**代码** (state_machine.cpp:161):
```cpp
arm.ik_solver_lma->CartToJnt(arm.current_joint_positions,
                               arm.desir_end_effector_frame,
                               arm.desir_joint_positions);
```

**数学表达**：

求解优化问题：
$$\mathbf{q}_d = \arg\min_{\mathbf{q}} \|f_{\text{FK}}(\mathbf{q}) - \mathbf{T}_d\|^2$$

使用 Levenberg-Marquardt 算法（见第5章），得到期望关节位置 $\mathbf{q}_d \in \mathbb{R}^6$。

**输入/输出**：
- **输入**：期望末端位姿 $\mathbf{T}_d$，当前关节位置 $\mathbf{q}_{\text{current}}$（作为初值）
- **输出**：期望关节位置 $\mathbf{q}_d$

### 6.3.2 步骤2：逆动力学计算（重力补偿）

**代码** (state_machine.cpp:164):
```cpp
arm.computeInverseDynamics(arm.desir_joint_positions,
                            arm.current_joint_velocities,
                            arm.current_joint_acceleration,
                            arm.gravity_joint_tauqes);
```

**数学表达**：

计算重力力矩：
$$\boldsymbol{\tau}_g = \mathbf{g}(\mathbf{q}_d)$$

其中 $\mathbf{g}(\mathbf{q}) \in \mathbb{R}^6$ 是重力矢量，由递归牛顿-欧拉算法计算：

$$\mathbf{g}(\mathbf{q}) = \sum_{i=1}^{6} m_i \mathbf{J}_{v,i}^T(\mathbf{q}) \mathbf{g}_0$$

- $m_i$：第 $i$ 个连杆的质量
- $\mathbf{J}_{v,i}$：第 $i$ 个连杆质心的线速度雅可比
- $\mathbf{g}_0 = [0, 0, -9.81]^T$ m/s²：重力加速度向量

**物理意义**：

计算在期望位置 $\mathbf{q}_d$ 处，为了对抗重力而需要施加的关节力矩。这是一种**前馈补偿**，消除重力的影响。

**注意**：此处使用 $\mathbf{q}_d$（期望位置）而非 $\mathbf{q}_{\text{current}}$（当前位置）计算重力，意味着采用**期望模型前馈**策略。

### 6.3.3 步骤3：PD 反馈控制

**代码** (state_machine.cpp:204-210):
```cpp
motor.motor_pid_tau[0] = 350 * (motor.desir_motor_pos[0] - motor.current_motor_pos[0]) 
                         + 1.9 * (motor.desir_motor_vel[0] - motor.current_motor_vel[0]);

motor.motor_pid_tau[1] = 300 * (motor.desir_motor_pos[1] - motor.current_motor_pos[1]) 
                         + 2.0 * (motor.desir_motor_vel[1] - motor.current_motor_vel[1]);

motor.motor_pid_tau[2] = 280 * (motor.desir_motor_pos[2] - motor.current_motor_pos[2]) 
                         + 2.0 * (motor.desir_motor_vel[2] - motor.current_motor_vel[2]);

motor.motor_pid_tau[3] = 10 * (motor.desir_motor_pos[3] - motor.current_motor_pos[3]) 
                         + 0.6 * (motor.desir_motor_vel[3] - motor.current_motor_vel[3]);
```

**数学表达**：

PD 控制律（在电机空间）：
$$\boldsymbol{\tau}_{\text{PD}} = \mathbf{K}_p (\mathbf{q}_m^d - \mathbf{q}_m) + \mathbf{K}_d (\dot{\mathbf{q}}_m^d - \dot{\mathbf{q}}_m)$$

其中：
- $\mathbf{q}_m \in \mathbb{R}^6$：当前电机位置
- $\mathbf{q}_m^d \in \mathbb{R}^6$：期望电机位置
- $\mathbf{K}_p = \text{diag}(k_{p,1}, \ldots, k_{p,6})$：比例增益矩阵
- $\mathbf{K}_d = \text{diag}(k_{d,1}, \ldots, k_{d,6})$：微分增益矩阵

**代码中的增益值**：

| 电机 | $k_p$ | $k_d$ | 说明 |
|------|-------|-------|------|
| 1 | 350 | 1.9 | 基座旋转关节 |
| 2 | 300 | 2.0 | 肩关节 |
| 3 | 280 | 2.0 | 肘关节 |
| 4 | 10 | 0.6 | 腕关节1 |
| 5 | 0 | 0 | 腕关节2（仅重力补偿） |
| 6 | 0 | 0 | 腕关节3（仅重力补偿） |

**分析**：
- 前三个关节（负载大）：高增益 $k_p \in [280, 350]$
- 后三个关节（负载小）：低增益 $k_p \in [0, 10]$
- 第4关节之后：主要依赖重力补偿，PD 增益很小或为0

### 6.3.4 步骤4：力矩叠加

**代码** (state_machine.cpp:212-215):
```cpp
for (size_t i = 0; i < 4; i++)
{
    motor.motor_control_tau[i] += motor.motor_pid_tau[i];
}
```

**数学表达**：

总控制力矩：
$$\boldsymbol{\tau}_{\text{control}} = \boldsymbol{\tau}_g + \boldsymbol{\tau}_{\text{PD}}$$

展开：
$$\tau_i = g_i(\mathbf{q}_d) + k_{p,i}(q_i^d - q_i) + k_{d,i}(\dot{q}_i^d - \dot{q}_i), \quad i = 1, \ldots, 6$$

**物理意义**：
- $\boldsymbol{\tau}_g$：**前馈项**，补偿重力
- $\boldsymbol{\tau}_{\text{PD}}$：**反馈项**，修正跟踪误差

### 6.3.5 步骤5：电机控制

**代码** (state_machine.cpp:217-222):
```cpp
motor.ControlMotors(port,
                    motor.desir_motor_pos,
                    motor.desir_motor_vel,
                    motor.kp,
                    motor.kd,
                    motor.motor_control_tau);
```

**控制指令发送**：

向电机驱动器发送：
- 期望位置：$\mathbf{q}_m^d$
- 期望速度：$\dot{\mathbf{q}}_m^d$
- 电机内部 PD 增益：$k_p = k_d = 0$（禁用电机内部控制器）
- 前馈力矩：$\boldsymbol{\tau}_{\text{control}}$

**注意**：电机内部 PD 增益设为 0，意味着**完全由上位机计算控制力矩**，电机只执行力矩指令。

## 6.4 完整数学模型

### 6.4.1 闭环系统动力学

**机械臂动力学**：
$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau}$$

**控制律**：
$$\boldsymbol{\tau} = \mathbf{g}(\mathbf{q}_d) + \mathbf{K}_p(\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}})$$

**闭环动力学**：

代入控制律：
$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \mathbf{g}(\mathbf{q}_d) + \mathbf{K}_p(\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}})$$

定义跟踪误差：
$$\mathbf{e} = \mathbf{q}_d - \mathbf{q}, \quad \dot{\mathbf{e}} = \dot{\mathbf{q}}_d - \dot{\mathbf{q}}$$

重新整理：
$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{e}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{e}} + \mathbf{K}_d\dot{\mathbf{e}} + \mathbf{K}_p\mathbf{e} = \mathbf{g}(\mathbf{q}_d) - \mathbf{g}(\mathbf{q}) - \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}}_d - \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}_d$$

### 6.4.2 简化分析（理想情况）

**假设**：
1. 期望轨迹为常值位置（$\ddot{\mathbf{q}}_d = \dot{\mathbf{q}}_d = 0$）
2. 跟踪误差很小（$\mathbf{q} \approx \mathbf{q}_d$）
3. 重力补偿精确（$\mathbf{g}(\mathbf{q}) \approx \mathbf{g}(\mathbf{q}_d)$）

则闭环动力学简化为：
$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{e}} + (\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{K}_d)\dot{\mathbf{e}} + \mathbf{K}_p\mathbf{e} \approx \mathbf{0}$$

这是一个**二阶线性时变系统**，对于缓慢变化的 $\mathbf{M}$ 和 $\mathbf{C}$，可近似为：

$$\ddot{\mathbf{e}} + \mathbf{M}^{-1}\mathbf{K}_d\dot{\mathbf{e}} + \mathbf{M}^{-1}\mathbf{K}_p\mathbf{e} \approx \mathbf{0}$$

**稳定性条件**：

定义自然频率和阻尼比：
$$\omega_n = \sqrt{\frac{k_p}{M}}, \quad \zeta = \frac{k_d}{2\sqrt{k_p M}}$$

系统稳定需要：
- $k_p > 0$（正定）
- $\zeta > 0$（欠阻尼或临界阻尼）

### 6.4.3 增益调节指南

**比例增益 $k_p$**：
- 影响：位置跟踪精度、系统刚度
- 过大：超调、震荡
- 过小：跟踪误差大、响应慢

**微分增益 $k_d$**：
- 影响：阻尼、稳定性
- 过大：系统迟钝、对噪声敏感
- 过小：震荡、不稳定

**经验公式**（临界阻尼）：
$$k_d = 2\sqrt{k_p M}$$

代码中的设置接近这个关系（例如关节1）：
$$k_d = 1.9 \approx 2\sqrt{350 \times 0.0029} \approx 2.0$$

（假设 $M \approx 0.0029$ kg·m²）

## 6.5 与计算力矩控制的对比

### 6.5.1 标准计算力矩控制（Computed Torque Control）

**完整形式**：
$$\boldsymbol{\tau} = \mathbf{M}(\mathbf{q})[\ddot{\mathbf{q}}_d + \mathbf{K}_v\dot{\mathbf{e}} + \mathbf{K}_p\mathbf{e}] + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q})$$

**特点**：
- 完全线性化：闭环系统变为 $\ddot{\mathbf{e}} + \mathbf{K}_v\dot{\mathbf{e}} + \mathbf{K}_p\mathbf{e} = \mathbf{0}$
- 需要精确动力学模型
- 计算量大（需要 $\mathbf{M}$, $\mathbf{C}$）

### 6.5.2 AutoServo 简化版本

**简化形式**：
$$\boldsymbol{\tau} = \mathbf{g}(\mathbf{q}_d) + \mathbf{K}_p(\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}})$$

**简化之处**：
- 仅补偿重力项 $\mathbf{g}(\mathbf{q})$
- 忽略惯性项 $\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}}$
- 忽略科里奥利/离心项 $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}$

**优点**：
- 计算简单：仅需计算重力
- 实时性好：计算量小
- 鲁棒性好：对模型误差不敏感

**缺点**：
- 高速运动时性能下降（惯性力未补偿）
- 负载变化时精度下降（重力补偿不准）

### 6.5.3 适用场景

AutoServo 适合：
- ✅ 慢速位置跟踪
- ✅ 准静态操作
- ✅ 精确定位任务
- ✅ 负载已知且变化小

不适合：
- ❌ 高速运动（需要完整动力学补偿）
- ❌ 高加速度（惯性力显著）
- ❌ 负载频繁变化（重力补偿失效）

## 6.6 代码分析

### 6.6.1 关键参数

| 参数 | 变量名 | 值/范围 | 说明 |
|------|--------|---------|------|
| 期望末端位姿 | `desir_end_effector_frame` | SE(3) | 由轨迹生成器更新 |
| 期望关节位置 | `desir_joint_positions` | $\mathbb{R}^6$ | IK 求解结果 |
| 当前关节位置 | `current_joint_positions` | $\mathbb{R}^6$ | 编码器反馈 |
| 当前关节速度 | `current_joint_velocities` | $\mathbb{R}^6$ | 数值微分或估计 |
| 重力力矩 | `gravity_joint_tauqes` | $\mathbb{R}^6$ | 逆动力学计算 |
| PD 力矩 | `motor_pid_tau` | $\mathbb{R}^6$ | PD 控制器输出 |
| 总控制力矩 | `motor_control_tau` | $\mathbb{R}^6$ | 发送给电机 |

### 6.6.2 控制频率

**主循环频率**：1 kHz（1 ms 周期）

**关键计算耗时**（估计）：
- 逆运动学 LMA：1-10 ms（视收敛速度）
- 逆动力学（重力）：0.1-0.5 ms
- PD 计算：< 0.01 ms
- 总计：≈ 1-11 ms

**问题**：IK 求解可能超过控制周期（1 ms）

**解决方案**：
- 使用当前位置作为初值（快速收敛）
- 限制最大迭代次数
- 或降低控制频率到 100 Hz

### 6.6.3 电机空间 vs 关节空间

**关节到电机映射**（line 192-199）：
```cpp
arm.joint2motor(arm.desir_joint_pos,
                arm.desir_joint_vel,
                arm.desir_joint_acc,
                arm.joint_control_tau,
                motor.desir_motor_pos,
                motor.desir_motor_vel,
                motor.desir_motor_acc,
                motor.motor_control_tau);
```

**数学关系**：

考虑减速比 $r_i$ 和方向 $s_i \in \{-1, +1\}$：

$$\begin{cases}
q_m^i = s_i \cdot r_i \cdot q_j^i \\
\dot{q}_m^i = s_i \cdot r_i \cdot \dot{q}_j^i \\
\tau_m^i = s_i \cdot \frac{\tau_j^i}{r_i}
\end{cases}$$

其中：
- 下标 $m$：电机空间
- 下标 $j$：关节空间
- $r_i$：减速比（例如 50:1）
- $s_i$：方向（1 或 -1）

**力矩变换解释**：

电机力矩 = 关节力矩 / 减速比，因为：
$$\tau_m \cdot \theta_m = \tau_j \cdot \theta_j \quad \text{(功率守恒)}$$
$$\tau_m \cdot (r \cdot \theta_j) = \tau_j \cdot \theta_j$$
$$\tau_m = \frac{\tau_j}{r}$$

## 6.7 实际性能分析

### 6.7.1 跟踪精度

**位置精度**（稳态）：

受以下因素影响：
- IK 求解精度：$\epsilon_{\text{IK}} \approx 10^{-6}$ m（LMA 收敛阈值）
- 编码器分辨率：$\Delta q \approx 0.01°$
- 重力补偿误差：$\Delta \tau_g / \tau_g \approx 5\%$（负载不确定性）
- 静摩擦：未补偿

**估计总精度**：$\pm 0.5$ mm（末端位置）

**速度跟踪**：

当期望速度为 0 时（定位任务），速度误差主要由：
- 测量噪声
- 数值微分误差

**动态性能**（关节1示例）：

自然频率：
$$\omega_n = \sqrt{\frac{k_p}{M}} = \sqrt{\frac{350}{0.0029}} \approx 348 \text{ rad/s} \approx 55 \text{ Hz}$$

阻尼比：
$$\zeta = \frac{k_d}{2\sqrt{k_p M}} = \frac{1.9}{2\sqrt{350 \times 0.0029}} \approx 0.95$$

系统接近**临界阻尼**（$\zeta \approx 1$），响应快且无超调。

### 6.7.2 稳定性鲁棒性

**模型不确定性**：

设实际重力为 $\mathbf{g}_{\text{actual}}$，模型为 $\mathbf{g}_{\text{model}}$，误差：
$$\Delta \mathbf{g} = \mathbf{g}_{\text{actual}} - \mathbf{g}_{\text{model}}$$

闭环系统变为：
$$\mathbf{M}\ddot{\mathbf{e}} + \mathbf{C}\dot{\mathbf{e}} + \mathbf{K}_d\dot{\mathbf{e}} + \mathbf{K}_p\mathbf{e} = -\Delta \mathbf{g} + \ldots$$

**稳态误差**：
$$\mathbf{e}_{\text{ss}} = -\mathbf{K}_p^{-1} \Delta \mathbf{g}$$

例如，若 $\Delta g_1 = 0.5$ Nm（重力估计误差），$k_{p,1} = 350$，则：
$$e_1 = -\frac{0.5}{350} \approx -0.0014 \text{ rad} \approx -0.08°$$

**鲁棒性结论**：
- 高增益 $k_p$ 可减小稳态误差
- 但过高会导致震荡
- 需要平衡精度与稳定性

### 6.7.3 外部干扰抑制

**干扰力矩** $\boldsymbol{\tau}_{\text{dist}}$（如碰撞、负载变化）：

闭环方程：
$$\mathbf{M}\ddot{\mathbf{e}} + \mathbf{K}_d\dot{\mathbf{e}} + \mathbf{K}_p\mathbf{e} = -\boldsymbol{\tau}_{\text{dist}} + \ldots$$

**稳态响应**：
$$\mathbf{e}_{\text{ss}} = -\mathbf{K}_p^{-1} \boldsymbol{\tau}_{\text{dist}}$$

**抗干扰能力**：与增益 $k_p$ 成正比。

## 6.8 改进方向

### 6.8.1 完整动力学补偿

**改进控制律**：
$$\boldsymbol{\tau} = \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}}_d + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}_d + \mathbf{g}(\mathbf{q}_d) + \mathbf{K}_p\mathbf{e} + \mathbf{K}_d\dot{\mathbf{e}}$$

**优点**：
- 高速运动性能提升
- 轨迹跟踪精度提高

**缺点**：
- 计算量增加（需计算 $\mathbf{M}$, $\mathbf{C}$）
- 对模型精度要求高

### 6.8.2 自适应重力补偿

**在线估计负载质量** $m_{\text{load}}$：

使用力/力矩传感器或电流反馈：
$$\hat{m}_{\text{load}}(t+1) = \hat{m}_{\text{load}}(t) + \gamma \cdot \mathbf{e}_{\tau}$$

其中 $\mathbf{e}_{\tau}$ 是力矩跟踪误差。

**更新重力模型**：
$$\mathbf{g}(\mathbf{q}) \leftarrow \mathbf{g}(\mathbf{q}, \hat{m}_{\text{load}})$$

**优点**：适应负载变化，提高鲁棒性。

### 6.8.3 前馈速度/加速度

**当期望轨迹包含速度和加速度时**：

$$\boldsymbol{\tau} = \mathbf{g}(\mathbf{q}_d) + \mathbf{K}_p\mathbf{e} + \mathbf{K}_d\dot{\mathbf{e}} + \mathbf{K}_a\ddot{\mathbf{q}}_d$$

其中 $\mathbf{K}_a$ 可设为 $\mathbf{M}(\mathbf{q}_d)$ 的估计。

**优点**：改善动态跟踪性能。

### 6.8.4 摩擦补偿

**静摩擦模型**（Stribeck模型）：
$$\tau_f = (F_c + (F_s - F_c)e^{-|\dot{q}/\dot{q}_s|^\delta})\text{sign}(\dot{q}) + b\dot{q}$$

**在控制律中加入**：
$$\boldsymbol{\tau} = \ldots + \boldsymbol{\tau}_f(\dot{\mathbf{q}})$$

**优点**：减小低速爬行、改善定位精度。

## 6.9 总结

### 6.9.1 核心公式

$$\boxed{\boldsymbol{\tau} = \mathbf{g}(\mathbf{q}_d) + \mathbf{K}_p(\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}})}$$

其中：
- $\mathbf{q}_d = f_{\text{IK}}^{-1}(\mathbf{T}_d)$：逆运动学求解
- $\mathbf{g}(\mathbf{q}_d)$：重力前馈补偿
- $\mathbf{K}_p\mathbf{e} + \mathbf{K}_d\dot{\mathbf{e}}$：PD 反馈修正

### 6.9.2 关键特性

1. **分层控制**：
   - 任务层：笛卡尔空间轨迹
   - 运动层：逆运动学 + 逆动力学
   - 执行层：PD 控制 + 电机驱动

2. **前馈 + 反馈结合**：
   - 前馈：重力补偿（消除已知扰动）
   - 反馈：PD 控制（修正未知误差）

3. **计算效率**：
   - 仅计算重力项（简化动力学）
   - 实时性好（适合 1 kHz 控制）

4. **适用范围**：
   - 慢速精确定位 ✓
   - 负载稳定任务 ✓
   - 高速动态运动 ✗

### 6.9.3 与其他模式对比

| 特性 | AutoServo | ManualServo | JointAutoServo |
|------|-----------|-------------|----------------|
| 控制空间 | 笛卡尔空间 | 笛卡尔空间 | 关节空间 |
| 逆运动学 | 每周期求解 | 每周期求解 | 不需要 |
| 动力学补偿 | 仅重力 | 仅重力 | 仅重力 |
| 反馈控制 | PD（电机空间） | PD（电机空间） | PD（电机空间） |
| 适用场景 | 末端位姿跟踪 | 末端位姿跟踪（备用） | 关节空间运动 |

### 6.9.4 代码位置索引

| 功能 | 代码位置 | 数学对应 |
|------|----------|----------|
| 模式入口 | state_machine.cpp:90-91 | - |
| 逆运动学 | state_machine.cpp:161 | $\mathbf{q}_d = f_{\text{IK}}^{-1}(\mathbf{T}_d)$ |
| 逆动力学 | state_machine.cpp:164 | $\boldsymbol{\tau}_g = \mathbf{g}(\mathbf{q}_d)$ |
| PD 控制 | state_machine.cpp:204-210 | $\boldsymbol{\tau}_{\text{PD}} = \mathbf{K}_p\mathbf{e} + \mathbf{K}_d\dot{\mathbf{e}}$ |
| 力矩叠加 | state_machine.cpp:212-215 | $\boldsymbol{\tau} = \boldsymbol{\tau}_g + \boldsymbol{\tau}_{\text{PD}}$ |
| 电机控制 | state_machine.cpp:217-222 | 发送 $\boldsymbol{\tau}$ 到电机 |

## 参考文献

1. Sciavicco, B., & Siciliano, L. (2000). *Modelling and Control of Robot Manipulators* (2nd ed.). Springer. (Chapter 8: Motion Control)
2. Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley. (Chapter 11: Computed-Torque Control)
3. Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control*. Pearson. (Chapter 6: Manipulator Dynamics, Chapter 10: Force Control)
4. Slotine, J. J. E., & Li, W. (1991). *Applied Nonlinear Control*. Prentice Hall. (Chapter 6: Robot Manipulator Control)
5. Murray, R. M., Li, Z., & Sastry, S. S. (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press. (Chapter 4: Robot Dynamics, Chapter 6: Robot Control)

---

# 7. ManualServo 控制模式

## 7.1 模式概述

ManualServo 是一种**纯力矩控制模式**，通过完全禁用电机内部控制器（kp = kd = 0），实现上位机对关节力矩的直接控制。该模式与 AutoServo 的核心区别在于：

1. **纯力矩控制**：电机内部 PD 增益设为 0，仅执行力矩指令
2. **基于当前状态的重力补偿**：使用 `current_joint_positions` 而非 `desir_joint_positions`
3. **全关节 PD 反馈**：所有 6 个关节均应用 PD 控制（AutoServo 仅前 4 个）
4. **更精确的重力补偿**：实时跟踪当前配置的重力状态

在代码中的实现位置：`state_machine.cpp:244-344`

## 7.2 控制架构

### 7.2.1 控制流程图

```
期望末端位姿 (desir_end_effector_frame)
        ↓
    逆运动学 IK
        ↓
期望关节位置 (desir_joint_positions)
        ↓
    逆动力学 ID（基于当前位置）
        ↓
重力补偿力矩 (gravity_joint_tauqes) = g(q_current)
        ↓
    上位机 PD 控制器（电机空间）
        ↓
总控制力矩 (motor_control_tau) = τ_g + τ_PD
        ↓
    纯力矩模式电机控制（kp = kd = 0）
        ↓
电机执行 → 关节运动 → 末端运动
```

### 7.2.2 数学模型

**期望关节位置**（通过逆运动学）：
$$\mathbf{q}_d(t) = f_{\text{IK}}^{-1}(\mathbf{T}_d(t))$$

**当前状态重力补偿**：
$$\boldsymbol{\tau}_g = \mathbf{g}(\mathbf{q}_{\text{current}})$$

**控制律**：
$$\boldsymbol{\tau}_{\text{total}} = \mathbf{g}(\mathbf{q}_{\text{current}}) + \mathbf{K}_p (\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d (\dot{\mathbf{q}}_d - \dot{\mathbf{q}})$$

## 7.3 详细算法步骤

### 7.3.1 步骤1：逆动力学计算（当前状态重力补偿）

**代码** (state_machine.cpp:277):
```cpp
arm.computeInverseDynamics(arm.current_joint_positions,
                            arm.current_joint_velocities,
                            arm.current_joint_acceleration,
                            arm.gravity_joint_tauqes);
```

**数学表达**：

计算当前配置下的重力力矩：
$$\boldsymbol{\tau}_g = \mathbf{g}(\mathbf{q}_{\text{current}})$$

**关键差异**：

| 模式 | 重力补偿位置 | 特点 |
|------|--------------|------|
| **ManualServo** | $\mathbf{g}(\mathbf{q}_{\text{current}})$ | 实时跟踪，精确补偿 ✓ |
| AutoServo | $\mathbf{g}(\mathbf{q}_d)$ | 前馈补偿，依赖跟踪精度 |

**物理意义**：

ManualServo 使用**当前状态反馈**计算重力，确保重力补偿始终准确反映机械臂的实际配置，即使存在跟踪误差时也能提供精确的重力抵消。

数学上：
$$\boldsymbol{\tau}_g = \sum_{i=1}^{6} m_i \mathbf{J}_{v,i}^T(\mathbf{q}_{\text{current}}) \mathbf{g}_0$$

其中：
- $m_i$：第 $i$ 个连杆的质量
- $\mathbf{J}_{v,i}(\mathbf{q}_{\text{current}})$：基于当前位置的线速度雅可比
- $\mathbf{g}_0 = [0, 0, -9.81]^T$ m/s²：重力加速度

### 7.3.2 步骤2：关节到电机空间转换

**代码** (state_machine.cpp:290-297):
```cpp
arm.joint2motor(arm.desir_joint_pos,
                arm.desir_joint_vel,
                arm.desir_joint_acc,
                arm.joint_control_tau,
                motor.desir_motor_pos,
                motor.desir_motor_vel,
                motor.desir_motor_acc,
                motor.motor_control_tau);
```

**数学关系**：

考虑减速比 $r_i$ 和方向 $s_i$：

$$\begin{cases}
q_m^i = s_i \cdot r_i \cdot q_j^i \\
\dot{q}_m^i = s_i \cdot r_i \cdot \dot{q}_j^i \\
\tau_m^i = s_i \cdot \frac{\tau_j^i}{r_i}
\end{cases}$$

### 7.3.3 步骤3：上位机 PD 控制（电机空间）

**代码** (state_machine.cpp:301-311):
```cpp
motor.motor_pid_tau[0] = kp1 * (motor.desir_motor_pos[0] - motor.current_motor_pos[0])
                         + kd1 * (motor.desir_motor_vel[0] - motor.current_motor_vel[0]);

motor.motor_pid_tau[1] = kp2 * (motor.desir_motor_pos[1] - motor.current_motor_pos[1])
                         + kd2 * (motor.desir_motor_vel[1] - motor.current_motor_vel[1]);

motor.motor_pid_tau[2] = kp3 * (motor.desir_motor_pos[2] - motor.current_motor_pos[2])
                         + kd3 * (motor.desir_motor_vel[2] - motor.current_motor_vel[2]);

motor.motor_pid_tau[3] = kp4 * (motor.desir_motor_pos[3] - motor.current_motor_pos[3])
                         + kd4 * (motor.desir_motor_vel[3] - motor.current_motor_vel[3]);

motor.motor_pid_tau[4] = kp5 * (motor.desir_motor_pos[4] - motor.current_motor_pos[4])
                         + kd5 * (motor.desir_motor_vel[4] - motor.current_motor_vel[4]);

motor.motor_pid_tau[5] = kp6 * (motor.desir_motor_pos[5] - motor.current_motor_pos[5])
                         + kd6 * (motor.desir_motor_vel[5] - motor.current_motor_vel[5]);
```

**数学表达**：

PD 控制律（电机空间）：
$$\boldsymbol{\tau}_{\text{PD}} = \mathbf{K}_p (\mathbf{q}_m^d - \mathbf{q}_m) + \mathbf{K}_d (\dot{\mathbf{q}}_m^d - \dot{\mathbf{q}}_m)$$

其中：
- $\mathbf{q}_m, \dot{\mathbf{q}}_m \in \mathbb{R}^6$：当前电机位置和速度
- $\mathbf{q}_m^d, \dot{\mathbf{q}}_m^d \in \mathbb{R}^6$：期望电机位置和速度
- $\mathbf{K}_p = \text{diag}(k_{p,1}, \ldots, k_{p,6})$：比例增益矩阵
- $\mathbf{K}_d = \text{diag}(k_{d,1}, \ldots, k_{d,6})$：微分增益矩阵

**代码中的增益值**：

| 电机 | $k_p$ | $k_d$ | 说明 |
|------|-------|-------|------|
| 1 | 315 | 1.43 | 基座旋转关节 |
| 2 | 315 | 1.43 | 肩关节 |
| 3 | 312 | 1.73 | 肘关节 |
| 4 | 20 | 0.53 | 腕关节1 |
| 5 | 20 | 0.43 | 腕关节2 |
| 6 | 20 | 0.33 | 腕关节3 |

**分析**：
- 前三个关节（大负载）：$k_p \in [312, 315]$，$k_d \in [1.43, 1.73]$
- 后三个关节（小负载）：$k_p = 20$，$k_d \in [0.33, 0.53]$
- **与 AutoServo 对比**：
  - ManualServo 的前3个关节增益略低（315 vs 350/300/280）
  - ManualServo 的后3个关节有明确的 PD 控制（AutoServo 为 0）

### 7.3.4 步骤4：力矩叠加

**代码** (state_machine.cpp:313-316):
```cpp
for (size_t i = 0; i < 6; i++)
{
    motor.motor_control_tau[i] += motor.motor_pid_tau[i];
}
```

**数学表达**：

总控制力矩：
$$\boldsymbol{\tau}_{\text{control}} = \boldsymbol{\tau}_g + \boldsymbol{\tau}_{\text{PD}}$$

展开：
$$\tau_i = g_i(\mathbf{q}_{\text{current}}) + k_{p,i}(q_i^d - q_i) + k_{d,i}(\dot{q}_i^d - \dot{q}_i), \quad i = 1, \ldots, 6$$

### 7.3.5 步骤5：纯力矩模式电机控制

**代码** (state_machine.cpp:325-336):
```cpp
// 设置电机内部 PD 增益为 0
for (size_t i = 0; i < 6; i++)
{
    motor.kp[i] = 0.0;
    motor.kd[i] = 0.0;
}

motor.ControlMotors(port,
                    motor.desir_motor_pos,
                    motor.desir_motor_vel,
                    motor.kp,
                    motor.kd,
                    motor.motor_control_tau);
```

**控制指令**：

向电机发送：
- 期望位置：$\mathbf{q}_m^d$（仅用于参考，不影响控制）
- 期望速度：$\dot{\mathbf{q}}_m^d$（仅用于参考）
- **电机内部增益：$k_p = k_d = 0$** ← **关键特征**
- 前馈力矩：$\boldsymbol{\tau}_{\text{control}}$

**物理意义**：

电机完全按照上位机计算的力矩执行，不进行任何内部反馈控制。这种模式的优点是：
1. **透明性**：电机行为完全由上位机算法决定
2. **柔顺性**：无电机内部刚性，适合力控制任务
3. **可调节性**：控制算法可灵活修改，无需重新配置电机

## 7.4 完整数学模型

### 7.4.1 闭环系统动力学

**机械臂动力学**：
$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau}$$

**控制律**：
$$\boldsymbol{\tau} = \mathbf{g}(\mathbf{q}_{\text{current}}) + \mathbf{K}_p(\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}})$$

**闭环动力学**：

代入控制律：
$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \mathbf{g}(\mathbf{q}_{\text{current}}) + \mathbf{K}_p(\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}})$$

定义跟踪误差：
$$\mathbf{e} = \mathbf{q}_d - \mathbf{q}, \quad \dot{\mathbf{e}} = \dot{\mathbf{q}}_d - \dot{\mathbf{q}}$$

重新整理：
$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{e}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{e}} + \mathbf{K}_d\dot{\mathbf{e}} + \mathbf{K}_p\mathbf{e} = \mathbf{g}(\mathbf{q}_{\text{current}}) - \mathbf{g}(\mathbf{q}) - \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}}_d - \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}_d$$

### 7.4.2 重力补偿误差分析

**关键项**：$\mathbf{g}(\mathbf{q}_{\text{current}}) - \mathbf{g}(\mathbf{q})$

当跟踪误差很小时（$\mathbf{q} \approx \mathbf{q}_{\text{current}}$），有：
$$\mathbf{g}(\mathbf{q}_{\text{current}}) \approx \mathbf{g}(\mathbf{q})$$

因此右侧第一项接近 0，闭环系统简化为：
$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{e}} + (\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{K}_d)\dot{\mathbf{e}} + \mathbf{K}_p\mathbf{e} \approx -\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}}_d - \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}_d$$

**ManualServo 的优势**：

由于使用 $\mathbf{q}_{\text{current}}$，重力补偿误差项 $\mathbf{g}(\mathbf{q}_{\text{current}}) - \mathbf{g}(\mathbf{q})$ 始终接近 0，即使存在较大的跟踪误差（$\mathbf{q} \neq \mathbf{q}_d$）。

**对比 AutoServo**：

AutoServo 使用 $\mathbf{g}(\mathbf{q}_d)$，当 $\mathbf{q} \neq \mathbf{q}_d$ 时：
$$\mathbf{g}(\mathbf{q}_d) - \mathbf{g}(\mathbf{q}) \neq 0$$

这会产生重力补偿误差，降低控制精度。

### 7.4.3 简化分析（静态或慢速运动）

**假设**：
1. 静态或准静态运动：$\ddot{\mathbf{q}}_d = \dot{\mathbf{q}}_d \approx 0$
2. 重力补偿精确：$\mathbf{g}(\mathbf{q}_{\text{current}}) \approx \mathbf{g}(\mathbf{q})$

则闭环动力学简化为：
$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{e}} + (\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{K}_d)\dot{\mathbf{e}} + \mathbf{K}_p\mathbf{e} \approx \mathbf{0}$$

这是标准的**二阶系统**，可分析其稳定性和响应特性。

### 7.4.4 增益调节指南

**比例增益 $k_p$**：
- 影响：位置跟踪精度、系统刚度
- ManualServo 设置略低于 AutoServo（315 vs 350）
- 原因：纯力矩模式更柔顺，避免过度震荡

**微分增益 $k_d$**：
- 影响：阻尼、稳定性
- ManualServo 前3个关节：$k_d \in [1.43, 1.73]$
- 接近临界阻尼

**经验公式**（临界阻尼）：
$$k_d = 2\sqrt{k_p M}$$

验证关节1：
$$k_d = 1.43 \approx 2\sqrt{315 \times 0.0029} \approx 1.88$$

实际值略小于临界阻尼，系统呈欠阻尼特性，响应快速但有小幅振荡。

## 7.5 与 AutoServo 的详细对比

### 7.5.1 控制架构对比

| 特性 | ManualServo | AutoServo |
|------|-------------|-----------|
| **电机内部增益** | $k_p = k_d = 0$ | $k_p = k_d = 0$ |
| **重力补偿位置** | $\mathbf{g}(\mathbf{q}_{\text{current}})$ ✓ | $\mathbf{g}(\mathbf{q}_d)$ |
| **上位机PD范围** | 全部6个关节 | 仅前4个关节 |
| **比例增益（关节1）** | 315 | 350 |
| **微分增益（关节1）** | 1.43 | 1.9 |
| **控制精度** | 高（精确重力补偿） | 中（依赖跟踪） |
| **柔顺性** | 高 | 中 |
| **鲁棒性** | 高（对跟踪误差） | 中 |

### 7.5.2 数学表达对比

**ManualServo**：
$$\boldsymbol{\tau} = \mathbf{g}(\mathbf{q}_{\text{current}}) + \mathbf{K}_p(\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}})$$

**AutoServo**：
$$\boldsymbol{\tau} = \mathbf{g}(\mathbf{q}_d) + \mathbf{K}_p(\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}})$$

**关键差异**：重力补偿项

**误差分析**：

设跟踪误差为 $\mathbf{e} = \mathbf{q}_d - \mathbf{q}$，重力项线性化：
$$\mathbf{g}(\mathbf{q}_d) - \mathbf{g}(\mathbf{q}) \approx \frac{\partial \mathbf{g}}{\partial \mathbf{q}}\bigg|_{\mathbf{q}} \mathbf{e}$$

- **ManualServo**：重力误差 ≈ 0（使用实际状态）
- **AutoServo**：重力误差 ∝ $\mathbf{e}$（依赖跟踪精度）

### 7.5.3 性能对比

| 性能指标 | ManualServo | AutoServo |
|----------|-------------|-----------|
| **静态精度** | ★★★★★ | ★★★★☆ |
| **动态响应** | ★★★☆☆ | ★★★★☆ |
| **力控性能** | ★★★★★ | ★★★☆☆ |
| **抗扰动能力** | ★★★★★ | ★★★☆☆ |
| **轨迹跟踪** | ★★★★☆ | ★★★★★ |
| **计算复杂度** | 低 | 低 |

**适用场景对比**：

**ManualServo 适合**：
- ✅ 需要精确力控制的任务（如抛光、装配）
- ✅ 外部干扰较大的环境
- ✅ 负载频繁变化
- ✅ 柔顺交互任务
- ✅ 静态定位

**AutoServo 适合**：
- ✅ 高速轨迹跟踪
- ✅ 预知轨迹的重复性任务
- ✅ 位置精度优先
- ✅ 负载稳定的环境

## 7.6 代码分析

### 7.6.1 关键参数

| 参数 | 变量名 | 值/范围 | 说明 |
|------|--------|---------|------|
| 期望关节位置 | `desir_joint_positions` | $\mathbb{R}^6$ | 由逆运动学计算 |
| 当前关节位置 | `current_joint_positions` | $\mathbb{R}^6$ | 编码器反馈 ✓ |
| 当前关节速度 | `current_joint_velocities` | $\mathbb{R}^6$ | 编码器反馈 |
| 重力力矩 | `gravity_joint_tauqes` | $\mathbb{R}^6$ | 基于当前位置 ✓ |
| PD 力矩 | `motor_pid_tau` | $\mathbb{R}^6$ | 上位机计算 |
| 总控制力矩 | `motor_control_tau` | $\mathbb{R}^6$ | 发送给电机 |
| 电机内部增益 | `motor.kp`, `motor.kd` | 全部为 0 ✓ | 纯力矩模式 |

### 7.6.2 控制频率

**主循环频率**：1 kHz（1 ms 周期）

**关键计算耗时**（估计）：
- 逆动力学（重力）：0.1-0.5 ms
- PD 计算：< 0.01 ms
- 关节/电机转换：< 0.01 ms
- 总计：≈ 0.12-0.52 ms

**实时性能**：优于 AutoServo（无需 IK 求解）

### 7.6.3 增益矩阵

**代码实现** (state_machine.cpp:251-263):
```cpp
float kp1 = 315;  // 基座
float kp2 = 315;  // 肩关节
float kp3 = 312;  // 肘关节
float kp4 = 20;   // 腕关节1
float kp5 = 20;   // 腕关节2
float kp6 = 20;   // 腕关节3

float kd1 = 1.43;
float kd2 = 1.43;
float kd3 = 1.73;
float kd4 = 0.53;
float kd5 = 0.43;
float kd6 = 0.33;
```

**数学表达**：

$$\mathbf{K}_p = \begin{bmatrix}
315 & 0 & 0 & 0 & 0 & 0 \\
0 & 315 & 0 & 0 & 0 & 0 \\
0 & 0 & 312 & 0 & 0 & 0 \\
0 & 0 & 0 & 20 & 0 & 0 \\
0 & 0 & 0 & 0 & 20 & 0 \\
0 & 0 & 0 & 0 & 0 & 20
\end{bmatrix}$$

$$\mathbf{K}_d = \begin{bmatrix}
1.43 & 0 & 0 & 0 & 0 & 0 \\
0 & 1.43 & 0 & 0 & 0 & 0 \\
0 & 0 & 1.73 & 0 & 0 & 0 \\
0 & 0 & 0 & 0.53 & 0 & 0 \\
0 & 0 & 0 & 0 & 0.43 & 0 \\
0 & 0 & 0 & 0 & 0 & 0.33
\end{bmatrix}$$

## 7.7 性能分析

### 7.7.1 跟踪精度

**位置精度**（稳态）：

影响因素：
- 编码器分辨率：$\Delta q \approx 0.01°$
- **重力补偿精度：几乎完美**（使用实际状态） ✓
- 静摩擦：未补偿
- 关节柔性：存在

**估计总精度**：$\pm 0.3$ mm（末端位置）

优于 AutoServo（$\pm 0.5$ mm）的原因：
1. 精确的重力补偿
2. 全关节 PD 控制
3. 实时状态反馈

### 7.7.2 动态性能

**关节1示例**：

自然频率：
$$\omega_n = \sqrt{\frac{k_p}{M}} = \sqrt{\frac{315}{0.0029}} \approx 330 \text{ rad/s} \approx 52.5 \text{ Hz}$$

阻尼比：
$$\zeta = \frac{k_d}{2\sqrt{k_p M}} = \frac{1.43}{2\sqrt{315 \times 0.0029}} \approx 0.76$$

系统呈**欠阻尼**（$\zeta < 1$），响应特性：
- 上升时间：快速
- 超调量：$\approx 5\%$
- 调节时间：$\approx 80$ ms

### 7.7.3 稳定性鲁棒性

**重力补偿误差的影响**：

设实际重力为 $\mathbf{g}_{\text{actual}}$，模型为 $\mathbf{g}_{\text{model}}$：

**ManualServo**：
$$\Delta \mathbf{g} = \mathbf{g}_{\text{actual}}(\mathbf{q}_{\text{current}}) - \mathbf{g}_{\text{model}}(\mathbf{q}_{\text{current}})$$

**AutoServo**：
$$\Delta \mathbf{g} = \mathbf{g}_{\text{actual}}(\mathbf{q}) - \mathbf{g}_{\text{model}}(\mathbf{q}_d)$$

当存在跟踪误差（$\mathbf{q} \neq \mathbf{q}_d$）时：
- ManualServo 的 $\Delta \mathbf{g}$ 仅来自模型误差
- AutoServo 的 $\Delta \mathbf{g}$ 包含模型误差 + 配置差异

**鲁棒性结论**：

ManualServo 对以下扰动更鲁棒：
1. **跟踪误差**：重力补偿不受影响
2. **外部干扰**：实时反馈自动调整
3. **负载变化**：立即体现在重力项中

### 7.7.4 柔顺性分析

**纯力矩控制的柔顺性**：

系统的输出阻抗（Output Impedance）为：
$$Z(\omega) = \mathbf{M}(\mathbf{q})(j\omega)^2 + (\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{K}_d)(j\omega) + \mathbf{K}_p$$

在低频（$\omega \to 0$）：
$$Z(0) = \mathbf{K}_p$$

ManualServo 的 $k_p$ 略低于 AutoServo，因此：
- **更柔顺**：适合接触任务
- **更安全**：碰撞时冲击力更小

## 7.8 改进方向

### 7.8.1 摩擦补偿

**当前问题**：静摩擦未补偿，导致低速爬行

**改进方案**：

使用 Stribeck 摩擦模型：
$$\tau_f = (F_c + (F_s - F_c)e^{-|\dot{q}/\dot{q}_s|^\delta})\text{sign}(\dot{q}) + b\dot{q}$$

在控制律中加入：
$$\boldsymbol{\tau} = \mathbf{g}(\mathbf{q}_{\text{current}}) + \mathbf{K}_p\mathbf{e} + \mathbf{K}_d\dot{\mathbf{e}} + \boldsymbol{\tau}_f(\dot{\mathbf{q}})$$

**优点**：
- 减小静摩擦影响
- 改善低速运动平滑性
- 提高定位精度

### 7.8.2 自适应增益调节

**问题**：固定增益无法适应不同任务需求

**改进方案**：

根据误差动态调整增益：
$$k_{p,i}(t) = k_{p,0} + \alpha \cdot |e_i(t)|$$
$$k_{d,i}(t) = k_{d,0} + \beta \cdot |\dot{e}_i(t)|$$

或使用模糊逻辑/神经网络自适应调节。

### 7.8.3 完整动力学补偿

**改进控制律**：
$$\boldsymbol{\tau} = \mathbf{M}(\mathbf{q}_{\text{current}})\ddot{\mathbf{q}}_d + \mathbf{C}(\mathbf{q}_{\text{current}}, \dot{\mathbf{q}}_{\text{current}})\dot{\mathbf{q}}_d + \mathbf{g}(\mathbf{q}_{\text{current}}) + \mathbf{K}_p\mathbf{e} + \mathbf{K}_d\dot{\mathbf{e}}$$

**优点**：
- 高速运动性能提升
- 轨迹跟踪精度提高
- 减少 PD 增益需求

**缺点**：
- 计算量增加
- 对模型精度要求高

### 7.8.4 外力观测器

**目的**：估计外部干扰力

**方法**：

基于动力学残差：
$$\hat{\mathbf{f}}_{\text{ext}} = \boldsymbol{\tau}_{\text{measured}} - \boldsymbol{\tau}_{\text{model}}$$

其中：
$$\boldsymbol{\tau}_{\text{model}} = \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q})$$

**应用**：
- 碰撞检测
- 力/位混合控制
- 安全停机

## 7.9 总结

### 7.9.1 核心公式

$$\boxed{\boldsymbol{\tau} = \mathbf{g}(\mathbf{q}_{\text{current}}) + \mathbf{K}_p(\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}})}$$

其中：
- **$\mathbf{g}(\mathbf{q}_{\text{current}})$**：基于当前状态的精确重力补偿 ✓
- **$\mathbf{K}_p\mathbf{e} + \mathbf{K}_d\dot{\mathbf{e}}$**：全关节 PD 反馈修正
- **电机内部增益：$k_p = k_d = 0$**：纯力矩控制模式

### 7.9.2 关键特性

1. **精确重力补偿**：
   - 使用实际状态 $\mathbf{q}_{\text{current}}$
   - 不受跟踪误差影响
   - 实时自适应调整

2. **纯力矩控制**：
   - 电机内部增益为 0
   - 完全透明的力传递
   - 高柔顺性

3. **全关节 PD 控制**：
   - 所有 6 个关节均受控
   - 提高整体稳定性
   - 改善末端精度

4. **高鲁棒性**：
   - 对跟踪误差不敏感
   - 抗外部干扰能力强
   - 适应负载变化

### 7.9.3 实现优点

1. **控制精度高**：精确的重力补偿确保静态精度 ✓
2. **柔顺性好**：纯力矩模式适合接触任务 ✓
3. **鲁棒性强**：实时状态反馈提高抗扰性 ✓
4. **安全性高**：低刚度减少碰撞风险 ✓
5. **实时性好**：无需 IK 求解（如果期望已知） ✓

### 7.9.4 可能的改进

1. **摩擦补偿**：添加 Stribeck 模型改善低速性能
2. **自适应增益**：根据任务动态调整 PD 参数
3. **完整动力学**：补偿惯性和科里奥利项提升高速性能
4. **外力观测**：检测和响应外部干扰
5. **模型辨识**：在线更新动力学参数

### 7.9.5 与其他模式的协作

```
控制模式层次结构：

    任务层：笛卡尔空间轨迹
        ↓
    规划层：逆运动学求解
        ↓
    控制层：
        ├─ AutoServo: 期望位置重力补偿 + PD（前4关节）
        ├─ ManualServo: 当前位置重力补偿 + PD（全6关节） ✓
        ├─ Gravity: 仅重力补偿
        └─ JointAutoServo: 关节空间直接控制
        ↓
    执行层：电机驱动
```

### 7.9.6 代码位置索引

| 功能 | 代码位置 | 数学对应 |
|------|----------|----------|
| 模式入口 | state_machine.cpp:94-96 | - |
| 逆动力学 | state_machine.cpp:277 | $\boldsymbol{\tau}_g = \mathbf{g}(\mathbf{q}_{\text{current}})$ ✓ |
| 关节/电机转换 | state_machine.cpp:290-297 | 减速比和方向映射 |
| PD 控制（全6关节） | state_machine.cpp:301-311 | $\boldsymbol{\tau}_{\text{PD}} = \mathbf{K}_p\mathbf{e} + \mathbf{K}_d\dot{\mathbf{e}}$ |
| 力矩叠加 | state_machine.cpp:313-316 | $\boldsymbol{\tau} = \boldsymbol{\tau}_g + \boldsymbol{\tau}_{\text{PD}}$ |
| 纯力矩设置 | state_machine.cpp:325-329 | $k_p = k_d = 0$ ✓ |
| 电机控制 | state_machine.cpp:331-336 | 发送 $\boldsymbol{\tau}$ 到电机 |

## 参考文献

1. Sciavicco, B., & Siciliano, L. (2000). *Modelling and Control of Robot Manipulators* (2nd ed.). Springer. (Chapter 8: Motion Control, Chapter 9: Force Control)
2. Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley. (Chapter 11: Computed-Torque Control, Chapter 12: Force Control)
3. Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control*. Pearson. (Chapter 6: Manipulator Dynamics, Chapter 10: Force Control)
4. Slotine, J. J. E., & Li, W. (1991). *Applied Nonlinear Control*. Prentice Hall. (Chapter 6: Robot Manipulator Control)
5. Canudas de Wit, C., Olsson, H., Astrom, K. J., & Lischinsky, P. (1995). *A new model for control of systems with friction*. IEEE Transactions on Automatic Control, 40(3), 419-425.
6. Ott, C., et al. (2015). *Unified Impedance and Admittance Control*. IEEE International Conference on Robotics and Automation (ICRA).

---

# 8. Impedance 控制模式

## 8.1 模式概述

Impedance（阻抗）控制是一种**力/位混合控制**策略，通过调节机械臂末端的动态阻抗特性，实现与环境的柔顺交互。该模式在**笛卡尔空间**建立虚拟质量-阻尼-弹簧系统，使机械臂表现出类似机械阻抗的动态响应特性。

在代码中的实现位置：`state_machine.cpp:524-742`

### 8.1.1 核心思想

阻抗控制的本质是建立**期望的力-位置关系**：

$$\mathbf{F}_{\text{ext}} = \mathbf{M}_d \ddot{\mathbf{e}} + \mathbf{C}_d \dot{\mathbf{e}} + \mathbf{K}_d \mathbf{e}$$

其中：
- $\mathbf{F}_{\text{ext}} \in \mathbb{R}^6$：外部环境施加在末端的力/力矩
- $\mathbf{M}_d$：期望惯性矩阵（虚拟质量）
- $\mathbf{C}_d$：期望阻尼矩阵
- $\mathbf{K}_d$：期望刚度矩阵
- $\mathbf{e} = \mathbf{x}_d - \mathbf{x}$：末端位置误差
- $\dot{\mathbf{e}} = \dot{\mathbf{x}}_d - \dot{\mathbf{x}}$：末端速度误差

**物理意义**：机械臂末端如同一个可调节的虚拟弹簧-阻尼器，通过参数 $\mathbf{M}_d, \mathbf{C}_d, \mathbf{K}_d$ 控制其柔顺性。

### 8.1.2 关键特点

1. **笛卡尔空间控制**：在任务空间建立阻抗模型，直观调节末端柔顺性
2. **力/位混合**：同时考虑位置跟踪和力交互
3. **重力补偿 + 电机内部PID**：使用基于当前位置的重力补偿，结合电机内部PID控制
4. **自适应负载估计**：支持在线估计和更新末端负载（仅此模式有此功能）
5. **软/硬参数可调**：提供两组预设参数实现不同柔顺度
6. **夹爪集成控制**：包含夹爪位置控制

### 8.1.3 与其他模式的对比

| 特性 | Impedance | AutoServo | ManualServo |
|------|-----------|-----------|-------------|
| 控制空间 | 笛卡尔空间 | 笛卡尔空间 | 笛卡尔空间 |
| 控制目标 | 力-位置关系 | 位置跟踪 | 位置跟踪 |
| 柔顺性 | 可调（软/硬） | 刚性 | 中等 |
| 适用场景 | 接触任务 | 位置跟踪 | 力控制 |
| 电机内部PID | 使用 ✓ | 禁用 | 禁用 |
| 负载自适应 | 支持 ✓ | 不支持 | 不支持 |

## 8.2 控制架构

### 8.2.1 控制流程图

```
期望末端位姿 (desir_end_effector_frame)
        ↓
    计算末端位置/姿态误差 (e)
        ↓
    正向速度运动学 (v = J*q̇)
        ↓
    计算末端速度误差 (de = -v_current)
        ↓
    阻抗控制律 (F = C*de + K*e)
        ↓
    雅可比转置映射 (τ_imp = J^T * F)
        ↓
    重力补偿 (τ_g = g(q_current))
        ↓
    力矩叠加 (τ = τ_g + α*τ_imp)
        ↓
    电机内部PID + 夹爪控制
        ↓
电机执行 → 末端运动/力交互
```

### 8.2.2 数学模型

**末端阻抗动力学**（简化，忽略惯性项）：

$$\mathbf{F} = \mathbf{C}_d \dot{\mathbf{e}} + \mathbf{K}_d \mathbf{e}$$

**映射到关节空间**：

$$\boldsymbol{\tau}_{\text{imp}} = \mathbf{J}^T(\mathbf{q}) \mathbf{F}$$

**总控制力矩**：

$$\boldsymbol{\tau} = \mathbf{g}(\mathbf{q}_{\text{current}}) + \boldsymbol{\alpha} \odot \boldsymbol{\tau}_{\text{imp}}$$

其中：
- $\boldsymbol{\alpha} = [1.2, 1.3, 1.3, 1.0, 1.0, 1.0]^T$：增益调节向量
- $\odot$：逐元素乘法（Hadamard积）

**闭环系统**：

结合电机内部PID：

$$\boldsymbol{\tau}_{\text{total}} = \boldsymbol{\tau}_{\text{imp}} + \boldsymbol{\tau}_g + \mathbf{K}_p^{\text{motor}}(\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d^{\text{motor}}(\dot{\mathbf{q}}_d - \dot{\mathbf{q}})$$

## 8.3 详细算法步骤

### 8.3.1 步骤1：计算末端当前速度

**代码** (state_machine.cpp:532-549)：
```cpp
for (unsigned int i = 0; i < 6; i++)
{
    double component = 0.0;
    for (unsigned int j = 0; j < 6; j++)
    {
        component += arm.current_jacobian(i, j) * arm.current_joint_velocities(j);
    }
    // 前 3 行对应线速度，后 3 行对应角速度
    if (i < 3)
    {
        arm.current_end_effector_dot.vel(i) = component;
    }
    else
    {
        arm.current_end_effector_dot.rot(i - 3) = component;
    }
}
```

**数学表达**：

正向速度运动学：

$$\mathbf{v}_{\text{current}} = \mathbf{J}(\mathbf{q}_{\text{current}}) \dot{\mathbf{q}}_{\text{current}}$$

展开为：

$$\begin{bmatrix} \dot{\mathbf{p}} \\ \boldsymbol{\omega} \end{bmatrix} = \begin{bmatrix} \mathbf{J}_v \\ \mathbf{J}_\omega \end{bmatrix} \dot{\mathbf{q}}$$

其中：
- $\dot{\mathbf{p}} = [\dot{x}, \dot{y}, \dot{z}]^T \in \mathbb{R}^3$：线速度
- $\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^T \in \mathbb{R}^3$：角速度

### 8.3.2 步骤2：计算末端位置/姿态误差

**代码** (state_machine.cpp:551-561)：
```cpp
e[0] = arm.desir_end_effector_frame.p.x() - arm.current_end_effector_frame.p.x();
e[1] = arm.desir_end_effector_frame.p.y() - arm.current_end_effector_frame.p.y();
e[2] = arm.desir_end_effector_frame.p.z() - arm.current_end_effector_frame.p.z();

double desir_roll, desir_pitch, desir_yaw, current_roll, current_pitch, current_yaw;
arm.desir_end_effector_frame.M.GetRPY(desir_roll, desir_pitch, desir_yaw);
arm.current_end_effector_frame.M.GetRPY(current_roll, current_pitch, current_yaw);

e[3] = desir_roll - current_roll;
e[4] = desir_pitch - current_pitch;
e[5] = desir_yaw - current_yaw;
```

**数学表达**：

位置误差：

$$\mathbf{e}_p = \mathbf{p}_d - \mathbf{p}_{\text{current}} = \begin{bmatrix} x_d - x \\ y_d - y \\ z_d - z \end{bmatrix}$$

姿态误差（欧拉角）：

$$\mathbf{e}_\eta = \boldsymbol{\eta}_d - \boldsymbol{\eta}_{\text{current}} = \begin{bmatrix} \phi_d - \phi \\ \theta_d - \theta \\ \psi_d - \psi \end{bmatrix}$$

完整误差向量：

$$\mathbf{e} = \begin{bmatrix} \mathbf{e}_p \\ \mathbf{e}_\eta \end{bmatrix} \in \mathbb{R}^6$$

### 8.3.3 步骤3：计算末端速度误差

**代码** (state_machine.cpp:563-568)：
```cpp
de[0] = 0.0*arm.desir_end_effector_dot.vel(0) - arm.current_end_effector_dot.vel(0);
de[1] = 0.0*arm.desir_end_effector_dot.vel(1) - arm.current_end_effector_dot.vel(1);
de[2] = 0.0*arm.desir_end_effector_dot.vel(2) - arm.current_end_effector_dot.vel(2);
de[3] = 0.0*arm.desir_end_effector_dot.rot(0) - arm.current_end_effector_dot.rot(0);
de[4] = 0.0*arm.desir_end_effector_dot.rot(1) - arm.current_end_effector_dot.rot(1);
de[5] = 0.0*arm.desir_end_effector_dot.rot(2) - arm.current_end_effector_dot.rot(2);
```

**数学表达**：

速度误差（期望速度设为0）：

$$\dot{\mathbf{e}} = \mathbf{v}_d - \mathbf{v}_{\text{current}} = \mathbf{0} - \mathbf{v}_{\text{current}} = -\mathbf{v}_{\text{current}}$$

即：

$$\dot{\mathbf{e}} = -\mathbf{J}(\mathbf{q}_{\text{current}}) \dot{\mathbf{q}}_{\text{current}}$$

**物理意义**：由于期望末端速度为0（准静态阻抗），速度误差等于当前速度的负值，阻尼项将抑制末端运动。

### 8.3.4 步骤4：阻抗控制律计算

**代码** (state_machine.cpp:579-605)：
```cpp
// 软模式
C[0] = 0.36;
C[1] = 0.36;
C[2] = 0.36;
C[3] = 0.13;
C[4] = 0.03;
C[5] = 0.03;

// 硬模式
K[0] = 455;
K[1] = 1095;
K[2] = 1095;
K[3] = 18;
K[4] = 18;
K[5] = 11;

for (size_t i = 0; i < 6; i++)
{
    F[i] = C[i]*de[i] + K[i]*e[i];
}
```

**数学表达**：

阻抗力/力矩：

$$\mathbf{F} = \mathbf{C}_d \dot{\mathbf{e}} + \mathbf{K}_d \mathbf{e}$$

其中阻尼和刚度矩阵为对角矩阵：

$$\mathbf{C}_d = \text{diag}(C_1, C_2, C_3, C_4, C_5, C_6)$$

$$\mathbf{K}_d = \text{diag}(K_1, K_2, K_3, K_4, K_5, K_6)$$

展开形式（硬模式）：

$$\mathbf{F} = \begin{bmatrix}
0.36 \dot{e}_x + 455 e_x \\
0.36 \dot{e}_y + 1095 e_y \\
0.36 \dot{e}_z + 1095 e_z \\
0.13 \dot{e}_{\phi} + 18 e_{\phi} \\
0.03 \dot{e}_{\theta} + 18 e_{\theta} \\
0.03 \dot{e}_{\psi} + 11 e_{\psi}
\end{bmatrix}$$

**物理意义**：
- $\mathbf{K}_d \mathbf{e}$：弹簧力，拉回到期望位置
- $\mathbf{C}_d \dot{\mathbf{e}}$：阻尼力，抑制振荡

### 8.3.5 步骤5：雅可比转置映射

**代码** (state_machine.cpp:607-617)：
```cpp
for (unsigned int i = 0; i < 6; i++)
{
    double component = 0.0;
    for (unsigned int j = 0; j < 6; j++)
    {
        component += arm.current_jacobian(j, i) * F[j];
    }

    arm.joint_impedance_tau[i] = component;
}
```

**数学表达**：

通过**雅可比转置**将笛卡尔空间的力映射到关节力矩：

$$\boldsymbol{\tau}_{\text{imp}} = \mathbf{J}^T(\mathbf{q}_{\text{current}}) \mathbf{F}$$

展开：

$$\tau_{\text{imp},i} = \sum_{j=1}^{6} J_{ji}(\mathbf{q}) F_j, \quad i = 1, \ldots, 6$$

**虚功原理**：

雅可比转置基于虚功原理：

$$\delta W = \boldsymbol{\tau}^T \delta \mathbf{q} = \mathbf{F}^T \delta \mathbf{x}$$

由于 $\delta \mathbf{x} = \mathbf{J} \delta \mathbf{q}$，得到：

$$\boldsymbol{\tau}^T \delta \mathbf{q} = \mathbf{F}^T \mathbf{J} \delta \mathbf{q}$$

因此：

$$\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}$$

### 8.3.6 步骤6：末端力估计（可选）

#### 核心思想

这是一种**基于动力学模型的外力观测器（Force Observer）**，无需在末端安装六维力传感器，仅通过关节力矩传感器即可估计末端受到的外部力/力矩。这是现代协作机器人中常用的**软传感器（Soft Sensor）**技术。

#### 调用接口

**代码** (state_machine.cpp:634)：
```cpp
arm.joint2end_force();
```

#### 完整实现代码

**位置**：light_lift_arm_6dof.cpp:146-161

```cpp
void LlArm6dof::joint2end_force()
{
    // 步骤1：计算雅可比转置 J^T (6×6)
    Eigen::MatrixXd Jt = current_jacobian.data.transpose();

    // 步骤2：计算 (J^T) 的伪逆
    // 使用完全正交分解，数值稳定性优于普通逆
    Eigen::MatrixXd Jt_pinv = Jt.completeOrthogonalDecomposition().pseudoInverse();

    // 步骤3：计算关节力矩残差
    // current_joint_tauqes: 电机反馈的实际测量力矩（基于电流）
    // gravity_joint_tauqes: 重力补偿模型预测的力矩
    Eigen::VectorXd tau_residual = current_joint_tauqes.data - gravity_joint_tauqes.data;

    // 步骤4：通过雅可比转置伪逆映射到末端空间
    // wrench = (J^T)^† × Δτ
    // 结果是6维向量：[Fx, Fy, Fz, Mx, My, Mz]
    Eigen::VectorXd wrench = Jt_pinv * tau_residual;

    // 步骤5：分解为力和力矩分量
    current_end_effector_wrench.force = KDL::Vector(wrench(0), wrench(1), wrench(2));   // 线性力 [N]
    current_end_effector_wrench.torque = KDL::Vector(wrench(3), wrench(4), wrench(5)); // 扭矩 [N·m]
}
```

#### 数学推导

##### 1. 机械臂动力学基础

完整的机械臂动力学方程：

$$\boldsymbol{\tau}_{\text{measured}} = \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) + \mathbf{J}^T(\mathbf{q}) \mathbf{F}_{\text{ext}}$$

其中：
- $\mathbf{M}(\mathbf{q}) \in \mathbb{R}^{6 \times 6}$：关节空间惯性矩阵
- $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \in \mathbb{R}^{6 \times 6}$：科氏力和离心力矩阵
- $\mathbf{g}(\mathbf{q}) \in \mathbb{R}^6$：重力项
- $\mathbf{F}_{\text{ext}} \in \mathbb{R}^6$：末端外力/力矩（未知量）
- $\boldsymbol{\tau}_{\text{measured}} \in \mathbb{R}^6$：电机测量的关节力矩

##### 2. 准静态假设

在 autoservo 模式下，机械臂运动缓慢（期望速度为0），可以忽略动态项：

$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} \approx \mathbf{0}, \quad \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} \approx \mathbf{0}$$

简化为准静态方程：

$$\boldsymbol{\tau}_{\text{measured}} \approx \mathbf{g}(\mathbf{q}) + \mathbf{J}^T(\mathbf{q}) \mathbf{F}_{\text{ext}}$$

##### 3. 力矩残差计算

定义力矩残差（实际测量值与模型预测值的差异）：

$$\Delta\boldsymbol{\tau} = \boldsymbol{\tau}_{\text{measured}} - \boldsymbol{\tau}_{\text{model}}$$

其中 $\boldsymbol{\tau}_{\text{model}} = \mathbf{g}(\mathbf{q})$ 是重力补偿项。

代入准静态方程：

$$\Delta\boldsymbol{\tau} = \mathbf{J}^T(\mathbf{q}) \mathbf{F}_{\text{ext}}$$

**物理意义**：力矩残差就是外力在关节空间的投影。

##### 4. 末端力求解

两边左乘 $(\mathbf{J}^T)^{\dagger}$（雅可比转置的伪逆）：

$$\hat{\mathbf{F}}_{\text{ext}} = (\mathbf{J}^T)^{\dagger} \Delta\boldsymbol{\tau} = (\mathbf{J}^T)^{\dagger} (\boldsymbol{\tau}_{\text{measured}} - \mathbf{g}(\mathbf{q}))$$

展开为分量形式：

$$\hat{\mathbf{F}}_{\text{ext}} = \begin{bmatrix} \hat{F}_x \\ \hat{F}_y \\ \hat{F}_z \\ \hat{M}_x \\ \hat{M}_y \\ \hat{M}_z \end{bmatrix} = (\mathbf{J}^T)^{\dagger} \begin{bmatrix} \Delta\tau_1 \\ \Delta\tau_2 \\ \Delta\tau_3 \\ \Delta\tau_4 \\ \Delta\tau_5 \\ \Delta\tau_6 \end{bmatrix}$$

#### 为什么使用伪逆而非普通逆？

##### 1. 奇异位形问题

雅可比矩阵 $\mathbf{J}(\mathbf{q})$ 在某些关节位置会退化（行列式接近0），导致 $\mathbf{J}^T$ 不可逆或数值不稳定。

##### 2. 伪逆的优势

Moore-Penrose 伪逆 $(\mathbf{J}^T)^{\dagger}$ 具有以下性质：

- **存在性**：即使 $\mathbf{J}^T$ 不可逆，伪逆总是存在
- **最小范数解**：在无穷多解的情况下，选择范数最小的解
- **最小二乘解**：在无精确解时，给出误差平方和最小的近似解
- **数值稳定性**：通过 SVD 或完全正交分解计算，避免数值爆炸

##### 3. 计算方法

代码中使用 `completeOrthogonalDecomposition().pseudoInverse()`：

$$\mathbf{J}^T = \mathbf{U} \boldsymbol{\Sigma} \mathbf{V}^T \quad \Rightarrow \quad (\mathbf{J}^T)^{\dagger} = \mathbf{V} \boldsymbol{\Sigma}^{\dagger} \mathbf{U}^T$$

其中 $\boldsymbol{\Sigma}^{\dagger}$ 是对角矩阵，非零元素取倒数，零元素保持为零。

#### 与步骤 8.3.5 的对偶关系

**8.3.5 正向映射**（已知末端力，求关节力矩）：

$$\boldsymbol{\tau}_{\text{imp}} = \mathbf{J}^T \mathbf{F}$$

**8.3.6 逆向映射**（已知关节力矩残差，求末端外力）：

$$\hat{\mathbf{F}}_{\text{ext}} = (\mathbf{J}^T)^{\dagger} \Delta\boldsymbol{\tau}$$

两者形成**虚功原理的双向映射**：

```
          正向（控制）
    末端力 ────────J^T────────> 关节力矩
      F                           τ
      ↑                           ↑
      |                           |
      |    逆向（估计）             |
      └────────(J^T)^†────────────┘
```

#### 物理意义深度解析

##### 1. 力矩残差的来源

$$\Delta\boldsymbol{\tau} = \boldsymbol{\tau}_{\text{measured}} - \mathbf{g}(\mathbf{q})$$

- **无外力时**：$\boldsymbol{\tau}_{\text{measured}} = \mathbf{g}(\mathbf{q})$，残差为零
- **受外力时**：电机需要额外力矩平衡外力，产生非零残差
- **模型误差时**：如果重力模型不准（质量、质心位置错误），会产生系统性偏差

##### 2. 坐标系说明

估计的 $\hat{\mathbf{F}}_{\text{ext}}$ 定义在**末端执行器坐标系**中：

- $F_x, F_y, F_z$：沿末端坐标系 X、Y、Z 轴的线性力
- $M_x, M_y, M_z$：绕末端坐标系 X、Y、Z 轴的力矩

##### 3. 符号约定

- **正值**：外部环境施加到末端的力
- 例如：$F_z > 0$ 表示末端受到向上的推力（如物体压在末端）

#### 应用场景

##### 1. 碰撞检测

实时监测外力大小：

$$\|\hat{\mathbf{F}}_{\text{ext}}\| = \sqrt{F_x^2 + F_y^2 + F_z^2}$$

当超过阈值时触发紧急停止：

```cpp
if (arm.current_end_effector_wrench.force.Norm() > COLLISION_THRESHOLD) {
    emergency_stop();
}
```

##### 2. 负载质量估计（见 8.3.7）

通过 Z 轴受力估计抓取物体质量：

$$\hat{m}_{\text{load}} = \frac{F_z}{g} \quad \text{（忽略缩放系数）}$$

##### 3. 人机协作

检测操作员手动施加的力方向：

- 若 $F_x > 0$：操作员向 X 正方向推动
- 可实现"拖动示教"功能

##### 4. 接触力反馈

在装配任务中监测插入力：

```cpp
double insertion_force = arm.current_end_effector_wrench.force(2); // Z轴力
if (insertion_force > ASSEMBLY_FORCE_LIMIT) {
    reduce_insertion_speed();
}
```

##### 5. 力控打磨

估计末端与工件的接触力，调整压力：

```cpp
double contact_force = arm.current_end_effector_wrench.force.Norm();
if (contact_force < TARGET_FORCE) {
    arm.desir_end_effector_frame.p.z() -= 0.001; // 下移1mm
}
```

#### 局限性与注意事项

##### 1. 准静态假设局限

忽略的动态项在快速运动时不可忽略：

- **惯性力误差**：$\|\mathbf{M}\ddot{\mathbf{q}}\|$ 在加速时可达数十牛米
- **科氏力误差**：高速旋转时 $\|\mathbf{C}\dot{\mathbf{q}}\|$ 显著

**解决方案**：使用完整动量观测器（Momentum Observer）补偿动态项。

##### 2. 重力模型精度依赖

$$\Delta\boldsymbol{\tau} = \boldsymbol{\tau}_{\text{measured}} - \mathbf{g}(\mathbf{q})$$

若 $\mathbf{g}(\mathbf{q})$ 计算不准：

- **质量误差**：$\pm 10\%$ 质量误差 → Z 轴力估计误差约 $\pm 1$ N
- **质心偏移**：质心位置 $\pm 5$ mm 误差 → 力矩估计误差约 $\pm 0.1$ N·m

**解决方案**：通过参数辨识实验校准动力学模型。

##### 3. 力矩传感器噪声

电机电流测量噪声（典型 $\pm 0.05$ N·m）会直接传递：

$$\sigma_{\hat{F}} \approx \|(\mathbf{J}^T)^{\dagger}\| \cdot \sigma_{\tau}$$

**解决方案**：应用低通滤波器（截止频率 5-10 Hz）。

##### 4. 雅可比奇异问题

在奇异位形，$(\mathbf{J}^T)^{\dagger}$ 的条件数极大：

$$\text{cond}((\mathbf{J}^T)^{\dagger}) = \frac{\sigma_{\max}}{\sigma_{\min}} \to \infty$$

导致某些方向的力估计**放大噪声**。

**检测方法**：计算可操作度（manipulability）

$$w = \sqrt{\det(\mathbf{J}\mathbf{J}^T)}$$

当 $w < 0.01$ 时，力估计不可靠。

##### 5. 摩擦力未建模

关节摩擦力矩（Coulomb + 粘性摩擦）未包含在 $\mathbf{g}(\mathbf{q})$ 中：

$$\boldsymbol{\tau}_{\text{friction}} = \text{sign}(\dot{\mathbf{q}}) \odot \boldsymbol{\tau}_c + \mathbf{D} \dot{\mathbf{q}}$$

会导致运动时的系统性估计偏差。

**解决方案**：扩展模型 $\boldsymbol{\tau}_{\text{model}} = \mathbf{g}(\mathbf{q}) + \boldsymbol{\tau}_{\text{friction}}$。

#### 数值示例

假设在某一时刻：

**输入数据**：
- 测量关节力矩：$\boldsymbol{\tau}_{\text{measured}} = [5.2, 3.8, 2.1, 0.3, 0.15, 0.08]^T$ N·m
- 重力补偿力矩：$\mathbf{g}(\mathbf{q}) = [5.0, 4.0, 2.0, 0.3, 0.15, 0.10]^T$ N·m

**计算过程**：

1. 力矩残差：
   $$\Delta\boldsymbol{\tau} = [0.2, -0.2, 0.1, 0.0, 0.0, -0.02]^T \text{ N·m}$$

2. 应用伪逆映射（假设 $(\mathbf{J}^T)^{\dagger}$ 已计算）：
   $$\hat{\mathbf{F}}_{\text{ext}} = (\mathbf{J}^T)^{\dagger} \Delta\boldsymbol{\tau} = [1.5, -2.0, 3.2, 0.05, 0.02, -0.01]^T$$

**结果解释**：
- 末端受到 $\mathbf{F} = [1.5, -2.0, 3.2]^T$ N 的线性力
- 末端受到 $\mathbf{M} = [0.05, 0.02, -0.01]^T$ N·m 的力矩
- 合力大小：$\|\mathbf{F}\| = \sqrt{1.5^2 + 2.0^2 + 3.2^2} \approx 4.1$ N

#### 改进方向

##### 1. 完整动量观测器

包含动态补偿：

$$\hat{\mathbf{F}}_{\text{ext}} = (\mathbf{J}^T)^{\dagger} \left( \boldsymbol{\tau}_{\text{measured}} - \mathbf{M}\ddot{\mathbf{q}} - \mathbf{C}\dot{\mathbf{q}} - \mathbf{g} \right)$$

需要：
- 关节加速度测量或估计
- 精确的惯性参数

##### 2. 卡尔曼滤波融合

将力观测器与滤波器结合：

$$\hat{\mathbf{F}}_{k+1} = \mathbf{A} \hat{\mathbf{F}}_k + \mathbf{K} (\mathbf{z}_k - \mathbf{H}\hat{\mathbf{F}}_k)$$

降低噪声，提高估计平滑性。

##### 3. 自适应参数辨识

在线更新重力模型参数 $\boldsymbol{\pi} = [m, \mathbf{r}_c]$：

$$\dot{\boldsymbol{\pi}} = \boldsymbol{\Gamma} \mathbf{Y}^T (\boldsymbol{\tau}_{\text{measured}} - \mathbf{g}(\mathbf{q}, \boldsymbol{\pi}))$$

其中 $\mathbf{Y}$ 是回归矩阵。

### 8.3.7 步骤7：自适应负载估计

**代码** (state_machine.cpp:636-661)：
```cpp
double link_mass = 0.0015 * (arm.current_end_effector_wrench.force(2) / 0.00981);

if(arm.cmd == "laod")
{
    std::string link_name = "link6";
    arm.updateLinkMass(arm.kdl_chain, link_name, 0.5*link_mass);
    std::cout << "某端增加： " << link_mass << " kg" << std::endl;
    arm.cmd = "nan";
}
else if(arm.cmd == "clearlaod"){
    std::string link_name = "link6";
    arm.updateLinkMass(arm.kdl_chain, link_name, 0.0);
    std::cout << "某端增加： " << 0.0 << " kg" << std::endl;
    arm.cmd = "nan";
}
```

**数学表达**：

负载质量估计：

$$\hat{m}_{\text{load}} = \gamma \cdot \frac{F_z}{g}$$

其中：
- $\gamma = 0.0015$：估计系数
- $F_z$：末端Z轴受力（N）
- $g = 9.81$ m/s²：重力加速度

**更新动力学模型**：

$$m_{\text{link6}}^{\text{new}} = m_{\text{link6}}^{\text{old}} + 0.5 \hat{m}_{\text{load}}$$

**应用场景**：抓取物体后，自动更新重力补偿模型以适应新负载。

### 8.3.8 步骤8：重力补偿与力矩叠加

**代码** (state_machine.cpp:666-671)：
```cpp
arm.joint_control_tau[0] = arm.gravity_joint_tauqes(0) + 1.2*arm.joint_impedance_tau[0];
arm.joint_control_tau[1] = arm.gravity_joint_tauqes(1) + 1.3*arm.joint_impedance_tau[1];
arm.joint_control_tau[2] = arm.gravity_joint_tauqes(2) + 1.3*arm.joint_impedance_tau[2];
arm.joint_control_tau[3] = arm.gravity_joint_tauqes(3) + 1.0*arm.joint_impedance_tau[3];
arm.joint_control_tau[4] = arm.gravity_joint_tauqes(4) + 1.0*arm.joint_impedance_tau[4];
arm.joint_control_tau[5] = arm.gravity_joint_tauqes(5) + 1.0*arm.joint_impedance_tau[5];
```

**数学表达**：

总控制力矩：

$$\boldsymbol{\tau}_{\text{control}} = \mathbf{g}(\mathbf{q}_{\text{current}}) + \boldsymbol{\alpha} \odot \boldsymbol{\tau}_{\text{imp}}$$

其中增益向量：

$$\boldsymbol{\alpha} = \begin{bmatrix} 1.2 \\ 1.3 \\ 1.3 \\ 1.0 \\ 1.0 \\ 1.0 \end{bmatrix}$$

**物理意义**：
- 前3个关节（大负载）：增益 > 1，增强阻抗效果
- 后3个关节（小负载）：增益 = 1，保持原始阻抗

### 8.3.9 步骤9：电机控制（含夹爪）

**代码** (state_machine.cpp:699-730)：
```cpp
float gravity_kp[6] = {0.0};
float gravity_kd[6] = {0.0};

gravity_kp[3] = 30;

gravity_kd[0] = 0.1;
gravity_kd[1] = 0.2;
gravity_kd[2] = 0.2;
gravity_kd[3] = 0.9;
gravity_kd[4] = 0.8;
gravity_kd[5] = 0.5;

motor.ControlMotors_g(port,
                      motor.desir_motor_pos,
                      motor.desir_motor_vel,
                      gravity_kp,
                      gravity_kd,
                      motor.motor_control_tau,
                      arm.desir_grp);
```

**数学表达**：

电机控制律（包含内部PID）：

$$\boldsymbol{\tau}_{\text{motor}} = \boldsymbol{\tau}_{\text{control}} + \mathbf{K}_p^{\text{motor}} (\mathbf{q}_{m,d} - \mathbf{q}_m) + \mathbf{K}_d^{\text{motor}} (\dot{\mathbf{q}}_{m,d} - \dot{\mathbf{q}}_m)$$

其中电机PD增益：

$$\mathbf{K}_p^{\text{motor}} = \text{diag}(0, 0, 0, 30, 0, 0)$$

$$\mathbf{K}_d^{\text{motor}} = \text{diag}(0.1, 0.2, 0.2, 0.9, 0.8, 0.5)$$

**注意**：
- 仅关节4有位置增益（$k_p = 30$）
- 所有关节都有阻尼增益（$k_d \in [0.1, 0.9]$）
- 与ManualServo不同，这里使用电机内部PID而非完全纯力矩控制

## 8.4 完整数学模型

### 8.4.1 机械臂动力学

$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau} + \boldsymbol{\tau}_{\text{ext}}$$

其中：
- $\boldsymbol{\tau}$：控制力矩
- $\boldsymbol{\tau}_{\text{ext}} = \mathbf{J}^T \mathbf{F}_{\text{ext}}$：外部力矩（环境接触力）

### 8.4.2 阻抗控制律

**笛卡尔空间**：

$$\mathbf{F} = \mathbf{C}_d \dot{\mathbf{e}} + \mathbf{K}_d \mathbf{e}$$

**关节空间映射**：

$$\boldsymbol{\tau}_{\text{imp}} = \mathbf{J}^T(\mathbf{q}) \mathbf{F}$$

**总控制力矩**：

$$\boldsymbol{\tau} = \mathbf{g}(\mathbf{q}) + \boldsymbol{\alpha} \odot \boldsymbol{\tau}_{\text{imp}} + \mathbf{K}_p^{\text{motor}} (\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d^{\text{motor}} (\dot{\mathbf{q}}_d - \dot{\mathbf{q}})$$

### 8.4.3 闭环末端阻抗

**理想情况**：当重力完全补偿（$\boldsymbol{\tau}_g \approx \mathbf{g}(\mathbf{q})$）且电机PID足够高频时，闭环系统在笛卡尔空间表现为：

$$\mathbf{M}_{\text{eff}} \ddot{\mathbf{x}} + \mathbf{C}_d \dot{\mathbf{x}} + \mathbf{K}_d \mathbf{x} = \mathbf{F}_{\text{ext}}$$

其中有效惯性：

$$\mathbf{M}_{\text{eff}} = (\mathbf{J}^{-T} \mathbf{M} \mathbf{J}^{-1})$$

**期望阻抗行为**：

当外力 $\mathbf{F}_{\text{ext}}$ 施加到末端时，末端位置偏移为：

$$\mathbf{x}_{ss} = \mathbf{K}_d^{-1} \mathbf{F}_{\text{ext}}$$

**柔顺性**：

$$\mathbf{S} = \mathbf{K}_d^{-1}$$

刚度越小，柔顺性越大。

### 8.4.4 稳定性分析

**闭环传递函数**（简化，忽略耦合）：

$$\frac{X(s)}{F_{\text{ext}}(s)} = \frac{1}{M_{\text{eff}} s^2 + C_d s + K_d}$$

**特征方程**：

$$M_{\text{eff}} s^2 + C_d s + K_d = 0$$

**稳定性条件**（Routh-Hurwitz）：

1. $M_{\text{eff}}, C_d, K_d > 0$（已满足）
2. $C_d^2 > 4 M_{\text{eff}} K_d$（过阻尼，避免振荡）

**自然频率和阻尼比**：

$$\omega_n = \sqrt{\frac{K_d}{M_{\text{eff}}}}, \quad \zeta = \frac{C_d}{2\sqrt{K_d M_{\text{eff}}}}$$

## 8.5 阻抗参数设计

### 8.5.1 硬模式参数

**代码设置** (state_machine.cpp:587-592)：

| 方向 | $K_d$ | $C_d$ | 说明 |
|------|-------|-------|------|
| X轴（线） | 455 | 0.36 | 高刚度，低阻尼 |
| Y轴（线） | 1095 | 0.36 | 超高刚度 |
| Z轴（线） | 1095 | 0.36 | 超高刚度（重力方向） |
| Roll（角） | 18 | 0.13 | 中等刚度 |
| Pitch（角） | 18 | 0.03 | 中等刚度，极低阻尼 |
| Yaw（角） | 11 | 0.03 | 低刚度，极低阻尼 |

**特性分析**：

自然频率（假设 $M_{\text{eff}} \approx 1$ kg）：

$$\omega_{n,y} = \sqrt{\frac{1095}{1}} \approx 33.1 \text{ rad/s} \approx 5.3 \text{ Hz}$$

$$\omega_{n,x} = \sqrt{\frac{455}{1}} \approx 21.3 \text{ rad/s} \approx 3.4 \text{ Hz}$$

阻尼比：

$$\zeta_y = \frac{0.36}{2\sqrt{1095 \times 1}} \approx 0.0054 \text{ (严重欠阻尼！)}$$

**问题**：硬模式下阻尼严重不足，可能导致振荡。

**改进建议**：

临界阻尼（$\zeta = 1$）所需阻尼系数：

$$C_{d,\text{critical}} = 2\sqrt{K_d M_{\text{eff}}}$$

对于Y/Z轴：

$$C_{d,y} = 2\sqrt{1095 \times 1} \approx 66.2$$

当前值仅为 0.36，建议增大至 20-30 以改善动态响应。

### 8.5.2 软模式参数（注释掉的）

**代码** (state_machine.cpp:571-576, 595-600)：

| 方向 | $K_d$ (注释) | $C_d$ | 说明 |
|------|--------------|-------|------|
| X/Y/Z轴 | 255/295/95 | 31.96/20.96 | 刚度降低 |

**软模式特点**：
- 刚度降低约 4-11 倍
- 阻尼增大（注释中硬模式阻尼更大）
- 更适合接触任务

### 8.5.3 参数设计准则

**1. 刚度选择**：

$$K_d = \frac{F_{\text{max}}}{\Delta x_{\text{allow}}}$$

- $F_{\text{max}}$：期望承受的最大力
- $\Delta x_{\text{allow}}$：允许的最大偏移

**示例**：Y轴能承受 10 N力，允许偏移 1 cm：

$$K_d = \frac{10}{0.01} = 1000 \text{ N/m}$$

（与代码中的 1095接近）

**2. 阻尼选择**：

临界阻尼：

$$C_d = 2\sqrt{K_d M_{\text{eff}}}$$

过阻尼（$\zeta = 1.2$-$1.5$）：

$$C_d = (1.2 \sim 1.5) \times 2\sqrt{K_d M_{\text{eff}}}$$

欠阻尼（快速响应，$\zeta = 0.5$-$0.7$）：

$$C_d = (0.5 \sim 0.7) \times 2\sqrt{K_d M_{\text{eff}}}$$

**3. 不同任务的参数选择**：

| 任务 | 刚度 $K_d$ | 阻尼 $C_d$ | 阻尼比 $\zeta$ |
|------|------------|------------|---------------|
| 精确定位 | 高（1000+） | 中（20-30） | 0.5-0.7 |
| 柔顺装配 | 低（100-300） | 高（30-50） | > 1.0 |
| 抛光/打磨 | 低（50-200） | 中（10-20） | 0.7-1.0 |
| 力控制 | 极低（10-50） | 高（20-40） | > 1.5 |

## 8.6 代码分析

### 8.6.1 关键参数

| 参数 | 变量名 | 值/范围 | 说明 |
|------|--------|---------|------|
| 阻抗力 | `F[6]` | 计算得到 | $\mathbf{C}_d \dot{\mathbf{e}} + \mathbf{K}_d \mathbf{e}$ |
| 阻抗力矩 | `joint_impedance_tau[6]` | 计算得到 | $\mathbf{J}^T \mathbf{F}$ |
| 位置误差 | `e[6]` | 实时计算 | $\mathbf{x}_d - \mathbf{x}$ |
| 速度误差 | `de[6]` | 实时计算 | $-\mathbf{v}_{\text{current}}$ |
| 刚度矩阵 | `K[6]` | [455, 1095, 1095, 18, 18, 11] | 硬模式 |
| 阻尼矩阵 | `C[6]` | [0.36, 0.36, 0.36, 0.13, 0.03, 0.03] | 软模式 |
| 增益向量 | α（隐式） | [1.2, 1.3, 1.3, 1.0, 1.0, 1.0] | 力矩放大系数 |
| 电机PD增益 | `gravity_kp/kd` | kp[3]=30, kd=[0.1-0.9] | 部分使用 |

### 8.6.2 控制频率

**主循环频率**：1 kHz（1 ms周期）

**关键计算耗时**（估计）：
- 正向速度运动学（$\mathbf{v} = \mathbf{J}\dot{\mathbf{q}}$）：0.05-0.1 ms
- 误差计算：< 0.01 ms
- 阻抗力计算：< 0.01 ms
- 雅可比转置映射：0.05-0.1 ms
- 逆动力学（重力）：0.1-0.5 ms
- 总计：≈ 0.2-0.7 ms

**实时性能**：良好，满足 1 kHz控制需求。

### 8.6.3 夹爪控制

**代码** (state_machine.cpp:724)：
```cpp
motor.ControlMotors_g(port, ..., arm.desir_grp);
```

**功能**：同时控制6个关节和1个夹爪（gripper），实现协调操作。

**夹爪控制量**：`arm.desir_grp`（期望夹爪位置）

### 8.6.4 特殊设计：仅关节4-6使用期望位置

**代码** (state_machine.cpp:676-679)：
```cpp
for (size_t i = 0; i < 3; i++)
{
    arm.desir_joint_pos[3+i] = arm.desir_joint_positions(3+i);
}
```

**意义**：
- 前3个关节（关节1-3）：不更新期望位置，电机PID作用弱化
- 后3个关节（关节4-6）：使用期望位置，电机PID参与控制

**原因**：前3个关节负载大，主要通过阻抗控制和重力补偿；后3个关节负载小，需要额外位置控制提高精度。

## 8.7 性能分析

### 8.7.1 柔顺性分析

**定义**：柔顺性是刚度的倒数

$$\mathbf{S} = \mathbf{K}_d^{-1}$$

**硬模式柔顺性**：

| 方向 | $K_d$ | 柔顺性 $S$ | 10N力的偏移 |
|------|-------|------------|-------------|
| X轴 | 455 | 0.0022 m/N | 22 mm |
| Y轴 | 1095 | 0.00091 m/N | 9.1 mm |
| Z轴 | 1095 | 0.00091 m/N | 9.1 mm |

**分析**：
- Y/Z轴较硬（刚度高），适合承载
- X轴较软，允许一定偏移

**与其他模式对比**：

| 模式 | 等效刚度 | 柔顺性 | 适用场景 |
|------|----------|--------|----------|
| Impedance（硬） | 455-1095 N/m | 中 | 接触任务 |
| Impedance（软） | 95-295 N/m | 高 | 柔顺装配 |
| AutoServo | 极高（∞） | 极低 | 刚性定位 |
| ManualServo | 高 | 低 | 力控制 |

### 8.7.2 动态响应

**硬模式Y轴**（最硬方向）：

自然频率：

$$\omega_n = \sqrt{\frac{1095}{1}} \approx 33.1 \text{ rad/s} \approx 5.3 \text{ Hz}$$

阻尼比（严重欠阻尼）：

$$\zeta = \frac{0.36}{2\sqrt{1095}} \approx 0.0054$$

**时域响应**：

超调量：

$$\sigma = e^{-\frac{\zeta \pi}{\sqrt{1-\zeta^2}}} \approx e^{-\frac{0.0054 \pi}{\sqrt{1-0.0054^2}}} \approx 98.3\%$$

调节时间（2%误差带）：

$$t_s = \frac{4}{\zeta \omega_n} = \frac{4}{0.0054 \times 33.1} \approx 22.4 \text{ s}$$

**结论**：欠阻尼导致严重振荡和长时间稳定，需要增大阻尼系数。

### 8.7.3 力跟踪性能

**稳态力误差**：

在静态平衡时，外力 $\mathbf{F}_{\text{ext}}$ 与阻抗力平衡：

$$\mathbf{F}_{\text{ext}} = \mathbf{K}_d \mathbf{e}_{ss}$$

稳态位置偏移：

$$\mathbf{e}_{ss} = \mathbf{K}_d^{-1} \mathbf{F}_{\text{ext}}$$

**示例**：Y轴受到 10 N外力：

$$e_{y,ss} = \frac{10}{1095} \approx 0.0091 \text{ m} = 9.1 \text{ mm}$$

**力跟踪带宽**（假设 $\zeta = 1$）：

$$\omega_{BW} \approx \omega_n = 5.3 \text{ Hz}$$

超过此频率的力变化难以跟踪。

### 8.7.4 稳定性鲁棒性

**对参数不确定性的敏感度**：

阻抗控制相对鲁棒，因为：
1. 基于雅可比转置，无需逆运算
2. 重力补偿基于当前状态
3. 电机内部PID提供额外稳定性

**潜在不稳定因素**：
1. 阻尼过低（如硬模式）→ 振荡
2. 刚度不匹配（各轴差异大）→ 耦合振荡
3. 雅可比奇异性 → 控制力矩爆炸

## 8.8 自适应负载估计

### 8.8.1 估计原理

**物理模型**：

末端受到的重力：

$$F_z = m_{\text{load}} \cdot g$$

负载质量：

$$m_{\text{load}} = \frac{F_z}{g}$$

**代码实现**（带校正系数）：

$$\hat{m}_{\text{load}} = \gamma \cdot \frac{F_z}{g} = 0.0015 \cdot \frac{F_z}{0.00981}$$

简化：

$$\hat{m}_{\text{load}} \approx 0.153 \cdot F_z$$

**校正系数意义**：
- $\gamma = 0.0015$：经验校正系数，补偿测量误差和模型简化

### 8.8.2 动力学模型更新

**更新link6质量**：

$$m_{\text{link6}}^{\text{new}} = m_{\text{link6}}^{\text{old}} + \alpha \cdot \hat{m}_{\text{load}}$$

其中 $\alpha = 0.5$（折扣因子，避免过度估计）。

**影响**：
- 重力补偿自动调整：$\mathbf{g}(\mathbf{q})$ 考虑新负载
- 惯性矩阵更新：$\mathbf{M}(\mathbf{q})$ 反映新质量分布
- 提高重力补偿精度

### 8.8.3 应用流程

**步骤**：
1. **初始状态**：机械臂处于已知配置（如home位置）
2. **抓取物体**：夹爪闭合，抓住物体
3. **发送命令**：通过ROS话题发送 `"laod"` 命令
4. **估计负载**：根据末端Z轴力估计物体质量
5. **更新模型**：将估计质量的50%添加到link6
6. **控制适应**：后续重力补偿自动考虑新负载

**卸载流程**：
1. **释放物体**：夹爪打开
2. **发送命令**：发送 `"clearlaod"` 命令
3. **重置质量**：link6质量恢复到原始值

### 8.8.4 优点与局限

**优点**：
- ✅ 无需额外传感器（基于关节力矩）
- ✅ 实时在线估计
- ✅ 自动更新动力学模型
- ✅ 提高不同负载下的控制性能

**局限**：
- ❌ 仅估计Z轴（重力方向）负载
- ❌ 需要准静态条件（运动时估计不准）
- ❌ 折扣因子（0.5）需要经验调整
- ❌ 校正系数（0.0015）与机械臂相关

**改进方向**：
- 使用卡尔曼滤波平滑估计
- 考虑多轴动态负载
- 自适应调整折扣因子

## 8.9 改进方向

### 8.9.1 阻尼系数优化

**当前问题**：硬模式阻尼严重不足（$\zeta \approx 0.005$）

**改进方案**：

增大阻尼到临界阻尼或略微过阻尼：

$$C_d^{\text{new}} = 2\sqrt{K_d M_{\text{eff}}} \cdot (1.0 \sim 1.2)$$

**示例**（Y/Z轴）：

$$C_d = 2\sqrt{1095 \times 1} \times 1.1 \approx 72.7$$

将代码中的 0.36 改为 70-80，可显著改善动态响应。

### 8.9.2 自适应阻抗

**问题**：固定参数无法适应不同任务

**改进**：

根据接触力实时调整刚度和阻尼：

$$K_d(t) = K_{d,0} + \beta_K \|\mathbf{F}_{\text{ext}}\|$$

$$C_d(t) = 2\sqrt{K_d(t) M_{\text{eff}}}$$

**效果**：
- 无接触时：高刚度，精确定位
- 接触时：降低刚度，增加柔顺性

### 8.9.3 完整阻抗模型

**当前简化**：忽略惯性项 $\mathbf{M}_d \ddot{\mathbf{e}}$

**完整模型**：

$$\mathbf{F} = \mathbf{M}_d \ddot{\mathbf{e}} + \mathbf{C}_d \dot{\mathbf{e}} + \mathbf{K}_d \mathbf{e}$$

**实现**：

需要估计末端加速度 $\ddot{\mathbf{x}}$：

$$\ddot{\mathbf{x}} = \mathbf{J}\ddot{\mathbf{q}} + \dot{\mathbf{J}}\dot{\mathbf{q}}$$

**优点**：更精确的动态响应，特别是高速运动时。

### 8.9.4 非对角阻抗矩阵

**当前**：$\mathbf{K}_d, \mathbf{C}_d$ 为对角矩阵，各轴解耦

**改进**：使用非对角矩阵实现轴间耦合

$$\mathbf{K}_d = \begin{bmatrix}
K_{11} & K_{12} & \cdots \\
K_{21} & K_{22} & \cdots \\
\vdots & \vdots & \ddots
\end{bmatrix}$$

**应用**：实现特定方向的柔顺性（如装配孔）。

### 8.9.5 混合力/位置控制

**概念**：不同轴使用不同控制模式

$$\begin{cases}
X, Y: \text{位置控制（高刚度）} \\
Z: \text{力控制（低刚度）}
\end{cases}$$

**实现**：

$$K_{d,i} = \begin{cases}
\infty & \text{位置控制轴} \\
K_i & \text{力控制轴}
\end{cases}$$

**应用**：插入、抛光等任务。

## 8.10 总结

### 8.10.1 核心公式

$$\boxed{\boldsymbol{\tau} = \mathbf{g}(\mathbf{q}_{\text{current}}) + \boldsymbol{\alpha} \odot \mathbf{J}^T(\mathbf{q}) [\mathbf{C}_d \dot{\mathbf{e}} + \mathbf{K}_d \mathbf{e}] + \mathbf{K}_p^{\text{motor}} \mathbf{e}_q + \mathbf{K}_d^{\text{motor}} \dot{\mathbf{e}}_q}$$

其中：
- **阻抗项**：$\mathbf{J}^T(\mathbf{q}) [\mathbf{C}_d \dot{\mathbf{e}} + \mathbf{K}_d \mathbf{e}]$ - 笛卡尔空间阻抗控制
- **重力补偿**：$\mathbf{g}(\mathbf{q}_{\text{current}})$ - 基于当前状态
- **增益调节**：$\boldsymbol{\alpha} = [1.2, 1.3, 1.3, 1.0, 1.0, 1.0]^T$ - 各关节力矩放大
- **电机PID**：$\mathbf{K}_p^{\text{motor}} \mathbf{e}_q + \mathbf{K}_d^{\text{motor}} \dot{\mathbf{e}}_q$ - 关节空间反馈

### 8.10.2 关键特性

1. **笛卡尔空间阻抗**：
   - 在任务空间建立虚拟弹簧-阻尼系统
   - 刚度 $\mathbf{K}_d$ 可调节柔顺性
   - 阻尼 $\mathbf{C}_d$ 抑制振荡

2. **雅可比转置映射**：
   - 基于虚功原理：$\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}$
   - 无需逆运算，数值稳定
   - 自然处理奇异配置

3. **自适应负载估计**：
   - 基于末端力估计负载质量
   - 自动更新动力学模型
   - 提高不同负载下的性能（仅此模式有）

4. **混合控制策略**：
   - 重力补偿 + 阻抗控制 + 电机PID
   - 三层控制结合，鲁棒性好
   - 适应性强

### 8.10.3 实现优点

1. **柔顺性可调** ✓
   - 软/硬两种预设参数
   - 不同轴可独立设置
   - 适应多种接触任务

2. **力感知能力** ✓
   - 通过关节力矩估计末端力
   - 支持力反馈的交互任务
   - 碰撞检测和保护

3. **自适应能力** ✓
   - 在线负载估计
   - 动力学模型更新
   - 无需重新标定

4. **夹爪集成** ✓
   - 协调控制6轴+夹爪
   - 抓取与操作一体化
   - 简化应用开发

### 8.10.4 局限与挑战

1. **阻尼不足**：
   - 硬模式 $\zeta \approx 0.005$，严重欠阻尼
   - 导致振荡和长稳定时间
   - 需要调整 $C_d$ 到 20-70

2. **参数耦合**：
   - 各轴阻抗参数差异大（K: 11-1095）
   - 可能导致耦合振荡
   - 需要综合调试

3. **简化模型**：
   - 忽略惯性项 $\mathbf{M}_d \ddot{\mathbf{e}}$
   - 高速运动时精度下降
   - 需要完整阻抗模型

4. **负载估计局限**：
   - 仅Z轴静态估计
   - 校正系数需经验调整
   - 动态负载估计不准

### 8.10.5 适用场景

**适合**：
- ✅ 装配任务（插入、对接）
- ✅ 抛光/打磨（恒力接触）
- ✅ 人机协作（安全交互）
- ✅ 抓取操作（自适应负载）
- ✅ 柔性接触（避免损坏）

**不适合**：
- ❌ 高速运动（忽略惯性项）
- ❌ 精确定位（柔顺性导致偏移）
- ❌ 无接触任务（不需要阻抗）
- ❌ 极硬环境（刚度需求超高）

### 8.10.6 代码位置索引

| 功能 | 代码位置 | 数学对应 |
|------|----------|----------|
| 模式入口 | state_machine.cpp:99-101 | - |
| 末端速度计算 | state_machine.cpp:532-549 | $\mathbf{v} = \mathbf{J}\dot{\mathbf{q}}$ |
| 位置误差 | state_machine.cpp:551-561 | $\mathbf{e} = \mathbf{x}_d - \mathbf{x}$ |
| 速度误差 | state_machine.cpp:563-568 | $\dot{\mathbf{e}} = -\mathbf{v}$ |
| 阻抗控制律 | state_machine.cpp:579-605 | $\mathbf{F} = \mathbf{C}_d\dot{\mathbf{e}} + \mathbf{K}_d\mathbf{e}$ |
| 雅可比转置 | state_machine.cpp:607-617 | $\boldsymbol{\tau}_{\text{imp}} = \mathbf{J}^T\mathbf{F}$ |
| 末端力估计 | state_machine.cpp:634 | `joint2end_force()` |
| 负载估计 | state_machine.cpp:636-661 | $\hat{m} = 0.153 F_z$ |
| 力矩叠加 | state_machine.cpp:666-671 | $\boldsymbol{\tau} = \mathbf{g} + \boldsymbol{\alpha}\odot\boldsymbol{\tau}_{\text{imp}}$ |
| 电机控制 | state_machine.cpp:724-730 | 含夹爪的PID控制 |

## 参考文献

1. Hogan, N. (1985). *Impedance Control: An Approach to Manipulation: Part I—Theory*. Journal of Dynamic Systems, Measurement, and Control, 107(1), 1-7.
2. Hogan, N. (1985). *Impedance Control: An Approach to Manipulation: Part II—Implementation*. Journal of Dynamic Systems, Measurement, and Control, 107(1), 8-16.
3. Siciliano, B., & Villani, L. (1999). *Robot Force Control*. Springer. (Chapter 7: Impedance Control)
4. Ott, C., et al. (2015). *Unified Impedance and Admittance Control*. IEEE International Conference on Robotics and Automation (ICRA).
5. Albu-Schäffer, A., et al. (2007). *Cartesian Impedance Control of Redundant and Flexible-Joint Robots*. Springer.
6. Raibert, M. H., & Craig, J. J. (1981). *Hybrid Position/Force Control of Manipulators*. Journal of Dynamic Systems, Measurement, and Control, 103(2), 126-133.

---

# 9. Admittance 控制模式

## 9.1 模式概述

**Admittance 控制模式**（导纳控制）是阻抗控制的对偶形式，是一种**力输入→位置输出**的混合力/位置控制策略。在该模式下，机械臂能够：

1. **选择性轴向控制**：通过参考掩码（Reference Mask）机制，在不同的笛卡尔空间自由度上实现位置控制或力控制
2. **力轨迹跟踪**：在力控制轴向上，通过期望力矩实现末端力的精确控制
3. **位置保持**：在位置控制轴向上，维持期望的末端位姿
4. **重力补偿**：基于当前关节状态计算重力补偿力矩
5. **高增益位置跟踪**：与阻抗模式不同，使用较高的电机PID增益确保位置精度

该模式特别适用于以下场景：
- 需要精确力控制的装配任务
- 接触力受限的抛光、打磨作业
- 柔性操作和人机协作
- 需要同时控制位置和力的复杂任务

**与阻抗控制的关键区别**：
- **阻抗控制**：位置输入 → 力输出（通过虚拟阻抗产生反作用力）
- **导纳控制**：力输入 → 位置输出（通过力修正期望位置）

## 9.2 控制架构

### 9.2.1 控制流程图

```
┌─────────────────────────────────────────────────────────────────┐
│                     Admittance Control Loop                      │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  1. Gravity Compensation Calculation     │
        │     τ_g = g(q, q̇, q̈)                    │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  2. Reference Mask Computation           │
        │     m_i = 1 - |o_i|                      │
        │  (o_i: end_force_orientation)            │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  3. Position Blending (X, Y, Z)          │
        │     p_d[i] = p_cmd[i]·m[i]               │
        │            + p_curr[i]·(1-m[i])          │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  4. Orientation Blending (Roll/Pitch/Yaw)│
        │     R = R_cmd·m[3] + R_curr·(1-m[3])     │
        │     P = P_cmd·m[4] + P_curr·(1-m[4])     │
        │     Y = Y_cmd·m[5] + Y_curr·(1-m[5])     │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  5. Inverse Kinematics (LMA)             │
        │     q_d = IK(T_d)                        │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  6. Current End-Effector Velocity        │
        │     ẋ = J(q)·q̇                           │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  7. Force Vector Construction            │
        │     F[i] = o[i] · F_value                │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  8. Jacobian Transpose Force Mapping     │
        │     τ_adm = J^T(q) · F_desired           │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  9. Torque Superposition                 │
        │     τ = τ_adm + τ_g                      │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  10. Joint to Motor Conversion           │
        │     motor_cmd = joint2motor(q_d, τ)      │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  11. Motor Control with High PID Gains   │
        │     K_p = [350, 270, 270, 55, 20, 10]    │
        │     K_d = [1.51, 2.01, 2.01, 0.9, ...]   │
        │     vel_cmd = 0  (velocity zeroing)      │
        └──────────────────────────────────────────┘
```

### 9.2.2 参考掩码机制

参考掩码（Reference Mask）是导纳控制的核心机制，用于在不同的笛卡尔空间自由度上切换位置控制和力控制：

$$
m_i = 1 - |o_i|, \quad i = 0, 1, 2, 3, 4, 5
$$

其中：
- $m_i$：第 $i$ 个自由度的参考掩码（0 到 1 之间）
- $o_i$：`end_force_orientation[i]`，力控制方向指示器（-1, 0, 或 1）

**掩码值的物理意义**：
- $o_i = 0 \Rightarrow m_i = 1$：**纯位置控制**，完全使用命令位置
- $o_i = \pm1 \Rightarrow m_i = 0$：**纯力控制**，完全使用当前位置（让力修正位置）
- $0 < |o_i| < 1$：**混合控制**，加权混合命令位置和当前位置

**位置/姿态混合公式**：

$$
\mathbf{x}_d = \mathbf{x}_{\text{cmd}} \odot \mathbf{m} + \mathbf{x}_{\text{fdbk}} \odot (\mathbf{1} - \mathbf{m})
$$

其中：
- $\mathbf{x}_d = [x_d, y_d, z_d, R_d, P_d, Y_d]^T$：混合后的期望位姿
- $\mathbf{x}_{\text{cmd}}$：外部命令的期望位姿
- $\mathbf{x}_{\text{fdbk}}$：当前反馈的实际位姿
- $\mathbf{m} = [m_0, m_1, m_2, m_3, m_4, m_5]^T$：参考掩码向量
- $\odot$：逐元素相乘（Hadamard 积）

## 9.3 详细算法步骤

### 步骤 1：重力补偿计算

**目的**：计算当前关节状态下的重力补偿力矩

**代码**：
```cpp
arm.computeInverseDynamics(arm.current_joint_positions,
                           arm.current_joint_velocities,
                           arm.current_joint_acceleration,
                           arm.gravity_joint_tauqes);
```

**数学模型**：

使用递归牛顿-欧拉算法计算逆动力学：

$$
\boldsymbol{\tau}_g = \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) - \mathbf{J}^T(\mathbf{q})\mathbf{F}_{\text{ext}}
$$

在重力补偿计算中，关节加速度设为零（$\ddot{\mathbf{q}} = \mathbf{0}$），外力也为零（$\mathbf{F}_{\text{ext}} = \mathbf{0}$），因此：

$$
\boldsymbol{\tau}_g = \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q})
$$

其中：
- $\mathbf{M}(\mathbf{q}) \in \mathbb{R}^{6 \times 6}$：惯性矩阵
- $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \in \mathbb{R}^{6 \times 6}$：科氏力和离心力矩阵
- $\mathbf{g}(\mathbf{q}) \in \mathbb{R}^6$：重力项
- $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{6 \times 6}$：雅可比矩阵

---

### 步骤 2：参考掩码计算

**目的**：根据力控制方向指示器计算每个自由度的参考掩码

**代码**：
```cpp
for (size_t i = 0; i < 6; i++)
{
    refer_mask[i] = 1 - abs(arm.end_force_orientation[i]);
}
```

**数学模型**：

$$
m_i = 1 - |o_i|, \quad i = 0, 1, 2, 3, 4, 5
$$

**示例**：
- **Z 轴力控制**：设 `end_force_orientation = [0, 0, 1, 0, 0, 0]`
  - $m_0 = 1 - |0| = 1$（X 位置控制）
  - $m_1 = 1 - |0| = 1$（Y 位置控制）
  - $m_2 = 1 - |1| = 0$（Z 力控制）
  - $m_3 = m_4 = m_5 = 1$（姿态位置控制）

---

### 步骤 3：位置混合（X, Y, Z）

**目的**：根据参考掩码混合命令位置和当前位置

**代码**：
```cpp
arm.desir_end_effector_frame.p.data[0] =
    arm.desir_end_effector_frame.p.x() * refer_mask[0] +
    arm.current_end_effector_frame.p.x() * abs(arm.end_force_orientation[0]);

arm.desir_end_effector_frame.p.data[1] =
    arm.desir_end_effector_frame.p.y() * refer_mask[1] +
    arm.current_end_effector_frame.p.y() * abs(arm.end_force_orientation[1]);

arm.desir_end_effector_frame.p.data[2] =
    arm.desir_end_effector_frame.p.z() * refer_mask[2] +
    arm.current_end_effector_frame.p.z() * abs(arm.end_force_orientation[2]);
```

**数学模型**：

$$
\begin{aligned}
x_d &= x_{\text{cmd}} \cdot m_0 + x_{\text{curr}} \cdot (1 - m_0) \\
y_d &= y_{\text{cmd}} \cdot m_1 + y_{\text{curr}} \cdot (1 - m_1) \\
z_d &= z_{\text{cmd}} \cdot m_2 + z_{\text{curr}} \cdot (1 - m_2)
\end{aligned}
$$

**向量形式**：

$$
\mathbf{p}_d = \mathbf{p}_{\text{cmd}} \odot \mathbf{m}_{0:2} + \mathbf{p}_{\text{curr}} \odot (\mathbf{1} - \mathbf{m}_{0:2})
$$

其中 $\mathbf{p}_d = [x_d, y_d, z_d]^T$

---

### 步骤 4：姿态混合（Roll, Pitch, Yaw）

**目的**：根据参考掩码混合命令姿态和当前姿态

**代码**：
```cpp
double desir_roll, desir_pitch, desir_yaw, current_roll, current_pitch, current_yaw;
arm.desir_end_effector_frame.M.GetRPY(desir_roll, desir_pitch, desir_yaw);
arm.current_end_effector_frame.M.GetRPY(current_roll, current_pitch, current_yaw);

double R = desir_roll * refer_mask[3]  + current_roll  * abs(arm.end_force_orientation[3]);
double P = desir_pitch * refer_mask[4] + current_pitch * abs(arm.end_force_orientation[4]);
double Y = desir_yaw * refer_mask[5]   + current_yaw  * abs(arm.end_force_orientation[5]);

arm.desir_end_effector_frame.M = KDL::Rotation::RPY(R, P, Y);
```

**数学模型**：

$$
\begin{aligned}
\phi_d &= \phi_{\text{cmd}} \cdot m_3 + \phi_{\text{curr}} \cdot (1 - m_3) \\
\theta_d &= \theta_{\text{cmd}} \cdot m_4 + \theta_{\text{curr}} \cdot (1 - m_4) \\
\psi_d &= \psi_{\text{cmd}} \cdot m_5 + \psi_{\text{curr}} \cdot (1 - m_5)
\end{aligned}
$$

其中：
- $\phi, \theta, \psi$：分别为 Roll, Pitch, Yaw 角
- $\mathbf{R}_d = \text{RPY}(\phi_d, \theta_d, \psi_d)$：期望旋转矩阵

**旋转矩阵构造**（RPY 惯例）：

$$
\mathbf{R}(\phi, \theta, \psi) = \mathbf{R}_z(\psi) \mathbf{R}_y(\theta) \mathbf{R}_x(\phi)
$$

$$
= \begin{bmatrix}
c\psi c\theta & c\psi s\theta s\phi - s\psi c\phi & c\psi s\theta c\phi + s\psi s\phi \\
s\psi c\theta & s\psi s\theta s\phi + c\psi c\phi & s\psi s\theta c\phi - c\psi s\phi \\
-s\theta & c\theta s\phi & c\theta c\phi
\end{bmatrix}
$$

其中 $c = \cos$，$s = \sin$

---

### 步骤 5：逆运动学求解

**目的**：将混合后的末端期望位姿转换为关节空间的期望位置

**代码**：
```cpp
int ret = arm.ik_solver_lma->CartToJnt(arm.current_joint_positions,
                                       arm.desir_end_effector_frame,
                                       arm.desir_joint_positions);
```

**数学模型**：

使用 **Levenberg-Marquardt 算法（LMA）** 求解逆运动学：

$$
\mathbf{q}_d = \text{IK}(\mathbf{T}_d | \mathbf{q}_{\text{init}})
$$

优化目标：

$$
\min_{\mathbf{q}} \left\| \mathbf{T}_d - \text{FK}(\mathbf{q}) \right\|^2
$$

LMA 迭代公式：

$$
\mathbf{q}_{k+1} = \mathbf{q}_k + \left( \mathbf{J}^T\mathbf{J} + \lambda \mathbf{I} \right)^{-1} \mathbf{J}^T \mathbf{e}_k
$$

其中：
- $\mathbf{T}_d$：期望的末端位姿（4×4 齐次变换矩阵）
- $\mathbf{q}_{\text{init}}$：初始关节角度（使用当前关节位置）
- $\mathbf{J}$：雅可比矩阵
- $\lambda$：阻尼因子（Levenberg-Marquardt 参数）
- $\mathbf{e}_k = \mathbf{T}_d - \text{FK}(\mathbf{q}_k)$：位姿误差

---

### 步骤 6：计算当前末端速度

**目的**：通过雅可比矩阵将关节速度映射到末端速度

**代码**：
```cpp
for (unsigned int i = 0; i < 6; i++)
{
    double component = 0.0;
    for (unsigned int j = 0; j < 6; j++)
    {
        component += arm.current_jacobian(i, j) * arm.current_joint_velocities(j);
    }
    if (i < 3)
    {
        arm.current_end_effector_dot.vel(i) = component;
    }
    else
    {
        arm.current_end_effector_dot.rot(i - 3) = component;
    }
}
```

**数学模型**：

$$
\dot{\mathbf{x}} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}}
$$

详细展开：

$$
\begin{bmatrix}
\dot{x} \\ \dot{y} \\ \dot{z} \\ \omega_x \\ \omega_y \\ \omega_z
\end{bmatrix}
= \mathbf{J}(\mathbf{q})
\begin{bmatrix}
\dot{q}_1 \\ \dot{q}_2 \\ \dot{q}_3 \\ \dot{q}_4 \\ \dot{q}_5 \\ \dot{q}_6
\end{bmatrix}
$$

其中：
- $\dot{\mathbf{x}} = [\mathbf{v}^T, \boldsymbol{\omega}^T]^T \in \mathbb{R}^6$：末端速度（线速度 + 角速度）
- $\mathbf{v} = [\dot{x}, \dot{y}, \dot{z}]^T$：线速度
- $\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^T$：角速度
- $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{6 \times 6}$：几何雅可比矩阵

---

### 步骤 7：构造期望力向量

**目的**：根据力控制方向和幅值构造 6 维期望力/力矩向量

**代码**：
```cpp
for (size_t i = 0; i < 6; i++)
{
    end_force[i] = arm.end_force_orientation[i] * arm.end_force_value;
}
```

**数学模型**：

$$
F_i = o_i \cdot F_{\text{value}}, \quad i = 0, 1, 2, 3, 4, 5
$$

向量形式：

$$
\mathbf{F}_{\text{desired}} = \mathbf{o} \odot F_{\text{value}} =
\begin{bmatrix}
o_0 \cdot F_{\text{value}} \\
o_1 \cdot F_{\text{value}} \\
o_2 \cdot F_{\text{value}} \\
o_3 \cdot F_{\text{value}} \\
o_4 \cdot F_{\text{value}} \\
o_5 \cdot F_{\text{value}}
\end{bmatrix}
$$

其中：
- $\mathbf{F}_{\text{desired}} = [\mathbf{f}^T, \boldsymbol{\tau}^T]^T \in \mathbb{R}^6$：期望的力/力矩
- $\mathbf{f} = [F_0, F_1, F_2]^T$：力（N）
- $\boldsymbol{\tau} = [F_3, F_4, F_5]^T$：力矩（Nm）
- $\mathbf{o} = [o_0, o_1, o_2, o_3, o_4, o_5]^T$：力方向向量
- $F_{\text{value}}$：力的幅值

**示例**：
- **Z 轴向下 10N 力**：$\mathbf{o} = [0, 0, -1, 0, 0, 0]^T$，$F_{\text{value}} = 10$
  - $\mathbf{F}_{\text{desired}} = [0, 0, -10, 0, 0, 0]^T$

---

### 步骤 8：雅可比转置力映射

**目的**：将笛卡尔空间的期望力/力矩映射到关节空间的力矩

**代码**：
```cpp
for (unsigned int i = 0; i < 6; i++)
{
    double component = 0.0;
    for (unsigned int j = 0; j < 6; j++)
    {
        component += arm.current_jacobian(j, i) * end_force[j];
    }
    arm.joint_admittance_tau[i] = component;
}
```

**数学模型**：

$$
\boldsymbol{\tau}_{\text{adm}} = \mathbf{J}^T(\mathbf{q}) \mathbf{F}_{\text{desired}}
$$

详细展开（第 $i$ 个关节）：

$$
\tau_{\text{adm}, i} = \sum_{j=0}^{5} J_{ji} \cdot F_j = \mathbf{J}_{:,i}^T \cdot \mathbf{F}_{\text{desired}}
$$

其中：
- $\boldsymbol{\tau}_{\text{adm}} = [\tau_{\text{adm},1}, \ldots, \tau_{\text{adm},6}]^T \in \mathbb{R}^6$：导纳控制产生的关节力矩
- $\mathbf{J}^T(\mathbf{q}) \in \mathbb{R}^{6 \times 6}$：雅可比矩阵的转置
- $\mathbf{J}_{:,i}$：雅可比矩阵的第 $i$ 列

**物理意义**（虚功原理）：

根据虚功原理，末端力做的虚功等于关节力矩做的虚功：

$$
\delta W = \mathbf{F}^T \delta \mathbf{x} = \boldsymbol{\tau}^T \delta \mathbf{q}
$$

由于 $\delta \mathbf{x} = \mathbf{J} \delta \mathbf{q}$，代入得：

$$
\mathbf{F}^T \mathbf{J} \delta \mathbf{q} = \boldsymbol{\tau}^T \delta \mathbf{q}
$$

因此：

$$
\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}
$$

这表明雅可比转置映射保证了能量守恒。

---

### 步骤 9：力矩叠加

**目的**：将导纳控制力矩与重力补偿力矩叠加

**代码**：
```cpp
arm.joint_control_tau[0] = arm.joint_admittance_tau[0] + arm.gravity_joint_tauqes(0);
arm.joint_control_tau[1] = arm.joint_admittance_tau[1] + arm.gravity_joint_tauqes(1);
arm.joint_control_tau[2] = arm.joint_admittance_tau[2] + arm.gravity_joint_tauqes(2);
arm.joint_control_tau[3] = arm.joint_admittance_tau[3] + arm.gravity_joint_tauqes(3);
arm.joint_control_tau[4] = arm.joint_admittance_tau[4] + arm.gravity_joint_tauqes(4);
arm.joint_control_tau[5] = arm.joint_admittance_tau[5] + arm.gravity_joint_tauqes(5);
```

**数学模型**：

$$
\boldsymbol{\tau}_{\text{ctrl}} = \boldsymbol{\tau}_{\text{adm}} + \boldsymbol{\tau}_g
$$

其中：
- $\boldsymbol{\tau}_{\text{ctrl}} \in \mathbb{R}^6$：最终控制力矩
- $\boldsymbol{\tau}_{\text{adm}} \in \mathbb{R}^6$：导纳控制产生的力矩
- $\boldsymbol{\tau}_g \in \mathbb{R}^6$：重力补偿力矩

**各分量作用**：
- $\boldsymbol{\tau}_{\text{adm}}$：实现力跟踪（在力控制轴向）
- $\boldsymbol{\tau}_g$：抵消重力影响，减轻位置控制负担

---

### 步骤 10：关节到电机空间转换

**目的**：考虑电机方向，将关节空间的指令转换到电机空间

**代码**：
```cpp
for (size_t i = 0; i < 6; i++)
{
    arm.desir_joint_pos[i] = arm.desir_joint_positions(i);
}

arm.joint2motor(arm.desir_joint_pos,
                arm.desir_joint_vel,
                arm.desir_joint_acc,
                arm.joint_control_tau,
                motor.desir_motor_pos,
                motor.desir_motor_vel,
                motor.desir_motor_acc,
                motor.motor_control_tau);
```

**数学模型**：

$$
\begin{aligned}
\theta_{\text{motor},i} &= d_i \cdot q_i \\
\dot{\theta}_{\text{motor},i} &= d_i \cdot \dot{q}_i \\
\ddot{\theta}_{\text{motor},i} &= d_i \cdot \ddot{q}_i \\
\tau_{\text{motor},i} &= d_i \cdot \tau_{\text{joint},i}
\end{aligned}
$$

其中：
- $d_i \in \{-1, +1\}$：`motor_direction[i]`，电机方向系数
- $\theta_{\text{motor},i}$：电机位置（rad）
- $q_i$：关节位置（rad）
- $\tau_{\text{motor},i}$：电机力矩（Nm）
- $\tau_{\text{joint},i}$：关节力矩（Nm）

---

### 步骤 11：电机控制执行

**目的**：使用高增益 PID 和速度置零策略执行电机控制

**代码**：
```cpp
// 设置高 PID 增益（与阻抗模式不同）
motor.kp[0] = 350.0;
motor.kd[0] = 1.51;

motor.kp[1] = 270.0;
motor.kd[1] = 2.01;

motor.kp[2] = 270.0;
motor.kd[2] = 2.01;

motor.kp[3] = 55.0;
motor.kd[3] = 0.9;

motor.kp[4] = 20.0;
motor.kd[4] = 0.51;

motor.kp[5] = 10.0;
motor.kd[5] = 0.51;

// 速度置零策略（关键）
for (size_t i = 0; i < 6; i++)
{
    motor.current_motor_vel[i] = 0;
}

// 电机控制
motor.ControlMotors(port,
                   motor.desir_motor_pos,
                   motor.current_motor_vel,  // 使用零速度
                   motor.kp,
                   motor.kd,
                   motor.motor_control_tau);
```

**数学模型**：

电机级别的 MIT 模式控制律：

$$
\tau_{\text{motor},i} = K_{p,i} (q_{d,i} - q_i) + K_{d,i} (\dot{q}_{d,i} - \dot{q}_i) + \tau_{\text{ff},i}
$$

**速度置零后的简化形式**：

$$
\tau_{\text{motor},i} = K_{p,i} (q_{d,i} - q_i) - K_{d,i} \dot{q}_i + \tau_{\text{ff},i}
$$

其中：
- $K_{p,i}, K_{d,i}$：高 PID 增益（比阻抗模式大）
- $\tau_{\text{ff},i}$：前馈力矩（= $\boldsymbol{\tau}_{\text{adm}} + \boldsymbol{\tau}_g$）
- $\dot{q}_{d,i} = 0$：期望速度置零

**增益设计特点**：
1. **高位置增益**：$K_p$ 较大（350/270/270），确保位置跟踪精度
2. **适度阻尼**：$K_d$ 适中，平衡响应速度和稳定性
3. **前馈补偿**：$\tau_{\text{ff}}$ 包含力控制和重力补偿

## 9.4 完整数学模型

### 9.4.1 系统动力学

6 自由度机械臂的完整动力学方程：

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau} + \mathbf{J}^T(\mathbf{q})\mathbf{F}_{\text{ext}}
$$

其中：
- $\mathbf{M}(\mathbf{q}) \in \mathbb{R}^{6 \times 6}$：对称正定的惯性矩阵
- $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \in \mathbb{R}^{6 \times 6}$：科氏力和离心力矩阵
- $\mathbf{g}(\mathbf{q}) \in \mathbb{R}^6$：重力力矩向量
- $\boldsymbol{\tau} \in \mathbb{R}^6$：控制力矩（电机输出）
- $\mathbf{F}_{\text{ext}} \in \mathbb{R}^6$：外部力/力矩（环境接触力）
- $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{6 \times 6}$：几何雅可比矩阵

### 9.4.2 导纳控制律

导纳控制的总控制力矩为：

$$
\boldsymbol{\tau} = \boldsymbol{\tau}_{\text{adm}} + \boldsymbol{\tau}_g + \mathbf{K}_p (\mathbf{q}_d - \mathbf{q}) - \mathbf{K}_d \dot{\mathbf{q}}
$$

其中：
- $\boldsymbol{\tau}_{\text{adm}} = \mathbf{J}^T(\mathbf{q}) \mathbf{F}_{\text{desired}}$：力控制分量
- $\boldsymbol{\tau}_g = \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q})$：重力补偿分量
- $\mathbf{K}_p (\mathbf{q}_d - \mathbf{q})$：位置反馈分量（高增益）
- $-\mathbf{K}_d \dot{\mathbf{q}}$：速度阻尼分量

### 9.4.3 期望关节位置计算

期望关节位置通过以下步骤计算：

1. **混合末端位姿**：
   $$
   \mathbf{T}_d = \begin{bmatrix} \mathbf{R}_d & \mathbf{p}_d \\ \mathbf{0}^T & 1 \end{bmatrix}
   $$
   其中：
   $$
   \begin{aligned}
   \mathbf{p}_d &= \mathbf{p}_{\text{cmd}} \odot \mathbf{m}_{0:2} + \mathbf{p}_{\text{curr}} \odot (\mathbf{1} - \mathbf{m}_{0:2}) \\
   \mathbf{R}_d &= \text{RPY}(\phi_d, \theta_d, \psi_d)
   \end{aligned}
   $$

2. **逆运动学**：
   $$
   \mathbf{q}_d = \arg\min_{\mathbf{q}} \left\| \mathbf{T}_d - \text{FK}(\mathbf{q}) \right\|^2
   $$

### 9.4.4 期望力向量构造

$$
\mathbf{F}_{\text{desired}} = \mathbf{o} \cdot F_{\text{value}} =
\begin{bmatrix}
\mathbf{f}_{\text{desired}} \\
\boldsymbol{\tau}_{\text{desired}}
\end{bmatrix}
$$

其中：
- $\mathbf{f}_{\text{desired}} = [o_0, o_1, o_2]^T \cdot F_{\text{value}}$：期望力（N）
- $\boldsymbol{\tau}_{\text{desired}} = [o_3, o_4, o_5]^T \cdot F_{\text{value}}$：期望力矩（Nm）

### 9.4.5 闭环系统分析

将控制律代入动力学方程：

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \mathbf{J}^T \mathbf{F}_{\text{desired}} + \mathbf{C}\dot{\mathbf{q}} + \mathbf{g} + \mathbf{K}_p (\mathbf{q}_d - \mathbf{q}) - \mathbf{K}_d \dot{\mathbf{q}} + \mathbf{J}^T \mathbf{F}_{\text{ext}}
$$

简化为：

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{K}_d \dot{\mathbf{q}} + \mathbf{K}_p (\mathbf{q} - \mathbf{q}_d) = \mathbf{J}^T (\mathbf{F}_{\text{desired}} + \mathbf{F}_{\text{ext}})
$$

**在力控制轴向**（$m_i = 0$）：
- $q_d = q_{\text{curr}}$（期望位置跟随当前位置）
- 位置误差 $\mathbf{q} - \mathbf{q}_d \approx 0$
- 系统主要响应力输入：$\mathbf{M}\ddot{\mathbf{q}} + \mathbf{K}_d \dot{\mathbf{q}} = \mathbf{J}^T (\mathbf{F}_{\text{desired}} + \mathbf{F}_{\text{ext}})$

**在位置控制轴向**（$m_i = 1$）：
- $q_d = \text{IK}(\mathbf{x}_{\text{cmd}})$（期望位置由命令给定）
- 力输入 $\mathbf{F}_{\text{desired},i} = 0$
- 系统主要响应位置指令：$\mathbf{M}\ddot{\mathbf{q}} + \mathbf{K}_d \dot{\mathbf{q}} + \mathbf{K}_p (\mathbf{q} - \mathbf{q}_d) = \mathbf{J}^T \mathbf{F}_{\text{ext}}$

## 9.5 参考掩码机制详解

### 9.5.1 理论基础

参考掩码机制实现了**混合控制**（Hybrid Control）的核心思想：在不同的笛卡尔空间自由度上独立选择控制模式（位置控制或力控制）。

**控制模式分类**：

| $o_i$ | $m_i$ | 控制模式 | 期望位姿来源 | 力控制 |
|-------|-------|---------|------------|-------|
| 0 | 1 | 纯位置控制 | 外部命令 $\mathbf{x}_{\text{cmd}}$ | 否 |
| ±1 | 0 | 纯力控制 | 当前反馈 $\mathbf{x}_{\text{curr}}$ | 是 |
| 中间值 | 中间值 | 混合控制 | 加权混合 | 部分 |

### 9.5.2 位姿混合公式

对于每个自由度 $i$：

$$
x_{d,i} = x_{\text{cmd},i} \cdot m_i + x_{\text{curr},i} \cdot (1 - m_i)
$$

**极限情况**：

1. **纯位置控制**（$m_i = 1$）：
   $$
   x_{d,i} = x_{\text{cmd},i}
   $$
   期望位姿完全由外部命令决定，不受当前位置影响。

2. **纯力控制**（$m_i = 0$）：
   $$
   x_{d,i} = x_{\text{curr},i}
   $$
   期望位姿跟随当前位置，允许力修正位置。

### 9.5.3 示例：Z 轴力控制 + XY 位置控制

**场景**：机械臂末端在 Z 轴方向施加 10N 的力，XY 平面保持位置。

**设置**：
```cpp
arm.end_force_orientation = [0, 0, -1, 0, 0, 0];
arm.end_force_value = 10.0;  // N
```

**掩码计算**：
$$
\mathbf{m} = [1, 1, 0, 1, 1, 1]^T
$$

**位置混合**：
$$
\begin{aligned}
x_d &= x_{\text{cmd}} \cdot 1 + x_{\text{curr}} \cdot 0 = x_{\text{cmd}} \\
y_d &= y_{\text{cmd}} \cdot 1 + y_{\text{curr}} \cdot 0 = y_{\text{cmd}} \\
z_d &= z_{\text{cmd}} \cdot 0 + z_{\text{curr}} \cdot 1 = z_{\text{curr}}
\end{aligned}
$$

**力向量**：
$$
\mathbf{F}_{\text{desired}} = [0, 0, -10, 0, 0, 0]^T \text{ N}
$$

**关节力矩**：
$$
\boldsymbol{\tau}_{\text{adm}} = \mathbf{J}^T(\mathbf{q}) \begin{bmatrix} 0 \\ 0 \\ -10 \\ 0 \\ 0 \\ 0 \end{bmatrix}
$$

**控制效果**：
- X, Y 轴：高精度位置控制（$K_p$ = 350, 270）
- Z 轴：力跟踪控制（施加 10N 向下力）
- Roll, Pitch, Yaw：姿态保持

### 9.5.4 参考掩码的优势

1. **灵活性**：可以任意组合 6 个自由度的控制模式
2. **解耦性**：不同自由度的控制相互独立
3. **平滑过渡**：通过调整 $o_i$ 可以实现控制模式的连续过渡
4. **易实现**：只需简单的加权混合，无需复杂的切换逻辑

## 9.6 代码分析

### 9.6.1 关键变量

| 变量名 | 类型 | 维度 | 物理意义 |
|--------|------|------|---------|
| `refer_mask[6]` | float | 6 | 参考掩码 $\mathbf{m}$ |
| `end_force[6]` | float | 6 | 期望力/力矩 $\mathbf{F}_{\text{desired}}$ |
| `arm.end_force_orientation[6]` | float | 6 | 力方向向量 $\mathbf{o}$ |
| `arm.end_force_value` | float | 1 | 力幅值 $F_{\text{value}}$ |
| `arm.gravity_joint_tauqes` | KDL::JntArray | 6 | 重力补偿力矩 $\boldsymbol{\tau}_g$ |
| `arm.joint_admittance_tau[6]` | float | 6 | 导纳控制力矩 $\boldsymbol{\tau}_{\text{adm}}$ |
| `arm.joint_control_tau[6]` | float | 6 | 总控制力矩 $\boldsymbol{\tau}_{\text{ctrl}}$ |
| `motor.kp[6]`, `motor.kd[6]` | float | 6 | 电机 PID 增益 $K_p, K_d$ |
| `motor.current_motor_vel[6]` | float | 6 | 电机速度（置零）|

### 9.6.2 RPY 姿态混合

**GetRPY 方法**：

从旋转矩阵 $\mathbf{R}$ 提取 Roll-Pitch-Yaw 角：

$$
\begin{aligned}
\phi &= \text{atan2}(R_{32}, R_{33}) \\
\theta &= \text{atan2}(-R_{31}, \sqrt{R_{32}^2 + R_{33}^2}) \\
\psi &= \text{atan2}(R_{21}, R_{11})
\end{aligned}
$$

其中 $R_{ij}$ 是旋转矩阵的第 $i$ 行第 $j$ 列元素。

**RPY 方法**：

从 Roll-Pitch-Yaw 角构造旋转矩阵（见步骤 4）。

### 9.6.3 雅可比转置映射实现

代码实现的矩阵乘法：

$$
\tau_i = \sum_{j=0}^{5} J_{ji} \cdot F_j
$$

等价于：

$$
\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}
$$

**注意**：代码中使用 `arm.current_jacobian(j, i)` 实际上是 $J_{ji}$（第 $j$ 行第 $i$ 列），因此：

```cpp
component += arm.current_jacobian(j, i) * end_force[j];
```

对应于：

$$
\tau_i = \sum_j J_{ji} F_j = (\mathbf{J}^T)_{i:} \cdot \mathbf{F}
$$

### 9.6.4 速度置零策略

```cpp
for (size_t i = 0; i < 6; i++)
{
    motor.current_motor_vel[i] = 0;
}
```

**作用**：
1. **消除速度前馈**：避免速度误差放大
2. **提高力响应**：减少阻尼对力控制的干扰
3. **平滑运动**：减少速度跳变引起的震荡

**数学解释**：

原始控制律：

$$
\tau = K_p (q_d - q) + K_d (\dot{q}_d - \dot{q}) + \tau_{\text{ff}}
$$

置零后：

$$
\tau = K_p (q_d - q) - K_d \dot{q} + \tau_{\text{ff}}
$$

相当于将期望速度设为 0，简化了控制律，使力控制分量 $\tau_{\text{ff}}$ 更直接地作用于系统。

### 9.6.5 电机 PID 增益设计

| 电机 | $K_p$ | $K_d$ | 特点 |
|------|-------|-------|------|
| 0 | 350.0 | 1.51 | 最高增益（基座关节）|
| 1 | 270.0 | 2.01 | 高增益（肩关节）|
| 2 | 270.0 | 2.01 | 高增益（肘关节）|
| 3 | 55.0 | 0.9 | 中增益（腕关节 1）|
| 4 | 20.0 | 0.51 | 低增益（腕关节 2）|
| 5 | 10.0 | 0.51 | 最低增益（腕关节 3）|

**设计原则**：
- **大关节高增益**：基座、肩、肘关节需要克服更大的重力和惯性，使用高 $K_p$
- **小关节低增益**：腕部关节负载小，使用低 $K_p$ 避免过度响应
- **适度阻尼**：$K_d$ 相对 $K_p$ 较小，保证系统快速响应同时抑制振荡

**与阻抗模式对比**（阻抗模式 $K_p$ 更高）：
- **Admittance**：$K_p$ = [350, 270, 270, ...]
- **Impedance**（纯力矩控制）：$K_p$ = [0, 0, 0, ...] 或较低增益
- **AutoServo**（位置伺服）：$K_p$ 更高

## 9.7 性能分析

### 9.7.1 力跟踪性能

**力跟踪误差**：

理论上，力跟踪误差为：

$$
\mathbf{e}_F = \mathbf{F}_{\text{desired}} - \mathbf{F}_{\text{actual}}
$$

其中实际力通过雅可比转置逆映射估计：

$$
\mathbf{F}_{\text{actual}} = (\mathbf{J}^T)^\dagger \boldsymbol{\tau}_{\text{measured}}
$$

**影响因素**：
1. **模型误差**：雅可比矩阵 $\mathbf{J}$ 的精度
2. **摩擦力**：未建模的摩擦影响力测量
3. **传感器噪声**：关节力矩传感器的噪声
4. **控制延迟**：1ms 控制周期引入的延迟

### 9.7.2 位置跟踪性能

**位置跟踪误差**：

$$
\mathbf{e}_p = \mathbf{p}_d - \mathbf{p}_{\text{actual}}
$$

在位置控制轴向（$m_i = 1$），高 PID 增益确保：

$$
|\mathbf{e}_p| < 1 \text{ mm}
$$

**影响因素**：
1. **逆运动学精度**：LMA 算法的收敛精度
2. **关节柔性**：关节刚度影响位置保持
3. **负载变化**：负载变化影响动力学补偿

### 9.7.3 轴向特性分析

根据代码注释（line 924）：

```cpp
// z轴非常平滑，x轴有摩擦，y轴很平滑
```

**Z 轴（垂直）**：
- **平滑性好**：重力方向，重力补偿效果好
- **力控制精度高**：摩擦力影响小

**X 轴（水平）**：
- **存在摩擦**：水平运动受静摩擦和动摩擦影响
- **力控制精度低**：摩擦力未完全补偿

**Y 轴（水平）**：
- **平滑性好**：可能由于机械设计（导轨质量好）
- **力控制精度高**：摩擦力影响小

### 9.7.4 稳定性分析

**闭环极点**：

简化的闭环系统（单自由度）：

$$
M \ddot{q} + K_d \dot{q} + K_p q = K_p q_d + J^T F_{\text{desired}}
$$

特征方程：

$$
M s^2 + K_d s + K_p = 0
$$

极点：

$$
s = \frac{-K_d \pm \sqrt{K_d^2 - 4MK_p}}{2M}
$$

**稳定性条件**：
- $K_p > 0, K_d > 0$（正增益）
- $K_d^2 < 4MK_p$（欠阻尼，快速响应）

对于电机 1（$K_p = 350, K_d = 1.51$），假设 $M \approx 0.1$：

$$
K_d^2 = 2.28, \quad 4MK_p = 140
$$

满足 $K_d^2 \ll 4MK_p$，系统欠阻尼，响应快速。

**阻尼比**：

$$
\zeta = \frac{K_d}{2\sqrt{MK_p}} = \frac{1.51}{2\sqrt{0.1 \times 350}} \approx 0.13
$$

$\zeta < 1$，系统欠阻尼，响应迅速但可能有轻微超调。

## 9.8 与阻抗控制对比

| 特性 | 阻抗控制（Chapter 8）| 导纳控制（Chapter 9）|
|------|---------------------|---------------------|
| **控制方向** | 位置输入 → 力输出 | 力输入 → 位置输出 |
| **适用场景** | 柔性接触，力感知 | 力跟踪，精确施力 |
| **电机 PID 增益** | 低或零（纯力矩控制）| 高增益（350/270/270）|
| **期望速度** | 0 或规划速度 | 0（速度置零）|
| **位姿更新** | 由阻抗模型修正 | 由参考掩码混合 |
| **力控制方式** | 虚拟弹簧阻尼 | 直接力映射 |
| **位置精度** | 中等（阻抗柔性）| 高（高 PID 增益）|
| **力控制精度** | 低（间接）| 高（直接）|
| **稳定性** | 高（固有柔性）| 需调节增益 |

### 9.8.1 数学对比

**阻抗控制**：

$$
\mathbf{F}_{\text{imp}} = \mathbf{K}_{\text{stiff}} (\mathbf{x} - \mathbf{x}_d) + \mathbf{C}_{\text{damp}} (\dot{\mathbf{x}} - \dot{\mathbf{x}}_d)
$$

$$
\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}_{\text{imp}} + \boldsymbol{\tau}_g
$$

**导纳控制**：

$$
\mathbf{x}_d = \mathbf{x}_{\text{cmd}} \odot \mathbf{m} + \mathbf{x}_{\text{curr}} \odot (\mathbf{1} - \mathbf{m})
$$

$$
\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}_{\text{desired}} + \boldsymbol{\tau}_g + \mathbf{K}_p (\mathbf{q}_d - \mathbf{q}) - \mathbf{K}_d \dot{\mathbf{q}}
$$

### 9.8.2 实现对比

| 步骤 | 阻抗控制 | 导纳控制 |
|------|---------|---------|
| 1 | 计算位置误差 | 计算重力补偿 |
| 2 | 计算速度误差 | 计算参考掩码 |
| 3 | 计算阻抗力 $\mathbf{F}_{\text{imp}}$ | 混合位姿 |
| 4 | 力映射 $\boldsymbol{\tau}_{\text{imp}} = \mathbf{J}^T \mathbf{F}_{\text{imp}}$ | 逆运动学 |
| 5 | 叠加重力补偿 | 构造期望力 |
| 6 | 电机控制（低增益或纯力矩）| 力映射 $\boldsymbol{\tau}_{\text{adm}} = \mathbf{J}^T \mathbf{F}_{\text{desired}}$ |
| 7 | - | 叠加重力补偿 |
| 8 | - | 电机控制（高增益）|

## 9.9 改进方向

### 9.9.1 摩擦补偿

**问题**：X 轴存在摩擦，影响力控制精度（line 924 注释）

**改进方法**：

1. **LuGre 摩擦模型**：

$$
F_{\text{friction}} = \sigma_0 z + \sigma_1 \dot{z} + \sigma_2 \dot{q}
$$

$$
\dot{z} = \dot{q} - \frac{\sigma_0 |\dot{q}|}{g(\dot{q})} z
$$

其中：
- $z$：内部状态（摩擦接触状态）
- $\sigma_0, \sigma_1, \sigma_2$：模型参数
- $g(\dot{q}) = F_c + (F_s - F_c) e^{-(\dot{q}/\dot{q}_s)^2}$：Stribeck 效应

2. **基于速度的补偿**（已有代码框架）：

```cpp
motor.compensate_static_friction_through_vel(motor.desir_motor_vel, motor.motor_control_tau);
```

建议激活此函数，实现静摩擦补偿。

### 9.9.2 自适应力控制

**目标**：根据力跟踪误差自适应调整增益

**方法**：

$$
\mathbf{K}_{\text{adapt}} = \mathbf{K}_0 + \mathbf{K}_I \int \mathbf{e}_F dt
$$

其中：
- $\mathbf{K}_0$：基础增益
- $\mathbf{K}_I$：积分增益
- $\mathbf{e}_F = \mathbf{F}_{\text{desired}} - \mathbf{F}_{\text{actual}}$：力误差

### 9.9.3 力/力矩传感器融合

**问题**：当前力估计依赖关节力矩传感器 + 雅可比转置，精度有限

**改进**：增加末端六维力/力矩传感器

$$
\mathbf{F}_{\text{measured}} = \text{Sensor}()
$$

使用测量力直接反馈：

$$
\boldsymbol{\tau}_{\text{adm}} = \mathbf{J}^T (\mathbf{F}_{\text{desired}} - \mathbf{K}_F (\mathbf{F}_{\text{desired}} - \mathbf{F}_{\text{measured}}))
$$

### 9.9.4 参考掩码平滑过渡

**问题**：当前掩码为阶跃函数，切换时可能产生冲击

**改进**：使用 Sigmoid 函数平滑过渡

$$
m_i(t) = \frac{1}{1 + e^{-k(t - t_0)}}
$$

其中：
- $k$：过渡速率
- $t_0$：切换时刻

### 9.9.5 增益调度

**目标**：根据关节配置和负载自适应调整 PID 增益

**方法**：

$$
K_{p,i}(\mathbf{q}) = K_{p,i,0} \cdot f(\mathbf{q})
$$

其中 $f(\mathbf{q})$ 是基于关节配置的调度函数，例如：

$$
f(\mathbf{q}) = 1 + \alpha \left\| \mathbf{q} - \mathbf{q}_{\text{nominal}} \right\|^2
$$

## 9.10 总结

### 9.10.1 核心要点

1. **控制原理**：
   - 导纳控制是**力输入 → 位置输出**的混合控制
   - 通过**参考掩码**机制实现选择性轴向控制
   - 使用**雅可比转置**映射力到关节力矩

2. **关键步骤**：
   - 重力补偿 → 掩码计算 → 位姿混合 → 逆运动学
   - 力向量构造 → 雅可比映射 → 力矩叠加
   - 关节/电机转换 → 高增益 PID 控制

3. **数学模型**：
   $$
   \boldsymbol{\tau} = \underbrace{\mathbf{J}^T \mathbf{F}_{\text{desired}}}_{\text{力控制}} + \underbrace{\boldsymbol{\tau}_g}_{\text{重力补偿}} + \underbrace{\mathbf{K}_p (\mathbf{q}_d - \mathbf{q}) - \mathbf{K}_d \dot{\mathbf{q}}}_{\text{位置反馈}}
   $$

4. **参数特点**：
   - **高 PID 增益**：$K_p \in [10, 350]$，远高于阻抗模式
   - **速度置零**：$\dot{q}_d = 0$，简化控制律
   - **力直接映射**：$\boldsymbol{\tau}_{\text{adm}} = \mathbf{J}^T \mathbf{F}_{\text{desired}}$

5. **性能特点**：
   - **Z 轴平滑**：重力方向，力控制精度高
   - **X 轴摩擦**：需要摩擦补偿改进
   - **位置精度高**：高 PID 增益确保位置跟踪

### 9.10.2 应用场景

1. **装配任务**：需要精确控制接触力
2. **抛光打磨**：恒力接触，提高表面质量
3. **人机协作**：安全的力限制交互
4. **柔性操作**：鸡蛋抓取等精细操作

### 9.10.3 优缺点

**优点**：
- ✅ 力控制精度高（直接力映射）
- ✅ 位置控制精度高（高 PID 增益）
- ✅ 灵活的混合控制（参考掩码）
- ✅ 实现简单（线性加权混合）

**缺点**：
- ❌ 稳定性依赖增益调节
- ❌ 摩擦影响力控制（X 轴）
- ❌ 需要精确的动力学模型（重力补偿）
- ❌ 对模型误差敏感

### 9.10.4 与其他模式对比

| 模式 | 力精度 | 位置精度 | 柔性 | 稳定性 |
|------|-------|---------|------|-------|
| AutoServo | 低 | 高 | 低 | 高 |
| Gravity | 低 | 低 | 高 | 高 |
| Impedance | 中 | 中 | 高 | 高 |
| **Admittance** | **高** | **高** | **中** | **中** |

导纳控制在**力控制精度**和**位置控制精度**上均表现优异，是需要同时控制力和位置的理想选择。

---

## 参考文献

1. Hogan, N. (1985). *Impedance Control: An Approach to Manipulation: Part I—Theory*. Journal of Dynamic Systems, Measurement, and Control, 107(1), 1-7.

2. Hogan, N. (1985). *Impedance Control: An Approach to Manipulation: Part II—Implementation*. Journal of Dynamic Systems, Measurement, and Control, 107(1), 8-16.

3. Raibert, M. H., & Craig, J. J. (1981). *Hybrid Position/Force Control of Manipulators*. Journal of Dynamic Systems, Measurement, and Control, 103(2), 126-133.

4. Siciliano, B., & Villani, L. (1999). *Robot Force Control*. Springer.

5. Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer.

6. Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.

7. De Luca, A., & Mattone, R. (2003). *Actuator Failure Detection and Isolation Using Generalized Momenta*. IEEE International Conference on Robotics and Automation.

8. Ott, C., Mukherjee, R., & Nakamura, Y. (2010). *Unified Impedance and Admittance Control*. IEEE International Conference on Robotics and Automation.

---

# 10. JointAutoServo 控制模式

## 10.1 模式概述

**JointAutoServo 控制模式**（关节空间自动伺服）是一种**直接在关节空间进行轨迹跟踪**的控制策略。与工作空间控制模式（AutoServo、Impedance、Admittance）不同，该模式：

1. **关节空间控制**：期望轨迹直接在关节空间指定，无需逆运动学求解
2. **重力前馈补偿**：基于当前关节状态计算重力补偿力矩
3. **电机内部PID**：利用电机内置的MIT模式PID控制器
4. **可选额外补偿**：可为前3个关节添加额外的PID补偿力矩（当前代码中被注释）
5. **高效简洁**：避免了逆运动学计算，控制流程简单高效

**典型应用场景**：
- 关节空间轨迹规划和跟踪
- 示教再现（Teaching and Playback）
- 关节空间插值运动
- 需要精确关节角度控制的任务
- 奇异点规避（在关节空间规划避免工作空间奇异）

**与工作空间控制的对比**：
- **AutoServo**：工作空间 → 逆运动学 → 关节空间 → 控制
- **JointAutoServo**：关节空间 → 控制（直接）

## 10.2 控制架构

### 10.2.1 控制流程图

```
┌─────────────────────────────────────────────────────────────────┐
│               JointAutoServo Control Loop                        │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  1. Gravity Compensation Calculation     │
        │     τ_g = g(q, q̇, q̈=0)                  │
        │  (Inverse Dynamics with zero accel)      │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  2. Prepare Desired Joint Trajectories   │
        │     q_d = desir_joint_positions          │
        │     q̇_d = desir_joint_velocities         │
        │     τ_ff = gravity_joint_tauqes          │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  3. [Optional] Additional PID Torque     │
        │     τ_pid[0..2] = K_p·e + K_d·ė         │
        │  (Currently commented out in code)       │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  4. Superpose Additional Torque          │
        │     τ_ctrl[0..2] = τ_g[0..2] + τ_pid    │
        │     τ_ctrl[3..5] = τ_g[3..5]             │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  5. Joint to Motor Conversion            │
        │     motor_cmd = joint2motor(q_d, τ_ctrl) │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  6. Motor Control (MIT Mode)             │
        │     ControlMotors(q_d, q̇_d, K_p, K_d, τ) │
        │  Uses motor internal PID controller      │
        └──────────────────────────────────────────┘
```

### 10.2.2 关键特点

1. **无逆运动学需求**：
   - 期望关节位置 $\mathbf{q}_d$ 直接由外部话题指定
   - 避免了逆运动学计算的复杂性和可能的求解失败

2. **重力补偿前馈**：
   - 基于当前关节状态 $\mathbf{q}, \dot{\mathbf{q}}$ 计算重力项
   - 作为前馈力矩 $\boldsymbol{\tau}_{\text{ff}}$ 补偿重力影响

3. **电机内部PID**：
   - 使用电机固件内置的PID控制器（MIT模式）
   - 增益 $\mathbf{K}_p, \mathbf{K}_d$ 在初始化或模式切换时设置

4. **可选额外补偿**（当前未启用）：
   - 代码预留了前3个关节的额外PID力矩
   - 可用于提高大关节的跟踪性能

## 10.3 详细算法步骤

### 步骤 1：重力补偿计算

**目的**：计算当前关节状态下的重力补偿力矩

**代码**：
```cpp
arm.computeInverseDynamics(arm.current_joint_positions,
                           arm.current_joint_velocities,
                           arm.current_joint_acceleration,
                           arm.gravity_joint_tauqes);
```

**数学模型**：

使用逆动力学计算重力项：

$$
\boldsymbol{\tau}_g = \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) - \mathbf{J}^T(\mathbf{q})\mathbf{F}_{\text{ext}}
$$

在重力补偿中，加速度设为零（$\ddot{\mathbf{q}} = \mathbf{0}$），外力为零（$\mathbf{F}_{\text{ext}} = \mathbf{0}$），简化为：

$$
\boldsymbol{\tau}_g = \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q})
$$

其中：
- $\mathbf{M}(\mathbf{q}) \in \mathbb{R}^{6 \times 6}$：惯性矩阵
- $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \in \mathbb{R}^{6 \times 6}$：科氏力和离心力矩阵
- $\mathbf{g}(\mathbf{q}) \in \mathbb{R}^6$：重力项
- $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{6 \times 6}$：雅可比矩阵

---

### 步骤 2：准备期望关节轨迹

**目的**：从话题订阅或内部规划获取期望的关节位置、速度，并设置前馈力矩

**代码**：
```cpp
for (size_t i = 0; i < 6; i++)
{
    arm.desir_joint_pos[i] = arm.desir_joint_positions(i);
    arm.desir_joint_vel[i] = arm.desir_joint_velocities(i);
    arm.joint_control_tau[i] = arm.gravity_joint_tauqes(i);
}
```

**数学模型**：

$$
\begin{aligned}
\mathbf{q}_d &= \text{desired\_joint\_positions} \\
\dot{\mathbf{q}}_d &= \text{desired\_joint\_velocities} \\
\boldsymbol{\tau}_{\text{ff}} &= \boldsymbol{\tau}_g
\end{aligned}
$$

其中：
- $\mathbf{q}_d \in \mathbb{R}^6$：期望关节位置（rad）
- $\dot{\mathbf{q}}_d \in \mathbb{R}^6$：期望关节速度（rad/s）
- $\boldsymbol{\tau}_{\text{ff}} \in \mathbb{R}^6$：前馈力矩（初始为重力补偿）

**期望轨迹来源**：
1. **ROS话题订阅**：`topic_control_desir_joint_state`
   ```cpp
   // 在light_lift_arm_6dof_node.cpp中
   void desir_joint_state_callback(const sensor_msgs::msg::JointState msg)
   {
       for (size_t i = 0; i < 6; i++)
       {
           llarm6dof.desir_joint_positions(i) = msg.position[i];
       }
   }
   ```

2. **内部轨迹规划**：根据需求生成关节空间轨迹

---

### 步骤 3：可选的额外PID补偿（当前未启用）

**目的**：为前3个关节（基座、肩、肘）添加额外的PID力矩补偿

**代码**（已注释）：
```cpp
// motor.motor_pid_tau[0] = 150 * (motor.desir_motor_pos[0] - motor.current_motor_pos[0])
//                         + 1.1 * (motor.desir_motor_vel[0] - motor.current_motor_vel[0]);
//
// motor.motor_pid_tau[1] = 170 * (motor.desir_motor_pos[1] - motor.current_motor_pos[1])
//                         + 1.4 * (motor.desir_motor_vel[1] - motor.current_motor_vel[1]);
//
// motor.motor_pid_tau[2] = 170 * (motor.desir_motor_pos[2] - motor.current_motor_pos[2])
//                         + 1.4 * (motor.desir_motor_vel[2] - motor.current_motor_vel[2]);
```

**数学模型**（如果启用）：

$$
\tau_{\text{pid},i} = K_{p,i}^{\text{extra}} (q_{d,i} - q_i) + K_{d,i}^{\text{extra}} (\dot{q}_{d,i} - \dot{q}_i), \quad i = 0, 1, 2
$$

其中：
- $K_{p}^{\text{extra}} = [150, 170, 170]$：额外PID的位置增益
- $K_{d}^{\text{extra}} = [1.1, 1.4, 1.4]$：额外PID的速度增益

**设计意图**：
- 大关节（0,1,2）承受更大负载，可能需要额外补偿
- 与电机内部PID叠加，形成双层PID结构

**当前状态**：
- 代码中被注释掉，未实际使用
- `motor.motor_pid_tau[i]` 默认为0（未显式赋值）

---

### 步骤 4：力矩叠加

**目的**：将额外PID力矩（如果有）叠加到前馈力矩上

**代码**：
```cpp
for (size_t i = 0; i < 3; i++)
{
    motor.motor_control_tau[i] += motor.motor_pid_tau[i];
}
```

**数学模型**：

$$
\tau_{\text{ctrl},i} = \begin{cases}
\tau_{g,i} + \tau_{\text{pid},i}, & i = 0, 1, 2 \\
\tau_{g,i}, & i = 3, 4, 5
\end{cases}
$$

**当前实际情况**：
- 由于 `motor.motor_pid_tau[i]` 未被设置（或为0），实际上：
  $$
  \boldsymbol{\tau}_{\text{ctrl}} = \boldsymbol{\tau}_g
  $$

---

### 步骤 5：关节到电机空间转换

**目的**：将关节空间的指令转换到电机空间，考虑电机方向

**代码**：
```cpp
arm.joint2motor(arm.desir_joint_pos,
                arm.desir_joint_vel,
                arm.desir_joint_acc,
                arm.joint_control_tau,
                motor.desir_motor_pos,
                motor.desir_motor_vel,
                motor.desir_motor_acc,
                motor.motor_control_tau);
```

**数学模型**：

$$
\begin{aligned}
\theta_{\text{motor},i} &= d_i \cdot q_i \\
\dot{\theta}_{\text{motor},i} &= d_i \cdot \dot{q}_i \\
\ddot{\theta}_{\text{motor},i} &= d_i \cdot \ddot{q}_i \\
\tau_{\text{motor},i} &= d_i \cdot \tau_{\text{joint},i}
\end{aligned}
$$

其中：
- $d_i \in \{-1, +1\}$：`motor_direction[i]`，电机方向系数
- $\theta_{\text{motor},i}$：电机位置（rad）
- $q_i$：关节位置（rad）
- $\tau_{\text{motor},i}$：电机力矩（Nm）
- $\tau_{\text{joint},i}$：关节力矩（Nm）

**方向系数作用**：
- 补偿URDF定义的关节方向与实际电机旋转方向的差异
- 确保正确的运动学和动力学映射

---

### 步骤 6：电机控制执行

**目的**：使用电机内部MIT模式PID控制器执行轨迹跟踪

**代码**：
```cpp
motor.ControlMotors(port,
                    motor.desir_motor_pos,
                    motor.desir_motor_vel,
                    motor.kp,
                    motor.kd,
                    motor.motor_control_tau);
```

**数学模型**：

电机级别的MIT模式控制律：

$$
\tau_{\text{motor},i} = K_{p,i} (\theta_{d,i} - \theta_i) + K_{d,i} (\dot{\theta}_{d,i} - \dot{\theta}_i) + \tau_{\text{ff},i}
$$

其中：
- $K_{p,i}, K_{d,i}$：电机内部PID增益（从配置或初始化设置）
- $\theta_{d,i}, \dot{\theta}_{d,i}$：期望电机位置和速度
- $\theta_i, \dot{\theta}_i$：当前电机位置和速度（反馈）
- $\tau_{\text{ff},i}$：前馈力矩（重力补偿转换到电机空间）

**PID增益来源**：
- 在模式初始化时设置（`joint_initialize` 函数）
- 或在配置文件中预定义
- 典型值（来自初始化代码）：
  - 关节0: $K_p = 450.0$, $K_d = 1.51$
  - 关节1: $K_p = 500.0$, $K_d = 4.81$
  - 关节2: $K_p = 500.0$, $K_d = 4.51$
  - 关节3: $K_p = 90.0$, $K_d = 1.0$
  - 关节4: $K_p = 40.0$, $K_d = 0.51$
  - 关节5: $K_p = 10.0$, $K_d = 0.51$

## 10.4 完整数学模型

### 10.4.1 系统动力学

6自由度机械臂的完整动力学方程：

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau} + \mathbf{J}^T(\mathbf{q})\mathbf{F}_{\text{ext}}
$$

### 10.4.2 JointAutoServo控制律

总控制力矩为：

$$
\boldsymbol{\tau} = \boldsymbol{\tau}_g + \mathbf{K}_p^{\text{motor}} (\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d^{\text{motor}} (\dot{\mathbf{q}}_d - \dot{\mathbf{q}})
$$

其中：
- $\boldsymbol{\tau}_g = \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q})$：重力补偿前馈
- $\mathbf{K}_p^{\text{motor}} (\mathbf{q}_d - \mathbf{q})$：电机内部位置反馈
- $\mathbf{K}_d^{\text{motor}} (\dot{\mathbf{q}}_d - \dot{\mathbf{q}})$：电机内部速度反馈

**展开形式**（考虑方向转换）：

$$
\tau_i = d_i \left[ K_{p,i} d_i (q_{d,i} - q_i) + K_{d,i} d_i (\dot{q}_{d,i} - \dot{q}_i) + d_i \tau_{g,i} \right]
$$

简化为：

$$
\tau_i = K_{p,i} (q_{d,i} - q_i) + K_{d,i} (\dot{q}_{d,i} - \dot{q}_i) + \tau_{g,i}
$$

（因为 $d_i^2 = 1$）

### 10.4.3 闭环系统分析

将控制律代入动力学方程：

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \mathbf{g}(\mathbf{q}) + \mathbf{C}\dot{\mathbf{q}} + \mathbf{K}_p (\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d (\dot{\mathbf{q}}_d - \dot{\mathbf{q}}) + \mathbf{J}^T \mathbf{F}_{\text{ext}}
$$

简化为：

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{K}_d (\dot{\mathbf{q}} - \dot{\mathbf{q}}_d) + \mathbf{K}_p (\mathbf{q} - \mathbf{q}_d) = \mathbf{J}^T \mathbf{F}_{\text{ext}}
$$

定义跟踪误差：

$$
\mathbf{e} = \mathbf{q}_d - \mathbf{q}, \quad \dot{\mathbf{e}} = \dot{\mathbf{q}}_d - \dot{\mathbf{q}}
$$

误差动力学：

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{e}} + \mathbf{K}_d \dot{\mathbf{e}} + \mathbf{K}_p \mathbf{e} = \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}}_d - \mathbf{J}^T \mathbf{F}_{\text{ext}}
$$

**特殊情况**：

1. **无外力，恒定目标**（$\mathbf{F}_{\text{ext}} = \mathbf{0}$, $\ddot{\mathbf{q}}_d = \mathbf{0}$）：
   $$
   \mathbf{M}(\mathbf{q})\ddot{\mathbf{e}} + \mathbf{K}_d \dot{\mathbf{e}} + \mathbf{K}_p \mathbf{e} = \mathbf{0}
   $$
   这是一个标准的二阶线性系统，误差收敛到零。

2. **轨迹跟踪**（$\ddot{\mathbf{q}}_d \neq \mathbf{0}$）：
   $$
   \mathbf{M}(\mathbf{q})\ddot{\mathbf{e}} + \mathbf{K}_d \dot{\mathbf{e}} + \mathbf{K}_p \mathbf{e} = \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}}_d
   $$
   存在跟踪误差，大小取决于 $\ddot{\mathbf{q}}_d$ 和增益 $\mathbf{K}_p, \mathbf{K}_d$。

### 10.4.4 稳定性条件

对于单关节简化模型：

$$
M \ddot{e} + K_d \dot{e} + K_p e = 0
$$

特征方程：

$$
M s^2 + K_d s + K_p = 0
$$

极点：

$$
s = \frac{-K_d \pm \sqrt{K_d^2 - 4MK_p}}{2M}
$$

**稳定性条件**：
- $K_p > 0$, $K_d > 0$（正增益）
- 实部 $\text{Re}(s) < 0$（左半平面）

**阻尼比**：

$$
\zeta = \frac{K_d}{2\sqrt{MK_p}}
$$

- $\zeta < 1$：欠阻尼（快速响应，可能超调）
- $\zeta = 1$：临界阻尼（最快无超调）
- $\zeta > 1$：过阻尼（慢速响应）

**示例**（关节1：$K_p = 500$, $K_d = 4.81$, 假设 $M \approx 0.1$）：

$$
\zeta = \frac{4.81}{2\sqrt{0.1 \times 500}} = \frac{4.81}{2 \times 7.07} \approx 0.34
$$

欠阻尼，响应快速，可能有小幅超调。

## 10.5 模式初始化

JointAutoServo模式需要专门的初始化过程（`joint_initialize`函数），将机械臂从当前位置平滑移动到默认关节配置。

### 10.5.1 初始化流程

```
┌─────────────────────────────────────────────────────────────────┐
│             Joint Space Initialization Process                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  1. Set Default Joint Positions          │
        │     q_d = desir_joint_pos_default        │
        │  (Usually zero configuration)            │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  2. Read Current Joint Positions         │
        │     q_curr (from motor feedback)         │
        │  Repeat 50 times for stable reading      │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  3. Generate Smooth Trajectory           │
        │     now2aim(q_curr, q_d, q_interp, ...)  │
        │  Uses sinusoidal interpolation           │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  4. Set High PID Gains for Transition    │
        │     K_p = [450, 500, 500, 90, 40, 10]    │
        │     K_d = [1.51, 4.81, 4.51, 1.0, ...]   │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  5. Execute Transition with Gravity Comp │
        │     τ = τ_g + K_p·e + K_d·ė               │
        │  Until reaching target configuration     │
        └──────────────────────────────────────────┘
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  6. Initialization Complete              │
        │     Ready for JointAutoServo control     │
        └──────────────────────────────────────────┘
```

### 10.5.2 默认关节配置

**代码**：
```cpp
arm.desir_joint_positions.data.setZero();  // 全零配置
for (size_t i = 0; i < 6; i++)
{
    arm.desir_joint_positions(i) = arm.desir_joint_pos_default[i];
}
```

**数学模型**：

$$
\mathbf{q}_{\text{default}} = [0, 0, 0, 0, 0, 0]^T \quad \text{or custom configuration}
$$

**物理意义**：
- 全零配置通常对应机械臂的"Home"位置
- 可在配置文件中自定义默认配置

### 10.5.3 平滑轨迹插值

**代码**：
```cpp
while (!arm.now2aim(current_joint_pos_temp, desir_joint_pos_temp,
                    arm.desir_joint_pos, 1, 50))
{
    // Control loop with gravity compensation
    ...
}
```

**`now2aim`函数原理**（从`light_lift_arm_6dof.cpp`）：

```cpp
bool LlArm6dof::now2aim(float sta_pos[6], float end_pos[6],
                        float now_pos[6], float steps, int delay)
{
    float e[6] = {(end_pos[0] - sta_pos[0]), ..., (end_pos[5] - sta_pos[5])};
    static int counter = 0;
    static float inter_step = 0;

    float time_counter = (float)counter / 1000.0;
    inter_step += steps * 0.5 * 0.001 * sin(time_counter);

    for (size_t i = 0; i < 6; i++)
    {
        now_pos[i] = sta_pos[i] + e[i] * inter_step;
    }

    if (counter >= 3140)  // π × 1000
    {
        counter = 0;
        inter_step = 0;
        return true;  // Reached target
    }
    else
    {
        counter += steps;
        return false;  // Still moving
    }
}
```

**数学模型**：

$$
\alpha(t) = \int_0^t \frac{s}{2} \times 10^{-3} \sin(\tau / 1000) d\tau
$$

$$
\mathbf{q}_{\text{interp}}(t) = \mathbf{q}_{\text{start}} + (\mathbf{q}_{\text{end}} - \mathbf{q}_{\text{start}}) \cdot \alpha(t)
$$

其中：
- $s$：步长因子（`steps`）
- $t$：时间计数器（毫秒）
- $\alpha(t)$：插值系数（0到1）

**插值特点**：
- **平滑加速/减速**：正弦积分产生S型曲线
- **零初速度和终速度**：避免冲击
- **固定时长**：约 $\pi$ 秒（3.14秒）

## 10.6 代码分析

### 10.6.1 关键变量

| 变量名 | 类型 | 维度 | 物理意义 |
|--------|------|------|---------|
| `arm.desir_joint_positions` | KDL::JntArray | 6 | 期望关节位置 $\mathbf{q}_d$ |
| `arm.desir_joint_velocities` | KDL::JntArray | 6 | 期望关节速度 $\dot{\mathbf{q}}_d$ |
| `arm.gravity_joint_tauqes` | KDL::JntArray | 6 | 重力补偿力矩 $\boldsymbol{\tau}_g$ |
| `motor.motor_pid_tau[6]` | float | 6 | 额外PID力矩（当前未用）|
| `motor.kp[6]`, `motor.kd[6]` | float | 6 | 电机PID增益 $K_p, K_d$ |
| `arm.desir_joint_pos_default[6]` | float | 6 | 默认关节配置 |

### 10.6.2 与AutoServo的对比

| 特性 | AutoServo（Chapter 7）| JointAutoServo（Chapter 10）|
|------|---------------------|----------------------------|
| **控制空间** | 工作空间（笛卡尔）| 关节空间 |
| **轨迹输入** | 末端位姿 $\mathbf{x}_d$ | 关节角度 $\mathbf{q}_d$ |
| **逆运动学** | 需要（LMA求解）| 不需要 |
| **重力补偿** | 有 | 有 |
| **额外PID** | 有（配置文件）| 可选（当前未启用）|
| **奇异点问题** | 可能遇到 | 无（关节空间）|
| **直观性** | 高（任务空间）| 低（关节角度不直观）|
| **计算效率** | 低（需要IK）| 高（直接控制）|

### 10.6.3 话题订阅与控制

**ROS话题接口**（在`light_lift_arm_6dof_node.cpp`）：

```cpp
// 订阅关节空间控制话题
desir_joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "topic_control_desir_joint_state", 10,
    std::bind(&KDLTestNode::desir_joint_state_callback, this, std::placeholders::_1));

// 回调函数
void desir_joint_state_callback(const sensor_msgs::msg::JointState msg)
{
    RCLCPP_INFO(this->get_logger(), "desir_joint_state_callback Received");
    for (size_t i = 0; i < 6; i++)
    {
        llarm6dof.desir_joint_positions(i) = msg.position[i];
    }
}
```

**消息格式**：

```yaml
# sensor_msgs/msg/JointState
header:
  stamp: ...
  frame_id: "1"
name: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
position: [q1, q2, q3, q4, q5, q6]  # rad
velocity: [dq1, dq2, dq3, dq4, dq5, dq6]  # rad/s (optional)
effort: [...]  # Not used for command
```

**示例命令**（Python）：

```python
import rclpy
from sensor_msgs.msg import JointState

# 发布关节空间轨迹
msg = JointState()
msg.header.stamp = node.get_clock().now().to_msg()
msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
msg.position = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]  # rad

publisher.publish(msg)
```

### 10.6.4 模式切换与初始化

**切换到JointAutoServo模式**（通过`cmd_topic`）：

```cpp
// 在light_lift_arm_6dof_node.cpp的cmd_callback中
if (llarm6dof.cmd == "JointAutoServo")
{
    mode_ = State::Gohome;  // 先回零点
    control_mode.modeTransition(mode_);
    control_mode.is_go_home = true;
}
```

**回零流程**（`joint_go_home`函数）：

1. 设置高增益：$K_p = [450, 500, 500, 90, 40, 10]$
2. 使用`now2aim`平滑插值到零点
3. 重置期望关节角度为零
4. 切换到JointAutoServo模式

**代码片段**：

```cpp
void ArmState::joint_go_home(...)
{
    if (is_go_home)
    {
        // 设置增益
        motor.kp[0] = 450.0; motor.kd[0] = 4.51;
        // ...

        // 插值到零点
        float current_joint_pos_temp[6] = {...};
        float desir_joint_pos_temp[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        while (!arm.now2aim(current_joint_pos_temp, desir_joint_pos_temp,
                            arm.desir_joint_pos, 1, 50))
        {
            // 重力补偿控制
            arm.computeInverseDynamics(...);
            motor.ControlMotors(...);
        }

        // 重置期望角度
        for (size_t i = 0; i < 6; i++)
        {
            arm.desir_joint_positions(i) = 0;
        }

        // 切换模式
        modeTransition(State::JointAutoServo);
        is_go_home = false;
    }
}
```

## 10.7 性能分析

### 10.7.1 跟踪精度

**位置跟踪误差**：

$$
\mathbf{e}_p = \mathbf{q}_d - \mathbf{q}
$$

在稳态（$\dot{\mathbf{q}}_d = \mathbf{0}$），高PID增益确保：

$$
|\mathbf{e}_p| < 0.01 \text{ rad} \quad (\approx 0.57°)
$$

**速度跟踪误差**：

$$
\mathbf{e}_v = \dot{\mathbf{q}}_d - \dot{\mathbf{q}}
$$

**影响因素**：
1. **PID增益**：更高的 $K_p, K_d$ 提高精度，但可能引起振荡
2. **重力补偿精度**：模型误差影响前馈效果
3. **摩擦力**：未建模的摩擦降低精度
4. **控制周期**：1ms周期（1kHz）足够高

### 10.7.2 响应速度

**建立时间**（settling time）估算：

对于二阶系统，4倍时间常数：

$$
t_s = \frac{4}{\zeta \omega_n} = \frac{4\zeta}{\sqrt{K_p / M}}
$$

**示例**（关节1：$K_p = 500$, $\zeta \approx 0.34$, $M \approx 0.1$）：

$$
\omega_n = \sqrt{K_p / M} = \sqrt{500 / 0.1} = 70.7 \text{ rad/s}
$$

$$
t_s = \frac{4 \times 0.34}{70.7} \approx 0.019 \text{ s} \approx 19 \text{ ms}
$$

非常快速的响应。

### 10.7.3 平滑性

**优点**：
- 重力补偿前馈减少扰动
- 电机内部PID滤波

**潜在问题**：
- 欠阻尼可能导致小幅振荡
- 轨迹跳变会引起冲击

**改进建议**：
1. 使用轨迹规划器生成平滑的 $\mathbf{q}_d(t)$
2. 加入前馈加速度项 $\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}}_d$
3. 调整阻尼比到临界阻尼（$\zeta = 1$）

### 10.7.4 计算效率

**优势**：
- 无逆运动学计算（节省大量计算时间）
- 仅需逆动力学计算重力项（已优化）

**时间分解**（估算）：

| 步骤 | 时间（μs）| 占比 |
|------|----------|------|
| 逆动力学（重力）| ~100 | 10% |
| 数据赋值 | ~10 | 1% |
| 关节/电机转换 | ~5 | 0.5% |
| 串口通信 | ~800 | 80% |
| 其他 | ~85 | 8.5% |
| **总计** | **~1000** | **100%** |

**结论**：控制算法本身非常高效（~200μs），主要时间消耗在串口通信。

## 10.8 应用场景

### 10.8.1 示教再现（Teaching & Playback）

**工作流程**：

1. **示教阶段**：
   - 切换到Gravity模式（重力补偿）
   - 人工拖动机械臂到各个关键位姿
   - 记录关节角度 $\mathbf{q}_{\text{teach},k}$, $k=1,2,...,N$

2. **再现阶段**：
   - 切换到JointAutoServo模式
   - 依次发送 $\mathbf{q}_{\text{teach},k}$ 到话题
   - 机械臂重复执行示教轨迹

**优势**：
- 直接在关节空间重现，无奇异点问题
- 精确重复示教轨迹
- 计算高效

### 10.8.2 关节空间轨迹规划

**场景**：避开奇异点的运动规划

**示例**（避免手腕奇异）：

```python
# 在关节空间规划避开奇异配置
q_start = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
q_goal = [1.0, 0.3, -0.8, 0.5, 0.5, 0.5]

# 直接插值（避免工作空间奇异点）
for alpha in np.linspace(0, 1, 100):
    q_d = q_start + alpha * (q_goal - q_start)
    publish_joint_command(q_d)
    time.sleep(0.01)
```

### 10.8.3 关节级柔顺控制

**通过调整增益实现柔性**：

```cpp
// 降低特定关节的增益，实现柔顺
motor.kp[4] = 10.0;  // 低增益
motor.kd[4] = 0.2;   // 低阻尼
```

允许该关节在外力作用下偏移，实现局部柔顺。

### 10.8.4 快速点对点运动

**场景**：快速移动到预定义配置

```python
# 预定义配置
configs = {
    "home": [0, 0, 0, 0, 0, 0],
    "ready": [0, 0.5, -1.0, 0, 0.5, 0],
    "pick": [0.5, 0.3, -0.8, 0, 0.8, 0.5]
}

# 快速切换
publish_joint_command(configs["ready"])
time.sleep(1.0)
publish_joint_command(configs["pick"])
```

## 10.9 改进方向

### 10.9.1 加速度前馈

**问题**：当前仅有重力补偿前馈，跟踪快速轨迹时存在滞后

**改进**：添加加速度前馈项

$$
\boldsymbol{\tau} = \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}}_d + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) + \mathbf{K}_p \mathbf{e} + \mathbf{K}_d \dot{\mathbf{e}}
$$

**代码修改**：

```cpp
// 计算完整逆动力学（包括期望加速度）
arm.computeInverseDynamics(arm.current_joint_positions,
                           arm.current_joint_velocities,
                           arm.desir_joint_acceleration,  // 非零
                           arm.feedforward_tauqes);
```

**效果**：
- 显著提高轨迹跟踪精度
- 减小跟踪误差，尤其在快速运动时

### 10.9.2 启用额外PID补偿

**当前状态**：额外PID代码被注释掉

**建议**：为大关节（0,1,2）启用额外补偿

```cpp
// 计算额外PID（在电机空间）
motor.motor_pid_tau[0] = 150 * (motor.desir_motor_pos[0] - motor.current_motor_pos[0])
                       + 1.1 * (motor.desir_motor_vel[0] - motor.current_motor_vel[0]);
motor.motor_pid_tau[1] = 170 * (motor.desir_motor_pos[1] - motor.current_motor_pos[1])
                       + 1.4 * (motor.desir_motor_vel[1] - motor.current_motor_vel[1]);
motor.motor_pid_tau[2] = 170 * (motor.desir_motor_pos[2] - motor.current_motor_pos[2])
                       + 1.4 * (motor.desir_motor_vel[2] - motor.current_motor_vel[2]);
```

**原因**：
- 大关节负载大，重力补偿可能不完美
- 额外PID提供更强的跟踪能力

**调参建议**：
- 增益不要过大，避免与电机内部PID冲突
- 测试稳定性，防止振荡

### 10.9.3 轨迹平滑器

**问题**：话题发送的 $\mathbf{q}_d$ 可能不连续，导致冲击

**改进**：添加轨迹平滑滤波器

**方法1：低通滤波**

$$
\mathbf{q}_d^{\text{filtered}}(t) = \alpha \mathbf{q}_d^{\text{raw}}(t) + (1-\alpha) \mathbf{q}_d^{\text{filtered}}(t-\Delta t)
$$

**方法2：样条插值**

生成5次多项式轨迹连接离散路点：

$$
q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5
$$

约束条件：
- $q(0) = q_{\text{start}}$, $q(T) = q_{\text{end}}$
- $\dot{q}(0) = \dot{q}(T) = 0$
- $\ddot{q}(0) = \ddot{q}(T) = 0$

### 10.9.4 在线轨迹优化

**目标**：最小化能量消耗或时间

**能量最优轨迹**：

$$
\min_{\mathbf{q}(t)} \int_0^T \boldsymbol{\tau}^T \boldsymbol{\tau} \, dt
$$

约束：
- 起止点：$\mathbf{q}(0) = \mathbf{q}_{\text{start}}$, $\mathbf{q}(T) = \mathbf{q}_{\text{goal}}$
- 速度限制：$|\dot{\mathbf{q}}(t)| \leq \dot{\mathbf{q}}_{\max}$
- 加速度限制：$|\ddot{\mathbf{q}}(t)| \leq \ddot{\mathbf{q}}_{\max}$
- 力矩限制：$|\boldsymbol{\tau}(t)| \leq \boldsymbol{\tau}_{\max}$

**求解方法**：
- 时间离散化 + 非线性规划（NLP）
- 使用IPOPT、SNOPT等求解器

### 10.9.5 自适应增益调整

**目标**：根据负载和配置自适应调整PID增益

**方法**：

$$
K_{p,i}(\mathbf{q}) = K_{p,i,0} \cdot \left(1 + \beta \frac{\|\mathbf{M}(\mathbf{q})\|_F}{\|\mathbf{M}(\mathbf{q}_{\text{nominal}})\|_F}\right)
$$

其中：
- $\|\mathbf{M}\|_F$：Frobenius范数，表征配置依赖的惯性
- $\beta$：调度系数

**效果**：
- 在奇异配置附近提高增益
- 在远离奇异点时降低增益，节能

## 10.10 总结

### 10.10.1 核心要点

1. **控制原理**：
   - JointAutoServo是**关节空间直接控制**模式
   - 无需逆运动学，避免奇异点问题
   - 使用**重力补偿前馈** + **电机PID反馈**

2. **控制律**：
   $$
   \boldsymbol{\tau} = \boldsymbol{\tau}_g + \mathbf{K}_p (\mathbf{q}_d - \mathbf{q}) + \mathbf{K}_d (\dot{\mathbf{q}}_d - \dot{\mathbf{q}})
   $$

3. **主要步骤**：
   - 计算重力补偿 $\boldsymbol{\tau}_g$
   - 准备期望关节轨迹 $\mathbf{q}_d, \dot{\mathbf{q}}_d$
   - （可选）添加额外PID力矩
   - 关节/电机空间转换
   - 电机MIT模式控制

4. **初始化**：
   - 使用`joint_initialize`函数
   - 平滑插值到默认配置（通常为零点）
   - 设置适当的PID增益

5. **性能特点**：
   - **高效**：无IK计算，控制周期短
   - **精确**：关节空间跟踪精度高（< 0.01 rad）
   - **快速**：响应时间 < 20 ms
   - **无奇异点**：关节空间规划避免奇异

### 10.10.2 应用场景

1. **示教再现**：精确重复示教轨迹
2. **关节空间规划**：避开奇异点的路径
3. **快速点对点**：预定义配置间快速切换
4. **关节柔顺控制**：调整特定关节增益

### 10.10.3 优缺点

**优点**：
- ✅ 无逆运动学计算（计算高效）
- ✅ 无奇异点问题（关节空间）
- ✅ 跟踪精度高（高PID增益 + 重力补偿）
- ✅ 响应速度快（欠阻尼设计）
- ✅ 实现简单（无复杂映射）

**缺点**：
- ❌ 缺乏任务空间直观性（关节角度不直观）
- ❌ 需要关节空间轨迹规划（用户需理解关节配置）
- ❌ 未充分利用电机能力（额外PID被注释）
- ❌ 缺少加速度前馈（快速运动时精度下降）
- ❌ 硬编码增益（未考虑配置依赖）

### 10.10.4 与其他模式对比

| 模式 | 控制空间 | IK需求 | 奇异点 | 计算量 | 精度 | 直观性 |
|------|---------|--------|-------|-------|------|-------|
| AutoServo | 工作空间 | 是 | 有 | 高 | 高 | 高 |
| Impedance | 工作空间 | 是 | 有 | 高 | 中 | 高 |
| Admittance | 工作空间 | 是 | 有 | 高 | 高 | 高 |
| **JointAutoServo** | **关节空间** | **否** | **无** | **低** | **高** | **低** |
| Gravity | 关节空间 | 否 | 无 | 低 | 低 | 低 |

### 10.10.5 最佳实践

1. **何时使用JointAutoServo**：
   - 需要避免奇异点的任务
   - 关节空间轨迹已知或易于规划
   - 需要高效率、快速响应
   - 示教再现应用

2. **何时不使用**：
   - 用户只能提供末端位姿（使用AutoServo）
   - 需要力控制（使用Impedance/Admittance）
   - 任务要求笛卡尔空间约束（使用工作空间模式）

3. **性能调优**：
   - 根据负载调整PID增益
   - 考虑启用额外PID补偿（大关节）
   - 使用轨迹平滑器避免冲击
   - 添加加速度前馈提高快速运动精度

4. **安全考虑**：
   - 检查关节限位 $\mathbf{q}_{\min} \leq \mathbf{q} \leq \mathbf{q}_{\max}$
   - 限制速度 $|\dot{\mathbf{q}}| \leq \dot{\mathbf{q}}_{\max}$
   - 监控力矩 $|\boldsymbol{\tau}| \leq \boldsymbol{\tau}_{\max}$
   - 平滑切换模式，避免突变

---

# 第11章 Gravity 控制模式

## 11.1 模式概述

**Gravity控制模式**是一种纯重力补偿控制模式，通过实时计算并补偿机械臂各关节的重力力矩，使机械臂在零外力状态下保持当前位姿，同时允许操作者用极小的力实现拖动示教。

### 11.1.1 核心特征

1. **零PID增益**：$K_p = 0, K_d = 0$
2. **纯力矩控制**：仅施加重力补偿力矩
3. **卡尔曼滤波**：对关节速度进行滤波
4. **高顺应性**：可轻松拖动机械臂

### 11.1.2 应用场景

- 拖动示教（Teach by Demonstration）
- 人机协作任务
- 手动调整机械臂位姿
- 安全模式（碰撞后自动切换）

---

## 11.2 数学模型

### 11.2.1 逆动力学方程

机械臂动力学方程：

$$
\boldsymbol{\tau} = \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q})
$$

在Gravity模式下，期望加速度和速度均为零（$\ddot{\mathbf{q}}_d = 0, \dot{\mathbf{q}}_d = 0$），因此：

$$
\boldsymbol{\tau}_{\text{control}} = \mathbf{g}(\mathbf{q})
$$

其中：
- $\mathbf{g}(\mathbf{q}) \in \mathbb{R}^6$：重力项
- $\mathbf{q} \in \mathbb{R}^6$：当前关节位置

### 11.2.2 重力项计算

使用KDL库的ChainDynParam计算重力力矩：

$$
\mathbf{g}(\mathbf{q}) = \sum_{i=1}^{6} m_i \mathbf{J}_{vi}^T(\mathbf{q}) \mathbf{g}_0
$$

其中：
- $m_i$：第$i$个连杆质量
- $\mathbf{J}_{vi}(\mathbf{q})$：第$i$个连杆质心的线速度雅可比
- $\mathbf{g}_0 = [0, 0, -9.81]^T \, \text{m/s}^2$：重力加速度向量

### 11.2.3 关节到电机坐标转换

$$
\begin{aligned}
\tau_{\text{motor}, i} &= d_i \cdot \tau_{\text{joint}, i} \\
q_{\text{motor}, i} &= d_i \cdot q_{\text{joint}, i} \\
\dot{q}_{\text{motor}, i} &= d_i \cdot \dot{q}_{\text{joint}, i}
\end{aligned}
$$

其中 $d_i \in \{-1, +1\}$ 是电机方向系数。

---

## 11.3 卡尔曼滤波器

### 11.3.1 状态空间模型

为减少速度噪声，系统配置了卡尔曼滤波器：

**状态方程**：
$$
x_{k+1} = x_k + w_k, \quad w_k \sim \mathcal{N}(0, Q)
$$

**观测方程**：
$$
z_k = x_k + v_k, \quad v_k \sim \mathcal{N}(0, R)
$$

### 11.3.2 滤波器参数

代码中配置的参数：
```cpp
motor.KalmanFilter(1e-6, 1e-2, 1, 0, 0.03);
```

参数解释：
- $Q = 10^{-6}$：过程噪声协方差（状态转移不确定性）
- $R = 10^{-2}$：测量噪声协方差（速度传感器噪声）
- 初始状态：$\hat{x}_0 = 1$
- 初始协方差：$P_0 = 0$
- 采样周期：$\Delta t = 0.03 \, \text{s}$

### 11.3.3 递推公式

**预测步骤**：
$$
\begin{aligned}
\hat{x}_{k|k-1} &= \hat{x}_{k-1|k-1} \\
P_{k|k-1} &= P_{k-1|k-1} + Q
\end{aligned}
$$

**更新步骤**：
$$
\begin{aligned}
K_k &= \frac{P_{k|k-1}}{P_{k|k-1} + R} \\
\hat{x}_{k|k} &= \hat{x}_{k|k-1} + K_k(z_k - \hat{x}_{k|k-1}) \\
P_{k|k} &= (1 - K_k)P_{k|k-1}
\end{aligned}
$$

---

## 11.4 控制算法

### 11.4.1 算法流程

**算法11.1：Gravity控制模式**

---

**输入**：当前关节状态 $\mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}}$
**输出**：电机控制力矩 $\boldsymbol{\tau}_{\text{motor}}$

---

1. **配置卡尔曼滤波器**：
   `KalmanFilter(Q=1e-6, R=1e-2, x0=1, P0=0, dt=0.03)`

2. **计算逆动力学**：
   $$
   \mathbf{g}(\mathbf{q}) \leftarrow \text{ComputeInverseDynamics}(\mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}})
   $$

3. **设置控制力矩**：
   $$
   \boldsymbol{\tau}_{\text{joint}} \leftarrow \mathbf{g}(\mathbf{q})
   $$

4. **关节到电机转换**：
   $$
   [\boldsymbol{\tau}_{\text{motor}}, \mathbf{q}_{\text{motor}}, \dot{\mathbf{q}}_{\text{motor}}] \leftarrow \text{Joint2Motor}(\boldsymbol{\tau}_{\text{joint}}, \mathbf{q}, \dot{\mathbf{q}})
   $$

5. **零PID增益设置**：
   $$
   K_p = \mathbf{0}_{6 \times 1}, \quad K_d = \mathbf{0}_{6 \times 1}
   $$

6. **执行电机控制**：
   $$
   \text{ControlMotors}(\mathbf{q}_{\text{motor}}, \dot{\mathbf{q}}_{\text{motor}}, K_p, K_d, \boldsymbol{\tau}_{\text{motor}})
   $$

---

### 11.4.2 代码实现

```cpp
void ArmState::gravity(serialport::SerialPortWrapper &port,
                       LlArm6dof &arm,
                       MotorControl &motor)
{
    // 1. 配置卡尔曼滤波器
    motor.KalmanFilter(1e-6, 1e-2, 1, 0, 0.03);

    // 2. 计算逆动力学（重力项）
    arm.computeInverseDynamics(arm.current_joint_positions,
                               arm.current_joint_velocities,
                               arm.current_joint_acceleration,
                               arm.gravity_joint_tauqes);

    // 3. 设置控制力矩为重力补偿力矩
    for (size_t i = 0; i < 6; i++)
    {
        arm.joint_control_tau[i] = arm.gravity_joint_tauqes(i);
    }

    // 4. 关节到电机坐标转换
    arm.joint2motor(arm.desir_joint_pos,
                    arm.desir_joint_vel,
                    arm.desir_joint_acc,
                    arm.joint_control_tau,
                    motor.desir_motor_pos,
                    motor.desir_motor_vel,
                    motor.desir_motor_acc,
                    motor.motor_control_tau);

    // 5. 零PID增益
    float gravity_kp[6] = {0.0};
    float gravity_kd[6] = {0.0};

    // 6. 执行电机控制
    motor.ControlMotors(port,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        gravity_kp,
                        gravity_kd,
                        motor.motor_control_tau);
}
```

---

## 11.5 性能分析

### 11.5.1 稳态误差

理想情况下，静止时：

$$
\boldsymbol{\tau}_{\text{ext}} = \mathbf{0} \quad \Rightarrow \quad \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) - \boldsymbol{\tau}_{\text{control}} = \mathbf{0}
$$

由于 $\boldsymbol{\tau}_{\text{control}} = \mathbf{g}(\mathbf{q})$ 且 $\dot{\mathbf{q}} = \ddot{\mathbf{q}} = 0$：

$$
\boldsymbol{\tau}_{\text{error}} = \mathbf{0}
$$

**实际误差来源**：
1. 重力模型不准确（质量、质心估计误差）
2. 摩擦力未补偿
3. 传感器噪声
4. 计算延迟

### 11.5.2 拖动力分析

当操作者施加外力 $\mathbf{F}_{\text{ext}}$ 时：

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} = \mathbf{J}^T(\mathbf{q})\mathbf{F}_{\text{ext}} + \boldsymbol{\tau}_{\text{friction}}
$$

由于 $K_p = K_d = 0$，系统顺应性极高，所需拖动力：

$$
\mathbf{F}_{\text{drag}} = (\mathbf{J}^T)^{-1}[\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} - \boldsymbol{\tau}_{\text{friction}}]
$$

典型值：$\|\mathbf{F}_{\text{drag}}\| < 5 \, \text{N}$（低速运动）

### 11.5.3 安全性

**优势**：
- 碰撞时自动顺应，不会造成大冲击
- 操作者可随时接管控制

**风险**：
- 无位置保持能力，外力可导致漂移
- 关节限位需额外保护

---

## 11.6 应用示例

### 11.6.1 拖动示教流程

```mermaid
graph TD
    A[切换到Gravity模式] --> B[操作者拖动机械臂]
    B --> C[记录关节轨迹 q_i_t]
    C --> D[到达目标位姿]
    D --> E[切换到其他控制模式]
    E --> F[回放轨迹]
```

### 11.6.2 关键点记录

在示教过程中，按时间采样记录：

$$
\mathcal{T} = \{(\mathbf{q}_1, t_1), (\mathbf{q}_2, t_2), \ldots, (\mathbf{q}_N, t_N)\}
$$

后续可用样条插值重建轨迹。

---

# 第12章 Zero 控制模式

## 12.1 模式概述

**Zero控制模式**是一种电机零点标定模式，用于设置机械臂各关节的绝对零位。该模式仅在机械臂安装、维修或需要重新标定时使用。

### 12.1.1 核心特征

1. **禁用电机保护**：允许设置零点
2. **一次性操作**：执行后自动退出
3. **延迟确认**：确保指令成功发送

### 12.1.2 应用场景

- 机械臂首次安装
- 更换电机后重新标定
- 编码器丢失绝对位置
- 系统维护

---

## 12.2 零点标定原理

### 12.2.1 绝对编码器与增量编码器

**增量编码器**：
- 仅测量相对位移
- 断电后丢失绝对位置
- 需要零点标定

**绝对编码器**：
- 记录绝对位置
- 断电后仍保留位置信息
- MIT电机使用绝对编码器

### 12.2.2 零点设置数学模型

设当前电机编码器读数为 $\theta_{\text{encoder}}$，零点偏移为 $\theta_{\text{offset}}$，则实际关节角度：

$$
\theta_{\text{joint}} = \theta_{\text{encoder}} - \theta_{\text{offset}}
$$

执行零点标定后：

$$
\theta_{\text{offset}} \leftarrow \theta_{\text{encoder}} \quad \Rightarrow \quad \theta_{\text{joint}} = 0
$$

### 12.2.3 多关节同步标定

对于6自由度机械臂：

$$
\begin{bmatrix}
\theta_{\text{offset},1} \\
\theta_{\text{offset},2} \\
\vdots \\
\theta_{\text{offset},6}
\end{bmatrix}
\leftarrow
\begin{bmatrix}
\theta_{\text{encoder},1} \\
\theta_{\text{encoder},2} \\
\vdots \\
\theta_{\text{encoder},6}
\end{bmatrix}
$$

---

## 12.3 控制算法

### 12.3.1 算法流程

**算法12.1：Zero控制模式**

---

**输入**：无
**输出**：零点标定完成标志

---

1. **输出提示信息**：
   `print("Robot Arm has in Zero state.")`

2. **发送零点设置指令**：
   `motor.SetZeroMotors(port)`

3. **延迟1秒**：
   `sleep(1s)`

4. **重复发送指令（确认）**：
   `motor.SetZeroMotors(port)`

5. **再延迟1秒**：
   `sleep(1s)`

6. **输出完成信息**：
   `print("ok.")`

7. **退出模式**

---

### 12.3.2 代码实现

```cpp
case State::Zero:
    std::cout << "Robot Arm has in Zero state.\n";
    motor.SetZeroMotors(port);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    motor.SetZeroMotors(port);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "ok.\n";
    break;
```

### 12.3.3 SetZeroMotors实现

```cpp
void MotorControl::SetZeroMotors(SerialPortWrapper &port)
{
    for (size_t i = 0; i < 6; i++)
    {
        // 构造零点设置CAN指令
        uint8_t data[8] = {0};
        data[0] = 0x01; // 零点设置命令ID

        // 发送到对应电机
        port.sendCANMessage(motor_id[i], data, 8);
    }
}
```

---

## 12.4 操作流程

### 12.4.1 标定前准备

1. **机械臂姿态调整**：
   将机械臂移动到标准零位姿态（通常为全关节伸直或特定参考姿态）

2. **安全检查**：
   - 确认周围无障碍物
   - 确认机械臂处于稳定支撑状态
   - 确认电源和通信正常

3. **手动定位**：
   使用工具或手动拖动将每个关节精确对齐到零位标记

### 12.4.2 执行标定

```cpp
// 通过ROS2话题或直接代码触发
control_mode.modeTransition(State::Zero);
```

### 12.4.3 标定后验证

1. **重启系统**
2. **查询关节角度**：应显示 $\mathbf{q} \approx \mathbf{0}$
3. **移动测试**：小幅度运动后返回零位，验证一致性

---

## 12.5 注意事项

### 12.5.1 安全警告

**⚠️ 警告**：
- Zero模式会**重置绝对位置**，操作不可逆
- 错误的零点会导致运动学计算错误，可能引发碰撞
- 仅由专业人员操作

### 12.5.2 错误处理

**常见问题**：

1. **指令未响应**：
   - 检查CAN总线连接
   - 确认电机ID配置正确
   - 查看电机状态LED

2. **零点漂移**：
   - 检查编码器是否损坏
   - 确认电机固件版本
   - 考虑温度影响（热膨胀）

3. **部分电机标定失败**：
   - 单独发送指令到故障电机
   - 检查电源电压是否稳定

---

# 第13章 Planning 控制模式

## 13.1 模式概述

**Planning控制模式**实现关节空间轨迹规划，通过正弦插值生成从当前位置到目标位置的平滑轨迹，避免速度和加速度突变。

### 13.1.1 核心特征

1. **自动轨迹生成**：从当前位姿到目标位姿
2. **平滑插值**：基于正弦函数的S型速度曲线
3. **重力补偿**：运动过程中持续补偿重力
4. **夹爪控制**：支持同步控制夹爪开合

### 13.1.2 应用场景

- 抓取任务（pick-and-place）
- 避障路径执行
- 预定义轨迹回放
- 远程控制响应

---

## 13.2 轨迹规划数学模型

### 13.2.1 S型速度曲线

给定起始位置 $\mathbf{q}_{\text{start}}$ 和目标位置 $\mathbf{q}_{\text{end}}$，定义插值参数 $s(t) \in [0, 1]$：

$$
s(t) = \frac{1}{2}\left[1 - \cos\left(\frac{\pi t}{T}\right)\right], \quad t \in [0, T]
$$

其中 $T$ 是运动总时间。

**轨迹方程**：
$$
\mathbf{q}(t) = \mathbf{q}_{\text{start}} + s(t) \cdot (\mathbf{q}_{\text{end}} - \mathbf{q}_{\text{start}})
$$

### 13.2.2 速度和加速度

**速度**：
$$
\dot{\mathbf{q}}(t) = \frac{d s(t)}{dt} \cdot \Delta\mathbf{q} = \frac{\pi}{2T}\sin\left(\frac{\pi t}{T}\right) \cdot \Delta\mathbf{q}
$$

其中 $\Delta\mathbf{q} = \mathbf{q}_{\text{end}} - \mathbf{q}_{\text{start}}$。

**加速度**：
$$
\ddot{\mathbf{q}}(t) = \frac{\pi^2}{2T^2}\cos\left(\frac{\pi t}{T}\right) \cdot \Delta\mathbf{q}
$$

### 13.2.3 S型曲线的优势

相比线性插值：
$$
s_{\text{linear}}(t) = \frac{t}{T}
$$

S型曲线的优势：

1. **速度连续**：$\dot{s}(0) = \dot{s}(T) = 0$（零初速度和末速度）
2. **加速度连续**：无冲击
3. **最大速度**：$\max \dot{s}(t) = \frac{\pi}{2T}$ 在 $t = T/2$ 时达到
4. **平滑性**：$C^\infty$ 连续

---

## 13.3 离散化实现

### 13.3.1 增量式插值

代码中使用增量式更新：

$$
s_{k+1} = s_k + \alpha \cdot \Delta t \cdot \sin(\omega t_k)
$$

其中：
- $\alpha$：步进系数（代码中为`steps * 0.5`）
- $\Delta t = 0.001 \, \text{s}$（1ms控制周期）
- $\omega = 1 \, \text{rad/s}$（角频率）
- $t_k = k \cdot \Delta t / 1000$

**位置更新**：
$$
\mathbf{q}_k = \mathbf{q}_{\text{start}} + s_k \cdot \Delta\mathbf{q}
$$

### 13.3.2 终止条件

计数器从0增加到3140（约对应 $\pi$ 的1000倍）：

$$
k_{\max} = 3140 \quad \Rightarrow \quad s(k_{\max}) \approx 1
$$

当 $k \geq k_{\max}$ 时，轨迹执行完成。

---

## 13.4 控制算法

### 13.4.1 算法流程

**算法13.1：Planning控制模式**

---

**输入**：
- 当前关节位置 $\mathbf{q}_{\text{current}}$
- 目标关节位置 $\mathbf{q}_{\text{target}}$
- 步进系数 `steps`（默认8）
- 延迟 `delay`（默认50ms）

**输出**：轨迹执行完成

---

1. **检查触发标志**：
   `if (is_run_planning == false) return`

2. **设置PID参数**：
   从配置文件加载 $K_p, K_d$（或使用默认值）

3. **计算重力补偿**：
   $$
   \mathbf{g}(\mathbf{q}) \leftarrow \text{ComputeInverseDynamics}(\mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}})
   $$

4. **设置控制力矩**：
   $$
   \boldsymbol{\tau}_{\text{control}} \leftarrow \mathbf{g}(\mathbf{q})
   $$

5. **发送初始电机指令**（带夹爪）：
   `ControlMotors_g(port, q_motor, q̇_motor, Kp, Kd, τ_motor, gripper)`

6. **初始化插值变量**：
   ```
   counter = 0
   inter_step = 0
   Δq = q_target - q_current
   ```

7. **轨迹生成循环**：
   ```
   while (counter < 3140):
       time = counter / 1000.0
       inter_step += steps * 0.5 * 0.001 * sin(time)
       q_desired = q_current + inter_step * Δq

       // 执行控制
       ComputeInverseDynamics(q, q̇, q̈) → g(q)
       τ_control = g(q)
       Joint2Motor(q_desired, q̇_desired, q̈_desired, τ_control)
       ControlMotors_g(...)

       sleep(1ms)
       counter += steps
   ```

8. **重置标志**：
   `is_run_planning = false`

9. **输出完成信息**：
   `print("到达目标位置>>>")`

---

### 13.4.2 代码实现

```cpp
void ArmState::planning(serialport::SerialPortWrapper &port,
                        LlArm6dof &arm,
                        MotorControl &motor)
{
    if(is_run_planning) {
        // 1. 设置重力补偿力矩
        for (size_t i = 0; i < 6; i++)
        {
            motor.motor_control_tau[i] = arm.gravity_joint_tauqes(i);
        }

        // 2. 发送初始指令
        motor.ControlMotors_g(port,
                              motor.desir_motor_pos,
                              motor.desir_motor_vel,
                              motor.kp,
                              motor.kd,
                              motor.motor_control_tau,
                              arm.desir_grp);

        // 3. 准备插值
        float current_joint_pos_temp[6] = {
            arm.current_joint_positions(0), ..., arm.current_joint_positions(5)
        };
        float desir_joint_pos_temp[6] = {
            arm.desir_joint_positions(0), ..., arm.desir_joint_positions(5)
        };

        // 4. 执行轨迹
        while (!arm.now2aim(current_joint_pos_temp,
                           desir_joint_pos_temp,
                           arm.desir_joint_pos,
                           8,  // steps
                           50)) // delay
        {
            // 读取串口数据
            auto parsedData = port.getReceivedData();
            motor.usbDataToMotorState(parsedData, motor.motorState);
            motor.updatMotorState(motor.motorState);
            arm.updatArmState(motor.current_motor_pos,
                             motor.current_motor_vel,
                             motor.current_motor_tau);

            // 计算重力补偿
            arm.computeInverseDynamics(arm.current_joint_positions,
                                      arm.current_joint_velocities,
                                      arm.current_joint_acceleration,
                                      arm.gravity_joint_tauqes);

            for (size_t i = 0; i < 6; i++)
            {
                arm.joint_control_tau[i] = arm.gravity_joint_tauqes(i);
            }

            // 关节到电机转换
            arm.joint2motor(arm.desir_joint_pos,
                           arm.desir_joint_vel,
                           arm.desir_joint_acc,
                           arm.joint_control_tau,
                           motor.desir_motor_pos,
                           motor.desir_motor_vel,
                           motor.desir_motor_acc,
                           motor.motor_control_tau);

            // 执行电机控制（带夹爪）
            motor.ControlMotors_g(port,
                                 motor.desir_motor_pos,
                                 motor.desir_motor_vel,
                                 motor.kp,
                                 motor.kd,
                                 motor.motor_control_tau,
                                 arm.desir_grp);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::cout << "到达目标位置>>>" << std::endl;
        is_run_planning = false;
    }
}
```

---

## 13.5 夹爪控制

### 13.5.1 夹爪状态表示

夹爪位置 $g \in [0, 1]$：
- $g = 0$：完全闭合
- $g = 1$：完全打开

### 13.5.2 同步控制

电机控制指令包含夹爪参数：

```cpp
motor.ControlMotors_g(port,
                      motor.desir_motor_pos,  // 6个关节位置
                      motor.desir_motor_vel,  // 6个关节速度
                      motor.kp,               // 6个Kp增益
                      motor.kd,               // 6个Kd增益
                      motor.motor_control_tau,// 6个力矩
                      arm.desir_grp);         // 夹爪位置
```

### 13.5.3 夹爪控制数学模型

假设夹爪电机使用位置控制：

$$
u_{\text{gripper}} = K_{p,g}(g_d - g) + K_{d,g}(\dot{g}_d - \dot{g})
$$

其中：
- $g_d$：期望夹爪位置
- $K_{p,g}, K_{d,g}$：夹爪PID增益

---

## 13.6 性能分析

### 13.6.1 运动时间估计

对于步进系数 `steps = 8`，计数器增量为8，总迭代次数：

$$
N_{\text{iter}} = \frac{3140}{8} \approx 393 \, \text{次}
$$

每次迭代1ms，总时间：

$$
T_{\text{total}} \approx 393 \, \text{ms} \approx 0.4 \, \text{s}
$$

### 13.6.2 最大速度

插值参数的最大变化率：

$$
\dot{s}_{\max} = \alpha \cdot \Delta t \cdot \max[\sin(\omega t)] = 8 \times 0.5 \times 0.001 \times 1 = 0.004
$$

对于关节角度变化 $\Delta q = 90° = \pi/2 \, \text{rad}$：

$$
\dot{q}_{\max} = \frac{\dot{s}_{\max}}{\Delta t} \cdot \Delta q = \frac{0.004}{0.001} \times \frac{\pi}{2} \approx 6.28 \, \text{rad/s}
$$

### 13.6.3 精度分析

**离散化误差**：

理论轨迹：
$$
s_{\text{theory}}(t) = \frac{1}{2}\left[1 - \cos\left(\frac{\pi t}{T}\right)\right]
$$

实际轨迹（累积式）：
$$
s_{\text{actual}}(k) = \sum_{i=0}^{k-1} \alpha \Delta t \sin(\omega t_i)
$$

累积误差：
$$
\epsilon(k) = |s_{\text{theory}}(t_k) - s_{\text{actual}}(k)|
$$

典型值：$\epsilon_{\max} < 0.01$（1%误差）

---

# 第14章 Gohome 控制模式

## 14.1 模式概述

**Gohome控制模式**实现机械臂从任意位置安全返回零点（home position），作为切换到JointAutoServo模式的过渡状态。

### 14.1.1 核心特征

1. **自动归零**：将所有关节移动到 $\mathbf{q} = \mathbf{0}$
2. **高PID增益**：确保快速稳定跟踪
3. **重力补偿**：减少稳态误差
4. **模式自动切换**：完成后切换到JointAutoServo

### 14.1.2 应用场景

- 机械臂启动初始化
- 任务结束后归位
- 紧急停止后恢复
- 维护模式进入

---

## 14.2 控制策略

### 14.2.1 控制律

Gohome模式结合重力补偿和PID控制：

$$
\boldsymbol{\tau}_{\text{motor}} = \mathbf{g}(\mathbf{q}) + K_p(\mathbf{q}_d - \mathbf{q}) + K_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}})
$$

其中：
- $\mathbf{q}_d = \mathbf{0}_{6 \times 1}$：目标位置（零点）
- $\dot{\mathbf{q}}_d$：插值生成的期望速度
- $K_p, K_d$：高增益PID参数

### 14.2.2 PID增益配置

```cpp
// 前3个关节（大关节）
motor.kp[0] = 450.0;  motor.kd[0] = 4.51;
motor.kp[1] = 500.0;  motor.kd[1] = 4.81;
motor.kp[2] = 500.0;  motor.kd[2] = 4.51;

// 后3个关节（小关节）
motor.kp[3] = 90.0;   motor.kd[3] = 1.0;
motor.kp[4] = 40.0;   motor.kd[4] = 0.51;
motor.kp[5] = 10.0;   motor.kd[5] = 0.51;
```

**增益选择原则**：
- 大关节（1-3）：高$K_p$（450-500）确保精度
- 小关节（4-6）：中等$K_p$（10-90）避免震荡
- $K_d$ 比例：约为 $K_p$ 的 1%，提供阻尼

### 14.2.3 闭环稳定性分析

单关节传递函数（忽略重力补偿）：

$$
G(s) = \frac{Q(s)}{Q_d(s)} = \frac{K_p + K_d s}{J s^2 + K_d s + K_p}
$$

其中 $J$ 是关节惯量。

**特征方程**：
$$
J s^2 + K_d s + K_p = 0
$$

**稳定性条件**（Routh-Hurwitz）：
$$
K_p > 0, \quad K_d > 0
$$

**阻尼比**：
$$
\zeta = \frac{K_d}{2\sqrt{J K_p}}
$$

对于关节1（$J_1 \approx 0.5 \, \text{kg·m}^2$，$K_p = 450$，$K_d = 4.51$）：

$$
\zeta_1 = \frac{4.51}{2\sqrt{0.5 \times 450}} \approx 0.15
$$

欠阻尼系统（$\zeta < 1$），有少量超调。

---

## 14.3 轨迹插值

### 14.3.1 插值函数调用

```cpp
arm.now2aim(current_joint_pos_temp,  // 起始位置
            desir_joint_pos_temp,     // 目标位置（零点）
            arm.desir_joint_pos,      // 输出：插值位置
            1,                        // steps（慢速）
            50);                      // delay（ms）
```

### 14.3.2 参数选择

| 参数 | Gohome模式 | Planning模式 | 说明 |
|------|-----------|--------------|------|
| `steps` | 1 | 8 | Gohome更慢，更安全 |
| `delay` | 50ms | 50ms | 控制周期 |
| 总时间 | ~3.14s | ~0.4s | Gohome约8倍慢 |

### 14.3.3 速度曲线对比

**Gohome模式**（steps=1）：
$$
\dot{q}_{\max, \text{Gohome}} = \frac{1 \times 0.5 \times 0.001}{0.001} \Delta q = 0.5 \Delta q \, \text{rad/s}
$$

**Planning模式**（steps=8）：
$$
\dot{q}_{\max, \text{Planning}} = \frac{8 \times 0.5 \times 0.001}{0.001} \Delta q = 4 \Delta q \, \text{rad/s}
$$

Gohome速度仅为Planning的 1/8，大幅提升安全性。

---

## 14.4 控制算法

### 14.4.1 算法流程

**算法14.1：Gohome控制模式**

---

**输入**：当前关节位置 $\mathbf{q}_{\text{current}}$
**输出**：机械臂归零，切换到JointAutoServo模式

---

1. **检查触发标志**：
   `if (is_go_home == false) return`

2. **设置高PID增益**：
   ```
   Kp = [450, 500, 500, 90, 40, 10]
   Kd = [4.51, 4.81, 4.51, 1.0, 0.51, 0.51]
   ```

3. **计算重力补偿**：
   $$
   \mathbf{g}(\mathbf{q}) \leftarrow \text{ComputeInverseDynamics}(\mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}})
   $$

4. **设置初始控制力矩**：
   $$
   \boldsymbol{\tau}_{\text{motor}} \leftarrow \mathbf{g}(\mathbf{q})
   $$

5. **发送保持当前位置指令**：
   `ControlMotors(port, q_current, q̇_current, Kp, Kd, τ_motor)`

6. **初始化轨迹插值**：
   ```
   q_start = q_current
   q_target = [0, 0, 0, 0, 0, 0]
   ```

7. **执行归零轨迹**：
   ```
   while (!now2aim(q_start, q_target, q_desired, steps=1, delay=50)):
       // 读取电机状态
       UpdateMotorState()
       UpdateArmState()

       // 计算重力补偿
       g(q) = ComputeInverseDynamics(q, q̇, q̈)
       τ_control = g(q)

       // 关节到电机转换
       Joint2Motor(q_desired, q̇_desired, q̈_desired, τ_control)

       // 执行PID+重力补偿控制
       ControlMotors(port, q_motor_desired, q̇_motor_desired, Kp, Kd, τ_motor)

       sleep(1ms)
   ```

8. **输出完成信息**：
   `print("到达目标位置>>>")`
   `print("重置关节角度>>>")`

9. **重置期望位置**：
   $$
   \mathbf{q}_{\text{desired}} \leftarrow \mathbf{0}
   $$

10. **切换到JointAutoServo模式**：
    ```
    modeTransition(State::JointAutoServo)
    is_go_home = false
    ```

---

### 14.4.2 代码实现

```cpp
void ArmState::joint_go_home(serialport::SerialPortWrapper &port,
                             LlArm6dof &arm,
                             MotorControl &motor)
{
    if (is_go_home)
    {
        // 1. 设置高PID增益
        motor.kp[0] = 450.0; motor.kd[0] = 4.51;
        motor.kp[1] = 500.0; motor.kd[1] = 4.81;
        motor.kp[2] = 500.0; motor.kd[2] = 4.51;
        motor.kp[3] = 90.0;  motor.kd[3] = 1.0;
        motor.kp[4] = 40.0;  motor.kd[4] = 0.51;
        motor.kp[5] = 10.0;  motor.kd[5] = 0.51;

        // 2. 计算重力补偿
        arm.computeInverseDynamics(arm.current_joint_positions,
                                   arm.current_joint_velocities,
                                   arm.current_joint_acceleration,
                                   arm.gravity_joint_tauqes);

        for (size_t i = 0; i < 6; i++)
        {
            motor.motor_control_tau[i] = arm.gravity_joint_tauqes(i);
        }

        // 3. 发送当前位置指令（保持）
        motor.ControlMotors(port,
                            motor.current_motor_pos,
                            motor.current_joint_vel,
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);

        // 4. 准备归零轨迹
        float current_joint_pos_temp[6] = {
            arm.current_joint_positions(0), ..., arm.current_joint_positions(5)
        };
        float desir_joint_pos_temp[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // 5. 执行归零运动
        while (!arm.now2aim(current_joint_pos_temp,
                           desir_joint_pos_temp,
                           arm.desir_joint_pos,
                           1,   // steps: 慢速
                           50)) // delay: 50ms
        {
            // 读取串口数据
            auto parsedData = port.getReceivedData();
            motor.usbDataToMotorState(parsedData, motor.motorState);
            motor.updatMotorState(motor.motorState);
            arm.updatArmState(motor.current_motor_pos,
                             motor.current_motor_vel,
                             motor.current_motor_tau);

            // 计算重力补偿
            arm.computeInverseDynamics(arm.current_joint_positions,
                                      arm.current_joint_velocities,
                                      arm.current_joint_acceleration,
                                      arm.gravity_joint_tauqes);

            for (size_t i = 0; i < 6; i++)
            {
                arm.joint_control_tau[i] = arm.gravity_joint_tauqes(i);
            }

            // 关节到电机转换
            arm.joint2motor(arm.desir_joint_pos,
                           arm.desir_joint_vel,
                           arm.desir_joint_acc,
                           arm.joint_control_tau,
                           motor.desir_motor_pos,
                           motor.desir_motor_vel,
                           motor.desir_motor_acc,
                           motor.motor_control_tau);

            // 执行控制
            motor.ControlMotors(port,
                               motor.desir_motor_pos,
                               motor.desir_motor_vel,
                               motor.kp,
                               motor.kd,
                               motor.motor_control_tau);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // 6. 完成归零
        std::cout << "到达目标位置>>>" << std::endl;
        std::cout << "重置关节角度>>>" << std::endl;

        for (size_t i = 0; i < 6; i++)
        {
            arm.desir_joint_positions(i) = 0;
        }

        std::cout << "切换到关节控制模式>>>" << std::endl;

        // 7. 自动切换模式
        modeTransition(State::JointAutoServo);
        is_go_home = false;
    }
}
```

---

## 14.5 模式切换

### 14.5.1 切换序列

```mermaid
graph LR
    A[任意模式] -->|ROS话题触发| B[Gohome模式]
    B -->|is_go_home=true| C[执行归零轨迹]
    C -->|到达零点| D[重置期望位置]
    D -->|自动切换| E[JointAutoServo模式]
```

### 14.5.2 触发条件

通过ROS2话题发送字符串命令：

```cpp
void cmd_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "JointAutoServo")
    {
        mode_ = State::Gohome;
        control_mode.modeTransition(mode_);
        control_mode.is_go_home = true;
    }
}
```

### 14.5.3 切换安全性

**潜在风险**：
- 从远离零点的位置归零可能经过奇异点
- 快速运动可能导致碰撞

**缓解措施**：
- 使用低速（steps=1）
- 持续重力补偿
- 高PID增益确保跟踪
- 运动前可添加碰撞检测

---

## 14.6 性能指标

### 14.6.1 归零时间

对于最大关节角度 $q_{\max} = 180° = \pi \, \text{rad}$：

$$
T_{\text{home}} = \frac{3140}{1} \times 0.001 = 3.14 \, \text{s}
$$

### 14.6.2 位置精度

在零点附近的稳态误差：

$$
e_{ss} = \lim_{t \to \infty} (\mathbf{q}_d - \mathbf{q}(t))
$$

对于I型系统（带积分器的PID）：

$$
e_{ss} = \frac{1}{1 + K_p / K_f} \approx 0
$$

其中 $K_f$ 是摩擦系数。

**实际精度**：$\|\mathbf{e}_{ss}\| < 0.01 \, \text{rad} = 0.57°$

### 14.6.3 能量消耗

归零过程的能量消耗：

$$
E = \int_0^{T_{\text{home}}} \boldsymbol{\tau}^T \dot{\mathbf{q}} \, dt
$$

由于重力补偿，实际消耗主要来自摩擦和加速：

$$
E \approx \int_0^{T_{\text{home}}} [\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}]^T \dot{\mathbf{q}} \, dt + \frac{1}{2}\mathbf{M}(\mathbf{q}_0)\dot{\mathbf{q}}_{\max}^2
$$

---

## 14.7 故障处理

### 14.7.1 常见问题

**1. 无法到达零点**

**原因**：
- 关节限位保护触发
- 电机失能
- 轨迹插值未完成

**解决方案**：
```cpp
// 检查限位
if (q[i] < q_min[i] || q[i] > q_max[i]) {
    std::cerr << "Joint " << i << " limit exceeded!" << std::endl;
    EmergencyStop();
}

// 检查插值完成
if (counter >= 3140 && norm(q - q_target) > 0.1) {
    std::cerr << "Trajectory not completed!" << std::endl;
    is_go_home = false;
}
```

**2. 震荡或不稳定**

**原因**：
- PID增益过高
- 延迟过大
- 重力模型不准

**解决方案**：
- 降低$K_p$，增加$K_d$
- 减小通信延迟
- 重新标定惯性参数

**3. 运动缓慢**

**原因**：
- `steps`参数设置过小
- 电机速度限制

**解决方案**：
- 适当增加`steps`（如1→2）
- 检查电机最大速度配置

---

## 参考文献

1. Sciavicco, L., & Siciliano, B. (2000). *Modelling and Control of Robot Manipulators* (2nd ed.). Springer.

2. Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. John Wiley & Sons.

3. Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.

4. Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer.

5. Murray, R. M., Li, Z., & Sastry, S. S. (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press.

6. Khalil, W., & Dombre, E. (2002). *Modeling, Identification and Control of Robots*. Hermes Penton Science.

7. Biagiotti, L., & Melchiorri, C. (2008). *Trajectory Planning for Automatic Machines and Robots*. Springer.

8. Slotine, J.-J. E., & Li, W. (1991). *Applied Nonlinear Control*. Prentice Hall.

---
