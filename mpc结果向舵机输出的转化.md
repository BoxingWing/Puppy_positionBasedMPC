#### MPC与位置控制电机结合遇到的问题

MPC根据前个周期的状态反馈，进而输出包括下个周期期望达到的足底相对位置$\pmb{p}$和足底接触力$\pmb{f}$。然而由于舵机只能控制关节位置，因此最直接的控制方式是忽视MPC输出结果中的足底接触力$\pmb{f}$而直接将足底相对位置$\pmb{p}$通过逆运动学反解之后得到的关节位置$\bar{\pmb{q}}$作为最终设定值：
$$
\bar{\pmb{q}}=\text{IK}(\pmb{p})=\text{MPC}_P(\pmb{q}[\text{N-1}])
$$
​    然而，由于舵机控制总是存在跟踪误差的，因此实际到达位置总与期望位置有差异，因此MPC的要求位置总是无法达到：
$$
\pmb{q}[\text{N}]=\text{MPC}_P(\pmb{q}[\text{N-1}])-\frac{1}{k}\pmb{J}^\text{T}\hat{\pmb{f}} < \text{MPC}_P(\pmb{q}[\text{N-1}])\\
$$
​    上式中的$\hat{\pmb{f}}$为实际的脚底接触力。

  因此在下个周期，由于需要跟踪质心位置，MPC会根据当前的关节角位置再次进行规划。当然，其趋势必然是增大脚底接触力进而使预计的关节角增量可以弥补关节角跟踪误差。假设MPC可以最终跟踪到期望的质心状态，则有：
$$
\pmb{q}_d=\text{MPC}'_P(\pmb{q}_d)-\frac{1}{k}\pmb{J}^\text{T}\pmb{f}_d
$$
即：
$$
\text{MPC}'_P(\pmb{q}_d)=\pmb{q}'_d=\pmb{q}_d+\frac{1}{k}\pmb{J}^\text{T}\pmb{f}_d
$$
因此有：
$$
\text{MPC}'_f(\pmb{q}_d)=\text{ID}(\pmb{q}_d,\pmb{q}'_d)>\pmb{f}_d
$$
  显然，对于正常MPC控制器和力矩控制电机而言，实现$\pmb{q}_d$对应的机身状态应该有：
$$
\left\{
\begin{aligned}
\text{MPC}_p(\pmb{q}_d)&=\pmb{q}_d \\
\text{MPC}_f(\pmb{q}_d)&=\pmb{f}_d
\end{aligned}
\right.
$$
   在使用位置控制舵机后变成了：
$$
\left\{
\begin{aligned}
\text{MPC}'_p(\pmb{q}_d)&=\pmb{q}'_d=\pmb{q}_d+\frac{1}{k}\pmb{J}^\text{T}\pmb{f}_d \\
\text{MPC}'_f(\pmb{q}_d)&=\text{ID}(\pmb{q}_d,\pmb{q}'_d)>\pmb{f}_d
\end{aligned}
\right.
$$
  因此引入了天然的模型误差，导致MPC无法根据原有模型进行正确规划。实验现象为各关节角度不断减小直到规划足底力发散至一个很大的值。



#### MPC与位置控制电机的结合方法尝试：与导纳控制结合

该方法的主要思路即通过MPC输出得到的足底接触力得到关节力矩输出，再将此转换为位置增量与MPC输出得到的关节位置相叠加，进而一定程度上消除舵机位置跟踪误差对于MPC控制的影响。

依据导纳控制，位置增量可按下计算：
$$
\left [
\begin{matrix}
\Delta q_1 \\
\Delta q_2 \\
\Delta q_3
\end{matrix}
\right]=
\left [
\begin{matrix}
\frac{1}{k_1+b_1s} & 0 & 0 \\
0 & \frac{1}{k_2+b_2s} & 0 \\
0 & 0 & \frac{1}{k_3+b_3s}
\end{matrix}
\right ]\pmb{J}^\text{T}
\left [
\begin{matrix}
f_x \\
f_y \\
f_z
\end{matrix}
\right ]
$$
则关节位置设定值如下：
$$
\pmb{q}_\text{cmd}=\text{MPC}_p+\Delta\pmb{q}
$$
不过， 为了与其他控制模块相结合，程序中会将此角度转换为脚底位置：
$$
\pmb{p}_\text{cmd}=\text{FK}(\pmb{q}_\text{cmd})
$$




















