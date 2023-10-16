# 6dof_arm_robot

The main goals of this project is to build a simulation of a Six Degrees of Freedom (6 DOF) robotic arm using OpenGL and control the robot based on Task Space Control Algorithm with line trajectory path. 
- The mathematical model of 6 DOF robotic arm and OpenGL template is obtained from Robotics and Mechatronics Lectures in University of Indonesia and were created by Dr. Abdul Muis, M.Eng (Autonomous Control Electronics (ACONICS) Research Group).

<h1>Mathematical Model</h1>

<h2>Denavit-Hartenberg Parameters</h2>

The Denavit-Hartenberg (D-H) parameters were determined as shown in the table below. These parameters can be used to determine forward kinematic expressions in $xyz$-axis by turning it to homogenous transformation matrix.
| $i$ | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$ |
| --- | --- | --- | --- | --- |
| 1 | $0^o$ | $0$ | $d_0$ | $\theta_1$ |
| 2 | $90^o$ | $a_1$ | $0$ | $\theta_2$ |
| 3 | $0^o$ | $a_2$ | $0$ | $\theta_3$ |
| 4 | $90^o$ | $a_3$ | $d_4$ | $\theta_4$ |
| 5 | $-90^o$ | $0$ | $0$ | $\theta_5$ |
| 6 | $90^o$ | $a_5$ | $d_6$ | $\theta_6$ |
| 7 | $0^o$ | $0$ | $d_7$ | $0$ |

<h2>Forward Kinematics</h2>

$x = L_1\cos(\theta_1) + L_5\left(\cos(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3))) - \sin(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$+ L_6\left(\sin(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3))) + \cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$+ L_7\left(\sin(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3))) + \cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$+ L_3(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) + L_4(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2)) + L_2\cos(\theta_1)\cos(\theta_2)$

$y = L_4(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2)) - L_3(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)) + L_1\sin(\theta_1)$
$- L_5\left(\cos(\theta_5)(\cos(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1))) + \sin(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$
$- L_6\left(\sin(\theta_5)(\cos(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1))) - \cos(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$
$- L_7\left(\sin(\theta_5)(\cos(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1))) - \cos(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right) + L_2\cos(\theta_2)\sin(\theta_1)$

$z = L_0 + L_3(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2)) - L_4(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3))$
$+ L_5\left(\sin(\theta_5)(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3)) + \cos(\theta_4)\cos(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2))\right)$
$- L_6\left(\cos(\theta_5)(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3)) - \cos(\theta_4)\sin(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2))\right)$
$- L_7\left(\cos(\theta_5)(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3)) - \cos(\theta_4)\sin(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2))\right) + L_2\sin(\theta_2)$

<h2>Jacobian Matrix</h2>

$$
J = \begin{pmatrix}
\frac{\partial x}{\partial \theta_1} & \frac{\partial x}{\partial \theta_2} & \frac{\partial x}{\partial \theta_3} & \frac{\partial x}{\partial \theta_4} & \frac{\partial x}{\partial \theta_5} & \frac{\partial x}{\partial \theta_6}\\
\frac{\partial y}{\partial \theta_1} & \frac{\partial y}{\partial \theta_2} & \frac{\partial y}{\partial \theta_3} & \frac{\partial y}{\partial \theta_4} & \frac{\partial y}{\partial \theta_5} & \frac{\partial y}{\partial \theta_6}\\
\frac{\partial z}{\partial \theta_1} & \frac{\partial z}{\partial \theta_2} & \frac{\partial z}{\partial \theta_3} & \frac{\partial z}{\partial \theta_4} & \frac{\partial z}{\partial \theta_5} & \frac{\partial z}{\partial \theta_6}\\
\end{pmatrix}
$$

where

$\frac{\partial x}{\partial \theta_1} = L_3(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)) - L_4(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2)) - L_1\sin(\theta_1)$
$+ L_5\left(\cos(\theta_5)(\cos(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1))) + \sin(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$
$+ L_6\left(\sin(\theta_5)(\cos(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1))) - \cos(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$
$+ L_7\left(\sin(\theta_5)(\cos(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1))) - \cos(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$
$- L_2\cos(\theta_2)\sin(\theta_1)$

$\frac{\partial x}{\partial \theta_2} = L_6\left(\cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) - \cos(\theta_4)\sin(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$- L_5\left(\sin(\theta_5)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) + \cos(\theta_4)\cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$+ L_7\left(\cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) - \cos(\theta_4)\sin(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$- L_3(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2)) + L_4(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) - L_2\cos(\theta_1)\sin(\theta_2)$

$\frac{\partial x}{\partial \theta_3} =L_6\left(\cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) - \cos(\theta_4)\sin(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$- L_5\left(\sin(\theta_5)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) + \cos(\theta_4)\cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$+ L_7\left(\cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) - \cos(\theta_4)\sin(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$- L_3(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2)) + L_4(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3))$

$\frac{\partial x}{\partial \theta_4} = L_5\cos(\theta_5)(\cos(\theta_4)\sin(\theta_1) - \sin(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)))$
$+ L_6\sin(\theta_5)(\cos(\theta_4)\sin(\theta_1) - \sin(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)))$
$+ L_7\sin(\theta_5)(\cos(\theta_4)\sin(\theta_1) - \sin(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3))$

$\frac{\partial x}{\partial \theta_5} = L_6\left(\cos(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) - \sin(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$- L_5\left(\sin(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) + \cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$+ L_7\left(\cos(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) - \sin(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$

$\frac{\partial x}{\partial \theta_6} = 0$

$\frac{\partial y}{\partial \theta_1} = L_1\cos(\theta_1) + L_5\left(\cos(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) - \sin(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$+ L_6\left(\sin(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) + \cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$+ L_7\left(\sin(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) + \cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$
$+ L_3(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) + L_4(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2)) + L_2\cos(\theta_1)\cos(\theta_2)$

$\frac{\partial y}{\partial \theta_2} = L_5\left(\sin(\theta_5)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)) - \cos(\theta_4)\cos(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$
$- L_4(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)) - L_3(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))$
$- L_6\left(\cos(\theta_5)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)) + \cos(\theta_4)\sin(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$
$- L_7\left(\cos(\theta_5)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)) + \cos(\theta_4)\sin(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$
$- L_2\sin(\theta_1)\sin(\theta_2)$

$\frac{\partial y}{\partial \theta_3} = L_5\left(\sin(\theta_5)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)) - \cos(\theta_4)\cos(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$
$- L_4(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)) - L_3(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))$
$- L_6\left(\cos(\theta_5)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)) + \cos(\theta_4)\sin(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$
$- L_7\left(\cos(\theta_5)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)) + \cos(\theta_4)\sin(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$

$\frac{\partial y}{\partial \theta_4} = - L_5\cos(\theta_5)(\cos(\theta_1)\cos(\theta_4) - \sin(\theta_4)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)))$
$- L_6\sin(\theta_5)(\cos(\theta_1)\cos(\theta_4) - \sin(\theta_4)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)))$
$- L_7\sin(\theta_5)(\cos(\theta_1)\cos(\theta_4) - \sin(\theta_4)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1))$

$\frac{\partial y}{\partial \theta_5} = L_5\left(\sin(\theta_5)(\cos(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)) - \cos(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$
$- L_6\left(\cos(\theta_5)(\cos(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)) + \sin(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$
$- L_7\left(\cos(\theta_5)(\cos(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\sin(\theta_1)\sin(\theta_2)\sin(\theta_3) - \cos(\theta_2)\cos(\theta_3)\sin(\theta_1)) + \sin(\theta_5)(\cos(\theta_2)\sin(\theta_1)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_1)\sin(\theta_2))\right)$

$\frac{\partial y}{\partial \theta_6} = 0$

$\frac{\partial z}{\partial \theta_1} = 0$

$\frac{\partial z}{\partial \theta_2} = L_3(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3)) + L_4(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2))$
$- L_5\left(\sin(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2)) - \cos(\theta_4)\cos(\theta_5)(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3))\right)$
$+ L_6\left(\cos(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2)) + \cos(\theta_4)\sin(\theta_5)(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3))\right)$
$+ L_7\left(\cos(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2)) + \cos(\theta_4)\sin(\theta_5)(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3))\right)$
$+ L_2\cos(\theta_2)$

$\frac{\partial z}{\partial \theta_3} = L_3(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3)) + L_4(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2))$
$- L_5\left(\sin(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2)) - \cos(\theta_4)\cos(\theta_5)(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3))\right)$
$+ L_6\left(\cos(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2)) + \cos(\theta_4)\sin(\theta_5)(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3))\right)$
$+ L_7\left(\cos(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2)) + \cos(\theta_4)\sin(\theta_5)(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3))\right)$

$\frac{\partial z}{\partial \theta_4} = - L_6\sin(\theta_4)\sin(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2))$
$- L_7\sin(\theta_4)\sin(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2))$
$- L_5\cos(\theta_5)\sin(\theta_4)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2))$

$\frac{\partial z}{\partial \theta_5} = L_5\left(\cos(\theta_5)(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3)) - \cos(\theta_4)\sin(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2))\right)$
$+ L_6\left(\sin(\theta_5)(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3)) + \cos(\theta_4)\cos(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2))\right)$
$+ L_7\left(\sin(\theta_5)(\cos(\theta_2)\cos(\theta_3) - \sin(\theta_2)\sin(\theta_3)) + \cos(\theta_4)\cos(\theta_5)(\cos(\theta_2)\sin(\theta_3) + \cos(\theta_3)\sin(\theta_2))\right)$

$\frac{\partial z}{\partial \theta_6} = 0$

<h2>Inverse Jacobian Matrix</h2>

For $J_{m \times n}: m < n$

Then $J^{-1}(\theta) = J^{T}{\theta} \cdot (J(\theta) \cdot J^{T}(\theta))^{-1}$

<h1>Control Flow</h1>

<p align="center">
  <img src="https://github.com/JordyMarcius/3dof_arm_robot/assets/65435469/9b5c04cd-e82b-430d-b883-5c217a2ae79c"/>
</p>

Flow of task space control: 
- Sensors read the angle position ($\theta_1, \theta_2, \theta_3$, $\theta_4, \theta_5, \theta_6$) of each servo motor.
- These angles ($\theta_1, \theta_2, \theta_3$, $\theta_4, \theta_5, \theta_6$) are used to calculate the coordinate ($x, y, z$) of end effector using forward kinematic expressions.
- Compute line trajectory path. If the control algorithm is implemented correctly, the end effector of the robot will follow this line repeatedly in a certain time.
- Calculate error of end effector coordinates.
- Input error to PID controller to calculate coordinate acceleration in $xyz$-axis ($\ddot{x}, \ddot{y}, \ddot{z}$).
- Input angle position ($\theta_1, \theta_2, \theta_3$) and coordinate acceleration ($\ddot{x}, \ddot{y}, \ddot{z}$) to inverse jacobian function. It will return the angle acceleration reference ($\ddot{\theta_1}, \ddot{\theta_2}, \ddot{\theta_3}$, $\ddot{\theta_4}, \ddot{\theta_5}, \ddot{\theta_6}$). 
- Calculate PWM signal from output signal of inverse jacobian function.
- Calculate angle speed in $xyz$-axis ($\dot{\theta_1}, \dot{\theta_2}, \dot{\theta_3}$, $\dot{\theta_4}, \dot{\theta_5}, \dot{\theta_6}$) using PWM signal and motor model.
- Calculate the motor position ($\theta_1, \theta_2, \theta_3$, $\theta_4, \theta_5, \theta_6$) by adding the previous angle position with integral of angle speed.
- Wait for the next loop and start from the first point.

<h1>Simulation</h1>

*Please click the start button in the right top of the .gif picture if the .gif motion was disabled.*

![xGif](https://github.com/JordyMarcius/6dof_arm_robot/assets/65435469/7d40f553-df2c-490b-805b-ad3338e81c14)
Figure 1. Robotic arm movement in $x$-axis

![yGif](https://github.com/JordyMarcius/6dof_arm_robot/assets/65435469/18be705d-e074-49ee-9fc8-aa304a801f37)
Figure 2. Robotic arm movement in $y$-axis

![zGif](https://github.com/JordyMarcius/6dof_arm_robot/assets/65435469/5de8fd9f-75a5-4632-ad4f-780af170e60a)
Figure 3. Robotic arm movement in $z$-axis

![fullGif](https://github.com/JordyMarcius/6dof_arm_robot/assets/65435469/d2a09e40-6b73-4a90-82db-13b7537c90ef)
Figure 4. Robotic arm movement in $xyz$-axis

<h1>Results Comparison</h1>

Below are the results comparison of an actual and reference of end effector coordinate. The purple line shows the reference coordinate and the green line shows the actual coordinate of end effector in $xyz$-axis. As can be seen in those figures, the end effector of this robotic arm can track the reference coordinate and follow the line trajectory. It indicates that the control algorithm was implemented correctly.

![image](https://github.com/JordyMarcius/6dof_arm_robot/assets/65435469/7f053aa2-68b0-4a0f-8439-d82d295c7736)
Figure 5. Comparison of $x$ coordinate

![image](https://github.com/JordyMarcius/6dof_arm_robot/assets/65435469/257f5bc6-eb7b-4a86-b3bf-0456dba98a3a)
Figure 6. Comparison of $y$ coordinate

![image](https://github.com/JordyMarcius/6dof_arm_robot/assets/65435469/b4e12c84-6b57-4ab3-bcc7-b36c112ca8c0)
Figure 7. Comparison of $z$ coordinate

![image](https://github.com/JordyMarcius/6dof_arm_robot/assets/65435469/c152668e-ed32-4820-a353-78597d8b3cb2)
Figure 8. Angle response
