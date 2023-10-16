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

$$
x = L_1\cos(\theta_1) + L_5\left(\cos(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3))) - \sin(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right) + L_6\left(\sin(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3))) + \cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right) L_7\left(\sin(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3))) + \cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right) + L_3(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) + L_4(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2)) + L_2\cos(\theta_1)\cos(\theta_2)
$$

x = L_1\cos(\theta_1) + L_5\left(\cos(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3))) - \sin(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right) \\
+ L_6\left(\sin(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3))) + \cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right) \\
+ L_7\left(\sin(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3))) + \cos(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right) \\
+ L_3(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3)) + L_4(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2)) + L_2\cos(\theta_1)\cos(\theta_2)

$x = L_1\cos(\theta_1) + L_5\left(\cos(\theta_5)(\sin(\theta_1)\sin(\theta_4) + \cos(\theta_4)(\cos(\theta_1)\cos(\theta_2)\cos(\theta_3) - \cos(\theta_1)\sin(\theta_2)\sin(\theta_3))) - \sin(\theta_5)(\cos(\theta_1)\cos(\theta_2)\sin(\theta_3) + \cos(\theta_1)\cos(\theta_3)\sin(\theta_2))\right)$


![xGif](https://github.com/JordyMarcius/6dof_arm_robot/assets/65435469/7d40f553-df2c-490b-805b-ad3338e81c14)

![yGif](https://github.com/JordyMarcius/6dof_arm_robot/assets/65435469/18be705d-e074-49ee-9fc8-aa304a801f37)

![zGif](https://github.com/JordyMarcius/6dof_arm_robot/assets/65435469/5de8fd9f-75a5-4632-ad4f-780af170e60a)

![fullGif](https://github.com/JordyMarcius/6dof_arm_robot/assets/65435469/d2a09e40-6b73-4a90-82db-13b7537c90ef)

