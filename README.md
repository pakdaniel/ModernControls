% Final Report (LaTex)

\documentclass[12pt]{article}
\usepackage[utf8]{inputenc}
\usepackage[margin=1in]{geometry}
\usepackage[svgnames]{xcolor}
\usepackage{color}
\usepackage{hyperref}
\usepackage{graphicx}
\usepackage{amsmath}
\usepackage{epsfig}
\usepackage{amssymb}
\usepackage{verbatim}
\usepackage{latexsym}
\usepackage{float}
\PassOptionsToPackage{hyphens}{url}\usepackage{hyperref}

%https://tex.stackexchange.com/questions/340717/ref-different-colors-for-figures-equations-etc

\hypersetup{
    colorlinks=true,
    linkcolor=blue,
    filecolor=blue,      
    urlcolor=blue
}


\begin{comment}
\usepackage{enumitem, amsmath, graphicx, wrapfig, float, booktabs, color, changepage, bigstrut, ragged2e, array, multirow}
\usepackage{amssymb}
\usepackage{xcolor}
\usepackage{comment}
\usepackage[svgnames]{xcolor}
\end{comment}


\newcommand{\codeblock}[1]{\colorbox{Gainsboro}{{\fontfamily{pcr}\selectfont #1}} \medskip}



\title{Trajectory Tracking for a 2 Degree of Freedom Robot Arm}
\author{Zhiyang Chen, Enea Dushaj, Daniel Pak}


\date{May 2021}

\begin{document}

\maketitle

\clearpage
\tableofcontents
\clearpage

\section{Abstract}

Robots are extremely versatile tools whose uses are endless. From industrial processes, to human assistance, to academic enlightenment, the study of robotics can lead to various applications. Robot arms can be used on a large scale for industrial processes to small scale like prosthetic bionic hands. This paper focuses on looking into control strategies to make a robust 2 degree of freedom, planar, vertical robot arm that can follow specified paths. 

This paper poses methods and a control infrastructure that generates a triangular trajectory for the 2 DOF robot arm to follow. Feedforward and feedback inputs are combined and input into the system. Additionally, noise is added to the input and output signal to make the system more realistic, measure the robustness of the system, and gauge the effectiveness of feedback control for this application. Furthermore, systematic error is introduced into the model in the form of inaccurate parameter estimation. Masses and lengths of each link is increased by 10\% to check the robustness of the system to incorrect parameters. A saturation input is incorporated into the model to simulate the maximum torque of a motor.  

Two controllers are implemented: LQR Control and Pole Placement. Different configurations of each controller, with and without systematic uncertainty, were tested. Additionally, a PID Controller for end effector position was wrapped into one LQR control configuration.




\newpage
\section{Model Description}
There are assumptions used to create this model.


\begin{enumerate}
    \item Mass is concentrated at the end of the joints
    \item Friction between joints and air resistance is neglected
    \item The robot arm is oriented vertically, so gravity acts as an external force
    \item Actuation effort is done by an idealized motor so the plant model of the motor is neglected
    \item Both joints are revolute with full {360\textdegree } range of motion
    \item The arm is infinitely stiff
    \item Knowledge of the full state is known, which consists of 2 angular positions and 2 angular velocities
\end{enumerate}

The free body diagram of the robot arm is shown in Figure \ref{fig:FBD}.

\begin{figure}[H]
\begin{center}
    \includegraphics[scale=0.7]{misc/robot_arm_FBD.png}
    \caption{Free Body Diagram for 2 DOF Robot Arm}
    \label{fig:FBD}
\end{center}
\end{figure}

The equations of motion for this 2 DOF robot arm can be determined by using the Euler-Lagrange Equations with external forces \cite{Modern_Robotics}. Knowing the torques applied to the robot arm at the joints to solve for the joint angle is called forward dynamics, which is stated as

\begin{equation}
    \tau = M(\theta)\ddot{\theta} + c(\theta,\dot{\theta}) + g(\theta)
    \label{eq:Forward Dynamics}
\end{equation}

where $\tau$ is 

\begin{equation}
    \tau = 
    \begin{bmatrix}
        \tau_{1} \\
        \tau_{2}
    \end{bmatrix} \hspace{5mm} \tau \in \mathbb{R}^{2}  \\
    \label{eq:Torque Input}
\end{equation}

and $\theta$ is 
\begin{equation}
    \theta = 
    \begin{bmatrix}
        \theta_{1} \\
        \theta_{2}
    \end{bmatrix} \hspace{5mm} \theta \in \mathbb{R}^{2}  \\
    \label{eq:Theta Vector}
\end{equation}



The components of each term in the forward dynamics equation are as follows. 

\begin{align*}
    M(\theta) &=
    \begin{bmatrix}
    m_{1}L_{1}^2+m_{2}(L_{1}^2+2L_{1}L_{2}\cos{\theta_{2}}+L_{2}^2) & m_{2}(L_{1}L_{2}\cos{\theta_2})+L_{2}^2) \\
    m_{2}(L_{1}L_{2}\cos{\theta_2})+L_{2}^2) & m_{2}L_{2}^2
    \end{bmatrix} \hspace{5mm} M \in \mathbb{R}^{2 \times 2}  \\
    c(\theta,\dot{\theta}) &= 
    \begin{bmatrix}
    -m_{2}L_{1}L_{2}\sin{\theta_{2}}(2\dot{\theta}_1\dot{\theta}_{2}+\dot{\theta}_{2}^2) \\
    m_{2}L_{1}L_{2}\dot{\theta}_{1}^2\sin{\theta_2}
    \end{bmatrix} \hspace{5mm} c \in \mathbb{R}^{2}\\ 
    g(\theta) &=
    \begin{bmatrix}
    (m_{1}+m_{2})L_{1}g\cos{\theta_1} +m_{2}gL_{2}\cos(\theta_{1}+\theta_{2}) \\
    m_{2}gL_{2}\cos(\theta_{1}+\theta_{2}) 
    \end{bmatrix} \hspace{5mm} g \in \mathbb{R}^{2}\\ 
\end{align*}


We define our state vector to constitute the joint angles and the joint velocities.

\begin{equation}
    x = 
    \begin{bmatrix}
        \theta_{1} \\
        \theta_{2} \\
        \dot{\theta}_{1} \\ 
        \dot{\theta}_{2}
    \end{bmatrix} \hspace{5mm} x \in \mathbb{R}^{4}
    \label{eq:State}
\end{equation}

Taking the time derivative on both sides of the state vector results in the following equation.

\begin{equation}
    \dot{x}  = 
    \begin{bmatrix}
    \dot{\theta}_{1} \\
    \dot{\theta}_{2} \\
    \ddot{\theta}_{1} \\ 
    \ddot{\theta}_{2}
    \end{bmatrix}
    = f(x,u) 
    \label{eq:Derivative of State}
\end{equation}

In Equation \ref{eq:Derivative of State}, $f(x,u)$ represents the system of nonlinear dynamics for the 2 DOF robot arm. ${\theta}_{1}$ and ${\theta}_{2}$ are also known by solving for $\ddot{\theta}$ in the forward dynamics. This is also known as the inverse dynamics and highlighted in Equation \ref{eq:Joint Acceleration}. 

\begin{equation}
    \ddot{\theta} = M^{-1}(\tau - c - g)  
    \label{eq:Joint Acceleration}
\end{equation}

Therefore, Equation \ref{eq:Joint Acceleration} can be substituted into Equation \ref{eq:Derivative of State} to get that

\begin{equation}
    f(x,u)  = 
    \begin{bmatrix}
    \dot{\theta}_{1} \\
    \dot{\theta}_{2} \\
    M^{-1}(\tau - c - g) \\ 
    \end{bmatrix}
    \hspace{5mm} f(x,u) \in \mathbb{R}^{4}
    \label{eq:Substitution into Derivative of State}
\end{equation}


\newpage 
\section{Controller Design}

\subsection{Control Scheme}

\begin{figure}[H] %H means here
\begin{center}
    \includegraphics[width=\textwidth]{misc/process_diagram.png}
    \caption{Process Block Diagram of 2 DOF Robot Arm}
    \label{fig:Block Diagram}
\end{center}
\end{figure}


In Figure \ref{fig:Block Diagram}, the control architecture is as follows. The specified end points of a line are converted into a trajectory between the two end points. The trajectory generation determines a feedfoward signal $u_{d}$ and the desired state $x_{d}$ of the robot arm. The feedback compensator, which is the controller, uses the difference in the measured output state to the desired state, and aims to correct it by creating the $u_{fb}$ signal. The input signal to the plant also accounts for input noise, and there is also sensor noise in the measured state at the output of the plant.


\begin{figure}[H] %H means here
\begin{center}
    \includegraphics[width=\textwidth]{misc/modified_process_diagram.png}
    \caption{Process Block Diagram with Camera to Know End-Effector Position}
    \label{fig:Modified Block Diagram}
\end{center}
\end{figure}
Because the previous controller scheme rely purely on the forward kinematics to deduce the robotic arm's end point position, it is very sensitive to changes in arm's length. To alleviate this problem, we incorporate a direct observer of the arm's end point, which could be a camera that tracks the end position of the arm. The accurate positional information is fed into a PI controller that generates input values to adjust the deviation in the end point's position.

\subsection{Trajectory Generation}
Referring back to Figure \ref{fig:Block Diagram}, the outputs of the trajectory generations include $u_{d}$ and $x_{d}$. $u_{d}$ in the case of two linkages robotic arm is $\begin{bmatrix} \tau_{1}(t) & \tau_{2}(t) \end{bmatrix}$, where $\tau_{1}$ represents the desired torque input for the first joint and $\tau_{2}$ represents the desired torque input for the second joint. $x_{d}$ in the case of the robotic arm is $\begin{bmatrix} \theta_{1}(t) & \theta_{2}(t) & \dot{\theta_{1}}(t) & \dot{\theta_{2}}(t) \end{bmatrix}$. Additionally, the equations of motion of the robotic arm relate $u_{d}$ and $x_{d}$ in the following manner $u_d = EOM(x_{d}, \ddot{\theta_{1}}, \ddot{\theta_{2}})$. The desired trajectory for the modeled robotic arm is going to be first generated as the path walked by the end position of the second linkage. This trajectory is interpreted in an Cartesian coordinate with two perpendicular axes $x$ and $y$. Simplify the end point of the robotic arm as a particle, the corresponding trajectory needs to include position, velocity, and acceleration as functions of time represented by the following one by six vector $\begin{bmatrix} x(t) & y(t) & \dot{x}(t) & \dot{y}(t) & \ddot{x}(t) & \ddot{y}(t) \end{bmatrix}$. For simplicity, we choose linear lines to be our desired paths, to be particular, we generate an equilateral triangle path for the robotic arm's end point to follow referring to Figure \ref{fig:arm end path}.

\begin{figure}[H] %H means here
\begin{center}
\minipage{0.4\textwidth}
    \includegraphics[width=\textwidth]{misc/triangle_path.png}
    \caption{Desired Path for the Robot Arm End Point}
    \label{fig:arm end path}
\endminipage\hfill
\end{center}
\end{figure}

A quadratic velocity profile is assigned to each edge alone the triangle path. The robotic' arm's tip will have desired velocity of zero at each vertex and maximum velocity at each center of the edges. To realize this speed profile, we utilized a scaling function $s(t)=\frac{3}{T^{2}} t^{2}-\frac{2}{T^{3}}t^{3}$, where $T$ is desired duration to execute the desired path. Following the the above function, we know $\dot{s}(t)=\frac{6}{T^{2}}t-\frac{6}{T^{3}}t^{2}$, and $\ddot{s}(t)=\frac{6}{T^{2}}-\frac{12}{T^{3}}t$. We can interpret the desired position alone the desired linear trajectory with Equation \ref{eq:linear_interpret} , where 
$\begin{bmatrix} x_{end} \\ y_{end} \\ \end{bmatrix}$ is the ending position of the linear trajectory and $\begin{bmatrix} x_{start} \\ y_{start} \\ \end{bmatrix}$ is the starting position of the linear trajectory. 
    
\begin{equation}
    \begin{bmatrix}
    x_{d} \\
    y_{d} \\
    \end{bmatrix} = s(t)(
    \begin{bmatrix}
    x_{end} \\
    y_{end} \\
    \end{bmatrix}-
    \begin{bmatrix}
    x_{start} \\
    y_{start} \\
    \end{bmatrix})
    \label{eq:linear_interpret}
\end{equation}
 
The desired end point velocity is interpreted alone the path using Equation \ref{eq:linear_interpret_v}.
\begin{equation}
    \begin{bmatrix}
    \dot{x_{d}} \\
    \dot{y_{d}} \\
    \end{bmatrix} = \dot{s}(t)(
    \begin{bmatrix}
    x_{end} \\
    y_{end} \\
    \end{bmatrix}-
    \begin{bmatrix}
    x_{start} \\
    y_{start} \\
    \end{bmatrix})
    \label{eq:linear_interpret_v}
\end{equation}

The desired end point acceleration is interpreted alone the path using Equation \ref{eq:linear_interpret_a}.

\begin{equation}
    \begin{bmatrix}
    \ddot{x_{d}} \\
    \ddot{y_{d}} \\
    \end{bmatrix} = \ddot{s}(t)(
    \begin{bmatrix}
    x_{end} \\
    y_{end} \\
    \end{bmatrix}-
    \begin{bmatrix}
    x_{start} \\
    y_{start} \\
    \end{bmatrix})
    \label{eq:linear_interpret_a}
\end{equation}
Then the generated trajectory will be converted from the Cartesian coordinate into angular form, particularly the angle values, angular velocity values, and angular acceleration values as functions of time represented by the following vector $\begin{bmatrix} \theta_{1}(t) & \theta_{2}(t) & \dot{\theta_{1}}(t) & \dot{\theta_{2}}(t) & \ddot{\theta_{1}}(t) & \ddot{\theta_{2}}(t) \end{bmatrix}$. The desired input is then calculated using equations of motion with angle positions, angular velocity, and angular accelerations as inputs.

The end position of the robotic arm is related to the joint angles by the following relationship, where l is a vector function of two dimensions.
\[
\begin{bmatrix}
x \\ 
y
\end{bmatrix}  
=
\begin{bmatrix}
L_{1}\cos(\theta_1) + L_{2}\cos(\theta_{1}+\theta_{2}) \\
L_{1}\sin(\theta_1) + L_{2}\sin(\theta_{1}+\theta_{2}) \\
\end{bmatrix} 
= l(\theta_{1}, \theta_{2})
\]

Taking derivatives of both sides of the equations and isolate out $\dot{\theta_{1}}$ and $\dot{\theta_{2}}$, we can get the following form (specific values please refer to the matlab code appendix) \[
\begin{bmatrix}
\dot{x} \\ 
\dot{y}
\end{bmatrix}  
=
J(\theta_{1}, \theta_{2})
\begin{bmatrix}
\dot{\theta_{1}} \\
\dot{\theta_{2}} \\
\end{bmatrix}  
\]
where $J$ represents the Jacobian matrix of the two linkages robotic arm system that maps angular velocities to the end point transnational velocities when being multiplied. Thus, using the Jacobian matrix, we can convert the desired velocities represented in Cartesian coordinate into desired angular velocities.

Multiplying the above equation by the inverse of the Jacobian, we can get 
\[
\begin{bmatrix}
\dot{\theta_{1}} \\ 
\dot{\theta_{2}}
\end{bmatrix}  
=
J^{-1}(\theta_{1}, \theta_{2})
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\end{bmatrix}  
\]
To get the desired angular accelerations, we take derivatives of both sides of the above equation, and combine all parts of the right hand side into one equation $FR$, resulting in 

\[
\begin{bmatrix}
\ddot{\theta_{1}} \\ 
\ddot{\theta_{2}}
\end{bmatrix}  
=
FR(\theta_{1}, \theta_{2}, \dot{\theta_{1}}, \dot{\theta_{2}}, \dot{x}, \dot{y}, \ddot{x}, \ddot{y})  
\]
The derivation above is carried out in MATLAB, and can be found in our script \\ \codeblock{jacobian\_derivative\_simplifying.m}.

We can then get the desired angular acceleration by inputting the parameters of function $FR$ (please refer to MATLAB appendix for detailed FR function). 

To generate the desired angle positions, Newton-Raphson numerical inverse kinematics solver is utilized. This solver takes the previous angle position values and the desired end point positions as inputs, and computes the desired angular positions \cite{Modern_Robotics} (please refer to modern robotics for detailed explanation). Refer to Equation \ref{NR_1}, $\theta_{1previous}$ and $\theta_{2previous}$ are the previous angular position, which are being iteratively updated by adding $\begin{bmatrix}
\Delta \theta_{1} \\
\Delta \theta_{2} \\
\end{bmatrix}$ until the difference between $\begin{bmatrix}
x_{desired} \\
y_{desired} \\
\end{bmatrix}$ and $l(\theta_{1previous}, \theta_{2previous})$ is small enough.
\begin{equation}\label{NR_1}
\begin{bmatrix}
\Delta \theta_{1} \\
\Delta \theta_{2} \\
\end{bmatrix} = (J(\theta_{1previous}, \theta_{2previous}))^{-1}(\begin{bmatrix}
x_{desired} \\
y_{desired} \\
\end{bmatrix}-l(\theta_{1previous}, \theta_{2previous}))
\end{equation}

After the desired angular position, desired angular velocity, and desired acceleration are calculated, they are used as input to calculate desired input with equations of motion.
The following diagram summarised the conversion process for generated trajectory from Cartesian coordinate to angular space.

\begin{figure}[H] 
\begin{center}
\minipage{0.9\textwidth}
    \includegraphics[width=\textwidth]{misc/desired_path_convertion.png}
    \caption{Flow Chart for Desired Path Conversion}
    \label{fig:path_conversion}
\endminipage\hfill
\end{center}
\end{figure}

\subsection{Linearized Error Dynamics}

The error in our system is between the measured state and the desired state. Ideally, the error between the desired state and the measured state goes to 0 as time goes to infinity \cite{Optimization_Based_Control}. The error in the 2 DOF robot arm system is 

\begin{equation}
    e = x - x_{d}
    \label{eq:error}
\end{equation}

The term $v$, the control input, is defined 

\begin{equation}
    v = u - u_{d}
    \label{eq:control input}
\end{equation}

The time derivative of the error can be written as 

\begin{equation}
    \dot{e} = \dot{x} - \dot{x}_{d} = f(x,u) - f(x_{d}, u_{d}) 
    \label{eq:error rate}
\end{equation}

Equation \ref{eq:error rate} can be rewritten by substituting Equations \ref{eq:error} and \ref{eq:control input}

\begin{equation}
    \dot{e} = f(e+x_{d},v+u_{d}) - f(x_{d},u_{d}) =  F(e,v,x_{d}(t),u_{d}(t))   
    \label{eq:Full Error Dynamics}
\end{equation}

Using Equation \ref{eq:Full Error Dynamics} and assuming e is small, the error can be linearized around $e=0$ into the form of 

\begin{equation}
    \dot{e} = Ae + Bv \hspace{5mm} A \in \mathbb{R}^{4 \times 4} \hspace{5mm} B \in \mathbb{R}^{4 \times 2}
\end{equation}

The matrix $A$ can be calculated by taking the derivative of Equation \ref{eq:Full Error Dynamics} with respect to the state vector in Equation \ref{eq:State}, while the matrix $B$ can be calculated by taking the derivative of Equation \ref{eq:Full Error Dynamics} with respect to the input torque vector Equation \ref{eq:Torque Input}. Therefore

\begin{equation}
    A = 
    \begin{bmatrix}
        \frac{dF}{d\theta_{1}} & \frac{dF}{d\theta_{2}} & \frac{dF}{d\dot{\theta}_{1}} & \frac{dF}{d\dot{\theta}_{2}}
    \end{bmatrix}
\end{equation}

\begin{equation}
    B = 
    \begin{bmatrix}
        \frac{dF}{d\tau_{1}} & \frac{dF}{d\tau_{2}} 
    \end{bmatrix}
\end{equation}

The linearized error dynamics was calculated in MATLAB, and can be found in our script \codeblock{linearize\_error\_dynamics.m}.


\subsection{Controller Selection}

The mission of the controller design is to design controllers that could regulate the error dynamics calculate above to reach zero error given enough time while satisfying performance requirements.
Two types of control strategies were utilized to correct for the difference in the measured output state and the desired state. These two strategies are pole placement and linear-quadratic regulator (LQR), both of which are full-state control.

For pole placement, different situations were looked at. The controllability matrix of the system has rank 4, meaning that this system is controllable. These situations are having 4 stable poles which are:

\[
\begin{bmatrix}
-1 & -0.5 & -0.1 & -2 \\
\end{bmatrix}
\]

3 stable and 1 marginally stable poles which are:
\[
\begin{bmatrix}
-1 & -0.5 & -0.1 & 0 \\
\end{bmatrix}
\]

and 3 stable and 1 unstable pole which are:
\[
\begin{bmatrix}
-1 & -0.5 & -0.1 & 1 \\
\end{bmatrix}
\]

For LQR, the same $Q$ and $R$ were chosen. However, more weight was given to $\theta_{1}$ and $\theta_{2}$. 

\[
Q =
\begin{bmatrix}
100 & 0 & 0 & 0 \\ 
0 & 100 & 0 & 0 \\
0 & 0 & 10 & 0 \\
0 & 0 & 0 & 10 \\
\end{bmatrix} 
\]

\[
R =
\begin{bmatrix}
1 & 0 \\ 
0 & 1
\end{bmatrix} 
\]

As mentioned in controller scheme section, a PI controller is implemented to alleviate the drawbacks of the old controller scheme. The error input for the PI controller is calculated by subtracting the current end point position $\begin{bmatrix} x_{current} \\ y_{current} \end{bmatrix}$ from the desired end point position $\begin{bmatrix} x_{desired} \\ y_{desired} \end{bmatrix}$. The controller output $u_{pi}$ is calculated following equation \ref{eq:pi controller} 

\begin{equation}
    u_{pi}=J^{-1}(\theta_{1}, \theta_{2})(K_{p}\cdot e+K_{i}\cdot\int_{0}^{t}e\cdot dt)
    \label{eq:pi controller}
\end{equation}

All controllers mentioned above are implemented in MATLAB, and can be found in our script \codeblock{controller\_design\_for\_error.m}

\newpage
\section{Result and Evaluation}

Different configurations of the controllers were simulated:
\begin{itemize}
    \item{LQR Control with No System Error}
    \item{LQR Control with 10\% System Error}
    \item{LQR Control with 10\% System Error and End Effector PID Control}
    \item{Pole Placement with Stable Poles (No System Uncertainty)}
    \item{Pole Placement with One Marginally Stable Pole (No System Uncertainty)}
\end{itemize}
\noindent NOTE: All control configurations have a noise input at the inlet and outlet of the system.

For each case, the error of each state variable with respect to its corresponding desired state variable is plotted, followed by the link angles with respect to time and the desired and actual trajectory being plotted next. Finally, for each scenario, the control input for controller (motor) 1 and 2, each placed at the base revolute joint and second revolute joint respectively, are plotted. 





\subsection{LQR Control - No System Error}

The first case is LQR control with no system uncertainty.

\begin{figure}[H] 
\begin{center}
    \includegraphics[scale = 0.7]{Plots/LQR_No_Systematic_Error/LQR Control No systematic error Error plot.jpg}
    \caption{Error Plot of LQR Control with No Systematic Error}
    \label{fig:LQR Control No Systematic error Error Plot}
\end{center}
\end{figure}

The error plot for this scenario shown in Figure \ref{fig:LQR Control No Systematic error Error Plot} depicts noisy error averaging around 0. The large spikes are for extremely small amounts of time. The overall error is negligible.

\begin{figure}[H] 
\begin{center}
    \includegraphics[width=\textwidth]{Plots/LQR_No_Systematic_Error/LQR Control No systematic error State Plot.png}
    \caption{State Plot of LQR Control with No Systematic Error}
    \label{fig:LQR Control No Systematic error State Plot}
\end{center}
\end{figure}

The state is closely tracked, with the actual joint angles perfectly overlapping the desired joint angles shown on the left graph in Figure \ref{fig:LQR Control No Systematic error Error Plot}. The graph on the right shows near perfect tracking of the desired end effector position. 

\begin{figure}[H] 
\begin{center}
    \includegraphics[width=\textwidth]{Plots/LQR_No_Systematic_Error/LQR Control No systematic error Controller Input.png}
    \caption{Controller Input of LQR Control with No Systematic Error}
    \label{fig:LQR Control No Systematic error Controller Input}
\end{center}
\end{figure}

The controller input for each motor is shown in Figure \ref{fig:LQR Control No Systematic error Controller Input}. The input is split up into the total input, $u$, as well as the feedforward component of the input, $u_{ff}$, and the feedback component, $u_{fb}$. One can see that the feedback portion of the controller input is essentially not adding much to the total input. The noise present in the graphs is from the noise that has been added to the controller input. This indicates that in a perfect system where everything about the system is known and there are no external hindrances, the system can be perfectly controlled by feedforward control alone, as expected.






\subsection{LQR Control - 10\% System Error}

This control scheme is LQR control with 10\% systematic uncertainty added to the masses and lengths of each link. 

\begin{figure}[H] 
\begin{center}
    \includegraphics[scale = 0.8]{Plots/LQR_Control_Systematic_Error/LQR Control Systematic error Error Plot.png}
    \caption{Error Plot of LQR Control with 10 Percent Systematic Error}
    \label{fig:LQR Control 10 Percent Systematic error Error Plot}
\end{center}
\end{figure}

The error plot for this scenario shown in Figure \ref{fig:LQR Control 10 Percent Systematic error Error Plot} depicts noticeable error for both angular positions and velocities. The large spikes are for extremely small amounts of time. The overall error is negligible. The maximum magnitude of the error for both angular positions and angular velocities is approximately |0.3| in the respective units (rad or rad/s respectively).

\begin{figure}[H] 
\begin{center}
    \includegraphics[width=\textwidth]{Plots/LQR_Control_Systematic_Error/LQR Control Systematic error State Plot.png}
    \caption{State Plot of LQR Control with 10 Percent Systematic Error}
    \label{fig:LQR Control 10 Percent Systematic error State Plot}
\end{center}
\end{figure}

The state plot over time is shown in Figure \ref{fig:LQR Control 10 Percent Systematic error State Plot}. The tracking of angular positions deviates over time, however it never goes fully unstable. The end effector position resembles a triangular shape, however it is not close to the desired result. This controller may be suitable for a hobbyist who cannot get accurate estimations of their system parameters, and needs something that is close to working.

\begin{figure}[H] %H means here
\begin{center}
   % \includegraphics[width=\textwidth]{Results Plots/LQR Control 10 Percent Systematic error/LQR Control 10 Percent Systematic error Controller Input.png}
    \includegraphics[width=\textwidth]{Plots/LQR_Control_Systematic_Error/LQR Control Systematic error Controller Input.png}
    \caption{Controller Input of LQR Control with 10 Percent Systematic Error}
    \label{fig:LQR Control 10 Percent Systematic error Controller Input}
\end{center}
\end{figure}

The controller input for each motor is shown in Figure \ref{fig:LQR Control 10 Percent Systematic error Controller Input}. The feedback control is being initiated in this scenario, and it helping to counteract the system error. Sharp increases or decreases in controller input indicate a transition in the direction of the robot arm to another leg of the triangle. One can see that the feedback controller amplifies the controller input in both the negative and positive direction. This is to make up for the increase in length and mass of the system links.




\subsection{LQR Control - 10\% System Error - PI Control}

This control scheme is LQR control with systematic uncertainty as well as a PI controller to regulate the location of the end effector. 

\begin{figure}[H] 
\begin{center}
    \includegraphics[scale = 0.22]{Plots/LQR_Control_Systematic_Error_PID_Control/LQR With System Error and Inverse Velocity Control PID Error Plot.png}
    \caption{Error Plot of LQR Control with Systematic Uncertainty and End Effector PI Control}
    \label{fig:Error Plot LQR Control Systematic Uncertainty End Effector PI Control}
\end{center}
\end{figure}

The error plot for this scenario shown in Figure \ref{fig:Error Plot LQR Control Systematic Uncertainty End Effector PI Control} depicts some error for both angular positions and velocities. The large angular velocity error in the beginning is due to the robot arms starting at different positions (due to geometry of starting at the same angles with different link length). The maximum magnitude of the error for both angular positions and angular velocities is approximately |0.15| ignoring large noise contributions and in the respective units (rad or rad/s respectively) and are for extremely small amounts of time.

\begin{figure}[H] 
\begin{center}
    \includegraphics[scale=0.22]{Plots/LQR_Control_Systematic_Error_PID_Control/LQR With System Error and Inverse Velocity Control PID State Plot.png}
    \includegraphics[scale=0.22]{Plots/LQR_Control_Systematic_Error_PID_Control/LQR With System Error and Inverse Velocity Control PID Final State.png}
    \caption{State Plot of LQR Control with Systematic Uncertainty and End Effector PI Control}
    \label{fig:State Plot LQR Control Systematic Uncertainty End Effector PI Control}
\end{center}
\end{figure}

The state plot over time is shown in Figure \ref{fig:State Plot LQR Control Systematic Uncertainty End Effector PI Control}. The tracking of angular positions deviates slightly over time, however it never goes fully unstable. The largest error in angular position is at the beginning, however, the end effector position tracks the desired triangular path relatively closely for most of the time. The largest error occurs on the bottom, horizontal leg of the triangle. This is because this is where the largest torque must be input into the system. 

\begin{figure}[H] 
\begin{center}
    \includegraphics[scale=0.22]{Plots/LQR_Control_Systematic_Error_PID_Control/LQR With System Error and Inverse Velocity Control PID Control Input 1.png}
    \includegraphics[scale=0.22]{Plots/LQR_Control_Systematic_Error_PID_Control/LQR With System Error and Inverse Velocity Control PID Control Input 2.png}
    \caption{Controller Input of LQR Control with Systematic Uncertainty and End Effector PI Control}
    \label{fig:Controller Input LQR Control Systematic Uncertainty End Effector PI Control}
\end{center}
\end{figure}

The controller input for each motor is shown in Figure \ref{fig:Controller Input LQR Control Systematic Uncertainty End Effector PI Control}. The feedback control is being utilized in this scenario, and it is helping to counteract the system error, as well as the end effector error. One can see the large spike in feedback control input at the beginning of the plots that help make up for the initial offset. 




\subsection{Pole Placement - Stable Poles - No System Error}

This control scheme utilizes pole placement with all stable poles selected as the poles of the system. The poles selected are : 
\[
\begin{bmatrix}
-1 & -0.5 & -0.1 & -2 \\
\end{bmatrix}
\]

\begin{figure}[H] 
\begin{center}
    \includegraphics[scale = 0.5]{Plots/Pole_Placement_Stable_Poles_No_Systematic_Uncertainty/Pole Placement Stable Poles No systematic Uncertainty Error Plot.png}
    \caption{Error Plot of Pole Placement with Stable Poles and No systematic Uncertainty}
    \label{fig:Error Plot Pole Placement Stable Poles No systematic Uncertainty}
\end{center}
\end{figure}

The error plot for this scenario shown in Figure \ref{fig:Error Plot Pole Placement Stable Poles No systematic Uncertainty} depicts noisy error averaging around 0. The large spikes are for extremely small amounts of time, and are relatively small. The overall error is negligible.


\begin{figure}[H] 
\begin{center}
    \includegraphics[scale=0.6]{Plots/Pole_Placement_Stable_Poles_No_Systematic_Uncertainty/Pole Placement Stable Poles No systematic Uncertainty State Plot.png}
    \includegraphics[scale=0.7]{Plots/Pole_Placement_Stable_Poles_No_Systematic_Uncertainty/Pole Placement Stable Poles No systematic Uncertainty Final State.PNG}
    \caption{State Plot of Pole Placement with Stable Poles and No systematic Uncertainty}
    \label{fig:State Plot Pole Placement Stable Poles No systematic Uncertainty}
\end{center}
\end{figure}

The state is closely tracked, with the actual joint angles perfectly overlapping the desired joint angles shown on the left graph in Figure \ref{fig:State Plot Pole Placement Stable Poles No systematic Uncertainty}. The graph on the right shows near perfect tracking of the desired end effector position, as expected with a pole placement controller with stable poles. 

\begin{figure}[H] 
\begin{center}
    \includegraphics[width=\textwidth]{Plots/Pole_Placement_Stable_Poles_No_Systematic_Uncertainty/Pole Placement Stable Poles No systematic Uncertainty Controller Input.png}
    \caption{Controller Input of Pole Placement with Stable Poles and No systematic Uncertainty}
    \label{fig:Controller Input Pole Placement Stable Poles No systematic Uncertainty}
\end{center}
\end{figure}

The controller input for each motor is shown in Figure \ref{fig:Controller Input Pole Placement Stable Poles No systematic Uncertainty}. One can see that the feedback portion of the controller input is essentially not adding much to the total input. This system could be controlled solely by feedforward control alone.


\subsection{Pole Placement - Marginally Stable Pole - No System Error}

This control scheme utilizes pole placement with all stable poles selected as the poles of the system, except for one marginally stable pole located at 0. The poles selected are : 
\[
\begin{bmatrix}
-1 & -0.5 & -0.1 & 0 \\
\end{bmatrix}
\]

\begin{figure}[H] 
\begin{center}
    \includegraphics[scale = 0.7]{Plots/Pole_Placement_Marginally_Stable_Poles_No_Systematic_Uncertainty/Pole Placement Marginally Stable Poles No systematic Uncertainty Error Plot.jpg}
    \caption{Error Plot of Pole Placement with Marginally Stable Poles and No systematic Uncertainty}
    \label{fig:Error Plot Pole Placement Marginally Stable Poles No systematic Uncertainty}
\end{center}
\end{figure}

The error plot for this scenario shown in Figure \ref{fig:Error Plot Pole Placement Marginally Stable Poles No systematic Uncertainty} depicts some error for both angular positions and velocities. The maximum magnitude of the error for both angular positions and angular velocities is approximately |0.05| ignoring large noise contributions.


\begin{figure}[H] 
\begin{center}
    \includegraphics[width=\textwidth]{Plots/Pole_Placement_Marginally_Stable_Poles_No_Systematic_Uncertainty/Pole Placement Marginally Stable Poles No systematic Uncertainty State Plots.png}
    \caption{State Plot of Pole Placement with Marginally Stable Poles and No systematic Uncertainty}
    \label{fig:State Plot Pole Placement Marginally Stable Poles No systematic Uncertainty}
\end{center}
\end{figure}

The state plot over time is shown in Figure \ref{fig:State Plot Pole Placement Marginally Stable Poles No systematic Uncertainty}. The tracking of angular positions deviates very slightly over time, however it remains relatively close tot he desired path.  The largest error occurs on the last leg of the system, which may be fro an accumulation of error throughout the entire simulation.


\begin{figure}[H] 
\begin{center}
    \includegraphics[width=\textwidth]{Plots/Pole_Placement_Marginally_Stable_Poles_No_Systematic_Uncertainty/Pole Placement Marginally Stable Poles No systematic Uncertainty Controller Input.png}
    \caption{Controller Input of Pole Placement with Marginally Stable Poles and No systematic Uncertainty}
    \label{fig:Controller Input Pole Placement Marginally Stable Poles No systematic Uncertainty}
\end{center}
\end{figure}

The controller input for each motor is shown in Figure \ref{fig:Controller Input Pole Placement Marginally Stable Poles No systematic Uncertainty}. The feedback control is being utilized slightly in this scenario towards the end of the simulation, and it is helping to counteract the system error, as well as the end effector error.\\ \\ \\ \\ \\



\textbf{NOTE:} Pole placement with unstable poles, as well as any pole placement controller with system uncertainty resulted in an unstable system. This indicates the the pole placements controller does not result in an inherently robust system when systematic uncertainty is included. 






\newpage
\section{Conclusions}

Based on the results from the simulation, it can be concluded that LQR control is relatively robust when system error is included. Additionally, LQR is very robust with input and output sensor noise. Pole placement control behaves as expected based on the desired poles to be placed. Marginal Stability tracks the desired path closely with small error. Pole placement is not robust to systematic error and Seems to be relatively stable with noise present.

LQR control produces gain margins that are infinitely large, and phase margins that are at least 60\textdegree \cite{LQR_Margins}.



\section{Practical Implementation}

In order to implement this system as a physical robot, materials and motors must be sourced. The proposed motor for this project is the Robotis DYNAMIXEL MX-106T, a fully integrated DC motor, driver, and controller unit with built in rotary encoder. Aluminum extrusions may be used for the robot links for their versatility, stiffness, and light weight. 

\begin{figure}[H] 
\begin{center}
    \includegraphics[scale=0.5]{misc/motor.png}
    \includegraphics[scale=0.2]{misc/aluminum extrusion.png}
    \caption{Robotis DYNAMIXEL MX-106T and 80/20 aluminum extrusions to be used for the practical implementation of this system}
    \label{fig:Error Plot LQR Control Systematic Uncertainty End Effector PID Control}
\end{center}
\end{figure}


\noindent A 0.5 lb motor, the limitations of this product include:
\begin{itemize}
    \item Baud Rate : 8,000 – 4,500,000 bits per second
    \item Resolution of Encoder : 4096 pulse/rev
    \item Stall Torque = 8.40 Nm
\end{itemize}

\noindent Using the lower bound of the baud rate as the available frequency of the motor:

Available Bandwidth = 8,000 Hz = 50,000 rad/s

\noindent One other limitation of this motor is is angular resolution at ~0.0879 degrees per pulse. This limits the accuracy achievable by the robot, and makes certain calculations done during the simulation obsolete since the resolution is simulated to be far finer. 

The stall torque  of the motor is 8.40 Nm. This limits the masses and lengths of each link that can be used. In this simulation, the maximum input is set at 50 Nm. In order to achieve similar results in a practical implementation, the masses and lengths of the motors need to be scaled down by approximately a factor of 6 to achieve similar results to the simulation. Finally, the motors being 0.5 lbs adds to the mass of the system, meaning higher torque outputting motors may be necessary for the proper functioning of the system. 

Regarding the implementation of the PI controller, the limiting factor will be the camera system's sampling rate. Given that our arm is 2D, a single camera is enough to track the end point's position. A common webcam has a maximum frame rate of 60 fps. This will limit our system's processing speed to 60 Hz. This bandwidth issue could be alleviated if more advanced camera is used.


\section{Future Work}

Future work with this project should include incorporating an observer to take steps towards a more realistic system. Different paths can also be generated, such as different concatenations of linear paths (square, zig zag, etc.), as well as paths with curvature such as a circle. Additionally, a filter could be added in an attempt to reduce the effect from the system noise. Furthermore, the angular resolution of the motor can be taken into account in this simulation for a more accurate representation of the practical implementation. Long term goals of the project include a physical implementation of the system using aluminum extrusions and torque control motors. 

\newpage
\bibliographystyle{unsrt}
\bibliography{citations}

\newpage
\section{MATLAB Code}
\url{https://github.com/eneadushaj/ME-451-Modern-Control-Final-Project.git}


\end{document}
