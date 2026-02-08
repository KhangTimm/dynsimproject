
# **Advanced Rotordynamic Simulation and Stability Analysis of a Twin-Spool Turbofan Engine: A Theoretical and Practical Synthesis**

## **1. Introduction and Motivation**

The propulsion systems of modern aviation, specifically high-bypass turbofan engines, represent some of the most complex mechanical assemblies in contemporary engineering. These systems operate under extreme conditions of thermal stress, aerodynamic loading, and, most critically, high rotational velocities. The structural integrity and operational safety of a jet engine are fundamentally governed by its dynamic behavior. Failure to accurately predict and manage these dynamics can lead to catastrophic failure modes, ranging from high-cycle fatigue of compressor blades to the disintegration of the rotor assembly due to unbounded resonance. Consequently, the study of rotordynamics—the branch of mechanics concerned with the behavior of rotating structures—is not merely an academic exercise but a stringent requirement for airworthiness certification.

This report documents the development, analysis, and tuning of a simplified yet physically representative numerical model of a twin-spool jet engine. The primary objective is to translate theoretical concepts from analytical dynamics—specifically the Principle of Virtual Work and Lagrange’s equations—into a practical simulation capable of predicting critical speeds and forced response amplitudes. The motivation for this specific analysis stems from the inherent coupling between the rotational and translational degrees of freedom in high-speed rotors, a phenomenon known as the gyroscopic effect. Unlike stationary structures where mass and stiffness matrices are constant, rotating systems exhibit speed-dependent characteristics. As the rotational velocity increases, gyroscopic moments act to stiffen or soften specific vibration modes, causing the natural frequencies of the system to bifurcate. This dependency renders static analysis insufficient; a full dynamic simulation using Campbell diagrams and harmonic response analysis is required to ensure the engine operates free of resonance within its flight envelope.

The selected model architecture is a dual-spool configuration, characteristic of engines such as the CFM56 or GE90. This system comprises a Low-Pressure (LP) rotor (fan and low-pressure turbine) and a High-Pressure (HP) rotor (compressor and high-pressure turbine), mechanically coupled through the engine casing and inter-shaft bearings. This architecture introduces significant complexity, satisfying the project constraints of involving at least three rigid bodies (LP Rotor, HP Rotor, Casing) and multiple degrees of freedom. By creating this model, we aim to determine the realistic operating speeds ($N1$ and $N2$), identify critical resonances, and tune system parameters such as bearing stiffness and damping coefficients to mitigate vibration amplitudes. This report serves not only as a technical document of the simulation results but also as a critical evaluation of the theoretical methods employed, contrasting the classical Lagrangian approach taught in lecture with the practical necessities of rotordynamic design.

## **2. Theoretical Framework and Methodological Basis**

The derivation of the equations of motion for the jet engine model is grounded in the principles of Analytical Dynamics, as outlined in the course "Dynamic Simulation for Vehicles, Machines, and Mechanisms" (LRG0112). Unlike vectorial mechanics, which relies on the isolation of free bodies and the summation of forces, analytical mechanics leverages scalar energy quantities—kinetic energy ($\mathcal{T}$) and potential energy ($\mathcal{V}$)—to derive the system's dynamics. This approach is particularly advantageous for complex multibody systems with constraints, as it naturally eliminates reaction forces that do not perform work.

### **2.1 The Principle of Virtual Work and D'Alembert’s Principle**

The foundation of our modeling approach is the Principle of Virtual Work (PVW). In a constrained system of $N$ particles, the motion is restricted by kinematic constraints (e.g., bearings holding a shaft). If we consider a virtual displacement $\delta \mathbf{r}_k$—a hypothetical, infinitesimal displacement that occurs consistent with the constraints at a fixed instant in time—the work done by the constraint forces is zero.
D'Alembert's principle extends this to dynamics by treating inertia terms as forces. The dynamic equilibrium can be stated as:

$$
\sum_{k=1}^{N} \left( m_k \mathbf{a}_k - \mathbf{F}_k^{ext} \right) \cdot \delta \mathbf{r}_k = 0
$$

Here, $m_k \mathbf{a}_k$ represents the inertia force vector, and $\mathbf{F}_k^{ext}$ represents the applied external forces. This equation states that the total virtual work of the inertia forces and the applied forces vanishes for any kinematically admissible displacement. This principle allows us to bypass the calculation of internal bearing reaction forces, focusing solely on the motion of the rotors relative to the inertial frame.

### **2.2 Lagrange’s Equations of the Second Kind**

While the PVW provides the condition for equilibrium, applying it directly to a system with complex constraints can be algebraically tedious. Therefore, we utilize Lagrange’s equations, which are derived directly from the PVW by expressing displacements in terms of a set of independent generalized coordinates, $\mathbf{q} = [q_1, q_2, \dots, q_n]^T$.

The kinetic energy of the system is defined as $\mathcal{T}(\mathbf{q}, \dot{\mathbf{q}}, t)$, and the potential energy as $\mathcal{V}(\mathbf{q})$. For a system with non-conservative forces defined by generalized forces $Q_j^{nc}$, the equations of motion are given by:

$$
\frac{d}{dt} \left( \frac{\partial \mathcal{T}}{\partial \dot{q}_j} \right) - \frac{\partial \mathcal{T}}{\partial q_j} + \frac{\partial \mathcal{V}}{\partial q_j} = Q_j^{nc}, \quad j = 1, \dots, n
$$

This formulation is robust and systematic. It requires only the scalar expressions of energy, which are often easier to formulate than vector forces in rotating frames. For our jet engine model, $Q_j^{nc}$ will include the effects of viscous damping in the bearings and any external aerodynamic excitations.

### **2.3 Rigid Body Kinematics and Gyroscopic Coupling**

The jet engine rotors are modeled as rigid bodies. According to the lecture material, the motion of a rigid body is described by the translation of its center of mass and the rotation about that center. The rotational kinetic energy is the source of the gyroscopic terms that are central to this project.
The orientation of a rotor disk is described by a rotation matrix $\mathbf{A}$ that transforms coordinates from the body-fixed frame ($B$) to the inertial frame ($I$). For rotordynamics, we typically use a set of angles describing the spin ($\phi$) and the tilt angles ($\vartheta, \psi$) about transverse axes. The angular velocity vector $\boldsymbol{\omega}$, expressed in the body frame, determines the rotational kinetic energy:

$$
\mathcal{T}_{rot} = \frac{1}{2} \boldsymbol{\omega}^T \mathbf{I}_B \boldsymbol{\omega}
$$

Where $\mathbf{I}_B$ is the inertia tensor. Crucially, because the rotor is spinning at a high velocity $\Omega = \dot{\phi}$, any tilting velocity ($\dot{\vartheta}$ or $\dot{\psi}$) changes the direction of the angular momentum vector. This change manifests in the Lagrange derivation as a "gyroscopic moment."
In the resulting linearized equations of motion, this appears as a skew-symmetric matrix $\mathbf{G}$ that couples the transverse coordinate directions (e.g., $x$ and $y$). The term is proportional to the polar moment of inertia $I_p$ and the spin speed $\Omega$:

$$
\mathbf{F}_{gyro} \propto \Omega I_p \begin{bmatrix} 0 & 1 \\ -1 & 0 \end{bmatrix} \begin{bmatrix} \dot{x} \\ \dot{y} \end{bmatrix}
$$

This coupling is what differentiates a rotating machine from a static beam. It causes the natural frequencies to split into forward and backward whirling modes, a phenomenon that we will analyze using Campbell diagrams.

## **3. Mechanical System Modeling**

To satisfy the project requirements of modeling a system with "at least three rigid bodies," we have conceptualized a simplified **Twin-Spool Turbofan Engine**. This model captures the essential dynamics of a commercial jet engine while remaining mathematically tractable for the scope of this simulation project.

### **3.1 System Topology and Rigid Bodies**

The system consists of three primary rigid bodies connected by elastic and damping elements (constraints):

1.  **Body 1: The Low-Pressure (LP) Rotor.** This body represents the N1 spool. It includes the large Fan disk at the front and the Low-Pressure Turbine (LPT) disk at the rear, connected by a long, slender inner shaft. In our lumped parameter model, we treat the entire LP assembly as a single rigid body with mass properties dominated by the large Fan.
2.  **Body 2: The High-Pressure (HP) Rotor.** This body represents the N2 spool. It includes the High-Pressure Compressor (HPC) and High-Pressure Turbine (HPT). The HP shaft is hollow and concentric with the LP shaft, rotating independently at a higher speed.
3.  **Body 3: The Engine Casing (Stator).** The casing provides the structural foundation. It supports the outer races of the main bearings. While often modeled as a fixed ground, treating it as a rigid body with its own mass allows us to investigate rotor-stator coupling and mount vibrations, fulfilling the "three rigid bodies" constraint.

**Scope of Simulation:**
While the topology includes three bodies, simulating the fully coupled 18-DOF system is beyond the scope of a harmonic analysis project. Therefore, we perform a **Decoupled Analysis** of the LP Rotor (Body 1). The HP Rotor (Body 2) and Casing (Body 3) are treated as boundary conditions. Their influence is captured via the effective stiffness and damping parameters of the bearings connecting them to the LP Rotor.

### **3.2 Constraints and Force Elements**

The bodies are interconnected by force elements that represent the bearings and structural compliance. In our mathematical model, we aggregate these into effective supports acting on the LP Rotor:

* **Front LP Support ($k_1, c_1$):** Represents **Bearing 1**. Connects the LP Fan shaft to the Engine Casing (Ground). Modeled as a linear spring and viscous damper at axial distance $l_1$ from the center of gravity.
* **Rear LP Support ($k_2, c_2$):** Represents **Bearing 5**. Connects the LP Turbine shaft to the Engine Casing (Ground). Modeled at axial distance $l_2$.
* **Inter-shaft Connection ($k_i, c_i$):** Represents **Bearing 3**. Connects the LP rotor to the HP rotor. Since the HP rotor is treated as a fixed boundary in this decoupled analysis, this element acts as an additional stiffness/damping contribution in parallel with the ground supports.

*Note: Bearings 2 and 4 support the HP rotor and are implicitly included in the boundary condition assumption.*

### **3.3 Origin at Center of Mass**

The origin of the coordinate system is set at the center of mass

### **3.4 Degrees of Freedom (DOFs)**

To capture the tilting dynamics and gyroscopic effects required by the prompt, each rotor must have at least rotational DOFs. However, to observe the full lateral rotordynamic behavior (whirling), we assign the following DOFs to the LP Rotor's Center of Gravity (CoG):

* Lateral translation in X ($x$)
* Lateral translation in Y ($y$)
* Rotation (pitch) about X ($\alpha$)
* Rotation (yaw) about Y ($\beta$)

This results in a 4-DOF system for the simulation, derived from the minimal coordinates of the LP rigid body.



---

## **4. Derivation of Equations of Motion**

We apply the Lagrangian method to derive the Equations of Motion (EOM) for the LP rotor.

### **4.1 Kinetic Energy of a Rotor**

Consider the LP rotor as a rigid body with mass $m$ and principal moments of inertia $I_d$ (diametral) and $I_p$ (polar). The rotor spins at a constant speed $\Omega$. The generalized coordinates are $\mathbf{q} = [x, y, \alpha, \beta]^T$. Following the standard Right-Hand Rule convention, $x, y$ are lateral displacements, $\alpha$ is the rotation around the X-axis (pitch), and $\beta$ is the rotation around the Y-axis (yaw).

The translational kinetic energy is simply:

$$
\mathcal{T}_{trans} = \frac{1}{2} m (\dot{x}^2 + \dot{y}^2)
$$

The rotational kinetic energy requires the angular velocity vector. The exact expression for kinetic energy including the spin $\Omega$ is:

$$
\mathcal{T}_{rot} = \frac{1}{2} I_d (\dot{\alpha}^2 + \dot{\beta}^2) + \frac{1}{2} I_p (\Omega^2 - 2\Omega \alpha \dot{\beta})
$$

*Note: The term $-2\Omega \alpha \dot{\beta}$ is the linearized approximation of the gyroscopic coupling energy. In the Lagrangian derivation, this term generates the skew-symmetric gyroscopic forces characteristic of rotating systems.*

**Total Kinetic Energy:**

$$
\mathcal{T} = \frac{1}{2} m (\dot{x}^2 + \dot{y}^2) + \frac{1}{2} I_d (\dot{\alpha}^2 + \dot{\beta}^2) + \frac{1}{2} I_p \Omega (\Omega - 2 \alpha \dot{\beta})
$$

### **4.2 Potential Energy and Rayleigh Dissipation**

The potential energy $\mathcal{V}$ is stored in the elastic bending of the shaft and the bearings. We assume an equivalent stiffness matrix $\mathbf{K}$ that couples translations and rotations (since a force at a bearing offset from the CoG yields a tilt).

$$
\mathcal{V} = \frac{1}{2} \mathbf{q}^T \mathbf{K} \mathbf{q}
$$

Dissipation is modeled using a Rayleigh dissipation function $\mathcal{F}$, which introduces the damping matrix $\mathbf{C}$:

$$
\mathcal{F} = \frac{1}{2} \dot{\mathbf{q}}^T \mathbf{C} \dot{\mathbf{q}}
$$

### **4.3 Application of Lagrange’s Equation**

We apply Lagrange's equations to the generalized coordinates $\mathbf{q} = [x, y, \alpha, \beta]^T$.
Recall the general formula:

$$
\frac{d}{dt} \left( \frac{\partial \mathcal{T}}{\partial \dot{q}_j} \right) - \frac{\partial \mathcal{T}}{\partial q_j} + \frac{\partial \mathcal{V}}{\partial q_j} + \frac{\partial \mathcal{F}}{\partial \dot{q}_j} = Q_j^{nc}
$$

**1. For Translation $x$ (Horizontal Lateral):**
* **Coupling:** Horizontal translation $x$ couples with the rotation about the vertical axis, $\beta$ (Yaw).

$$
\frac{d}{dt}(m \dot{x}) - 0 + \underbrace{\frac{\partial \mathcal{V}}{\partial x}}_{k_{xx} x + k_{x\beta} \beta} + c \dot{x} = F_x
$$

$$
\implies m \ddot{x} + c \dot{x} + k_{xx} x + k_{x\beta} \beta = F_x
$$

**2. For Translation $y$ (Vertical Lateral):**
* **Coupling:** Vertical translation $y$ couples with the rotation about the horizontal axis, $\alpha$ (Pitch).

$$
\frac{d}{dt}(m \dot{y}) - 0 + \underbrace{\frac{\partial \mathcal{V}}{\partial y}}_{k_{yy} y + k_{y\alpha} \alpha} + c \dot{y} = F_y
$$

$$
\implies m \ddot{y} + c \dot{y} + k_{yy} y + k_{y\alpha} \alpha = F_y
$$

**3. For Rotation $\alpha$ (Pitch about X-axis):**
* **Gyroscopic Effect:** The kinetic energy term relevant to gyroscopics is approximated as $\mathcal{T}_{gyro} \approx -I_p \Omega \alpha \dot{\beta}$.
* **Derivatives:**
    $$\frac{\partial \mathcal{T}}{\partial \dot{\alpha}} = I_d \dot{\alpha} \implies \frac{d}{dt}(I_d \dot{\alpha}) = I_d \ddot{\alpha}$$
    $$\frac{\partial \mathcal{T}}{\partial \alpha} = -I_p \Omega \dot{\beta}$$
* **Resulting Equation:**

$$
(I_d \ddot{\alpha}) - (-I_p \Omega \dot{\beta}) + c_r \dot{\alpha} + k_{\alpha \alpha} \alpha + k_{\alpha y} y = M_x
$$

$$
\implies I_d \ddot{\alpha} + I_p \Omega \dot{\beta} + c_r \dot{\alpha} + k_{\alpha \alpha} \alpha + k_{\alpha y} y = M_x
$$

**4. For Rotation $\beta$ (Yaw about Y-axis):**
* **Gyroscopic Effect:** Using the same kinetic energy approximation.
* **Derivatives:**
    $$\frac{\partial \mathcal{T}}{\partial \dot{\beta}} = I_d \dot{\beta} - I_p \Omega \alpha \implies \frac{d}{dt}(I_d \dot{\beta} - I_p \Omega \alpha) = I_d \ddot{\beta} - I_p \Omega \dot{\alpha}$$
    $$\frac{\partial \mathcal{T}}{\partial \beta} = 0$$
* **Resulting Equation:**

$$
(I_d \ddot{\beta} - I_p \Omega \dot{\alpha}) - 0 + c_r \dot{\beta} + k_{\beta \beta} \beta + k_{\beta x} x = M_y
$$

$$
\implies I_d \ddot{\beta} - I_p \Omega \dot{\alpha} + c_r \dot{\beta} + k_{\beta \beta} \beta + k_{\beta x} x = M_y
$$

---

### **4.4 Matrix Form of the Equations of Motion**

The behavior of the rotordynamic system is described by the matrix equation:

$$
\mathbf{M} \ddot{\mathbf{q}} + (\mathbf{C} + \Omega \mathbf{G}) \dot{\mathbf{q}} + \mathbf{K} \mathbf{q} = \mathbf{F}
$$

Where:
* $\mathbf{M}$ is the symmetric Mass/Inertia matrix.
* $\mathbf{C}$ is the symmetric Damping matrix.
* $\mathbf{G}$ is the skew-symmetric Gyroscopic matrix (proportional to rotor speed $\Omega$).
* $\mathbf{K}$ is the symmetric Stiffness matrix.
* $\mathbf{F}$ is the external force vector (e.g., Unbalance).

By assembling the four scalar equations derived in Section 4.3, we obtain the specific Equation of Motion for our rotor model. Note that the order of the state vector is $\mathbf{q} = [x, y, \alpha, \beta]^T$.

$$
\begin{bmatrix}
m & 0 & 0 & 0 \\
0 & m & 0 & 0 \\
0 & 0 & I_d & 0 \\
0 & 0 & 0 & I_d
\end{bmatrix}
\begin{bmatrix} \ddot{x} \\ \ddot{y} \\ \ddot{\alpha} \\ \ddot{\beta} \end{bmatrix}
+
\left(
\begin{bmatrix}
c_{xx} & 0 & 0 & 0 \\
0 & c_{yy} & 0 & 0 \\
0 & 0 & c_{\alpha \alpha} & 0 \\
0 & 0 & 0 & c_{\beta \beta}
\end{bmatrix}
+ \Omega
\begin{bmatrix}
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & +I_p \\
0 & 0 & -I_p & 0
\end{bmatrix}
\right)
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\alpha} \\ \dot{\beta} \end{bmatrix}
+
\begin{bmatrix}
k_{xx} & 0 & 0 & k_{x\beta} \\
0 & k_{yy} & k_{y\alpha} & 0 \\
0 & k_{\alpha y} & k_{\alpha \alpha} & 0 \\
k_{\beta x} & 0 & 0 & k_{\beta \beta}
\end{bmatrix}
\begin{bmatrix} x \\ y \\ \alpha \\ \beta \end{bmatrix}
=
\begin{bmatrix} F_x \\ F_y \\ M_x \\ M_y \end{bmatrix}
$$

### **Constitutive Relations for Stiffness and Damping**

To relate the general matrix terms ($k_{xx}$, etc.) to the physical bearing parameters, we expand them based on the system geometry.

**We define:**
* $k_1, k_2, k_i$: Stiffness of Front, Rear, and Inter-shaft bearings.
* $c_1, c_2, c_i$: Damping of Front, Rear, and Inter-shaft bearings.
* $l_1, l_2, l_i$: Axial distance from CoG to each bearing. ($l_1$ is positive for locations in front of the CoG; $l_2, l_i$ are negative for locations behind the CoG).

#### **1. Stiffness Matrix ($\mathbf{K}$)**
The stiffness matrix is fully populated and geometrically coupled. The off-diagonal terms represent the moment generated by the "lever arm" of the bearings relative to the CoG.

**Expanded Matrix:**
$$
\mathbf{K} =
\begin{bmatrix}
(k_1 + k_2 + k_i) & 0 & 0 & (k_1 l_1 - k_2 l_2 - k_i l_i) \\
0 & (k_1 + k_2 + k_i) & -(k_1 l_1 - k_2 l_2 - k_i l_i) & 0 \\
0 & -(k_1 l_1 - k_2 l_2 - k_i l_i) & (k_1 l_1^2 + k_2 l_2^2 + k_i l_i^2) & 0 \\
(k_1 l_1 - k_2 l_2 - k_i l_i) & 0 & 0 & (k_1 l_1^2 + k_2 l_2^2 + k_i l_i^2)
\end{bmatrix}
$$

* **Translational ($k_{xx}$):** Sum of all stiffnesses.
* **Rotational ($k_{\alpha \alpha}$):** Sum of stiffness $\times$ distance squared.
* **Coupling ($k_{x\beta}$):** Sum of stiffness $\times$ distance.

#### **2. Damping Matrix ($\mathbf{C}$)**
While physical dampers would create geometric coupling similar to springs, standard rotordynamic analysis often employs **Decoupled Damping**. We assume that the damping acts directly on the generalized modes (translation and tilt) rather than through geometric coupling. This simplifies tuning and avoids the complexity of estimating exact coupling coefficients for fluid film dampers.

**Expanded Matrix (Diagonal):**
$$
\mathbf{C} =
\begin{bmatrix}
c & 0 & 0 & 0 \\
0 & c & 0 & 0 \\
0 & 0 & c_r & 0 \\
0 & 0 & 0 & c_r
\end{bmatrix}
$$

* **Effective Translational Damping ($c$):** Aggregates the effect of all Squeeze Film Dampers ($c \approx c_1 + c_2 + c_i$).
* **Effective Rotational Damping ($c_r$):** Aggregates aerodynamic drag and structural loss ($c_r \approx c_1 l_1^2 + c_2 l_2^2$).