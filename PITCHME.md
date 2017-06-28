<center><div style="width:10%">
![](assets/figures/MoreGraphics/NTUA.jpg)
</div></center>

<div style="color:gray; font-size:0.5em; height:3em;">
National Technical University of Athens<br>
School of Mechanical Engineering<br>
Section of Mechanical Design and Automatic Control<br>
Control Systems Laboratory<br>
</div>

<br>


### Design and Development of SLAM Algorithms

<div style="color:gray; font-size: 1.3em">
Nikos Koukis<br>
</div>

<div style="color:gray; font-size: 0.8em; position: fixed; bottom: 0; left: 50%">
Master Thesis
</div>


---

## Presentation Contents

- Intro - What is SLAM? |
- Review of SLAM Algorithms |
- Single-Robot graphSLAM - GSoC Internship @MRPT |
- Multi-Robot graphSLAM |
- Conclusions - Future Directions |

---

## Intro - What is SLAM?

Goal of SLAM is to **simultaneously**:

- Construct consistent, incremental map
- Localize itself in that map

---

### SLAM Commercial/Academic Usages

- Self-driving cars |
- Domestic robots (Roomba, Neato) |
- Architectural applications (GeoSLAM) |
- Large-scale mapping applications (UAV Atmos) |
- Space applications (NASA Planetary Rovers) |

---?image=assets/figures/bulk/roomba.jpg&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/bulk/stanley.jpg&size=contain
<!-- .slide: data-background-transition="none" -->

<!--Just in case... https://www.sbir.gov/sbirsearch/detail/369703 -->
<!--Also see the Downloaded files -->
---?image=assets/figures/bulk/mars_rover.jpg&size=contain
<!-- .slide: data-background-transition="none" -->

---

<!--- --------- End of Intro - What is SLAM?-->


## Review of SLAM Algorithms

Can be divided according to
- Map representation |
- Processing scheme |


---

## Map Representation

- Feature-based SLAM |
- View-based SLAM |
- ... |

---?image=assets/figures/bulk/map_representation.png&size=contain
<!-- .slide: data-background-transition="none" -->

Note:
describe **briefly** the two variants.
Feature-based maps depend on specific landmarks. View-based handle raw sensor
data. No loss of info there
~2min

---


## Processing Scheme

- Extended Kalman Filter (EKF) |
- Particle-Filter (PF) |
- Graph-based approaches |
- ... |

Note:
State the purpose of this section!!!
Why do we pick the graph-based approaches
Describe what the EKF, PF **do**, do not analyze the math
~4min

---

### Extended Kalman Filter

##### Motion model

`\[ P(x_k | x_{k-1}, u_k) \Longleftrightarrow x_k = f(x_{k-1}, u_k) + w_k \]`


##### Observation model

`\[ P(z_k | x_{k}, m) \Longleftrightarrow z_k = h(x_{k}, m) + v_k \]`

---

#### Extended Kalman Filter

Compute joint posterior distribution:

<span style="font-size:0.8em">
`\[
\begin{align}
\begin{bmatrix}
  \hat{x}_{k | k}\\
  \hat{m}
\end{bmatrix}
&= E
\left[
  \begin{matrix}
      x_k \hfill \\
      \hat{m}_k
  \end{matrix}
  \, | \,
  Z_{0:k}
\right]
\\[1pt]
P_{k|k} =
\begin{bmatrix}
  P_{xx} & P_{xm}\\
  P^T_{xm} & P_{mm}
\end{bmatrix}_{k | k}
&= E
\left[
  \begin{matrix}
      \left(
      \begin{matrix}
          x_k - \hat{x}_k \\
          m - \hat{m}_k
      \end{matrix}
      \right)
	  \left(
	  \begin{matrix}
		  x_k - \hat{x}_k \\
		  m - \hat{m}_k
	  \end{matrix}
	  \right)^T &
  \end{matrix}
  |\, Z_{0:k}
\right]
\end{align}
\]`

</span>

Split into a **time** and an **observation** update.

---

#### Extended Kalman Filter

**Time Update**

<span style="font-size:0.8em">
`\[
\begin{align}
\hat{x}_{k|k-1} &= f(\hat{x}_{k-1|k-1}, u_k) \\
P_{xx, k|k-1} &= \nabla f P_{xx, k-1|k-1} \nabla f^T + Q_k
\end{align}
\]`
</span>


**Observation Update**

<span style="font-size:0.8em">
`\[
\begin{align}
\begin{bmatrix}
	\hat{x}_{k|k} \\
	\hat{m}_k
\end{bmatrix} &=
\begin{bmatrix}
	\hat{x}_{k|k-1} \\
	\hat{m}_{k-1}
\end{bmatrix}+W_k\big(z(k)-h(\hat{x}_{k|k-1},
\hat{m}_{k-1})  \big)\\
P_{k|k} &= P_{k|k-1}-W_k S_k W_k^T\\
where &\\
S_k &= \nabla hP_{k|k-1} \nabla h^T + R_k\\
W_k &= P_{k|k-1} \nabla h^TS_k^{-1}
\end{align}
\]`
</span>

---

#### Extended Kalman Filter

<div style="font-size:1.2em">
<ul>
  <li>Handles landmarks uncertainty and robot movement</li>
  <li>Linearize motion, observation models</li>
  <li>Computational, Storage complexity `\(\rightarrow\mathcal{O}(N^2)\)`</li>
  <li>Constructs exclusively landmarks-based maps</li>
  <li>Works in 2D/3D SLAM</li>
</ul>
</div>

---

### Particle Filter - FastSLAM

<ul>
 <li>Directly represent non-linear motion model, non-Gaussian pose distribution.</li>
 <li>Apply Rao-Blackwellisation (RB) to joint space probability;
 <ul>
    <li>Condition map computation on whole robot trajectory
    <li>Use a *set of particles* representing possible trajectory
    `\(X_{0:k}^{(i)}\)`, corresponding weight `\(w_k^{(i)}\)`</li>
</ul>

<br>

<span style="font-size:1.0em">
`\[
\begin{align}
  P(X_{0:k}, m | Z_{0:k}, U_{0:k}, x_0) =
  &P(m | X_{0:k}, Z_{0:k}) \times \\
  &P(X_{0:k} | Z_{0:k}, U_{0:k}, x_0)
\end{align}
\]`
</span>

Note:
All the EKF shit should take ~2min

---

### Particle Filter - FastSLAM

<ol>
  <li> <b>Proposal Distribution:</b> Draw for each particle, based on its prior
    poses, observations and latest control input.
  </li>
  <li><b>Sample weighting:</b> Samples are weighted according to an importance
    function.</li>
  <li><b>(Optional) resampling:</b>
    <ul>
        <li>Select prominent particles</li>
        <li>Eliminate the rest</li>
        <li>Reset the weights `\(\rightarrow w_k^{(i)} = \frac{1}{N}\)`</li>
    </ul>
</ol>

Note:
All the PF shit should take ~2min

---

### Particle Filter - FastSLAM

<ul>
  <li>FastSLAM produces landmarks-based maps - ``gmapping`` variant produces
  occupancy grid maps</li>
  <li>Doesn't extend in 3D; Computationally expensive</li>
  <li>Applies linearization to observation model</li>
  <li>Computational complexity: `\( \mathcal{O}\big((M \times log(N))\big) \)`,
      M = num of particles </li>
</ul>

---

### Graph-based SLAM

- Use graph to represent SLAM problem
  - Nodes `\( \rightarrow \)` *poses* of the robot
  - Edge between two nodes `\( \rightarrow \)`  *spatial constraint*
- Optimize for the whole trajectory - not only latest pose
- Optionally consider the robot poses exclusively (no landmarks)

<hr>

<div class="fragment">
Find node configuration for which the overall error of constraints is
minimized `\( \rightarrow \)` *least-squares problem*.
</div>


Note:
Take your time to explain this correctly - should take ~2min

Give an example on the edge addition.
"Assume two nodes A, B for which we store the corresponding laser scans. If we
can determine the 2D transformation to maximally align the laser scan of one
node to that of the other we can add an edge constraining those two nodes."

---

### Graph-based SLAM

Break overall problem down to:

<div style="font-size:1.0em">
<ul>
  <li>Frontend</li>
  <ul>
    <li>Constructs <i>the initial graph</i> from raw sensor data</li>
    <li>Deal with the <i>data association problem</i></li>
  </ul>
  <li>Backend</li>
  <ul>
    <li>Multivariate optimization scheme</li>
    <li>Minimize the error vector between <i>predicted</i>, <i>measured
    state</i></li>
    <li>Least-squares solver (Gauss-Newton, Levenberg-Marquardt, Gradient
    Descent)</li>
  </ul>
</ul>
</div>

---

### Graph-based SLAM

- `\( x = \left( x_1, x_2, \cdots x_T\right)^T \)`: Set of estimated robot
    trajectory poses (graph nodes).
- `\(z_{i,j}, \Omega_{i,j} \)`: Mean, information matrix of a *virtual
measurement*, associates two graph nodes.
- `\( \hat{z}_{i,j} \)`: Mean of *prediction* of a virtual measurement.
    Computed via initial poses of nodes `\( i, j \)`.

note:
On the dimensions of the terms:
e_{i,j} -> VECTOR of error terms for all node ID pairs that I have a measurement
\hat x -> VECTOR of initial guesses
\Delta x -> VECTOR of differences between the initial guesses and the real
values

c_{i,j} is a scalar  (e_{i,j}^T * X * e_{i,j}) and e_{i,j} is a vector


---

### Graph-based SLAM

Error multivariate function:

`\[ e(x_i, x_j) = e_{i,j} = z_{i,j} - \hat{z}_{i,j}(x_i, x_j) \]`


Log-likelihood of the virtual measurement:

<div style="font-size:1.0em">
`\[
\begin{align}
    l_{i,j} &\propto \big[ z_{i,j} - \hat{z}_{i,j}(x_i, x_j) \big]^T
    \Omega_{i,j}
    \big[ z_{i,j} - \hat{z}_{i,j}(x_i, x_j) \big] \\
    &=
    e_{i,j}^T \, \Omega_{i,j} \, e_{i,j}
\end{align}
\]`
</div>

---?image=assets/figures/bulk/virtual_measurements.png&size=contain

---

### Graph-based SLAM

`\( C \)`: Set of pairs of indices `\(i,j\)` for which a virtual measurement exists.

**Goal:** Find the vector of node positions `\( x^{\star} \)` that maximizes the
    log-likelihood of all observations in `\( C \)`

`\[
\mathbf{F}(x) =
\sum_{\langle i,j \rangle \in C}F_{i,j} =
\sum_{\langle i,j \rangle \in C}
e_{i,j}^T \, \Omega_{i,j} \, e_{i,j}
\]`

To express this in least-squares form:

`\[ x^{\star} = argmin_{x}\mathbf{F}(x) \]`


---

### Graph-based SLAM

1st order Taylor approximation of error function:

<div style="font-size:0.8em">
`\[
\begin{align}
e_{i,j}(\breve x_i + \Delta x_i, \breve x_j + \Delta x_j)
&= e_{i,j}(\breve x + \Delta x) \\
&\approx e_{i,j} + J_{i,j} \Delta x
\end{align}
\]`
</div>

`\( \mathbf{F} \)` can be rewritten as:

<div style="font-size:0.6em">
`\[
\begin{align}
\mathbf{F}_{i,j}(\breve x + \Delta x)
&= e_{i,j}(\breve x + \Delta x)^T \Omega_{i,j} e_{i,j}(\breve x + \Delta x) \\
&\approx (e_{i,j} + J_{i,j} \Delta x)^T \Omega_{i,j} (e_{i,j} + J_{i,j}
\Delta x) \\
&= \underbrace{e^T_{i,j}\Omega_{i,j}e_{i,j}}_{c_{i,j}} + 2
\underbrace{e^T_{i,j}\Omega_{i,j} J_{i,j}}_{b_{i,j}} \Delta x +
\Delta x ^T \underbrace{J^T_{i,j} \Omega_{i,j} J_{i,j}}_{H_{i,j}} \Delta x \\
&= c_{i,j} + 2b_{i,j} \Delta x + \Delta x^T H_{i,j} \Delta x
\end{align}
\]`
</div>

---

### Graph-based SLAM

Using latter expression, and by setting:

<div style="font-size:1.0em">
`\[

\begin{align*}
    c &= \sum c_{i,j} \\
    b &= \sum b_{i,j} \\
    H &= \sum H_{i,j}
\end{align*}

\]`
</div>

Rewrite as follows:

<div style="font-size:0.7em">
`\[

\begin{align}
    \mathbf{F}(\breve x + \Delta x)
    &= \sum_{\langle i,j \rangle \in C}F_{i,j}(\breve x + \Delta x) \\
    &\approx \sum_{\langle i,j \rangle \in C}\Big[ c_{i,j} + 2 b_{i,j} \Delta x
    + \Delta x ^T H_{i,j} \Delta x \Big] \\
    &= c + 2 b^T \Delta x + \Delta x^T H \Delta x
\end{align}

\]`
</div>

---

### Graph-based SLAM

Reach a formula suitable for optimization; Compute partial
derivative wrt `\( \Delta x \)`,  set it to `\( 0 \)`:

<div style="font-size:0.8em">
`\[
\begin{align}
    \frac{\partial  F(x + \Delta x)}{\partial{\Delta x}}
    &= \left(  H +  H^T \right) \Delta x + 2b
    \stackrel{\text{H symmetric}}{=} \notag \\
    &= 2 H \Delta x + 2b  = 0 \Rightarrow \notag \\
    \Delta x^{\star} &= - H^{-1} b
\end{align}
\]`
</div>


---

### Why Choose GraphSLAM?

<div style="font-size:1.0em">
<ul>
  <div class="fragment">
    <li> Modular design; clear distinction between:
    <ul>
      <li>Acquisition of measurements, initial graph construction</li>
      <li>Computations part - graph optimization</li>
    </ul>
  </div>
  <div class="fragment">
    <li>Supports variety of sensors</li>
  </div>
  <div class="fragment">
    <li>Same backend for 2D/3D constraints</li>
  </div>
  <div class="fragment">
    <li>Optimize for whole trajectory - Increased accuracy
  </div>
  <div class="fragment">
    <li>Not restrained to a particular map format:</li>
    <ul>
      <li>include landmarks <b>or</b></li>
      <li>execute <em>"mapping with known poses"</em>, construct the map
          by aligning measurements.</li>
    </ul>
  </div>
  <div class="fragment">
    <li>Computational complexity: <em>Linear</em> in number of edges</li>
  </div>
</ul>
</div>

Note:
State the benefits. Why is it fundamentally better?
2min

Mention that pose-graphSLAM supports all sensors that can provide inter-pose
constraints

---

<!--- --------- End of Review of SLAM Algorithms-->

## Single-Robot graphSLAM

- Google Summer of Code Internship @MRPT |
- Development Goals |
- Mathematical Formulation |
- Simulations |
- Real-time Experiments |

---

## GSoC Internship

Designed and implemented a complete single-robot graphSLAM framework in
[MRPT](http://mrpt.org) which also included:

- SLAM Error metric + visualization
- Implementation of Robust Loop Closure (LC) scheme initially [designed by
    Olson](https://pdfs.semanticscholar.org/ee30/49d302f56bcadaaec9f3d57642a32ff5224d.pdf) - ``CLoopCloserERD``.
- Use of a consistent [observation-grouping
    algorithm](http://ieeexplore.ieee.org/document/1641810) proposed by
    J.L.Blanco


---

## Development Goals

<div style="font-size=0.7em"/>
Build a tool to execute and visualize SLAM
<br><br>
<ul>
  <li>Generic/Extensible design</li>
    <ul>
      <li>Handle odometry, laser scans, (potentially) monocular/stereo images</li>
      <li>2D/3D SLAM</li>
      <li>Offline/online use:</li>
      <ul>
        <li>Simulated or prerecorded datasets</li>
        <li>Integration with ROS, real-time graphSLAM</li>
      </ul>
    </ul>
</ul>
</div>


Note:
How much am I going to write here?
Why exactly did I implement these things?
This should provide the **end product** of your work in sr-graphSLAM so that,
when you show them the specific math steps below, to know what they are about!

---

## UI Design Insight

<div style="font-size:1.0em"/>
Two options for visualizing the procedure
<ul>
  <li>MRPT Native application
    <ul>
      <li>Written in <i>OpenGL</i> + <i>wxWidgets</i></li>
      <li>Simulated, real-time setups</li>
      <li>Does not require <i>ROS</i> installation </li>
    </ul>
  </li>
  <li>ROS <i>Qt</i> GUI
    <ul>
      <li>Real-time setups</li>
      <li>Better integration with ROS ecosystem (<i>roscore</i>, <i>ros
          messages</i>, <i>rqt</i>, <i>tf2</i>)</li>
    </ul>
  </li>
</ul>
</div>

---?image=assets/figures/bulk/gui_main.png&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/bulk/ros_visuals.png&size=contain
<!-- .slide: data-background-transition="none" -->

---

### Iterative Closest Point - ICP

Assume two point clouds X, P. Find transformation `\((R, t)\)` which, if
applied to P will maximally align its points with those of cloud X.

`\[
\begin{align}
X &= \{ x_1, x_2, \cdots x_{N_x} \} \\
P &= \{ p_1, p_2, \cdots p_{N_p} \}
\end{align}
\]`


Potential error metric:

`\[
E(R,t) = \frac{1}{N_p} \sum_{i=1}^{N_p} \| x_i - R p_i -t\|^2
\]`



---

### Iterative Closest Point - ICP

- Starting from an initial estimation of the transform between the two point
    clouds, iteratively refine it by minimizing a suitable error metric.
- Used for registering the outputs of 2D/3D scanners; add new edges between
    already registered nodes.

---

### Iterative Closest Point - ICP

Find the centers of mass for each point cloud; substract if from every point:

`\[
\mu_x = \frac{1}{N_x}\sum_{i=1}^{N_x} x_i \,\,\,\,
\mu_p = \frac{1}{N_p}\sum_{i=1}^{N_p} p_i
\]`

<br>

`\[
X^{\prime} = \{ x_i - \mu_x \} = \{ x_i^\prime \} \,\,\,\,
P^{\prime} = \{ p_i - \mu_p \} = \{ p_i^\prime \}
\]`

---

### Iterative Closest Point - ICP

Let `\( W = \sum_{i=1}^{N_p} x^\prime_i p{^\prime}^T_i \)`. We denote its
*Singular Value Decomposition* (SVD) as:

<br><br>

`\[
W = U \begin{bmatrix}
\sigma_1 & 0 & 0 \\
0 & \sigma_2 & 0 \\
0 & 0 & \sigma_3
\end{bmatrix} V^T
\]`

Note:

U,V are orthogonal
sigma1 > sigma2 > sigma3

---

### Iterative Closest Point - ICP

Theorem:

If `\( rank(W) = 3 \)`, the optimal solution E(R,t) is unique and given by:

`\[
\begin{align}
R &= U V^T \\
t &= \mu_x - R \mu_p
\end{align}
\]`

Minimal value of error function:

`\[
E(R,t) = \sum_{i=1}^{N_p}\big(\|x^{\prime}_i \|^2 + \| y^{\prime}_i \|^2
-2(\sigma_1 + \sigma_2 + \sigma_3) \big)
\]`

---

### Iterative Closest Point - ICP

Algorithm stages:
<br>

<div style="font-size:1.0em"/>

<ul>
    <li> **Selection** of set of points in one or both meshes.</li>
    <li> **Matching** these points to samples in the other mesh.</li>
    <li> **Weighting** corresponding pairs appropriately.</li>
    <li> **Rejecting** certain pairs.</li>
    <li> Assigning an **error metric**.</li>
    <li> **Minimizing** error metric.</li>
</ul>
</div>

---?image=assets/figures/bulk/icp_example.png&size=contain

---

#### MRPT ``lib-graphslam`` design

- **CGraphSlamEngine:** Manages the overall execution
- **CRegistrationDeciderOrOptimizer:** Common parent of all deciders/optimizers 
- **CNodeRegistrationDecider (NRD):** Add *new nodes* to the graph
- **CEdgeRegistrationDecider (ERD):** Add edges *between already added nodes*
- **CGraphSlamOptimizer(GSO):** Optimize already constructed graph

<!--Ref: http://www.gravizo.com/ -->
---?image=assets/figures/dot/lib_hierarchy.png&size=contain

note:
Describe what's the actual separation of tasks. Why is this important? Give
examples!

---

### MRPT Simulation Examples

<!--https://github.com/gitpitch/gitpitch/wiki/Image-Animations-Workflows -->

---?image=assets/figures/gimp/mrpt_sr_slam_combined.png&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/gimp/mrpt_sr_slam_combined2.png&size=contain

---

<!-- .slide: class="center" -->
# Simulation Demo

---

![](https://www.youtube.com/embed/Pv0yvlzrcXk)

---

### Configuring the Application

- Don't meddle with source code
- Use of ``.ini`` files, tweak application behavior
  - Parameters of deciders, optimizer classes
  - Visualization parameters
  - ...

---?code=codes/sample.ini&lang=ini

---

#### Launching Application from the Command Line

- Specify the following:
  - dataset file
  - Ground-truth file
  - ``.ini`` configuration file
  - 2D/3D SLAM

```bash
$ graphslam-engine -r dataset.rawlog \
                   -g dataset.rawlog.GT.txt \
                   -i odometry_2DRangeScans.ini \
                   --2d
```

---

<!-- .slide: class="center" -->
### Real-Time Setup & Execution

---

### Real-time experiment

- For comparison compute ground-truth: 
    - `Aruco` static and moving markers
    - Use ``ar_sys`` for computing the camera(s) `\( \rightarrow \)` marker(s)
        transforms
- 2 scenarios
  - Odometry + laser scans
  - laser scans only

---?image=assets/figures/bulk/real_time_gt_setup.png&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/bulk/real_time_CFixedIntervalsNRD_CLoopCloserERD_gt.png&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/bulk/real_time_CICPCriteriaNRD_CICPCriteriaERD_gt.png&size=contain
<!-- .slide: data-background-transition="none" -->

note:
Mention the ar_sys faulty measurements in the end
Mention the camera range near the trash can

note:
Mention why we acquired the ground-truth

---

#### Configuring Robot for real-time graphSLAM

<div style="font-size:0.8em"/>

Generic way to define the processes that are launched in each of the
running robots. Account for:

<ul>
  <li>Robot movement</li>
  <li>Teleoperation (joystick, keyboard)</li>
  <li>Sensor acquisition (laser, camera)</li>
  <li>Network utilities (communication of agents in MR-SLAM)</li>
  <li>Various utilities (dataset recording)</li>
  <li>Software config (deciders/optimizer to use)</li>
</ul>

<br> <br>

Take care of multiple configurations (robot type, laser type)
</div>

<hr>

<div class="fragment", style="font-size:0.8em">
Use a *shell script* that defines environment variables. Source that in
every robot agent separately. Based on the variables set, launch the
corresponding processes.
</div>


---?code=codes/real_time_config.sh&lang=bash

---

<!--- --------- End of Single-Robot graphSLAM-->

<!-- .slide: class="center" -->
## Inter-Robot Communication

---

### Design requirements

<div style="font-size: 1.2em">
<ul>
  <div class="fragment">
    <li>Arbitrary number of agents in real-time experiment</li>
  </div>
  <div class="fragment">
    <li>Robust to communication, agent failure</li>
  </div>
  <div class="fragment">
    <li>Minimization of exchanged data between agents - Assume limited
        communications</li>
  </div>
  <div class="fragment">
    <li>No prior network infrastructure</li>
  </div>
</ul>
</div>

---

### Network Setup

<div style="font-size: 1.0em">
<ul>
  <li>Fully distributed ad-hoc network</li>
  <li>Robots run <b>their own separate</b> <code>roscore</code> instance.
  <li>Exchange messages with counterparts via the <code>multimaster_fkie</code>
  ROS package.
  </li>
  <ul>
    <li><b>Central node</b> runs as DNS server.</li>
    <li>Rerouting of multicast packets to the ad-hoc interface</li>
  </ul>
  <li> Provide access to the internet</li>
</ul>
</div>


note:

Previous bullets are quite to the point. Do describe what happens for each one of them and why it is needed

---

### Network Setup

Example: Configuring an agent to join the ad-hoc network


```sh
  # Get the 10.8.0.16 IP in the ad-hoc
  # Access internet via the 10.8.0.1 node
  # ad-hoc interface is wlan0
  $ [sudo] $(rospack find csl_hw_setup)/scripts/ad_hoc_network/setup_adhoc.py \
                                                                  -a 10.8.0.1 \
                                                                  -I 10.8.0.16 \
                                                                  -w wlan0
```

---

### Communication Protocol

<div style="font-size:0.8em">
  <ul>
    <li> Each robot executes single-robot graphSLAM.</li>
    <li>When in communication range robots exchange their local maps:</li>
    <ul>
      <li>Laser scan corresponding to latest registered graph node</li>
      <li>List of last X (optimized) node positions in own robot frame</li>
    </ul>
    <li>Receiver robot caches the transmitters data (can't use them yet -
      unknown relative position of other agent.</li>
    <li>When enough data (scans + node positions) have been gathered, build
      neighbor's map execute map-matching - find transformation between own and
      neighbor's frame of reference.
    </li>
    <li> If transformation is found integrate received measurements in own
      graph</li>
    <li> Continue standard mapping - Integrate rest of received nodes in
      batches of `\(Y\)` nodes
  </ul>
</div>

---

<!--- --------- End of Inter-Robot Communication-->

<!-- .slide: class="center" -->
## Multi-Robot graphSLAM

---

### Multi-hypothesis Map-matching

- Used to find the relative transformation from the robot's frame to the
    neighbor's frame. If found the incoming measurements can be integrated in
    own map
- Code implementation is provided via the MRPT `maps::CGridmapAligner`
    class.

---

### Multi-hypothesis Map-matching

<div style="font-size:0.8em">
  <ul>
    <li> Multi-hypothesis approach; Provides a multi-modal probability
    distribution for the transformation between two grid maps.</li>
    <li> Based on a modified RANSAC scheme; Non-deterministic algorithm</li>
    <li> Consists of a two-step matching procedure:
    <ul>
      <li> Grid-to-grid matching without any a priori information</li>
      <li> Point maps matching to refine the latter grid-matching estimation</li>
    </ul>
  </ul>
</div>

Note:
I didn't write this module. Say in short what it does and skip

---?image=assets/figures/bulk/jlblancoc_map_matching_scheme.png&size=contain

---

<!-- .slide: class="center" -->
## Example with 2 agents

---?image=assets/figures/bulk/map_merger_node_2_robots.png&size=contain

---

<!-- .slide: class="center" -->
## Example with 3 agents

---?image=assets/figures/bulk/map_merger_node_3_robots.png&size=contain

---

#### Simulation Setup & Results

<div style="font-size:0.8em">

We conducted multi-robot simulations in **Gazebo**.
<br><br>

<ul>
  <li>2 Mobile robots (Pioneer 2dx)</li>
  <li>Measurements utilized</li>
  <ul>
    <li>Odometry</li>
    <ul>
      <li>Odometry covariance (same for <code>x, y, yaw</code>): 0.1m </li>
    </ul>
  </ul>
  <li>Laser scanner</li>
  <ul>
    <li>Sensor range: [0.20 - 5.60]m </li>
    <li>Resolution: 0.01m </li>
    <li>Sensor noise (Gaussian)</li>
    <ul>
      <li>Mean: 0.0m</li>
      <li>Std. Deviation: 0.01m</li>
    </ul>
  </ul>
</ul>
</div>

---

<!-- .slide: class="center" -->
## Gazebo World

---?image=assets/figures/bulk/gazebo_simulation_env.png&size=contain

---

<!-- .slide: class="center" -->
## Map-matching

---?image=assets/figures/bulk/simulation_map_match.png&size=contain

---

<!-- .slide: class="center" -->
### Map-matching #2 - More obstacles

---?image=assets/figures/bulk/gazebo_simulation_env_more_obstacles.png&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/bulk/simulation_map_match_more_obstacles.png&size=contain
<!-- .slide: data-background-transition="none" -->

---


<!-- .slide: class="center" -->
# Simulation Demo

---

![](https://www.youtube.com/embed/4RKS2jrvsYE)

---

<!-- .slide: class="center" -->
# Simulation Demo #2

---

![](https://youtube.com/embed/6EjFP3jO4gM)

---

<!-- .slide: class="center" -->
## Experimental Results

---

<!-- .slide: class="center" -->
### Experiment - Around the pool

---?image=assets/figures/bulk/online_integration_nickkoukubuntu2.png&size=contain

---

<!-- .slide: class="center" -->
### Experiment - M building top floor

---?image=assets/figures/bulk/online_integration_nickkoukubuntu.png&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/bulk/online_integration_nickkoukubuntu_edited.png&size=contain
<!-- .slide: data-background-transition="none" -->

---

<!--- --------- End of Multi-Robot graphSLAM-->

<!-- .slide: class="center" -->
## Conclusions - Future Directions

---

## Thesis summary

- Thorough analysis of single, multi-robot graphSLAM strategies |
- Framework for single, multi-robot graphSLAM - offline or real-time |
- Communication scheme for multi-robot real-time experiment |
- Multiple tests in simulations and real-time |


note:
Tell the details as described in thesis text

---

## Future Directions

<div style="font-size:1.2">
<ul>
  <div class="fragment">
    <li>3rd party optimization framework</li>
    <ul>
      <li>g2o</li>
      <li>iSAM, iSAM2</li>
      <li>SE-Sync</li>
    </ul>
  </div>

  <div class="fragment">
    <li>Adaptive node registration decider / Node Reduction Scheme</li>
  </div>

  <div class="fragment">
    <li>Support visual sensors (Monocular, Stereo images)</li>
  </div>

  <div class="fragment">
    <li>3D Slam</li>
  </div>

  <div class="fragment">
    <li>Active exploration</li>
  </div>
</ul>
</div>

<!--- --------- End of Conclusions - Future Directions-->

---

### Asking for help

- [mrpt-graphslam docs](http://reference.mrpt.org/devel/group__mrpt__graphslam__grp.html)
- [ROS mrpt_graphslam_2d docs](http://wiki.ros.org/mrpt_graphslam_2d)
- In case of a question / bug report, open an issue on the corresponding Github page

---

## Acknowledgements

<div style="color:gray; font-size:2em; font-size: 1em">
Prof. Kostas J. Kyriakopoulos<br>
Dr. George Karras
</div>
<br>

<div style="color:gray; font-size:2em; font-size: 1em">
Dr. Jose-Luis Blanco - University of Almeria, MRPT Lead<br>
</div>
<br>

<div style="color:gray; font-size:2em; font-size: 1em">
CSL Team
</div>

<!--- --------- End of Acknowledgements-->

---

<!-- .slide: class="center" -->
### Thanks for your attention ![](https://raw.githubusercontent.com/github/gemoji/master/images/bowtie.png)

# Q&A
