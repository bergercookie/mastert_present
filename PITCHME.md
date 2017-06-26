
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

<div style="color:gray; font-size:2em; font-size: 1em">
Nikos Koukis<br>
</div>
<div style="color:gray; font-size:2em; font-size: 0.8em">
Master Thesis Presentation
</div>


<!--DONE-Works fine- Make this an offline version-->
<!--DONE-Works fine- Test presenter mode-->
<!--DONE-Works fine- Generate PDF-->
<!--DONE-Just use names and a list of citations in the last page- How I handle citations?-->

---

## Presentation Contents

- Intro - What is SLAM? |
- Review of SLAM Algorithms |
- Single-Robot graphSLAM - GSoC Internship @MRPT |
- Multi-Robot graphSLAM |
- Conclusions - Future Directions |

---

## Intro - What is SLAM?

---

## Definition


>Simultaneous localization and mapping (SLAM) is a process that aims
>to <b>localize an autonomous mobile robot</b> in a previously unexplored
>environment while <b>constructing a consistent and incremental map</b> of its
>environment
>
> <cite>Saeedi2016</cite>

---

### SLAM Commercial/Academic Usages

- Self-driving cars |
- Domestic robots (Roomba, Neato) |
- Architectural applications (GeoSLAM) |
- Large-scale mapping applications (UAV Atmos) |
- Space applications (NASA Planetary Rovers) |

---

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

Can be divided based  according to the
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
describe **briefly** the KF, PF. GraphSLAM is on its own in the next
~4min

---

#### Extended Kalman Filter

**Motion model:**

$$P(x\_k | x\_{k\_1}, u\_k) \Longleftrightarrow x\_k = f(x\_{k-1}, u\_k) + w\_k$$


**Observation model:**

$$P(z\_k | x\_{k\_1}, m) \Longleftrightarrow z\_k = h(x\_{k}, m) + v\_k$$

---

#### Extended Kalman Filter

We need to compute the joint posterior distribution:

<span style="font-size:0.8em">
$$
\begin{bmatrix}
  \hat{x}\_{k | k}\\\\
  \hat{m}
\end{bmatrix}
= E
\left[
  \begin{matrix}
      x\_k \hfill \\\\
      \hat{m}\_k
  \end{matrix}
  \, | \,
  Z\_{0:k}
\right]
\\\\[1pt]
P\_{k|k} =
\begin{bmatrix}
  P\_{xx} & P\_{xm}\\\\
  P^T\_{xm} & P\_{mm}
\end{bmatrix}\_{k | k}
= E
\left[
  \begin{matrix}
      \left(
      \begin{matrix}
          x\_k - \hat{x}\_k \\\\
          m - \hat{m}\_k
      \end{matrix}
      \right)
	  \left(
	  \begin{matrix}
		  x\_k - \hat{x}\_k \\\\
		  m - \hat{m}\_k
	  \end{matrix}
	  \right)^T &
  \end{matrix}
  |\\, Z\_{0:k}
\right]
$$
</span>

To do that, we split the latter into a **time** and an **observation** update.

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

<div style="font-size:0.8em">
<ul>
  <li>Handles uncertainty in landmarks and robot movement simultaneously (both
  are considered in the SLAM state)</li>
  <li>Handles the nonlinearities by linearizing the motion, observation models
  (not suitable for highly non-linear models)</li>
  <li>Motion step affects only the current position estimate not the map;
  Observation step requires the landmarks means and covariances to be
  computed</li>

  <center>Computational, Storage complexity `\(\rightarrow\mathcal{O}(N^2)\)`</center>

  <li>Constructs exclusively landmarks-based maps</li>
  <li>Works in either 2D or 3D SLAM</li>
</ul>
</div>

note:
mention that all the KF-variants suffer from one (or more) of its drawbacks
no need to perform time-update for stationary landmarks

---

### Particle Filter - FastSLAM

<ul>
 <li>Based on Monte Carlo sampling; directly represent the non-linear motion
   model, non-Gaussian pose distribution.</li>
 <li>Apply Rao-Blackwellisation (RB) to the joint space probability; Condition
   the map computation on the whole robot trajectory; Use a *set of particles*
   each one representing an possible trajectory `\(X_{0:k}^{(i)}\)` and a
   corresponding weight `\(w_k^{(i)}\)`</li>
</ul>

<span style="font-size:0.8em">
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
  <li><b>(Optional) resampling:</b> Select prominent particles - Eliminate the
    rest. Reset the weights `\(\rightarrow w_k^{(i)} = \frac{1}{N}\)`</li>
</ol>

Note:
All the PF shit should take ~2min

---

### Particle Filter - FastSLAM

<ul>
  <li>FastSLAM produces landmarks-based maps - ``gmapping`` variant produces
  occupancy grid maps</li>
  <li>No known working extension in 3D; Computationally expensive</li>
  <li>Still applies linearization to the observation model (as does EKF)</li>
  <li>Computational complexity: `\( \mathcal{O}\big((M \times log(N))\big) \)`,
      M = num of particles </li>
</ul>

---

### Graph-based SLAM

- Use a graph to represent the SLAM problem
  - Nodes correspond to *poses* of the robot during mapping
  - An edge between two nodes represents a *spatial constraint* (2D/3D
      transformation) between them.
- Optimize for the whole trajectory - not only the latest pose
- Option to consider the robot poses exclusively (no landmarks)

<hr>

<div class="fragment">
Find the node configuration for which the overall error of constraints is
minimized. This comes down to a *least-squares problem*.
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

<div style="font-size:0.8em">
<ul>
  <li>Frontend</li>
  <ul>
    <li>Constructs <i>the initial graph</i> from raw sensor data</li>
    <li>In case we also consider landmarks, it deals with the <i>data association
        problem</i></li>
  </ul>
  <li>Backend</li>
  <ul>
    <li>Multivariate optimization scheme</li>
    <li>Minimizes the error vector between the <i>predicted state</i> and the
      <i>measured state</i></li>
    <li>Comprises variants of least-squares solvers (Gauss-Newton,
      Levenberg-Marquardt, Gradient Descent)</li>
  </ul>
</ul>
</div>

---

### Graph-based SLAM

- `\( x = \left( x_1, x_2, \cdots x_T\right)^T \)`: Set of estimated robot
    trajectory poses (graph nodes).
- `\(z_{i,j}, \Omega_{i,j} \)`: Mean and information matrix of a *virtual
measurement* that associates two different graph nodes.

    - In pose-graph SLAM this is the transformation between two nodes: `\( i \rightarrow j \)`
- `\( \hat{z}_{i,j} \)`: Mean of the *prediction* of a virtual measurement.
    Computed via the initial poses of the nodes `\( i, j \)`.

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

<div style="font-size:0.8em">
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

Using the latter expression, and by setting:

<div style="font-size:0.6em">
`\[

\begin{align*}
    c &= \sum c_{i,j} \\
    b &= \sum b_{i,j} \\
    H &= \sum H_{i,j}
\end{align*}

\]`
</div>

We can rewrite as follows:

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

We have to reach to a formula suitable for optimization; Compute partial
derivative with regards to `\( \Delta x \)` and set it to `\( 0 \)`:

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

<div style="font-size:0.8em">
<ul>
  <div class="fragment">
    <li> Modular design; Makes a clear distinction between:
    <ul>
      <li>Acquisition of measurements, initial graph construction</li>
      <li>Computations part - graph optimization</li>
    </ul>
  </div>
  <div class="fragment">
    <li>Any sensor can be used as long as it provides inter-pose constraints</li>
  </div>
  <div class="fragment">
    <li>Backend works the same for 2D/3D constraints `\( \rightarrow \)` 2D/3D
        SLAM</li>
  </div>
  <div class="fragment">
    <li>Optimize for the whole trajectory - Increased accuracy
  </div>
  <div class="fragment">
    <li>Not restrained to a particular map format. We can:</li>
    <ul>
      <li>include landmarks in the mathematical formulation <b>or</b></li>
      <li>execute <em>"mapping with known poses"</em> and then construct the map
          by aligning the measurements.</li>
    </ul>
  </div>
  <div class="fragment">
    <li>Computational complexity: <em>Linear</em> in the number of edges</li>
  </div>
</ul>
</div>

Note:
State the benefits. Why is it fundamentally better?
2min

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
Build a tool to execute robustly and visualize SLAM
<br><br>
<ul>
  <li>Generic/Extensible design</li>
    <ul>
      <li>Handle odometry, laser scans, (potentially) monocular/stereo images etc.</li>
      <li>2D/3D SLAM</li>
      <li>Offline/online use:</li>
      <ul>
        <li>Offline use with simulated or prerecorded datasets</li>
        <li>Integration with ROS for real-time graphSLAM</li>
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

<div style="font-size:0.8em"/>
Two options for visualizing the procedure
<ul>
  <li>MRPT Native application
    <ul>
      <li>Written in <i>OpenGL</i> + <i>wxWidgets</i></li>
      <li>Can be used in both simulated, real-time setups</li>
      <li>Does not require installation of <i>ROS</i></li>
    </ul>
  </li>
  <li>ROS <i>Qt</i> GUI
    <ul>
      <li>Suitable for visualizing executing in real-time</li>
      <li>Better integration with the ROS ecosystem (<i>roscore</i>, <i>ros
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

Assume two point clouds X, P. Find the transformation `\((R, t)\)` which, if
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

<div style="font-size:0.8em"/>
Algorithm stages:
<ul>
    <li> **Selection** of some set of points in one or both meshes.</li>
    <li> **Matching** these points to samples in the other mesh.</li>
    <li> **Weighting** the corresponding pairs appropriately.</li>
    <li> **Rejecting** certain pairs by looking at each pair</li>
        individually or by considering an entire set of pairs.
    <li> Assigning an **error metric** based on the point pairs.</li>
    <li> **Minimizing** the error metric.</li>
</ul>
</div>

---

### Iterative Closest Point - ICP

Find the centers of mass for each point cloud; substract if from every point:

`\[
\mu_x = \frac{1}{N_x}\sum_{i=1}^{N_x} x_i \,\,\,\,
\mu_p = \frac{1}{N_p}\sum_{i=1}^{N_p} p_i
\]`

`\[
X^\prime = { x_i - \mu_x }= { x_i^\prime }
P^\prime = { p_i - \mu_p }= { p_i^\prime }
\]`

---

### Iterative Closest Point - ICP


---

---?image=assets/figures/bulk/icp_example.png&size=contain

---

#### Robust Loop-Closure (LC) Scheme

A robot returns to re-observe part of the already observed environment after a
large traverse. If this is detected it can be useful to reduce the accumulated
trajectory/map error.

---?image=assets/figures/bulk/gui_registered_loop_closure.png&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/bulk/gui_registered_loop_closure1.png&size=contain
<!-- .slide: data-background-transition="none" -->

---

#### MRPT ``lib-graphslam`` design

- **CGraphSlamEngine:** Class that manages the overall execution of graphSLAM
- **CRegistrationDeciderOrOptimizer:** Common parent of all decider/optimizer classes
- **CNodeRegistrationDecider (NRD):** Add *new nodes* to the graph according to a specific criterion
- **CEdgeRegistrationDecider (ERD):** Add edges *between already added nodes* in the graph according to a specific criterion
- **CGraphSlamOptimizer(GSO):** Optimize an already constructed graph

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

# Simulation Demo
---

![](https://www.youtube.com/embed/Pv0yvlzrcXk)

---

### Configuring the Application

- Users don't have to meddle with the source code / compilation
- Use of ``.ini`` files to tweak the application behavior
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

### Real-Time Setup & Execution

---

### Real-time experiment

- For comparison computed an estimation of the ground-truth path via `Aruco`
    static and moving markers; Use ``ar_sys`` for computing the
    camera(s) `\( \rightarrow \)` marker(s) transforms
- Tested it with the *Pioneer-2dx*, *Pioneer-2AT*, *Youbot* models
- We conducted the experiment in the top floor of the M building, NTUA
- 2 scenarios
  - Use odometry + laser scans
  - Use laser scans exclusively

---?image=assets/figures/bulk/real_time_gt_setup.png&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/bulk/real_time_CFixedIntervalsNRD_CLoopCloserERD_gt.png&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/bulk/real_time_CICPCriteriaNRD_CICPCriteriaERD_gt.png&size=contain
<!-- .slide: data-background-transition="none" -->

note:
Mention the ar_sys faulty measurements in the end
Mention the camera range near the trash can

---

#### Configuring Robot for real-time graphSLAM

<div style="font-size:0.8em"/>

We need a generic way to define the processes that are launched in each of the
running robots. These take care of:

<ul>
  <li>Robot movement</li>
  <li>Teleoperation (joystick, keyboard)</li>
  <li>Sensor acquisition (laser, camera)</li>
  <li>Various utilities (dataset recording)</li>
  <li>Network utilities (communication of agents in MR-SLAM)</li>
  <li>Software config (deciders/optimizer to use)</li>
</ul>

<br> <br>

Design should account for the multiple different options (robot type, laser
type)
</div>

<hr>

<div class="fragment", style="font-size:0.8em">
Use a *shell script* that defines a list of environment variables. Source that in
every robot agent separately. Based on the variables set, launch the
corresponding processes.
</div>


---?code=codes/real_time_config.sh&lang=bash

---

<!--- --------- End of Single-Robot graphSLAM-->

## Inter-Robot Communication

---

### Design requirements

<div style="font-size: 1.0em">
<ul>
  <div class="fragment">
    <li>Arbitrary number of agents in a real-time experiment</li>
  </div>
  <div class="fragment">
    <li>Robust to communication, agent failure</li>
  </div>
  <div class="fragment">
    <li>Minimization of exchanged data between the agents - Assume limited
        communications environment </li>
  </div>
  <div class="fragment">
    <li>No prior network infrastructure</li>
  </div>
</ul>
</div>

---

### Network Setup

<div style="font-size: 0.6em">
<ul>
  <li>Robots communicate over a fully distributed  ad-hoc network</li>
  <li>Automated script to register an <b>upstart job</b>; Configures the
    corresponding interface on ad-hoc mode on startup and when it is enabled -
    see
    [csl_hw_setup/ad_hoc_network](https://github.com/bergercookie/csl_mr_slam/tree/master/csl_hw_setup/scripts/ad_hoc_network)
  </li>
  <li>Robots run <b>their own separate</b> <code>roscore</code> instance. They
    exchange messages with their counterparts via the
    <code>multimaster_fkie</code> ROS package. This also requires the following:
  </li>
  <ul>
    <li><b>A central node</b> runs as a DNS server for name resolution of all the
      running agents.
    <li>Rerouting of multicast packets to the ad-hoc interface for
      multimaster_fkie processes to work as expected. </li>
  </ul>
  <li> Optionally provide access to the internet via a set of firewall rules
    and the central node as the middle man. </li>
</ul>
</div>

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

## Multi-Robot graphSLAM

---

### Multi-hypothesis Map-matching

- Used to find the relative transformation from the robot's frame to the
    neighbor's frame. If found the incoming measurements can be integrated in
    own map
- Algorithm is described in detail in <cite>Blanco2013</cite>
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


## Example with 2 agents

---?image=assets/figures/bulk/map_merger_node_2_robots.png&size=contain

---

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

## Gazebo World

---?image=assets/figures/bulk/gazebo_simulation_env.png&size=contain

---

## Map-matching

---?image=assets/figures/bulk/simulation_map_match.png&size=contain

---

## Map-matching #2 - More obstacles

---?image=assets/figures/bulk/gazebo_simulation_env_more_obstacles.png&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/bulk/simulation_map_match_more_obstacles.png&size=contain
<!-- .slide: data-background-transition="none" -->

---


# Simulation Demo

---

![](https://www.youtube.com/embed/4RKS2jrvsYE)

---

# Simulation Demo #2

---

![](https://youtube.com/embed/6EjFP3jO4gM)

---

## Experimental Results

---

### Experiment - Around the pool

---?image=assets/figures/bulk/online_integration_nickkoukubuntu2.png&size=contain

---

### Experiment - M building top floor

---?image=assets/figures/bulk/online_integration_nickkoukubuntu.png&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/bulk/online_integration_nickkoukubuntu_edited.png&size=contain
<!-- .slide: data-background-transition="none" -->

---

<!--- --------- End of Multi-Robot graphSLAM-->

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

<ul>
  <div class="fragment">
    <li>Integration of 3rd party optimization framework</li>
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
    <li>Implement deciders based on Visual sensors (Monocular, Stereo images)</li>
  </div>

  <div class="fragment">
    <li>Add full support for 3D Slam</li>
  </div>

  <div class="fragment">
    <li>Support for active exploration</li>
  </div>
</ul>

<!--- --------- End of Conclusions - Future Directions-->

---

### Open-source contributions

- [mrpt-graphslam](http://reference.mrpt.org/devel/namespacemrpt_1_1graphslam.html):
    Library that provides generic graphSLAM functionality.
- [graphslam-engine_app](http://www.mrpt.org/list-of-mrpt-apps/application-graphslamengine/):
    MRPT application that utilizes lib-graphslam to run single-robot simulated
    graphSLAM.
- [mrpt_graphslam_2d](http://wiki.ros.org/mrpt_graphslam_2d): ROS wrapper for
    the lib-graphslam library for real-time single, and multi-robot graphSLAM.
- [csl_mr_slam](https://github.com/bergercookie/csl_mr_slam): Suite of helper
    ROS packages that facilitate in simulated, real-time graphSLAM; provide
    automation scripts, launchfiles for Gazebo simulations, set of hardware
    sensors and drivers.
- [catkin_ws](https://github.com/bergercookie/catkin_ws): ROS set of packages
    that acts as a container and provides instructions for easier setup of
    graphSLAM

Stable versions of the above also exist in the [CSL Gogs
server](http://controlsystemslab.gr/code).

---

### Asking for help

- [mrpt-graphslam docs](http://reference.mrpt.org/devel/group__mrpt__graphslam__grp.html)
- [ROS mrpt_graphslam_2d docs](http://wiki.ros.org/mrpt_graphslam_2d)
- In case of a question / bug report, open an issue on the corresponding Github page


---

## References

<div style="font-size:0.5em;">
  <li> (Saeedi2016) Multiple-Robot Simultaneous Localization and Mapping: A Review
    - Saeedi, Sajad and Trentini, Michael and Seto, Mae and Li, Howard
  </li>
  <li> (Grisetti2010) A tutorial on graph-based SLAM - Grisetti, Giorgio
        and Kummerle, Rainer and Stachniss, Cyrill and Burgard, Wolfram
  </li>
  <li> (Blanco2013) A Robust , Multi-Hypothesis Approach to Matching Occupancy
  Grid Maps - Blanco, Jose-luis and Gonz, Javier and Fern, Juan-antonio
</div>

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

### Thanks for your attention ![](https://raw.githubusercontent.com/github/gemoji/master/images/bowtie.png)

# Q&A
