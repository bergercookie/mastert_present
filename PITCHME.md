
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


<!--TODO Add a date -->
<!--TODO Add current section in right/left footer-->
<!--DONE-Works fine- Make this an offline version-->
<!--DONE-Works-fine- Test presenter mode-->
<!--TODO How should I handle citations?-->

---

## Presentation Contents

- Intro - What is SLAM? |
- Review of SLAM Algorithms |
- Single-Robot graphSLAM |
- Multi-Robot graphSLAM |
- Conclusions - Future Directions |

---

## Intro - What is SLAM?

---

## Definition


>Simultaneous localization and mapping (SLAM) is a process that aims
>to localize an autonomous mobile robot in a previously unexplored environment while
>constructing a consistent and incremental map of its environment
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

Can be devided based on the **processing scheme** and **map representation**.

---

## Processing Scheme

- Extended Kalman Filter (EKF) |
- Particle-Filtering (PF) |
- Graph-based approaches |

Note:
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
\hat{m}_{k-1})  \big)\\\\
P_{k|k} &= P_{k|k-1}-W_k S_k W_k^T
\end{align}
\]`
</span>

<span style="font-size:0.8em">
`\[
\begin{align}
\S_k &= \nabla hP_{k|k-1} \nabla h^T + R_k\\
W_k &= P_{k|k-1} \nabla h^TS_k^{-1}
\end{align}
\]`
</span>


---

#### Extended Kalman Filter

- Handles uncertainty in landmarks and robot movement simultaneously (both are
    considered in the SLAM state)
- Handles the nonlinearities by linearizing the motion, observation models (not
	suitable for highly non-linear models)
- Motion step affects only the current position estimate not the map.
- Observation step requires the landmarks means and covariances to be
    computed:

Computational, Storage cost `\(\rightarrow\mathcal{O}(N^2)\)`

note:
mention that all the KF-variants suffer from one (or more) of its drawbacks
no need to perform time-update for stationary landmarks

---

### Particle Filter

TODO

---

### Graph-based SLAM


TODO

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

<!--- --------- End of Review of SLAM Algorithms-->


## Single-Robot graphSLAM

- Goals |
- Mathematical Formulation |
- Simulations |
- Real-time Experiments |

TODO

---

## Development Goals

- Generic/Extensible design |
  - Handle odometry, laser scans, monocular/stereo images etc. |
  - 2D/3D SLAM |
- Offline/online use |
  - Offline use with simulated or prerecorded datasets |
  - Integration with ROS for real-time graphSLAM |

Note:
How much am I going to write here?
Why exactly did I implement these things?
This should provide the **end product** of your work in sr-graphSLAM so that,
when you show them the specific math steps below, to know what they are about!

---

#### Levenberg-Marquardt Least-Squares Optimization

TODO

---

### Iterative Closest Point - ICP

TODO

---

#### Robust Loop-Closure (LC) Scheme

TODO

---

#### MRPT `lib-graphslam` design

- **CGraphSlamEngine:** Class that manages the overall execution of graphSLAM
- **CRegistrationDeciderOrOptimizer:** Common parent of all decider/optimizer classes
- **CNodeRegistrationDecider (NRD):** Add *new nodes* to the graph according to a specific criterion
- **CEdgeRegistrationDecider (ERD):** Add edges *between already added nodes* in the graph according to a specific criterion
- **CGraphSlamOptimizer(GSO):** Optimize an already constructed graph

---

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
- Use of `.ini` files to tweak the application behavior
  - Parameters of deciders, optimizer classes
  - Visualization parameters
  - ...

---?code=codes/sample.ini&lang=ini

---

#### Launching Application from the Command Line

- Specify the following:
  - dataset file
  - Ground-truth file
  - `.ini` configuration file
  - 2D/3D SLAM

```bash
$ graphslam-engine -r dataset.rawlog \
                   -g dataset.rawlog.GT.txt \
                   -i odometry_2DRangeScans.ini \
                   --2d
```

---

### Real-Time Setup & Execution

TODO

---


<!--- --------- End of Single-Robot graphSLAM-->

## Inter-Robot Communication

TODO

---

<!--- --------- End of Inter-Robot Communication-->

## Multi-Robot graphSLAM

TODO

---

### Multi-hypothesis Map-matching

TODO

---


## Example with 2 agents

---?image=assets/figures/bulk/map_merger_node_2_robots.png&size=contain

---

## Example with 3 agents

---?image=assets/figures/bulk/map_merger_node_3_robots.png&size=contain

---

#### Simulation Setup & Results

We conducted multi-robot simulations in **Gazebo**.

- 2 Mobile robots (Pioneer 2dx)
- Measurements utilized
  - Odometry
    - Odometry covariance (same for *x*, *y*, *yaw*): `\(0.1m\)`
  - Laser scanner:
    - Sensor range: `\([0.20 - 5.60]m\)`
    - resolution: `\(0.01m\)`
    - Sensor noise (Gaussian)
      - Mean: `\(0.0m\)`
      - Std. deviation: `\(0.01m\)`

---

## Gazebo World

---?image=assets/figures/bulk/simulation_env_gazebo.png&size=contain

---

# Simulation Demo
---

![](https://www.youtube.com/embed/4RKS2jrvsYE)

---

## Experimental Results

TODO

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
## References

  <div style="font-size:0.5em">
    <li>
      (Saeedi 2016) Multiple-Robot Simultaneous Localization and Mapping: A Review
      - Saeedi, Sajad and Trentini, Michael and Seto, Mae and Li, Howard
    </li>
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
