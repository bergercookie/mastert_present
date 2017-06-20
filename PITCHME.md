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

<div style="color:gray; font-size:2em; font-size: 0.5em">
Nikolaos Koukis<br>
Master Thesis Presentation
</div>


<!--TODO Add a date -->
<!--TODO Add current section in right/left footer-->

TODO

---

### Presentation Contents

- Intro - What is SLAM? |
- SLAM Algorithms - Why choose graphSLAM? |
- Single-robot graphSLAM |
- Multi-robot graphSLAM |
- Conclusions - Future Directions |

---

## Intro - What is SLAM?

TODO

---

### Definition

TODO

---

### SLAM Commercial/Academic Usages

TODO

---

### SLAM Examples

TODO

---

<!--- --------- End of Intro - What is SLAM?-->


## SLAM Algorithms - Why choose graphSLAM?

TODO

---

### SLAM map representation

- Feature-based SLAM |
- View-based SLAM |
- ...

Note:
describe **briefly** the two variants
~2min

TODO


---


### SLAM processing scheme

- Kalman Filter (KF) |
- Particle-Filtering (PF) |
- Graph-based approaches |

Note:
describe **briefly** the KF, PF. GraphSLAM is on its own in the next
~2min

TODO

---

<!--- --------- End of SLAM Algorithms - Why choose graphSLAM?-->


## Single-robot graphSLAM

+ Goals |
+ Mathematical Formulation |
+ Simulations |
+ Real-time Experiments |

TODO

---

### Single-robot Goals

TODO

---

#### Development Goals

- Generic/Extensible design
  - Handle odometry, laser scans, monocular/stero images etc. |
  - 2D/3D SLAM |
- Offline/online use
  - Offline use with simulated or prerecorded datasets |
  - Integration with ROS for real-time graphSLAM |

Note: How much am I going to write here?

Note:
Why exactly did I implement these things?
This should provide the **end product** of your work in sr-graphSLAM so that,
when you show them the specific math steps below, to know what they are about!

TODO

---

### Levenberg-Marquardt Least-Squares Optimization

TODO

---

### Iterative Closest Point - ICP

TODO

---

### Robust Loop-Closure (LC) Scheme

TODO

---


<!--- --------- End of Single-robot graphSLAM-->

## Multi-robot graphSLAM

TODO

---

<!--- --------- End of Multi-robot graphSLAM-->

## Conclusions - Future Directions

TODO

---

<!--- --------- End of Conclusions - Future Directions-->
