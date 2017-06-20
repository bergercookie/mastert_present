
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
<!--DONE-Works fine- Make this an offline version-->
<!--DONE-Works-fine- Test presenter mode-->

TODO

---

### Presentation Contents

- Intro - What is SLAM? |
- Review of SLAM Algorithms |
- Single-Robot graphSLAM |
- Multi-Robot graphSLAM |
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

<!--- --------- End of Intro - What is SLAM?-->


## SLAM Algorithms - Why Choose graphSLAM?

TODO

---

### Map Representation

- Feature-based SLAM |
- View-based SLAM |
- ... |

TODO

Note:
describe **briefly** the two variants
~2min

---


### Processing Scheme

- Kalman Filter (KF) |
- Particle-Filtering (PF) |
- Graph-based approaches |

TODO

Note:
describe **briefly** the KF, PF. GraphSLAM is on its own in the next
~2min

---

<!--- --------- End of SLAM Algorithms - Why choose graphSLAM?-->


## Single-Robot graphSLAM

- Goals |
- Mathematical Formulation |
- Simulations |
- Real-time Experiments |

TODO

---

#### Development Goals

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

### Levenberg-Marquardt Least-Squares Optimization

TODO

---

### Iterative Closest Point - ICP

TODO

---

### Robust Loop-Closure (LC) Scheme

TODO

---


### MRPT Simulation Examples

<!--https://github.com/gitpitch/gitpitch/wiki/Image-Animations-Workflows -->

---?image=assets/figures/gimp/mrpt_sr_slam_combined.png&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/gimp/mrpt_sr_slam_combined2.png&size=contain
<!-- .slide: data-background-transition="none" -->

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

### Simulation Results

TODO

---

### Experimental Results

TODO

---

<!--- --------- End of Multi-Robot graphSLAM-->

## Conclusions - Future Directions

---

### Thesis summary

- Thorough analysis of single, multi-robot graphSLAM strategies |
- Framework for single, multi-robot graphSLAM - offline or
    real-time |
- Communication scheme for multi-robot real-time experiment |
- Multiple tests in simulations and real-time |


note:
Tell the details as described in thesis text

---

### Future Directions

<div class="fragment">
- Integration of 3rd party optimization framework |
  - g2o |
  - iSAM, iSAM2 |
  - SE-Sync |
</div>

<div class="fragment">
- Adaptive node registration decider / Node Reduction Scheme
</div>


<div class="fragment">
- Implement deciders based on Visual sensors (Monocular, Stereo images)
</div>

<div class="fragment">
- Add full support for 3D Slam
</div>

<div class="fragment">
- Support for active exploration
</div>

<!--- --------- End of Conclusions - Future Directions-->

---

### Acknowledgements

<div style="color:gray; font-size:2em; font-size: 0.5em">
Prof. Kostas J. Kyriakopoulos<br>
Dr. George Karras
</div>

<!--- --------- End of Acknowledgements-->

---

### Thanks for your attention
:smile:

## Q&A
