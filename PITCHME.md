
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

<table>
<tr>
<td align="center" valign="top"><img alt="Neato" src="/assets/figures/bulk/neato.jpg"></td>
<td align="center" valign="top"><img alt="roomba" src="/assets/figures/bulk/roomba.jpg"></td>
<td rowspan="2" valign="middle">
<img alt="BSD3 License" src="http://img.shields.io/badge/license-BSD3-brightgreen.svg">
<br>
<img alt="[Join the chat at https://gitter.im/MRPT/mrpt" src="https://badges.gitter.im/Join%20Chat.svg">
</td>
</tr>
<tr>
<td colspan="2" align="center">CI: GNU/Linux & OSX</td>
<td align="center">CI: Windows</td>
</tr>
</table>



---

<!--- --------- End of Intro - What is SLAM?-->


## Review of SLAM Algorithms


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

<!--- --------- End of Review of SLAM Algorithms-->


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


---?image=assets/figures/bulk/map_merger_node_2_robots&size=contain
<!-- .slide: data-background-transition="none" -->
---?image=assets/figures/bulk/map_merger_node_3_robots&size=contain
<!-- .slide: data-background-transition="none" -->

---

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
- Framework for single, multi-robot graphSLAM - offline or real-time |
- Communication scheme for multi-robot real-time experiment |
- Multiple tests in simulations and real-time |


note:
Tell the details as described in thesis text

---

### Future Directions

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

### Acknowledgements

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
