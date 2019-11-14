# Particle Filters in C++

You know what particle filter is and how it works. In this lesson, you learn some important implementation details. Afterwards you will be able to understand how to implement a particle filter in the context of C++.

## Pseudocode

To implement a particle filter we start with the flowchart (see below), which represent the steps of the particle filter algorithm as well as its inputs.
<p align="right"> <img src="./img/1.jpg" style="right;" alt=" Pseudocode" width="600" height="400"> </p> 


Psuedo Code

This is an outline of steps you will need to take with the code in order to implement a particle filter for localizing an autonomous vehicle. The pseudo code steps correspond to the steps in the algorithm flow chart (see figure above), initialization, prediction, particle weight updates, and resampling. Python implementation of these steps was covered in this [repository](https://github.com/A2Amir/Particle-Filters).
 
1. At the initialization step (see above) we estimate our position from GPS input. The subsequent steps in the process will refine this estimate to localize our vehicle.

 <p align="right"> <img src="./img/2.jpg" style="right;" alt=" Pseudocode At the initialization step" width="600" height="400"> </p> 

2. During the prediction step we add the control input (yaw rate & velocity) for all particles
 
 <p align="right"> <img src="./img/3.jpg" style="right;" alt=" Pseudocode the prediction step" width="600" height="400"> </p> 

3. During the update step, we update our particle weights using map landmark positions and feature measurements.
 
 <p align="right"> <img src="./img/4.jpg" style="right;" alt=" Pseudocode the update step" width="600" height="400"> </p> 

4. During resampling we will resample M times (M is range of 0 to length_of_particleArray) drawing a particle i (i is the particle index) proportional to its weight. one implementation of this is covered in this [repository](https://github.com/A2Amir/Particle-Filters).

 <p align="right"> <img src="./img/5.jpg" style="right;" alt=" Pseudocode the  resampling step" width="600" height="400"> </p> 

 
5. The new set of particles represents the Bayes filter posterior probability. We now have a refined estimate of the vehicles position based on input evidence.

 <p align="right"> <img src="./img/6.jpg" style="right;" alt=" Pseudocode The new set of particles" width="600" height="400"> </p> 

## Initialization

