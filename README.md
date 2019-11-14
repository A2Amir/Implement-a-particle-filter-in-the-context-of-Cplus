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

The first thing you need to do to implement a particle filter is initialize all your particles. You have to decide how many particles you want to use. This parameter is mostly decided empirically but theoretically, the particle filter will exactly represent the Bayesian posterior distribution as the number of particles approaches infinity. If you have too few particles, you will not have enough to cover all of the high likelihood positions, so you might miss the correct position.
For instance, if each cell in the below presented  grid is equally likely to be the car’s position, and you only have three particles.
 
 <p align="right"> <img src="./img/7.jpg" style="right;" alt="  three particles" width="600" height="400"> </p> 


you would not have enough particles right now to represent this However, having too many particles will slow down your filter, and prevent it from localizing a self-driving car in real time.

There are two main ways you can initialize your particles:
* The first approach is to sample uniformly across the state space. For instance, you could divide the state space into a grid and put one particle in each cell. However, this approach is not very practical if the state space is too large, such as in the case of a self-driving car, where the state space spans essentially the entire land surface of the earth.

  <p align="right"> <img src="./img/8.jpg" style="right;" alt="  The first approach" width="600" height="400"> </p> 



* The second approach for initialization is to sample around some kind of initial estimate.For a self-driving car, this initial estimate can come from a global positioning system, or GPS but you cannot rely on GPS alone for localization, because of its low accuracy and reduced availability in certain environments.

   <p align="right"> <img src="./img/9.jpg" style="right;" alt="  The second approach" width="600" height="400"> </p> 

However, GPS can be very useful to provide initial rough estimate of your location.

## Implementation of Initialization


