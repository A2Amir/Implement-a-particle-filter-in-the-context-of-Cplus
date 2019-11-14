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

## 1. Initialization

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

### 1.1 Implementation of Initialization

The most practical way to initialize our particles and generate real time output, is to make an initial estimate using GPS input. As with all sensor based operations, this step is impacted by noise.

* Particles shall be implemented by sampling a Gaussian distribution, taking into account Gaussian sensor noise around the initial GPS position and heading estimates.

* Use the [C++ standard library normal distribution](https://en.cppreference.com/w/cpp/numeric/random/normal_distribution) and [C++ standard library random engine](http://www.cplusplus.com/reference/random/default_random_engine/) functions to sample positions around GPS measurements.

A function called ‘‘printSamples’’, which is based on the above criteria, that takes a GPS position (gps_x, gps_y) and an initial heading (theta) as input. The function prints out to the terminal 3 samples from a normal distribution with mean equal to the GPS position and initial heading measurements and standard deviation of 2 m for the x and y position and 0.05 radians for the heading of the car.

~~~c++
#include "iostream"
#include <random>
using std::normal_distribution;
void printSimples(double gps_x,double gps_y,double theta);
 int main(int argc, char const *argv[]) {
     double gps_x=4983;
     double gps_y=5029;
     double theta = 1.201;

     printSimples(gps_x,gps_y,theta);

     return 0;
 }
 void printSimples(double gps_x, double gps_y, double theta)
 {
     std::default_random_engine gen;
      //Set standard deviations for x, y, and theta
      double std_x = 2;
      double std_y = 2;
      double std_theta = 0.05;
        //  Create normal distributions for x, y and theta
     normal_distribution<double> dist_x(gps_x,std_x);
     normal_distribution<double> dist_y(gps_y,std_y);
     normal_distribution<double> dist_theta(theta,std_theta);

     for (int i = 0; i < 3; ++i)
     {
         double sample_x=dist_x(gen);
         double sample_y=dist_y(gen);
         double sample_theta=dist_theta(gen);
         // Print your samples to the terminal.
         std::cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " "
          << sample_theta << std::endl;
      }

 }
~~~
Result:

Sample 1 4982.76 5030.37 1.20266, 
Sample 2 4980.83 5026.85 1.23824,
Sample 3 4983.07 5029.93 1.30723,

## 2. Prediction Step

The next part of the filter needs to implement is the prediction step. For the prediction step, we will use what you learned in the [motion models lesson](https://github.com/A2Amir/Motion-Model-of-a-Car) to predict where the car will be at the next time step.

<p align="right"> <img src="./img/10.jpg" style="right;" alt=" the prediction step" width="600" height="400"> </p> 

 
For each particle, we will have to update the particle’s location based on velocity and yaw rate measurements. To account for the uncertainty in the control input, in the project we will also add Gaussian noise to the velocity and yaw rate.
The equations for updating x, y and the yaw angle when the yaw rate is not equal to zero:
 
<p align="center"> <img src="./img/16.jpg" style="right;" alt=" to update the particle’s location based on velocity and yaw rate measurements" width="400" height="200"> </p> 



If you want to practice calculating the prediction of a car’s position, assuming a basic motion model and disregarding sensor uncertainty, you can check this jupyter notebook.

