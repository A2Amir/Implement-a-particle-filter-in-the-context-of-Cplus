
/**
 * print_samples_sol.cpp
 *
 * SOLUTION CODE
 *
 * Print out to the terminal 3 samples from a normal distribution with
 * mean equal to the GPS position and IMU heading measurements and
 * standard deviation of 2 m for the x and y position and 0.05 radians
 * for the heading of the car.
 *
 * Author: Amir ziaee
 */
#include "iostream"
#include <random>// Need this for sampling from distributions
using std::normal_distribution;

/**
 * Prints samples of x, y and theta from a normal distribution
 * @param gps_x   GPS provided x position
 * @param gps_y   GPS provided y position
 * @param theta   GPS provided yaw
 */
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
