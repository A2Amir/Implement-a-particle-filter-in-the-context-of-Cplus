/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include "assert.h"

#include "helper_functions.h"

using std::string;
using std::vector;
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles
  normal_distribution<double> dist_x(x,std[0]);
  normal_distribution<double> dist_y(y,std[1]);
  normal_distribution<double> dist_theta(theta,std[2]);
  random_device rd;
  default_random_engine gen(rd());
  for (int i = 0; i < num_particles; i++)
  {
    Particle currentParticle;
    currentParticle.id=i;
    currentParticle.x=dist_x(gen);
    currentParticle.y=dist_y(gen);
    currentParticle.theta=dist_theta(gen);
    currentParticle.weight=1.0;

    particles.push_back(currentParticle);
    weights.push_back(currentParticle.weight);

  }
  is_initialized=true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

   for (int i = 0; i < num_particles; i++)
   {
        double particle_x=particles[i].x;
        double particle_y=particles[i].y;
        double particle_theta=particles[i].theta;
        double pred_x;
        double pred_y;
        double pred_theta;
        if (fabs(yaw_rate)<0.001)
        {
            pred_x=particle_x+velocity*cos(particle_theta)*delta_t;
            pred_y=particle_y+velocity*sin(particle_theta)*delta_t;
            pred_theta=particle_theta;
        }else
        {
            pred_x=particle_x+(velocity/yaw_rate)*(sin(particle_theta+(yaw_rate*delta_t))-sin(particle_theta));
            pred_y=particle_y+(velocity/yaw_rate)*(cos(particle_theta)-cos(particle_theta+ (yaw_rate*delta_t)));
            pred_theta=particle_theta+(yaw_rate*delta_t);

        }
        normal_distribution<double> dist_x(pred_x,std_pos[0]);
        normal_distribution<double> dist_y(pred_y,std_pos[1]);
        normal_distribution<double> dist_theta(pred_theta,std_pos[2]);

        random_device rd;
        default_random_engine gen(rd());
        // Set particle state
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);


   }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations)
{
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */
   for (LandmarkObs &observation : observations)
   {
       double minDist = numeric_limits<double>::max();

       for (LandmarkObs &prediction : predicted)
        {
            double xDistance = observation.x - prediction.x;
            double yDistance = observation.y - prediction.y;

            double distance = xDistance * xDistance + yDistance * yDistance;

            if (distance < minDist)
            {
                minDist = distance;
                observation.id = prediction.id;
            }
        }
   }

}

std::vector<LandmarkObs> ParticleFilter::FindnearbyLandmarks(double x, double y, double range, std::vector<Map::single_landmark_s> landmarks)
{
    std::vector<LandmarkObs> result;
    for (Map::single_landmark_s landmark : landmarks)
    {
        double xDelta = x - landmark.x_f;
        double yDelta = y - landmark.y_f;

        if (xDelta * xDelta + yDelta * yDelta <= range * range)
        {
            result.push_back(LandmarkObs{landmark.id_i, landmark.x_f, landmark.y_f});
        }
    }
    return result;
}

std::vector<LandmarkObs> ParticleFilter::transform(std::vector<LandmarkObs> observations, double x, double y, double theta)
{
    vector<LandmarkObs> result;
    for (LandmarkObs observation: observations) {
        double xNew = cos(theta) * observation.x - sin(theta) * observation.y + x;
        double yNew = sin(theta) * observation.x + cos(theta) * observation.y + y;
        result.push_back({observation.id, xNew, yNew});
    }
    return result;
}

double
ParticleFilter::MultivariateGaussian(const LandmarkObs &prediction, const LandmarkObs &observation,double std_landmark[])
{
    // Re-calculate weight of the particle
    const double sig_x = std_landmark[0];
    const double sig_y = std_landmark[1];

    // Make sure the std dev doesn't change
    assert (sig_x == std_landmark[0]);
    assert (sig_y == std_landmark[1]);

    // calculate normalization term
    const double gauss_norm = (1 / (2 * M_PI * sig_x * sig_y));

    double exponent =
            ((observation.x - prediction.x) * (observation.x - prediction.x)) / (2 * sig_x * sig_x) +
            ((observation.y - prediction.y) * (observation.y - prediction.y)) / (2 * sig_y * sig_y);

    return gauss_norm * exp(-exponent);
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
    // Clear weights
    weights.clear();



    // Go over all particles
    for (Particle &particle : particles) {

        std::vector<LandmarkObs> predictions = FindnearbyLandmarks(particle.x, particle.y, sensor_range,
                                                                    map.landmark_list);
        /*
                     Step1, transformation from OBSx to TOBSx.
                            The observations are given in the VEHICLE'S coordinate system.
                            These codes would first convert the VEHICLE's coordinate
                            obs to the MAP's coord.

         */
        std::vector<LandmarkObs> transposedObservations = transform(observations, particle.x, particle.y,
                                                                    particle.theta);

        // Associate landmarks and observations
        dataAssociation(predictions, transposedObservations);

        // Re-calculate weight of the particle
        particle.weight = 1;
        for (LandmarkObs &observation : transposedObservations)
        {

            // Find the prediction for this observation
            auto it = std::find_if(predictions.begin(), predictions.end(),[&observation](const LandmarkObs &prediction)
                        {return prediction.id == observation.id;});

            // If prediction is found, calculate Multivariate-Gaussian Probability and multiply with previous
            if (it != predictions.end()) {
                particle.weight *= MultivariateGaussian(*it, observation, std_landmark);
            }
        }

        // Update weights vector for re-sampling
        weights.push_back(particle.weight);
    }

    assert(weights.size() == particles.size());

}


void ParticleFilter::resample()
{
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
   default_random_engine gen;

    vector<Particle> new_particles;

    vector<double> weights;
    for(int i=0; i<num_particles; i++)
    {
      weights.push_back(particles[i].weight);
    }
    std::uniform_real_distribution<double> unirealdist(0.0,1.0);
    int index   = int(unirealdist(gen) * num_particles);
    double beta = 0.0;
    double mw   = *max_element(weights.begin(), weights.end());
    for(int i=0; i<num_particles; i++)
    {
      beta += unirealdist(gen) * 2.0 * mw;
      while(beta > weights[index])
      {
        beta -= weights[index];
        index = (index+1) % num_particles;
      }
      new_particles.push_back(particles[index]);
    }
    particles = new_particles;
}



void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
