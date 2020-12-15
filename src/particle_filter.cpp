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

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::discrete_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    /**
    * TODO: Set the number of particles. Initialize all particles to
    *   first position (based on estimates of x, y, theta and their uncertainties
    *   from GPS) and all weights to 1.
    * TODO: Add random Gaussian noise to each particle.
    * NOTE: Consult particle_filter.h for more information about this method
    *   (and others in this file).
    */
    

    num_particles = 21;  // TODO: Set the number of particles
    // set particle positions sampling from Gaussian dist
    // different normal dist per each variable (x,y, theta)
    std::default_random_engine gen;
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    
    for (int i=0; i<num_particles; i++) {
        Particle part;
        //sample variables from normal dist
        part.x = dist_x(gen);
        part.y = dist_y(gen);
        part.theta = dist_theta(gen);
        part.weight = 1.0;
        //
        particles.push_back(part);
    }
    
    is_initialized = true;
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

    std::default_random_engine gen;
    
    for (int i=0; i<num_particles; i++) {
        if (fabs(yaw_rate) < 0.000001) {
            particles[i].x += velocity * delta_t * cos( particles[i].theta );
            particles[i].y += velocity * delta_t * sin( particles[i].theta );
        } else {
            particles[i].x = particles[i].x + velocity/yaw_rate * ( sin(particles[i].theta + yaw_rate*delta_t ) - sin(particles[i].theta) );
            particles[i].y = particles[i].y + velocity/yaw_rate * ( cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t) );
            particles[i].theta = particles[i].theta + yaw_rate * delta_t;
        }
        //add noise by taking a sample from normal dist with mean as variable value and std as given by respective function parameter
        normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
        normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
        normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);
        //
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
        
    //for each "predicted" landmark (== landmark position "measured" with lidar/rader etc.) find closest data point on map ("observations" vector)
    for (int i =0; i<observations.size(); i++) {
        //find closest using nearest neighbour
        int min_id= -1;
        double min_dist= -1;
        for (int j=0;j<predicted.size();j++) {
            //calculate distance
            double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
            //update if smaller than min distance so far (or if first element)
            if (distance < min_dist || min_id == -1) {
                min_dist = distance;
                min_id = predicted[j].id; //j; //
            }
        }
        //write the closest landmark id into predicted object
        observations[i].id = min_id;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
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
    weights.clear();
    for (int i = 0; i < num_particles; i++) {
        vector<LandmarkObs> map_landmark_in_range;
        //1. select only those map observations that are within particle range (according to sensor_range (using euclidean dist)
        for (int j=0; j< map_landmarks.landmark_list.size(); j++) {
            double distance = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);
            if (distance <= sensor_range ) {
                LandmarkObs landmark_in_range;
                landmark_in_range.id = map_landmarks.landmark_list[j].id_i;
                landmark_in_range.x = map_landmarks.landmark_list[j].x_f;
                landmark_in_range.y = map_landmarks.landmark_list[j].y_f;
                map_landmark_in_range.push_back(landmark_in_range);
            }
        }

        //2. calculate landmark observation from particle viewpoint
        vector<LandmarkObs> observations_particle;
        //vector<LandmarkObs> observations_particle;
        //convert every observation from car coord to map coord aligning with particle map coords
        for (int j=0;j< observations.size(); j++) {
            double *coords = car2map(observations[j].x, observations[j].y, particles[i].x, particles[i].y, particles[i].theta);
            LandmarkObs predicted_landmark_obs;
            predicted_landmark_obs.id = observations[j].id;
            predicted_landmark_obs.x = coords[0];
            predicted_landmark_obs.y = coords[1];
            observations_particle.push_back(predicted_landmark_obs);
        }

        //3. make data association between particle observation coords and real map landmarks (sets landmark IDs inside "observations_particle" structs
        dataAssociation(map_landmark_in_range, observations_particle);

        //4. update particle probability weight based on difference between the calculated landmark position from particle point of view and actual (associated) map landmark position
        double partial_weight;
        double final_weight=1.0;
        for (int j=0;j< observations_particle.size(); j++) { //calculate partial weight for every detected landmark
            LandmarkObs actual_landmark; //= map_landmark_in_range[observations_particle[j].id];
            for (unsigned int k = 0; k < map_landmark_in_range.size(); k++){ //++k
              if (map_landmark_in_range[k].id == observations_particle[j].id){
                  actual_landmark = map_landmark_in_range[k];
                  break;
              }
            }
//            LandmarkObs actual_landmark = map_landmark_in_range[observations_particle[j].id]; //optmisation to store index rather than scrolling through all IDs intestingly doesnt change the execution time almost at all...
            partial_weight = multiv_prob(observations_particle[j].x, observations_particle[j].y,
                                         actual_landmark.x, actual_landmark.y,
                                         std_landmark[0], std_landmark[1]);
            
            if (partial_weight == 0.0) { //add protection to avoid exact zero and replace with some small value instead
                final_weight = 0.000001;
            } else {
                final_weight *= partial_weight;
            }
        }
        particles[i].weight = final_weight;
        weights.push_back(final_weight); //add weights to single vector for easier resampling later
    }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

    //sample particles using discreet distribution with probabilities as indicated by weights
    std::default_random_engine gen;
    discrete_distribution<int> distr(weights.begin(), weights.end());
    std::vector<Particle> new_particles;
    for (int i =0; i< particles.size(); i++) {
        int idx = distr(gen); //sample index from distribution
        Particle new_particle = particles[idx];
        new_particles.push_back(new_particle);
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

//some helper functions

/**
 Calculate map coordinate of observation using: map coords of a particle; and car coordinates of the observation to convert
 x_observation x coord in car coordinates
 y_observation y coord in car coordinates
 x_particle x coord of particle in map coordinates
 y_particle y coord of particle in map coordinates
 theta heading of particle
 */
double* ParticleFilter::car2map(double x_observation, double y_observation, double x_particle, double y_particle, double theta) {
    double* coords = new double[2];
    // transform to map x coordinate
    coords[0] = x_particle + (cos(theta) * x_observation) - (sin(theta) * y_observation);
    // transform to map y coordinate
    coords[1] = y_particle + (sin(theta) * x_observation) + (cos(theta) * y_observation);
    return coords;
}

//                   double mu_x, double mu_y) {
/**
 Calculate probability for observation coords (given observation coords and real coords; and std for probability distribution)
 x_observation x coord of landmark coming fomr measurement (converted to map coordinates)
 y_observation y coord of landmark coming fomr measurement (converted to map coordinates)
 x_actual actual x position of the landmark
 y_actual actual y position of the landmark
 std_x standard deviation in measurement of x coord
 std_y standard deviation in measurement of y coord
 */
double ParticleFilter::multiv_prob(double x_observation, double y_observation,
                   double x_actual, double y_actual,
                   double std_x, double std_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * std_x * std_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_observation - x_actual, 2) / (2 * pow(std_x, 2)))
               + (pow(y_observation - y_actual, 2) / (2 * pow(std_y, 2)));
  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);
  return weight;
}
