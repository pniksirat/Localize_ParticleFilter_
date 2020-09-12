/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 * 			Parna N
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

using namespace std;

#include "helper_functions.h"

using std::string;
using std::vector;
std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 300;  // TODO: Set the number of particles
 
  std::normal_distribution<double> dist_x(x,std[0]);
  std::normal_distribution<double> dist_y(y,std[1]);
  std::normal_distribution<double> dist_theta(theta,std[2]);
  
  
  
  for (int i; i<num_particles; ++i){
    
  	Particle particle;
    std::vector<int> associat;
  	std::vector<double> s_x;
 	std::vector<double> s_y;
    s_x={};
    s_y={};
    
    particle.id=i;
    /**create the particle with a random Gaussian noise  */
    particle.x=dist_x(gen);
    particle.y=dist_y(gen);
    particle.theta=dist_theta(gen);
    /**initialize the weight to one*/
    particle.weight=1.0;
   // particle.associations=associat;
   // particle.sense_x=s_x;
    //particle.sense_y=s_y;
    
    particles.push_back(particle);
    weights.push_back(particle.weight);
     
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
  
 
  
  for (int i=0; i<num_particles; ++i){
    
    /**adding the measurement to each particle (motion model) */
    if (fabs(yaw_rate)>0.001){
    	
  		particles[i].x=particles[i].x + velocity/yaw_rate*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
    	particles[i].y=particles[i].y + velocity/yaw_rate*(cos(particles[i].theta)-cos(particles[i].theta+delta_t*yaw_rate));
    	particles[i].theta=particles[i].theta+yaw_rate*delta_t;
    }
    else{
    	particles[i].x+=velocity*delta_t*cos(particles[i].theta);
    	particles[i].y+=velocity*delta_t*sin(particles[i].theta);
    }
    
    /**adding the random Gaussian noise to each particle */
    	std::normal_distribution<double> noise_x(particles[i].x,std_pos[0]);
  		std::normal_distribution<double> noise_y(particles[i].y,std_pos[1]);
 		std::normal_distribution<double> noise_theta(particles[i].theta,std_pos[2]);
    	particles[i].x=noise_x(gen);
   		particles[i].y=noise_y(gen);
   		particles[i].theta=noise_theta(gen);
    	
    	//cout<<"Prediction_i"<<particles[i].x<<endl;
 		//cout<<"Prediction_i "<<particles[i].y<<endl;
 		//cout<<"Prediction_i "<<particles[i].weight<<endl;
 
 
 // cout<<"Prediction"<<particles[200].x<<endl;
 // cout<<"Prediction"<<particles[200].y<<endl;
 // cout<<"Prediction"<<particles[200].weight<<endl;
    
      /**
 
  for (Particle &particle: particles){
  		particle.x=particle.x + velocity/yaw_rate*(sin(particle.theta+yaw_rate*delta_t)-sin(particle.theta));
    	
    	particle.y=particle.y + velocity/yaw_rate*(cos(particle.theta)-cos(particle.theta+delta_t*yaw_rate));
    	particle.theta=particle.theta+yaw_rate*delta_t;
    	cout<<"here"<<endl;
    
    	std::normal_distribution<double> noise_x(particle.x,std_pos[0]);
  		std::normal_distribution<double> noise_y(particle.y,std_pos[1]);
 		std::normal_distribution<double> noise_theta(particle.theta,std_pos[2]);
    	particle.x=noise_x(gen);
   		particle.y=noise_y(gen);
   		particle.theta=noise_theta(gen);
    	cout<<"Prediction_i"<<particle.x<<endl;
  		cout<<"Prediction_i"<<particle.y<<endl;
  		cout<<"Prediction_i"<<particle.weight<<endl;
   */    	
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
}
std::vector<int> ParticleFilter::dataAssociation_(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  vector<int> associat_id;
  for (auto obs:observations){
  		vector<LandmarkObs> nearest_mark;
    	double min_dist=100000000;
    	int L_count;
    	int L_id=-1;
  		for (auto L_mark:predicted){
        	double nearestN=dist(obs.x,obs.y,L_mark.x,L_mark.y);
          	L_count+=1;
          	if (nearestN<min_dist){
            	min_dist=nearestN;
              	//nearest_mark=L_mark;
              	L_id=L_count;
            }
        
        }
    	associat_id.push_back(L_id);
  
  }

  return associat_id;
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
  
  
  for (int i=0; i<num_particles; ++i){
    //vector<LandmarkObs> observations_temp
    double Probabilty=1;
    /**for each observation convert the local measurement to global find the closest  landmark neighbour *****/
    for (auto obs: observations){
  		/**Vehicle coordinate system to map coordinate system for each particle */
  		 double X_map = particles[i].x + (cos(particles[i].theta) * obs.x) - (sin(particles[i].theta) * obs.y);
      	 double Y_map = particles[i].y + (sin(particles[i].theta) * obs.x) + (cos(particles[i].theta) * obs.y);

      	// for each observation find the closest landmark and estimate the probability
      	Map::single_landmark_s landmark_closest;
      	double mindist=sensor_range*sensor_range;
      	//find the most likely landmark for the observation from all landmarks
        for (auto m_landmarks : map_landmarks.landmark_list){
          //double check_range=sqrt((m_landmarks.x_f - particles[i].x)*(m_landmarks.x_f -particles[i].x) + (m_landmarks.y_f -particles[i].y )*(m_landmarks.y_f - particles[i].y ));
          double check_range=dist(m_landmarks.x_f, m_landmarks.y_f, particles[i].x,particles[i].y);
          if(check_range<=sensor_range){
            //check the observation distance from the landmark
            double dist = sqrt((m_landmarks.x_f - X_map)*(m_landmarks.x_f - X_map) + (m_landmarks.y_f - Y_map )*(m_landmarks.y_f - Y_map ));
    		if (dist < mindist) {
        		mindist = dist;
              	// the landmark most resembles to the observation
        		landmark_closest = m_landmarks;
    		}
           
          }  
          
        }
        
       double W=multiv_prob(std_landmark[0], std_landmark[1], X_map, Y_map, landmark_closest.x_f, landmark_closest.y_f);
      	  if (W==0){
  				Probabilty*=0.00001;
            	
          }
      	  else{
          	 Probabilty*=W;
          }
        
        
      	
    }
  //set the particle wight after estimation of probability of all observations with the colsest landmark  
  
  particles[i].weight=Probabilty;
  weights[i]=Probabilty;
  //cout<<i<<" Weights****"<<Probabilty<<endl;
  }

}


void ParticleFilter::resample_wheel(){


  std::vector<Particle> P;
  std::vector<double> W_temp;
  Particle P_temp;
  double max_weight=*max_element(weights.begin(), weights.end());
  
  uniform_real_distribution<double> beta_gen(0,max_weight*2);
  uniform_int_distribution<int> index_gen(1, num_particles - 1);
  
  //index between 1-N
  //int index=int(rand() %num_particles+1); 
  int index=index_gen(gen);
  
 // auto max_weight=max(weights.begin(), weights.end());
  double beta=0;
  for (int i=0; i<num_particles; ++i){
    
    //beta+=rand()*2* max_weight;
    beta+=beta_gen(gen);
    //cout<<"beta,*****"<<beta<<endl;
    //cout<<"max W,*****"<<max_weight<<endl;
    while(beta>weights[index]){
    	beta=beta-weights[index];
      	index= (index+1)%num_particles;
      	
    }
    P_temp=particles[index];
    P_temp.id=i;
    //cout<<"resampling*****"<<endl;
    //cout<<"X:"<<particles[index].x<<"w:"<<particles[index].weight<<endl;
    
    P.push_back(P_temp);
    W_temp.push_back(particles[index].weight);
      
  
  }
  // reset the partickes and W by resampled P

  weights=W_temp;
  particles=P;
  //cout<<"resampling x: "<<particles[num_particles-1].x<<endl;

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
 
     std::vector<Particle> resampled_particles(num_particles);

     /* std::discrete_distribution produces random integers on the interval [0, n), 
     where the probability of each individual integer i is defined as w i/S, that is 
     the weight of the ith integer divided by the sum of all n weights.*/
     std::discrete_distribution<size_t> dist_W_index(weights.begin(), weights.end());

     //resample particles with respect to their weight 
     for (int i = 0; i < num_particles; i++) {
         resampled_particles[i] = particles[dist_W_index(gen)];
     }

     
     particles = resampled_particles;
  
 
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