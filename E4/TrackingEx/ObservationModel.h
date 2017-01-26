#pragma once
#include "Particle.h"
#include <random>

class ObservationModel{

public:
  
/**
* \brief  the constructor
* \param      img image used to learn the observation model
*/
  ObservationModel(cv::Mat img, double lambda_);
  
/**
* \brief  calculate the likelihood of a particle
* \param      p the particle we want to calculate the likelihood for
* \return the likelihood
*/
  double likelihood(cv::Mat img, Particle p);

};