#pragma once
#include "Particle.h"
#include <random>

class MotionModel{

public:

/**
* \brief  the constructor
* \param      stdXY_ the standard deviation of the movement in xy direction
* \param      stdSize_ the standard deviation the change in size
*/
  MotionModel(double stdXY_, double stdSize_);
  
/**
* \brief  randomly move a particle according to the motion model
* \param      p particle to be moved
* \param      engine pseudo random number engine used for moving particles
* \return return moved particle
*/
  Particle move(Particle p, std::mt19937& engine);

  
  //standard deviations for x- and y-motion, and for the change in size
  double stdXY, stdSize;
  
  // distribution for x- and y-motion
  std::normal_distribution<double> distributionXY;
  
  // distribution for change in size 
  std::normal_distribution<double> distributionSize;
};