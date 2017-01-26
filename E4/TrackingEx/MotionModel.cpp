
#include "MotionModel.h"
  MotionModel::MotionModel(double stdXY_, double stdSize_){
    // IMPLEMENT
    // create nomral distributions using std::normal_distribution<double> class for the motion
    // in x, y, and the change of the window size
    // store the distributions
  }
  
  
  Particle MotionModel::move(Particle p, std::mt19937& engine){
        Particle result(0,0,0);
	// IMPLEMENT
	// move the particle randomly according to the motion model defined by the stored distributions
	// return the particle
	return result;
  }