
#include "MotionModel.h"
  MotionModel::MotionModel(double stdXY_, double stdSize_) {
      this->stdXY = stdXY_;
      this->stdSize = stdSize_;
      this->distributionXY = std::normal_distribution<double>(0.0, stdXY_);
      this->distributionSize = std::normal_distribution<double>(0.0, stdSize_);
  }
  
  
  Particle MotionModel::move(Particle p, std::mt19937& engine){
      Particle result(0,0,0);
      result.x = p.x + this->distributionXY(engine);
      result.y = p.y + this->distributionXY(engine);
      result.size = p.size + this->distributionSize(engine);
      result.weight = p.weight;
      return result;
  }
