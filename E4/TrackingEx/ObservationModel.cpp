#include "ObservationModel.h"
#include <math.h>

  void calculateHist(cv::Mat& img, cv::Mat& hist){
    int histSize[3];
    float hranges[2];
    const float* ranges[3];
    int channels[3];
    // Prepare arguments for a color histogram
    histSize[0] = 5; histSize[1] = histSize[2] = 25;
    hranges[0] = 0.0; // BRG range
    hranges[1] = 255.0;
    ranges[0] = hranges; // all channels have the same range
    ranges[1] = hranges;
    ranges[2] = hranges;
    channels[0] = 0; // the three channels
    channels[1] = 1;
    channels[2] = 2;
    
    cv::calcHist(&img,
	1, // histogram of 1 image only
	channels, // the channel used
	cv::Mat(), // no mask is used
	hist, // the resulting histogram
	3, // it is a 3D histogram
	histSize, // number of bins
	ranges // pixel value range
    );
  }

  ObservationModel::ObservationModel(cv::Mat img, double lambda_){
      calculateHist(img, this->hist);
      cv::normalize(this->hist, this->hist, 1, 0, cv::NORM_L1);
      this->lambda = lambda_;
  }
  
  double ObservationModel::likelihood(cv::Mat img, Particle p){
      cv::Mat currentFrame = p.getSubImg(img);
      cv::Mat currentHist;
      calculateHist(currentFrame, currentHist);

      double distance = cv::compareHist(currentHist, this->hist, CV_COMP_BHATTACHARYYA);
    	
      return  exp(- this->lambda * distance);
  }

