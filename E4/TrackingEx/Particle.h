#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

class Particle{
public:
  
/**
* \brief  particle constructor
* \param      x_ the initial x-position of the window
* \param      y_ the initial y-position of the window
* \param      size_ the initial size of the window
*/
  Particle(double x_,double y_,double size_);
  
/**
* \brief  draw the window into an image
* \param      image image to draw in    
* \param      color color of borders
*/
  void draw(cv::Mat& image, cv::Scalar color);
  
  /**
* \brief  crop the content of the window
* \param      image to cut out    
* \return     a cropped version
*/
  cv::Mat getSubImg(cv::Mat& image);
  
// x,y, and size describe the state; weight holds the weight of the particle  
  double x,y,size, weight;
};