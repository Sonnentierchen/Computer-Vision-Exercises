#include "Particle.h"

Particle::Particle(double x_,double y_, double size_){
  x=x_;
  y=y_;
  size=size_;
  weight=1;
  
}

void Particle::draw(cv::Mat& image, cv::Scalar color){
  cv::rectangle(image,cv::Point2d(x-size,y-size),cv::Point2d(x+size,y+size),color);
}

cv::Mat Particle::getSubImg(cv::Mat& image){
  int xmine=std::min( std::max(0,(int)(x-size)), image.cols-1);
  int ymine=std::min( std::max(0,(int)(y-size)), image.rows-1);
  int xmaxe=std::max( std::min(image.cols-1,(int)(x+size)), 0);
  int ymaxe=std::max( std::min(image.rows-1,(int)(y+size)), 0);

  if((xmaxe-xmine)*(ymaxe-ymine)<10){
    cv::Mat retImg(10,10,CV_8UC3);
    retImg.setTo(cv::Scalar(255,255,255));
    return retImg;
  }
  cv::Mat subimg(image,cv::Rect(cv::Point2d(xmine,ymine),cv::Point2d(xmaxe,ymaxe) ) );
  return subimg;
}
  



