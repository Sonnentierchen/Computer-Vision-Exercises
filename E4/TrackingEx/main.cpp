#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <map>
#include "Particle.h"
#include "MotionModel.h"
#include "ObservationModel.h"


/**
* \brief  Resample particles using their weights
* \param      particles vector of particles for resampling
* \param      numberOfParticles the number of particles the should be resampled
* \param      engine pseudo random number engine to be used for sampling
* \return     a vector of resampled particles; its length is 'numberOfParticles'
*/
std::vector<Particle> resampleParticles(std::vector<Particle>& particles, int numberOfParticles, std::mt19937 engine)
{
    // IMPLEMENT
    return particles;
}

/**
* \brief  Proecess the frame using the particle filter. The function will also display several images and draw some results.
* \param      frame  current rgb-frame; mainly used to display results
* \param      frameLab current frame in Lab color space; used to calculate histograms for the observation model
* \param      particles  current set of particles; it will be modified according to the next frame
* \param      mm statistical model describing the motion from one frame to the next
* \param      om statistical model describing likelihood of the observation given a certain state
* \param      engine pseudo random number engine to be used in motion model and for resampling
*/
void processFrame(cv::Mat& frame, cv::Mat& frameLab, std::vector<Particle>& particles, MotionModel& mm, ObservationModel* om, std::mt19937& engine )
{
    
    // check if we have an observation model
    if(om!=NULL)	//we do tracking in this case
    {
                
	//calculate weights (the observation likelihood) for particles using the observation model
	//IMPLEMENT

        Particle meanP(0,0,0);
	//calculate mean particle using the weights
	//IMPLEMENT
        
	//resample the particles using their weights
        particles= resampleParticles(particles, particles.size(), engine);
 
        // cutout boundingbox
        cv::Mat cutout(200,200,CV_8UC3);
        cv::resize(meanP.getSubImg(frame),cutout,cv::Size(200,200));
        cv::imshow("subimage", cutout);
       
	// draw bounding box for every n-th particle in red into the RGB-frame
	// IMPLEMENT
	  
	// draw boundingbox of mean particle in blue into the RGB-frame
        // IMPLEMENT
	
	//move the particles according to the motion model
        //IMPLEMENT
    }
    // in case we don't have an observation model and are not tracking we draw the bounding box in the center
    // it will be used to learn the observation model
    else{
      Particle center(frame.cols/2,frame.rows/2, 50 );
      center.draw(frame,cv::Scalar(255,255,0));
    }
    
    //display the images
    cv::imshow("current frame in RGB", frame);
    cv::imshow("current frame in Lab", frameLab);
}

/**
* \brief  Convert an interger to a string 
* \param      number	the number to convert
* \param      minLength the string will be padded with leading zeros intil this length os reached
* \return     string
*/
std::string intToString(int number, int minLength)
{
    std::stringstream ss; //create a stringstream
    ss << number; //add number to the stream
    std::string out = ss.str();
    while((int)out.length() < minLength) out = "0" + out;
    return out; //return a string with the contents of the stream
}


/**
* \brief  perform tracking on a recorded sequence of images
* \param      seq the path to the sequence
*/
void trackSequence(std::string seq)
{
    // this vector will hold the particles
    std::vector <Particle> particles;
    int numOfP= 250;	// The number of particles to use
    MotionModel mm(10,2); // init the motion model
    ObservationModel* om  = NULL;	// we initialize the observation model with NULL beacause we are not tracking yet.
    std::mt19937 engine; // engine for pseudo random numbers
    // this vector will hold the frames
    std::vector<cv::Mat> rgbs;
    //load all images
    for(int i=0; i<500; i++)
    {
        cv::Mat img;
        img= cv::imread(seq+"/"+ intToString(i,3)+".jpg");
        rgbs.push_back(img);
    }
    
    long ticks= cv::getTickCount();
    //iterate over all images
    for(int i=0; i<500; i++)
    {
        // the rgb image an a Lab version
        cv::Mat frame, frameLab;
        frame= rgbs[i];
        //convert the frame to Lab space
        cv::cvtColor(frame, frameLab, CV_BGR2Lab);
	
	
	processFrame(frame,frameLab,particles,mm,om,engine);
	cv::waitKey(om==NULL? 0:1);
	
        // if there is no observation model -> learn one and start tracking
        if(om==NULL)
        {
            // A bounding box in the center -> It will be used to learn the observation model.
            Particle center(frame.cols/2,frame.rows/2, 50 );
            std::cout <<"start tracking\n";
            //to start tracking we have to train a new observation model from the center region of the frame
            om= new ObservationModel(center.getSubImg(frameLab),50.0);
            //we reinitialize all particles at the center location
            particles.clear();
            for(int i=0; i<numOfP; i++)
                particles.push_back(center);
        }
        
	
        
    }
   
}

/**
* \brief  perform live tracking using a camera
*/
void liveTracking()
{
    std::mt19937 engine; // engine for pseudo random numbers
    int numOfP= 250;	// The number of particles to use
    std::vector <Particle> particles;
    MotionModel mm(10,1); // init the motion model
    ObservationModel* om  = NULL;	// we initialize the observation model with NULL beacause we are not tracking yet.
    
    cv::VideoCapture cap;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
        return;
    
    // init frame counter
    int c=0;
    
    
    long ticks=cv::getTickCount(); //init tick counter
    
    // main loop
    for(;;)
    {
        // the rgb image an a Lab version
        cv::Mat frame, frameLab;
        // capture the image from the camera
        cap >> frame;
	
        if( frame.empty() ) break; // end of video stream
        
        //convert the frame to Lab space
        cv::cvtColor(frame, frameLab, CV_BGR2Lab);
	
        processFrame(frame,frameLab,particles,mm,om,engine);
        
	// wait one millisec for input
        int key= cv::waitKey(1);
        if(key == 27 ) break; // stop capturing by pressing ESC
        // if a key was pressed -> we switch wetween tracking and not tracking
        if(key != -1)
        {
            if(om!=NULL)
            {
                std::cout <<"\nstop tracking\n";
                delete om;
                om=NULL;
            }
            else
            {
                // A bounding box in the center -> It will be used to learn the observation model.
                Particle center(frame.cols/2,frame.rows/2, 50 );
                std::cout <<"\nstart tracking\n";
                //to start tracking we have to train a new observation model from the center region of the frame, we use lambda=50
                om= new ObservationModel(center.getSubImg(frameLab),50.0);
                //we reinitialize all particles at the center location
                particles.clear();
                for(int i=0; i<numOfP; i++)
                    particles.push_back(center);
            }
        }
        
        if(c%10==0){	//output fps every 10 frames
	  double time=(cv::getTickCount()-ticks)/cv::getTickFrequency();
	  ticks=cv::getTickCount();
	  
	  for(int j=0; j<50; j++)   std::cout<<"\b";
	  std::cout<< "fps:"<<((double)(10.0)/time );
	  std::cout <<std::flush;
	}
	c++; //increase frame counter
    }
    return;
}


int main(int argc, char** argv)
{
    if(argc<2) liveTracking(); // if no argument is provided start live tracking
    else trackSequence(argv[1]); // do tracking in a recorded sequence
    return 0;
}