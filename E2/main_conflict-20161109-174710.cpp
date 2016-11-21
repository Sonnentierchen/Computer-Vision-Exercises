#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <math.h>
using namespace cv;
using namespace std;

#define DIRECTION_Y 0
#define DIRECTION_X 1

/** the images */
Mat src, blurred, gray;
Mat dst, dst_norm, dst_norm_scaled;
Mat derivativeX;
Mat derivativeY;

int cornerThreshold = 200;
int max_cornerThreshold = 255;

const char* originalImage = "original_image";
const char* cornerImage = "corner_image";

const double matrixSobelX[3][3] = {{1, 0, -1}, {3, 0, -3}, {1, 0, -1}};
const double matrixSobelY[3][3] = {{1, 3, 1}, {0, 0, 0}, {-1, -3, -1}};

int sobelNormalizerX = 0;
int sobelNormalizerY = 0;

/** method preload */
void cornerHarrisDetector(Mat &sourceImage);
void detectCorners(Mat &src, Mat &out, double k);
void sobel(Mat &src, Mat &out, int direction);
void applyFilter(int x, int y, Mat &src, Mat &out, const double (*filter)[3], double normalizer);

int main( int argc, char** argv ) {
    /** read image name and threshold from command line */
    char* filename = argc >= 2 ? argv[1] : (char*)"../data/fruits.jpg";
    src = imread(filename, 1);
    if (argc > 3) {
        cornerThreshold = atoi(argv[2]);
        cornerThreshold = cornerThreshold > max_cornerThreshold ? max_cornerThreshold : cornerThreshold;
    }
    if( src.empty() )
    {
        cout << "Image empty. Usage: corner <image_name>\n";
        return 0;
    }

    dst = Mat::zeros( src.size(), CV_8U );
    derivativeX = Mat::zeros( src.size(), CV_8U );
    derivativeY = Mat::zeros( src.size(), CV_8U );

    cvtColor( src, gray, CV_BGR2GRAY );
    namedWindow(originalImage, CV_WINDOW_AUTOSIZE);
    imshow(originalImage, src);
    cornerHarrisDetector(gray);
    return 0;
}

void cornerHarrisDetector(Mat &sourceImage) {
    /** detector parameters */
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            sobelNormalizerX += abs(matrixSobelX[i][j]);
            sobelNormalizerY += abs(matrixSobelY[i][j]);
        }
    }

    GaussianBlur(sourceImage, blurred, Size(5, 5), 2);
    detectCorners(sourceImage, dst, k);

    /// Normalizing
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );
    namedWindow( cornerImage, CV_WINDOW_AUTOSIZE );
    imshow( cornerImage, dst_norm_scaled );
    waitKey(0);
}

void detectCorners(Mat &src, Mat &out, double k) {
    sobel(src, derivativeX, DIRECTION_Y);
    out = derivativeX;
    //sobel(src, derivativeY, DIRECTION_Y);
/*
    for (int row = 0; row < src.size().height; row++) {
        for (int column = 0; column < src.size().width; column++) {
            uchar _derivativeX = derivativeX.at<uchar>(row, column);
            uchar _derivativeY = derivativeY.at<uchar>(row, column);
            int harrisMatrix[2][2] = {{_derivativeX * _derivativeX, _derivativeX * _derivativeY},
                              {_derivativeX * _derivativeY, _derivativeY * _derivativeY}};
            int determinant = harrisMatrix[0][0] * harrisMatrix[1][1] - harrisMatrix[1][0] * harrisMatrix[0][1];
            int trace = harrisMatrix[0][0] + harrisMatrix[1][1];
            trace *= trace;
            double result = determinant - k * trace;
            out.at<double>(row, column) = result;
        }
    }
    */
}

void sobel(Mat &src, Mat &out, int direction) {
    const double (*filter)[3];
    double normalizer = 0;
    if (direction == DIRECTION_X) {
        filter = matrixSobelX;
        normalizer = (double) 1 / sobelNormalizerX;
    }
    else if (direction == DIRECTION_Y) {
        filter = matrixSobelY;
        normalizer = (double) 1 / sobelNormalizerY;
    }

    Size imageSize = src.size();
    for (int row = 0; row < imageSize.height; row++) {
        for (int column = 0; column < imageSize.width; column++) {
            applyFilter(row, column, src, out, filter, normalizer);
        }
    }
}

void applyFilter(int row, int column, Mat &src, Mat &out, const double (*filter)[3], double normalizer) {
    int neighborRow, neighborColumn;
    double result = 0;
    Size imageSize = src.size();

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (filter[i][j] != 0) {
                neighborRow = row + i - 1;
                neighborColumn = column + j  - 1;
                if(neighborRow >= 0 && neighborRow < imageSize.height && neighborColumn >= 0 && neighborColumn < imageSize.width){
                    uchar pixelValue = src.at<uchar>(neighborRow, neighborColumn);
                    double filterValue = filter[i][j];
                    result += filterValue * pixelValue;
                }
            }
        }
    }

    result *= normalizer;
    result += 128;

    out.at<uchar>(neighborRow, neighborColumn) = result;
}
