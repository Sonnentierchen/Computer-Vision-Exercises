#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>
using namespace cv;
using namespace std;
using namespace chrono;

#define DIRECTION_Y 0
#define DIRECTION_X 1

/** the images */
Mat src, gray, corners;
Mat output, output2, output_normalized, output_scaled, output_suppressed;

int cornerThreshold = 1;

vector<double> matrixSobelX_1({1, 2, 1});
vector<double> matrixSobelX_2({-1, 0, 1});

vector<vector<double>> matrixSobelX({{1, 2, 1}, {0, 0, 0}, {-1, -2, -1}});
vector<vector<double>> matrixSobelY({{1, 0, -1}, {2, 0, -2}, {1, 0, -1}});

vector<vector<double>> gaussian({{1.0 / 16, 1.0 / 8, 1.0 / 16}, {1.0 / 8, 1.0 / 4, 1.0 / 8},
                                 {1.0 / 16, 1.0 / 8, 1.0 / 16}});

void calculateGradient(Mat input, Mat output, int direction);
uchar getGradientAt(Mat input, vector<vector<double>> *filter, int row, int column);
void detectCorners(Mat input, Mat output, Mat gradientX, Mat gradientY);
float horizontalGradient(Mat input, vector<double> *filterH, int row, int column);
float verticalGradient(Mat input, vector<double> *filterV, int row, int column);
void nonMaxSuppression(Mat input, Mat output);

int main( int argc, char** argv ) {
    /** read image name and threshold from command line */
    char* filename = argc >= 2 ? argv[1] : (char*)"../data/fruits.jpg";
    src = imread(filename, 1);
    if( src.empty() )
    {
        cout << "Image empty. Usage: corner <image_name>\n";
        return 0;
    }

    output = Mat::zeros( src.size(), CV_32F );
    output_suppressed = Mat::zeros( src.size(), CV_8U );
    src.copyTo(corners);

    cvtColor( src, gray, CV_BGR2GRAY );
    namedWindow("original_image", CV_WINDOW_AUTOSIZE);
    imshow("original_image", src);

    Mat gradientX = Mat::zeros( src.size(), CV_32F );
    Mat gradientY = Mat::zeros( src.size(), CV_32F );

    calculateGradient(gray, gradientX, 0);
    calculateGradient(gray, gradientY, 1);

    convertScaleAbs(gradientX, gradientX);
    convertScaleAbs(gradientY, gradientY);


    detectCorners(gray, output, gradientX, gradientY);
    normalize(output, output_normalized, 0, 255, NORM_MINMAX, CV_32F, Mat());
    convertScaleAbs(output_normalized, output_scaled);

    nonMaxSuppression(output_scaled, output_suppressed);

    for (int row = 0; row < output_suppressed.size().height; row++) {
        for (int column = 0; column < output_suppressed.size().width; column++) {
            if (output_suppressed.at<uchar>(row, column) > cornerThreshold) {
                circle(corners, Point(column, row), 5, Scalar(100), 2, 8, 0);
            }
        }
    }

    namedWindow("detected_corners", CV_WINDOW_AUTOSIZE);
    imshow("detected_corners", corners);

    waitKey(0);
    return 0;
}

void nonMaxSuppression(Mat input, Mat output) {
    Size size = input.size();
    input.copyTo(output);
    for (int row = 0; row < src.size().height; row++) {
        for (int column = 0; column < src.size().width; column++) {
            for (int rowOffset = -1; rowOffset < 2; rowOffset++) {
                for (int columnOffset = -1; columnOffset < 2; columnOffset++) {
                    int neighborRow = row + rowOffset;
                    int neighborColumn = column + columnOffset;

                    if (neighborRow == row && neighborColumn == column)
                        continue;

                    if (neighborRow >= 0 && neighborRow < size.height
                            && neighborColumn >= 0 && neighborRow < size.width) {
                        float current = input.at<uchar>(row, column);
                        float neighbor = input.at<uchar>(neighborRow, neighborColumn);
                        if (current <= neighbor) {
                            output.at<uchar>(row, column) = 0;
                        }
                    }
                }
            }
        }
    }
}

void detectCorners(Mat input, Mat output, Mat gradientX, Mat gradientY) {
    Size inputSize = input.size();
    float result = 0;
    for (int row = 0; row < inputSize.height; row++) {
        for (int column = 0; column < inputSize.width; column++) {
            float xx = 0;
            float yy = 0;
            float xy = 0;
            for (int rowOffset = -1; rowOffset < 2; rowOffset++) {
                for (int columnOffset = -1; columnOffset < 2; columnOffset++) {
                    int neighborRow = row + rowOffset;
                    int neighborColumn = column + columnOffset;
                    if (neighborRow >= 0 && neighborRow < inputSize.height
                            && neighborColumn >= 0 && neighborColumn < inputSize.width) {
                        uchar x = gradientX.at<uchar>(neighborRow, neighborColumn);
                        uchar y = gradientY.at<uchar>(neighborRow, neighborColumn);
                        xx += x * x * gaussian[rowOffset + 1][columnOffset + 1];
                        yy += y * y * gaussian[rowOffset + 1][columnOffset + 1];
                        xy += x * y * gaussian[rowOffset + 1][columnOffset + 1];
                    }
                }
            }
            xx /= 9;
            yy /= 9;
            xy /= 9;
            float determinant = xx * yy - xy * xy;
            float trace = xx + yy;
            trace *= trace;
            result = determinant - 0.04 * trace;
            output.at<float>(row, column) = result;
        }
    }
}

void calculateGradient(Mat input, Mat output, int direction) {
    vector<vector<double>> *filter;
    if (direction == 0)
        filter = &matrixSobelX;
    if (direction == 1)
        filter = &matrixSobelY;
    Size inputSize = input.size();
    int height = inputSize.height;
    int width = inputSize.width;

    for (int row = 0; row < height; row++) {
        for (int column = 0; column < width; column++) {
            float gradient = getGradientAt(input, filter, row, column);
            output.at<float>(row, column) = gradient;
        }
    }
}

float horizontalGradient(Mat input, vector<double> *filterH, int row, int column) {
    double result = 0;
    int filterOffset = filterH->size() / 2;

    for (int offsetRow = -filterOffset; offsetRow < filterOffset + 1; offsetRow++) {
        int neighborRow = row + offsetRow;
        if (neighborRow >= 0 && neighborRow < input.size().height) {
            Scalar neighborPixel = input.at<float>(neighborRow, column);
            double filterValue = filterH->at(offsetRow + 1);
            double _result = neighborPixel.val[0] * filterValue;
            result += _result;
        }
    }

    if (result < 0)
        result = 0;
    else if (result > 255)
        result = 255;

    return result;
}

float verticalGradient(Mat input, vector<double> *filterV, int row, int column) {
    double result = 0;
    int filterOffset = filterV->size() / 2;

    for (int offsetColumn = -filterOffset; offsetColumn < filterOffset + 1; offsetColumn++) {
        int neighborColumn = column + offsetColumn;
        if (neighborColumn >= 0 && neighborColumn < input.size().height) {
            Scalar neighborPixel = input.at<float>(row, neighborColumn);
            double filterValue = filterV->at(offsetColumn + 1);
            double _result = neighborPixel.val[0] * filterValue;
            result += _result;
        }
    }

    if (result < 0)
        result = 0;
    else if (result > 255)
        result = 255;

    return result;
}

uchar getGradientAt(Mat input, vector<vector<double>> *filter, int row, int column) {
    double result = 0;

    for (int offsetRow = -1; offsetRow < 2; offsetRow++) {
        for (int offsetColumn = -1; offsetColumn < 2; offsetColumn++) {
            int neighborRow = row + offsetRow;
            int neighborColumn = column + offsetColumn;
            if (neighborRow >= 0 && neighborRow < input.size().height
                    && neighborColumn >= 0 && neighborColumn < input.size().width) {
                Scalar neighborPixel = input.at<uchar>(neighborRow, neighborColumn);
                double filterValue = filter->at(offsetRow + 1).at(offsetColumn + 1);
                double _result = neighborPixel.val[0] * filterValue;
                result += _result;
            }
        }
    }

    if (result < 0)
        result = 0;
    else if (result > 255)
        result = 255;

    return result;
}
