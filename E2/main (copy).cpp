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
Mat output, output2, output_normalized, output_scaled, output_normalized2, output_scaled2;

int cornerThreshold = 240;
int max_cornerThreshold = 255;

vector<double> matrixSobelX_1({1, 2, 1});
vector<double> matrixSobelX_2({-1, 0, 1});

vector<vector<double>> matrixSobelX({{1, 2, 1}, {0, 0, 0}, {-1, -2, -1}});
vector<vector<double>> matrixSobelY({{1, 0, -1}, {2, 0, -2}, {1, 0, -1}});

vector<vector<double>> gaussian({{1.0 / 16, 1.0 / 8, 1.0 / 16}, {1.0 / 8, 1.0 / 4, 1.0 / 8},
                                 {1.0 / 16, 1.0 / 8, 1.0 / 16}});

void calculateGradient(Mat input, Mat output, int direction);
uchar getGradientAt(Mat input, vector<vector<double>> *filter, int row, int column);
void detectCorners(Mat input, Mat output, Mat gradientX, Mat gradientY);

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

    output = Mat::zeros( src.size(), CV_32F );
    src.copyTo(corners);

    cvtColor( src, gray, CV_BGR2GRAY );
    namedWindow("original_image", CV_WINDOW_AUTOSIZE);
    imshow("original_image", src);

    Mat gradientX = Mat::zeros( src.size(), CV_8U );
    Mat gradientY = Mat::zeros( src.size(), CV_8U );

    calculateGradient(gray, gradientX, 0);
    calculateGradient(gray, gradientY, 1);

    detectCorners(gray, output, gradientX, gradientY);
    normalize(output, output_normalized, 0, 255, NORM_MINMAX, CV_8U, Mat());
    convertScaleAbs(output_normalized, output_scaled);

    for (int row = 0; row < src.size().height; row++) {

        for (int column = 0; column < src.size().width; column++) {
            Scalar value = output_scaled.at<uchar>(row, column);
            if (value.val[0] > cornerThreshold)
                circle(corners, Point(column, row), 5, Scalar(100), 2, 8, 0);
        }
    }

    namedWindow("detected_corners", CV_WINDOW_AUTOSIZE);
    imshow("detected_corners", corners);

    waitKey(0);
    return 0;
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
            uchar gradient = getGradientAt(input, filter, row, column);
            output.at<uchar>(row, column) = gradient;
        }
    }
}

uchar horizontalGradient(Mat ipnut, vector<double> *filterH, int row, int column) {

}

uchar verticalGradient(Mat input, vector<double> *filterV, int row, int column) {

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
