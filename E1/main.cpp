#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <vector>
#include <list>
using namespace cv;
using namespace std;

struct Node {
    Point position;
    int weight;

    Node(Point position, int weight) {
        this->position = position;
        this->weight = weight;
    }
};

Mat image0, image;
Point startingPoint, endingPoint;
short pointsSet = 0;
int edgeThresh = 1;

int minIndex(vector<int> &weights, vector<bool> &visited, int n) {
   // Initialize min value
   int min = INT_MAX;
   int minIndex;

   for (int i = 0; i < n; i++) {
       if (visited[i] == false && weights[i] <= min) {
           min = weights[i];
           minIndex = i;
       }
   }

   return minIndex;
}

Node* getNodeForPosition(list<Node>& nodes, Point position) {
    for (list<Node>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
        Node node = *it;
        if (node.position.x == position.x && node.position.y == position.y)
            return &*it;
    }

    return nullptr;
}

void insertSorted(list<Node>& nodes, Point neighbor, int neighborWeight) {
    Node node(neighbor, neighborWeight);
    for (list<Node>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
        Node listNode = *it;
        if (node.weight < listNode.weight) {
            nodes.insert(it, node);
            return;
        }
    }

    nodes.push_back(node);
}

void dijkstra(Mat gradient) {
    Size imageSize = gradient.size();
    int width = imageSize.width;
    int height = imageSize.height;
    vector<vector<int>> weights(height, vector<int>(width));
    vector<vector<bool>> visited(height, vector<bool>(width));
    // store the parents of the pixels
    vector<vector<Point>> parents(height, vector<Point>(width));
    list<Node> nodes;

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            weights[i][j] = INT_MAX;
            visited[i][j] = false;
            parents[i][j] = Point(-1, -1);
        }
    }

    weights[startingPoint.y][startingPoint.x] = 0;
    Node currentNode = Node(startingPoint, 0);
    nodes.push_front(currentNode);

    while(nodes.size() > 0) {

        // retrieve current node that we need to work on
        currentNode = nodes.front();
        nodes.pop_front();

        visited[currentNode.position.y][currentNode.position.x] = true;

        // visit each neighbor by adding an offset to the position of the current node
        for (int i = -1; i < 2; i++) {
            for (int j = -1; j < 2; j++) {
                // this offset would work on the current node itself so skip
                if (i == 0 && j == 0)
                    continue;
                Point currPos = currentNode.position;
                // retrieve neighbor by applying offset
                Point neighbor = Point(currPos.y + i, currPos.x + j);

                // continue if coordinates are outside of image
                if (neighbor.x < 0 || neighbor.x > width - 1 || neighbor.y < 0 || neighbor.y > height - 1)
                    continue;

                // if the neighbor has been visited (i.e. gone through this loop) there is no need to consider it anymore
                if (!visited[neighbor.y][neighbor.x]) {
                    // retrieve the neighbors weight, i.e. the gradient value of the pixel
                    Vec3b neighborWeightVector = gradient.at<Vec3b>(neighbor.y, neighbor.x);
                    // calculate the weight by adding the rgb values of the gradient
                    int neighborWeight = neighborWeightVector[0] + neighborWeightVector[1] + neighborWeightVector[2];
                    int currentWeight = weights[currPos.y][currPos.x];
                    // check if the weight of the path passing this node to the neighbor is less than the (maybe) existing path
                    if (neighborWeight + currentWeight < weights[neighbor.y][neighbor.x]) {
                        //if so set weight of the neighbor
                        weights[neighbor.y][neighbor.x] = currentWeight;
                        // check if node has already been added to the list of nodes
                        Node* exisitingNode = getNodeForPosition(nodes, neighbor);
                        if (exisitingNode != nullptr) {
                            // if so only update the information
                            exisitingNode->weight = neighborWeight;
                        } else {
                            // if not (e.g. if this is the first time this node has been considered a neighbor in the run of this algorithm) add a new node
                            insertSorted(nodes, neighbor, neighborWeight);
                        }

                        // set the parent of the neighbor to this node
                        parents[neighbor.y][neighbor.x] = currentNode.position;
                    }
                }
            }
        }

        if (currentNode.position == endingPoint) {
            // we found our goal node
            break;
        }
    }

    Point currentPoint = currentNode.position;
    while (currentPoint != Point(-1, -1)) {
        Vec3b color = image.at<Vec3b>(currentPoint.y, currentPoint.x);
        color[0] = 255;
        color[1] = 255;
        color[2] = 255;
        image.at<Vec3b>(currentPoint.y, currentPoint.x) = color;
        currentPoint = parents[currentPoint.y][currentPoint.x];
    }

    imshow("image", image);
    // current node is end point
}

void calculateRoute() {
    Mat smoothed, edge, laplacian, result;
    GaussianBlur(image, smoothed, Size(5, 5), 1, 1);
    Canny(smoothed, edge, 100, 100);
    Laplacian(edge, laplacian, CV_16S, 5);
    //convertScaleAbs(laplacian, result);
    imshow("gradient", laplacian);
    dijkstra(laplacian);
}

static void onMouse( int event, int x, int y, int, void* ) {
    if( event != EVENT_LBUTTONDOWN )
        return;
    cout << "Event: left mouse down\n";
    if (pointsSet == 0) {
        startingPoint = Point(x, y);
        pointsSet++;
    }
    else if (pointsSet == 1) {
        endingPoint = Point(x, y);
        pointsSet++;
        calculateRoute();
    }
    //imshow("image", image);
}

int main( int argc, char** argv ) {
    char* filename = argc >= 2 ? argv[1] : (char*)"../data/fruits.jpg";
    image0 = imread(filename, 1);
    if( image0.empty() )
    {
        cout << "Image empty. Usage: ffilldemo <image_name>\n";
        return 0;
    }
    image0.copyTo(image);
    namedWindow("image", 0);
    imshow("image", image);
    setMouseCallback( "image", onMouse, 0 );
    waitKey(0);
    return 0;
}
