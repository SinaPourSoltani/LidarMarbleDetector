#include <opencv2/opencv.hpp>
#include <iostream>

#include "LidarMarbleDetector.h"

#define NUM_DATA_POINTS 200

double * lidarData = new double[NUM_DATA_POINTS];
Mat image;

using namespace std;
using namespace cv;

double * getData(string path, int thisMuch){
    ifstream inFile;
    inFile.open(path);
    auto * data = new double [thisMuch];
    char inputStr[5];
    if (!inFile.is_open()) {
        cerr << "Unable to open file datafile.txt";
        exit(1);   // call system to stop
    } else {
        int i = 0;
        do{
            double output = 0;
            inFile >> inputStr;

            if (inputStr[2] == '.')
                output = 10;
            else
                output = inputStr[0] - 48 + 0.1 * (inputStr[2] - 48) + 0.01 * (inputStr[3] - 48);

            data[i++] = output;
        }while (!inFile.eof() && i < thisMuch);
    }
    inFile.close();
    return data;
}

void onMouse(int event, int x, int y, int flags, void* param){
    if(event == EVENT_LBUTTONDOWN){
        cout << "[" << x << "," << y << "]" << endl;
    }
}

int main() {
    LidarMarbleDetector ldm = LidarMarbleDetector(lidarData, NUM_DATAPTS,800,800);
    setMouseCallback(WINDOW_NAME, onMouse);

    for (int i = 0; i < 6; ++i) {
        double *lidarData = getData("../data/TEST" + to_string(i + 1) + ".txt", NUM_DATAPTS);
        double target_dir_in_rad = ldm.setLidarData(lidarData);
        cout << "Target direction: " << (target_dir_in_rad != NO_TARGET ? to_string(target_dir_in_rad) : "No target.") << endl;
        //imwrite("../images/img" + to_string(i+1) + ".png", ldm._image);
        waitKey();
    }
    return 0;
}