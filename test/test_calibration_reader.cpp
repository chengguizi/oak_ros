#include <cstdio>
#include <iostream>
#include <string>

// Includes common necessary includes for development using depthai library
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/EepromData.hpp"
#include "depthai/depthai.hpp"

void printMatrix(std::vector<std::vector<float>> matrix) {
    using namespace std;
    std::string out = "[";
    for(auto row : matrix) {
        out += "[";
        for(auto val : row) out += to_string(val) + ", ";
        out = out.substr(0, out.size() - 2) + "]\n";
    }
    out = out.substr(0, out.size() - 1) + "]\n\n";
    cout << out;
}

int main(int argc, char** argv) {
    using namespace std;

    // Connect Device
    dai::Device device;

    dai::CalibrationHandler calibData = device.readCalibration();
    // calibData.eepromToJsonFile(filename);
    std::vector<std::vector<float>> intrinsics;
    int width, height;

    cout << "Intrinsics from default Intrinsics function:" << endl;
    std::tie(intrinsics, width, height) = calibData.getDefaultIntrinsics(dai::CameraBoardSocket::RIGHT);
    printMatrix(intrinsics);

    cout << "Width: " << width << endl;
    cout << "Height: " << height << endl;

    cout << "Stereo baseline distance: " << calibData.getBaselineDistance() << " cm" << endl;

    cout << "Mono FOV from camera specs: " << calibData.getFov(dai::CameraBoardSocket::LEFT)
         << ", calculated FOV: " << calibData.getFov(dai::CameraBoardSocket::LEFT, false) << endl;

    
    cout << "Stereo Left Rectification Rotation: " << endl;
    printMatrix(calibData.getStereoLeftRectificationRotation());

    cout << "Stereo Right Rectification Rotation: " << endl;
    printMatrix(calibData.getStereoRightRectificationRotation());

    cout << "Intrinsics from getCameraIntrinsics function full resolution:" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT);
    printMatrix(intrinsics);

    cout << "Intrinsics from getCameraIntrinsics function 1280 x 720:" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 1280, 720);
    printMatrix(intrinsics);

    cout << "Intrinsics from getCameraIntrinsics function 720 x 450:" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 720);
    printMatrix(intrinsics);

    cout << "Intrinsics from getCameraIntrinsics function 600 x 1280:" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 600, 1280);
    printMatrix(intrinsics);

    std::vector<std::vector<float>> extrinsics;

    cout << "Extrinsics from left->right test:" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::LEFT, dai::CameraBoardSocket::RIGHT);
    printMatrix(extrinsics);

    cout << "Extrinsics from right->left test:" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RIGHT, dai::CameraBoardSocket::LEFT);
    printMatrix(extrinsics);

    cout << "Extrinsics from right->rgb test:" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RIGHT, dai::CameraBoardSocket::RGB);
    printMatrix(extrinsics);

    cout << "Extrinsics from rgb->right test:" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RGB, dai::CameraBoardSocket::RIGHT);
    printMatrix(extrinsics);

    cout << "Extrinsics from left->rgb test:" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::LEFT, dai::CameraBoardSocket::RGB);
    printMatrix(extrinsics);

    return 0;
}