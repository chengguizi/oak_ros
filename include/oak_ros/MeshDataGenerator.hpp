#pragma once

#include <vector>

#include <depthai/depthai.hpp>

#include <opencv2/calib3d.hpp>

template <typename _Tp> static cv::Mat_<_Tp> toMat(const std::vector<std::vector<_Tp>> vecIn) {
    cv::Mat_<_Tp> matOut(vecIn.size(), vecIn.at(0).size());
    for (int i = 0; i < matOut.rows; ++i) {
        for (int j = 0; j < matOut.cols; ++j) {
            matOut(i, j) = vecIn.at(i).at(j);
        }
    }
    return matOut;
}
template <typename _Tp> static std::vector<std::vector<_Tp>> toVec(const cv::Mat_<_Tp> matIn) {
    std::vector<std::vector<_Tp>> vecOut(matIn.rows);
    for (int i = 0; i < matIn.rows; ++i) {
        vecOut[i].resize(matIn.cols);
        for (int j = 0; j < matIn.cols; ++j) {
            vecOut[i][j] = matIn.at(i, j);
        }
    }
    return vecOut;
}

cv::Size_<int> getImageSize(dai::MonoCameraProperties::SensorResolution resolution) {

    cv::Size_<int> imageSize;
    if (resolution == dai::MonoCameraProperties::SensorResolution::THE_400_P) {
            imageSize = cv::Size_<int>(640, 400);
        }else if (resolution == dai::MonoCameraProperties::SensorResolution::THE_480_P) {
            imageSize = cv::Size_<int>(640, 480);
        }else if (resolution == dai::MonoCameraProperties::SensorResolution::THE_720_P) {
            imageSize = cv::Size_<int>(1280, 720);
        }else if (resolution == dai::MonoCameraProperties::SensorResolution::THE_800_P) {
            imageSize = cv::Size_<int>(1280, 800);
        }else {
            throw std::runtime_error("sensor resolution not implemented");
        }
    
    return imageSize;
}

cv::Size_<int> getImageSize(dai::ColorCameraProperties::SensorResolution resolution) {

    cv::Size_<int> imageSize;
    if (resolution == dai::ColorCameraProperties::SensorResolution::THE_1080_P) {
            imageSize = cv::Size_<int>(1920, 1080);
        }else if (resolution == dai::ColorCameraProperties::SensorResolution::THE_1200_P) {
            imageSize = cv::Size_<int>(1920, 1200);
        }else {
            throw std::runtime_error("sensor resolution not implemented");
        }
    
    return imageSize;
}

template <typename _T, typename _Tp>
void getRt(std::vector<std::vector<_T>> &T, cv::Mat_<_Tp> &R, cv::Mat_<_Tp> &t) {
    R = cv::Mat_<_Tp>(3, 3);
    t = cv::Mat_<_Tp>(3, 1);

    R(0, 0) = T[0][0];
    R(0, 1) = T[0][1];
    R(0, 2) = T[0][2];
    R(1, 0) = T[1][0];
    R(1, 1) = T[1][1];
    R(1, 2) = T[1][2];
    R(2, 0) = T[2][0];
    R(2, 1) = T[2][1];
    R(2, 2) = T[2][2];

    t(0) = T[0][3] / 100.;
    t(1) = T[1][3] / 100.;
    t(2) = T[2][3] / 100.;
}

class OakMeshDataGenerator {

  public:
    // initialise R1, R2, P1, P2, Q
    void getRectificationTransformFromOpenCV(dai::CalibrationHandler calibData,
                                             dai::CameraBoardSocket socketLeft,
                                             dai::CameraBoardSocket socketRight, dai::MonoCameraProperties::SensorResolution resolution, float alpha);

    cv::Mat_<float> getNewM(){return m_newM;}
    cv::Mat getR2(){return m_R2;}
    
    void calculateMeshData(const int meshStep, std::vector<std::uint8_t> &dataLeft,
                           std::vector<std::uint8_t> &dataRight);

    cv::Mat getMaskRight() {return m_maskRight;}

  private:
    cv::Mat_<float> m_M1, m_M2;
    cv::Mat m_R1, m_R2, m_P1, m_P2, m_Q;
    cv::Mat_<float> m_newM; // essentially is m_P2's K matrix
    cv::Mat_<uint8_t> m_maskRight;
    cv::Mat_<float> m_D1, m_D2;
    cv::Size m_imageSize;
};

void OakMeshDataGenerator::getRectificationTransformFromOpenCV(dai::CalibrationHandler calibData,
                                                               dai::CameraBoardSocket socketLeft,
                                                               dai::CameraBoardSocket socketRight, dai::MonoCameraProperties::SensorResolution resolution, float alpha) {

    std::cout << "getRectificationTransformFromOpenCV" << std::endl;

    assert (alpha >= 0 && alpha <= 1);

    // Obtain Size
    {
        // dai::CameraInfo infoLeft = calibData.getEepromData().cameraData.at(socketLeft);
        // // dai::CameraInfo infoRight = calibData.getEepromData().cameraData.at(socketRight);
        // m_imageSize = cv::Size_<int>(infoLeft.width, infoLeft.height);

        m_imageSize = getImageSize(resolution);

        std::cout << "Mesh Generator uses image size " << m_imageSize << std::endl;
    }

    // Obtain Intrinsics
    {
        std::vector<std::vector<float>> intrinsicsLeft, intrinsicsRight;
        intrinsicsLeft = calibData.getCameraIntrinsics(socketLeft, m_imageSize.width, m_imageSize.height);
        intrinsicsRight = calibData.getCameraIntrinsics(socketRight, m_imageSize.width, m_imageSize.height);

        m_M1 = toMat(intrinsicsLeft);
        m_M2 = toMat(intrinsicsRight);
    }

    std::cout << "M1" << std::endl << m_M1 << std::endl;
    std::cout << "M2" << std::endl << m_M2 << std::endl;

    // Obtain Extrinsics
    cv::Mat_<double> R, t; // stereoRectify requires double precision for R t
    {
        std::vector<std::vector<float>> T;
        T = calibData.getCameraExtrinsics(socketLeft, socketRight, false);

        getRt(T, R, t);
    }

    std::cout << "R" << std::endl << R << std::endl;
    std::cout << "t" << std::endl << t << std::endl;

    // Obtain Distortion

    {
        dai::CameraModel cameraModelLeft = calibData.getDistortionModel(socketLeft);
        dai::CameraModel cameraModelRight = calibData.getDistortionModel(socketRight);

        std::vector<float> distortionLeft, distortionRight;
        distortionLeft = calibData.getDistortionCoefficients(socketLeft);
        distortionRight = calibData.getDistortionCoefficients(socketRight);

        if (cameraModelLeft == dai::CameraModel::Perspective &&
            cameraModelRight == dai::CameraModel::Perspective) {
            m_D1 = cv::Mat_<float>(8, 1);
            m_D2 = cv::Mat_<float>(8, 1);

            for (int i = 0; i < 8; i++) {
                m_D1(i) = distortionLeft[i];
                m_D2(i) = distortionRight[i];
            }

            std::cout << "Initialise left distortion = " << m_D1.t() << std::endl;
            std::cout << "Initialise right distortion = " << m_D2.t() << std::endl;

            std::cout << "Initialise rectification R and P matrices with perspective camera model."
                      << std::endl;

            cv::stereoRectify(m_M1, m_D1, m_M2, m_D2, m_imageSize, R, t, m_R1, m_R2, m_P1, m_P2,
                              m_Q, cv::CALIB_ZERO_DISPARITY, alpha);
            

        } else if (cameraModelLeft == dai::CameraModel::Fisheye &&
                   cameraModelRight == dai::CameraModel::Fisheye) {

            m_D1 = cv::Mat_<float>(4, 1);
            m_D2 = cv::Mat_<float>(4, 1);

            for (int i = 0; i < 4; i++) {
                m_D1(i) = distortionLeft[i];
                m_D2(i) = distortionRight[i];
            }

            std::cout << "Initialise left distortion = " << m_D1.t() << std::endl;
            std::cout << "Initialise right distortion = " << m_D2.t() << std::endl;

            std::cout << "Initialise rectification R and P matrices with fisheye camera model."
                      << std::endl;

            cv::fisheye::stereoRectify(m_M1, m_D1, m_M2, m_D2, m_imageSize, R, t, m_R1, m_R2, m_P1,
                                       m_P2, m_Q, cv::CALIB_ZERO_DISPARITY, cv::Size(), alpha);
            

        } else {
            throw std::runtime_error("Not Implemented distortion model");
        }
    }

    std::cout << "R1" << std::endl << m_R1 << std::endl;
    std::cout << "R2" << std::endl << m_R2 << std::endl;
    std::cout << "P1" << std::endl << m_P1 << std::endl;
    std::cout << "P2" << std::endl << m_P2 << std::endl;
    std::cout << "Q" << std::endl << m_Q << std::endl;

    m_newM = cv::Mat_<float>(m_P2, cv::Range(0, 3), cv::Range(0, 3));
    // m_newM = m_M2;
}

void OakMeshDataGenerator::calculateMeshData(const int meshStep,
                                             std::vector<std::uint8_t> &dataLeft,
                                             std::vector<std::uint8_t> &dataRight) {

    std::cout << "calculateMeshData " << std::endl;

    std::cout << "m_newM" << std::endl << m_newM << std::endl;

    cv::Mat_<float> mapXL, mapYL;
    cv::Mat_<float> mapXR, mapYR;

    m_maskRight = cv::Mat_<uint8_t>(m_imageSize, 0);
    auto m_dummyWhite = cv::Mat_<uint8_t>(m_imageSize, 255);

    if (m_D1.total() == 8 && m_D2.total() == 8) {
        // perspective camera
        cv::initUndistortRectifyMap(m_M1, m_D1, m_R1, m_newM, m_imageSize, CV_32FC1, mapXL, mapYL);
        cv::initUndistortRectifyMap(m_M2, m_D2, m_R2, m_newM, m_imageSize, CV_32FC1, mapXR, mapYR);
    } else if (m_D1.total() == 4 && m_D2.total() == 4) {
        // fisheye camera
        cv::fisheye::initUndistortRectifyMap(m_M1, m_D1, m_R1, m_newM, m_imageSize, CV_32FC1, mapXL,
                                             mapYL);
        cv::fisheye::initUndistortRectifyMap(m_M2, m_D2, m_R2, m_newM, m_imageSize, CV_32FC1, mapXR,
                                             mapYR);
    } else {
        throw std::runtime_error("Not Implemented distortion model");
    }

    cv::remap(m_dummyWhite, m_maskRight, mapXR, mapYR, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

    cv::erode(m_maskRight, m_maskRight, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)),
     cv::Point(-1,-1),
     1,
     cv::BORDER_CONSTANT,
     cv::Scalar(0));

    // cv::imshow("remap image", m_maskRight);
    // cv::waitKey(1);

    // we assume the image size is a multiple of the step size
    assert(mapXL.rows % meshStep == 0);
    assert(mapXL.cols % meshStep == 0);

    std::vector<float> meshLeft, meshRight;

    for (int i = 0; i <= mapXL.rows; i += meshStep) {
        for (int j = 0; j <= mapXL.cols; j += meshStep) {
            if (i == mapXL.rows && j == mapXL.cols) {
                // bottom-right
                meshLeft.push_back(mapYL(i - 1, j - 1));
                meshLeft.push_back(mapXL(i - 1, j - 1));

                meshRight.push_back(mapYR(i - 1, j - 1));
                meshRight.push_back(mapXR(i - 1, j - 1));
            } else if (i == mapXL.rows) {
                // bottom
                meshLeft.push_back(mapYL(i - 1, j));
                meshLeft.push_back(mapXL(i - 1, j));

                meshRight.push_back(mapYR(i - 1, j));
                meshRight.push_back(mapXR(i - 1, j));
            } else if (j == mapXL.cols) {
                // right
                meshLeft.push_back(mapYL(i, j - 1));
                meshLeft.push_back(mapXL(i, j - 1));

                meshRight.push_back(mapYR(i, j - 1));
                meshRight.push_back(mapXR(i, j - 1));
            } else {
                meshLeft.push_back(mapYL(i, j));
                meshLeft.push_back(mapXL(i, j));

                meshRight.push_back(mapYR(i, j));
                meshRight.push_back(mapXR(i, j));
            }
        }

        if ((mapXL.cols % meshStep) % 2 != 0) {
            meshLeft.push_back(0);
            meshLeft.push_back(0);

            meshRight.push_back(0);
            meshRight.push_back(0);
        }
    }

    std::uint8_t* pL = reinterpret_cast<std::uint8_t *>(meshLeft.data());
    std::uint8_t* pR = reinterpret_cast<std::uint8_t *>(meshRight.data());

    for (int k = 0 ; k < sizeof(float) * meshLeft.size(); k++) {
        dataLeft.push_back(pL[k]);
        dataRight.push_back(pR[k]);
    }
    
}