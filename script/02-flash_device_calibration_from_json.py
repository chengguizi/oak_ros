#!/usr/bin/env python3

from pathlib import Path

import depthai as dai
import argparse

import numpy as np

import cv2

import json
from scipy.spatial.transform import Rotation as R
import sophus as sp


# about camera type
# Perspective = 0, Fisheye = 1, Equirectangular = 2, RadialDivision = 3 

basaltJsonFile = str((Path(__file__).parent / Path('./calibration-imu.json')).resolve().absolute())

parser = argparse.ArgumentParser()
parser.add_argument('basaltJsonFile', nargs='?', help="Path to Kalibr file in yaml", default=basaltJsonFile)
args = parser.parse_args()

def getExtrinsicTransform(T_imu_cam):
    t_imu_cam = np.array([T_imu_cam['px'], T_imu_cam['py'], T_imu_cam['pz']])
    R_imu_cam = R.from_quat([T_imu_cam['qx'], T_imu_cam['qy'], T_imu_cam['qz'], T_imu_cam['qw']]) # scalar-last (x, y, z, w) format

    return sp.SE3(R_imu_cam.as_matrix(), t_imu_cam)

def getIntrinsicMatrix(intrinsics):
    ret = np.eye(3)

    ret[0,0] = intrinsics['fx']
    ret[1,1] = intrinsics['fy']
    ret[0,2] = intrinsics['cx']
    ret[1,2] = intrinsics['cy']

    return ret

def getDistortionCoeffsFisheye(intrinsics):
    D = np.zeros(14)
    D[0:4] = [intrinsics['k1'], intrinsics['k2'], intrinsics['k3'], intrinsics['k4']]

    return D


def getRt(T):

    return T[0:3, 0:3], T[0:3, 3]


# Connect device
with dai.Device() as device:

    calibData = dai.CalibrationHandler()

    # calibData.setBoardInfo("BW1098OBC", "R0M0E0")
    # calibData.setBoardInfo("OAK-D-LITE", "R0M0E0")

    # eepromData = calibData.getEepromData()

    # print(eepromData.boardName)
    # print(eepromData.boardRev)
    # print(eepromData.cameraData)
    # print(eepromData.imuExtrinsics)
    # print(eepromData.stereoRectificationData)
    # print(eepromData.version)

    # exit(0)

    with open(args.basaltJsonFile, 'r') as stream:
        try:
            basalJson = json.load(stream)['value0']
        except:
            print("Error occurred for json")
            exit(1)

    leftId = dai.CameraBoardSocket.LEFT # 1
    rightId = dai.CameraBoardSocket.RGB # 2

    # STEP 1 - intrinsic

    leftCameraType = basalJson['intrinsics'][0]['camera_type']
    rightCameraType = basalJson['intrinsics'][1]['camera_type']

    leftIntrinsic = getIntrinsicMatrix(basalJson['intrinsics'][0]['intrinsics'])
    rightIntrinsic = getIntrinsicMatrix(basalJson['intrinsics'][1]['intrinsics'])


    print("\nIntrinsic Matrices")
    print(leftIntrinsic)
    print(rightIntrinsic)

    leftResolution = basalJson['resolution'][0]
    rightResolution = basalJson['resolution'][1]

    print('\nResolutions')
    print(leftResolution)
    print(rightResolution)

    calibData.setCameraIntrinsics(leftId, leftIntrinsic.tolist(), leftResolution[0], leftResolution[1])
    calibData.setCameraIntrinsics(rightId, rightIntrinsic.tolist(), rightResolution[0], rightResolution[1])


    # STEP 2 - distortion

    # calibData.setCameraType(leftId, dai.CameraModel.Perspective)
    # calibData.setCameraType(rightId, dai.CameraModel.Perspective)

    
    

    leftD = getDistortionCoeffsFisheye(basalJson['intrinsics'][0]['intrinsics'])
    rightD = getDistortionCoeffsFisheye(basalJson['intrinsics'][1]['intrinsics'])

    print(f"\nDistortion Coeffs ({leftCameraType}, {rightCameraType})")
    print(leftD)
    print(rightD)

    calibData.setDistortionCoefficients(leftId, leftD.tolist())
    calibData.setDistortionCoefficients(rightId, rightD.tolist())

    if (leftCameraType == 'kb4' and rightCameraType == 'kb4'):
        print('\nSelect Fisheye Camera Type')
        calibData.setCameraType(leftId, dai.CameraModel.Fisheye)
        calibData.setCameraType(rightId, dai.CameraModel.Fisheye)
    else:
        raise RuntimeError(f"unknown camera type {leftCameraType} {rightCameraType}")

    # STEP 3 - extrinsic

    T_imu_left = getExtrinsicTransform(basalJson['T_imu_cam'][0])
    T_imu_right = getExtrinsicTransform(basalJson['T_imu_cam'][1])

    print('\nExtrinsic Matrices')
    print(T_imu_left.matrix())
    print(T_imu_right.matrix())

    right_T_left = T_imu_right.inverse() * T_imu_left

    leftR, leftt = getRt(right_T_left.matrix())

    print("\nLeft Camera Extrinsic")
    print(leftR)
    print(leftt)

    # translation in cm
    calibData.setCameraExtrinsics(leftId, rightId, leftR, leftt * 100, [-5.5,0,0])

    calibData.setFov(leftId, 105)
    calibData.setFov(rightId, 105)

    # STEP 3A - imu extrinsic

    T_right_imu = T_imu_right.inverse()
    imuR, imut = getRt(T_right_imu.matrix())

    spec4CamParallel = [-2,0,-3.5]

    calibData.setImuExtrinsics(rightId, imuR, imut * 100, spec4CamParallel)

    # STEP 4 - rectification

    # The step 4 is not necessary, as we shall calculate online

    R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(leftIntrinsic, leftD[0:4], rightIntrinsic, rightD[0:4], (leftResolution[0], leftResolution[1]), leftR, leftt, cv2.CALIB_ZERO_DISPARITY)

    print(f"R1\n{R1}")
    print(f"R2\n{R2}")
    print(f"P1\n{P1}")
    print(f"P2\n{P2}")
    print(f"Q\n{Q}")

    calibData.setStereoLeft(leftId, R1.tolist())
    calibData.setStereoRight(rightId, R2.tolist())

    status = device.flashCalibration(calibData)
    if status:
        print('Calibration Flash Successful')
    else:
        print('Calibration Flash Failed!!!')