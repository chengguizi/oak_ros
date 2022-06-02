#!/usr/bin/env python3

from pathlib import Path

import depthai as dai
import argparse

import numpy as np

import cv2

import yaml

# about camera type
# Perspective = 0, Fisheye = 1, Equirectangular = 2, RadialDivision = 3 

# calibJsonFile = str((Path(__file__).parent / Path('./depthai_calib.json')).resolve().absolute())
kalibrYamlFile = str((Path(__file__).parent / Path('./camchain-nominal.yaml')).resolve().absolute())

parser = argparse.ArgumentParser()
# parser.add_argument('calibJsonFile', nargs='?', help="Path to V6 calibration file in json", default=calibJsonFile)
parser.add_argument('kalibrYamlFile', nargs='?', help="Path to Kalibr file in yaml", default=kalibrYamlFile)
args = parser.parse_args()


def getIntrinsicMatrix(kalibrIntrinsic):
    ret = np.eye(3)

    ret[0,0] = kalibrIntrinsic[0]
    ret[1,1] = kalibrIntrinsic[1]
    ret[0,2] = kalibrIntrinsic[2]
    ret[1,2] = kalibrIntrinsic[3]

    return ret

def getRt(T):

    return T[0:3, 0:3], T[0:3, 3] * 100


# Connect device
with dai.Device() as device:

    calibData = dai.CalibrationHandler()

    # calibData.setBoardInfo("BW1098OBC", "R0M0E0")
    # calibData.setBoardInfo("OAK-D-LITE", "R0M0E0")

    eepromData = calibData.getEepromData()

    # print(eepromData.boardName)
    # print(eepromData.boardRev)
    # print(eepromData.cameraData)
    # print(eepromData.imuExtrinsics)
    # print(eepromData.stereoRectificationData)
    # print(eepromData.version)

    # exit(0)

    with open(args.kalibrYamlFile, 'r') as stream:
        try:
            kalibrYaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # STEP 1 - intrinsic

    leftIntrinsic = getIntrinsicMatrix(kalibrYaml['cam1']['intrinsics'])
    rightIntrinsic = getIntrinsicMatrix(kalibrYaml['cam0']['intrinsics'])

    print("\nIntrinsic Matrices")
    print(leftIntrinsic)
    print(rightIntrinsic)


    leftId = dai.CameraBoardSocket.LEFT # 1
    rightId = dai.CameraBoardSocket.RIGHT # 2

    # left2Id = dai.CameraBoardSocket.CAM_D # 1
    # right2Id = dai.CameraBoardSocket.RIGHT # 2


    calibData.setCameraIntrinsics(leftId, leftIntrinsic.tolist(), 640, 400)
    calibData.setCameraIntrinsics(rightId, rightIntrinsic.tolist(), 640, 400)

    # calibData.setCameraIntrinsics(left2Id, leftIntrinsic.tolist(), 640, 400)
    # calibData.setCameraIntrinsics(right2Id, rightIntrinsic.tolist(), 640, 400)

    # STEP 2 - distortion

    # calibData.setCameraType(leftId, dai.CameraModel.Perspective)
    # calibData.setCameraType(rightId, dai.CameraModel.Perspective)

    calibData.setCameraType(leftId, dai.CameraModel.Fisheye)
    calibData.setCameraType(rightId, dai.CameraModel.Fisheye)

    leftD = np.zeros(14)
    rightD = np.zeros(14)
    leftD[0:4] = kalibrYaml['cam0']['distortion_coeffs']
    rightD[0:4] = kalibrYaml['cam1']['distortion_coeffs']

    print("\nDistortion Coeffs (fisheye / equi / kb4)")
    print(leftD)
    print(rightD)

    calibData.setDistortionCoefficients(leftId, leftD.tolist())
    calibData.setDistortionCoefficients(rightId, rightD.tolist())

    # calibData.setDistortionCoefficients(left2Id, leftD.tolist())
    # calibData.setDistortionCoefficients(right2Id, rightD.tolist())

    # STEP 3 - extrinsic

    right_T_left = np.array(kalibrYaml['cam1']['T_cn_cnm1'])
    # right_T_left = np.linalg.inv(left_T_right)

    leftR, leftt = getRt(right_T_left)
    # rightR, rightt = getRt()

    print("\nLeft Camera Extrinsic")
    print(leftR)
    print(leftt)

    calibData.setCameraExtrinsics(leftId, rightId, leftR, leftt, [-7.5,0,0])
    # 4 camera front
    # calibData.setCameraExtrinsics(leftId, rightId, leftR, leftt, [-7.75,0.625,-2.8])

    # calibData.setCameraExtrinsics(left2Id, right2Id, leftR, leftt, [-7.75,0.625,-2.8])

    calibData.setFov(leftId, 72.9)
    calibData.setFov(rightId, 72.9)

    # STEP 4 - rectification

    R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(leftIntrinsic, leftD[0:4], rightIntrinsic, rightD[0:4], (640, 400), leftR, leftt, 0)

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