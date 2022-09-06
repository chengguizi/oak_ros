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

class Camera:
    def __init__(self):
        self.type = None
        self.intrinsic = None # numpy 3x3 Matrix
        self.resolution = None
        self.extrinsic = None
        self.distortion = None
        self.index = None

        self.socket = None
        self.fov = 105

    def __init__(self, json, index, socket):
        self.index = index
        self.socket = socket


        self.type = json['intrinsics'][self.index]['camera_type']
        self.intrinsic = getIntrinsicMatrix(json['intrinsics'][self.index]['intrinsics'])
        self.resolution = json['resolution'][self.index]
        self.extrinsic = getExtrinsicTransform(json['T_imu_cam'][self.index])
        self.distortion = getDistortionCoeffsFisheye(json['intrinsics'][self.index]['intrinsics'])


def addCameraCalibPair(calibData, childCamera, parentCamera):

    print(f"adding camera pair {childCamera.socket} to {parentCamera.socket}")
    print(f"with type {childCamera.type} and {parentCamera.type}")

    calibData.setCameraIntrinsics(childCamera.socket, childCamera.intrinsic.tolist(), childCamera.resolution[0], childCamera.resolution[1])
    calibData.setCameraIntrinsics(parentCamera.socket, parentCamera.intrinsic.tolist(), parentCamera.resolution[0], parentCamera.resolution[1])
    
    if (childCamera.type == 'kb4' and parentCamera.type == 'kb4'):
        # Fisheye type has index 1
        calibData.setCameraType(childCamera.socket, dai.CameraModel.Fisheye)
        calibData.setCameraType(parentCamera.socket, dai.CameraModel.Fisheye)
    else:
        raise RuntimeError("Only fisheye type implemented")

    calibData.setDistortionCoefficients(childCamera.socket, childCamera.distortion.tolist())
    calibData.setDistortionCoefficients(parentCamera.socket, parentCamera.distortion.tolist())

    # extrinsics
    T_imu_child = childCamera.extrinsic
    T_imu_parent = parentCamera.extrinsic

    parent_T_child = T_imu_parent.inverse() * T_imu_child
    R_, t_ = getRt(parent_T_child.matrix())
    calibData.setCameraExtrinsics(childCamera.socket, parentCamera.socket, R_, t_ * 100, [-1,0,0])

    print(f"{childCamera.index} in {parentCamera.index} \n{parent_T_child.matrix()}")

def addImuCalibPairInternal(calibData, camera, T_imu_cam):

    imuR, imut = getRt(T_imu_cam.inverse().matrix())

    specs = [0,0,0]
    calibData.setImuExtrinsics(camera.socket, imuR, imut * 100, specs)

# Connect device
with dai.Device() as device:

    # calibData = device.readCalibration()
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

    with open(args.basaltJsonFile, 'r') as stream:
        try:
            jsonStream = json.load(stream)['value0']
        except:
            print("Error occurred for json")
            exit(1)

    mainAR = Camera(jsonStream, 2, dai.CameraBoardSocket.CAM_D)
    leftOV = Camera(jsonStream, 1, dai.CameraBoardSocket.CAM_B)
    rightOV = Camera(jsonStream, 0, dai.CameraBoardSocket.CAM_A)

    addCameraCalibPair(calibData, mainAR, rightOV)
    addCameraCalibPair(calibData, leftOV, mainAR)

    # set a dummy calibration for imu first
    R_imu_cam = np.array([  [0,0,-1],
                            [-1,0,0],
                            [0,1,0]])
    t_imu_cam = np.array([-0.065, 0, 0])
    T_imu_cam = sp.SE3(R_imu_cam, t_imu_cam)
    addImuCalibPairInternal(calibData, mainAR, T_imu_cam)

    calibData.eepromToJsonFile("temp.json")

    try:
        device.flashCalibration2(calibData)
        print('Successfully flashed calibration')
    except Exception as ex:
        print(f'Failed flashing calibration: {ex}')