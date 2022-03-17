#!/usr/bin/env python3

import blobconverter
import cv2
import depthai
import numpy as np  # numpy - manipulate the packet data returned by depthai
import math
from math import cos, sin, tan, pi
from paho.mqtt import client as mqtt_client
from time import sleep
import time
from scipy.optimize import minimize
import datetime

broker = '10.44.99.11'
port = 1883
pubTopic = "/sensors/camera"
subTopic = "/robot/camera"
client_id = "4499-NANO"

sub_client_id = "4499-NANO"

def connect_mqtt():
    # client_id = "44H99"
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)
    # Set Connecting Client ID
    client = mqtt_client.Client(client_id)
    client.username_pw_set("4499", "4499")
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def connect_mqttSub():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
           print("Connected to MQTT Broker!")
        else:
           print("Failed to connect, return code %d\n", rc)
        #Set Connecting Client ID
    client = mqtt_client.Client(sub_client_id)
    client.username_pw_set("4499", "4499")
    client.on_connect = on_connect
    client.connect(broker)
    return client

def publish(client, msg):
    msg_count = 0
    result = client.publish(pubTopic, msg)
    # result: [0, 1]
    status = result[0]
    if(status != 0):
        print("Failed to send message to Topic")
    msg_count += 1

def subscribe(client):
    def on_message(client, userdata, msg):
        print("Received " + str(msg) + " from Topic " + str(subTopic))

    # print("Listening")
    client.subscribe(subTopic, 2)
    client.on_message = on_message

def on_change(value):
    print(value)

def getRotX(theta):
	return np.array([
		[1,0,0,0],
		[0,cos(theta),-sin(theta),0],
		[0,sin(theta),cos(theta),0],
		[0,0,0,1],
	])
def getRotY(theta):
	return np.array([
		[cos(theta),0,-sin(theta),0],
		[0,1,0,0],
		[-sin(theta),cos(theta),0,0],
		[0,0,0,1],
	])
def getRotZ(theta):
	return np.array([
		[cos(theta),-sin(theta),0,0],
		[sin(theta),cos(theta),0,0],
		[0,0,1,0],
		[0,0,0,1],
	])
def leftRightFlip():
	return np.array([
		[-1,0,0,0],
		[0,1,0,0],
		[0,0,1,0],
		[0,0,0,1]
	])

def getCircleOfBestFitCenter(xList, yList):
    inputList = []
    dList = []
    for i in range(0, len(xList)):
        inputList.append([xList[i], yList[i], 1])
        dList.append([(xList[i] ** 2) + (yList[i] ** 2)])
    
    inputMatrix = np.array(inputList)
    dMatrix = np.array(dList)

    outputMatrix = (np.linalg.pinv(inputMatrix)) @ dMatrix

    xCenter = outputMatrix[0]/2
    yCenter = outputMatrix[1]/2

    rSquared = math.sqrt(((4 * outputMatrix[2]) + (outputMatrix[0] ** 2) + (outputMatrix[1] ** 2)))/2

    centers = [[xCenter, yCenter, rSquared]]

    return centers 

def reduceBestFitError(var, xList, yList):
    error = 0
    for x,y in zip(xList, yList):
        error += ((((x-var[0]) ** 2) + ((y-var[1]) ** 2) - 26**2) ** 2)
    return error


def convertCoordinates(x, y, z):
    input_point = np.array([x, y, z, 1]).T
    transform = getRotZ(pi/2) @ getRotX(pi/2 - cameraElevation) @ leftRightFlip()
    mapped_point = transform @ input_point
    # print(input_point)
    # print(mapped_point)
    return mapped_point

cameraResolutionWidth = 640.0
cameraResolutionHeight = 360.0

HFOV = 69
VFOV = 36

focalLength = cameraResolutionWidth/(2 * tan(HFOV/2))

# Camera elevation from horizontal, remember to change
cameraElevation = 40 * pi/180.0

cameraMountAngleFromUpright = 50 * pi/180

targetHeightDifference = 81.625

# MobilenetSSD label texts
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

# Create pipeline
pipeline = depthai.Pipeline()

# cam_rgb = pipeline.create(depthai.node.ColorCamera)
# cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
# cam_rgb.setInterleaved(False)
# cam_rgb.setIspScale(1, 3)
# cam_rgb.setPreviewSize(640, 360)
# cam_rgb.setIspScale(2, 3)

# xout_rgb = pipeline.create(depthai.node.XLinkOut)
# xout_rgb.setStreamName("rgb")
# cam_rgb.preview.link(xout_rgb.input)

# Define source and output
cam_rgb = pipeline.create(depthai.node.ColorCamera)
cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_12_MP)
cam_rgb.setInterleaved(False)
cam_rgb.setIspScale(1,4) # 4056x3040 -> 812x608
cam_rgb.setPreviewSize(1014, 760)
cam_rgb.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)
# Slightly lower FPS to avoid lag, as ISP takes more resources at 12MP
cam_rgb.setFps(25)

xoutIsp = pipeline.create(depthai.node.XLinkOut)
xoutIsp.setStreamName("isp")
cam_rgb.isp.link(xoutIsp.input)

# Use ImageManip to resize to 300x300 with letterboxing
manip = pipeline.create(depthai.node.ImageManip)
manip.setMaxOutputFrameSize(270000) # 300x300x3
manip.initialConfig.setResizeThumbnail(300, 300)
cam_rgb.preview.link(manip.inputImage)

controlIn = pipeline.create(depthai.node.XLinkIn)
controlIn.setStreamName('control')
controlIn.out.link(cam_rgb.inputControl)

lowerH = 92
upperH = 91

lowerS = 75
upperS = 255

lowerV = 109
upperV = 255

expTime = 20000
sensIso = 500

# Define a source - two mono (grayscale) cameras
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()
spatialLocationCalculator = pipeline.createSpatialLocationCalculator()

xoutDepth = pipeline.createXLinkOut()
xoutSpatialData = pipeline.createXLinkOut()
xinSpatialCalcConfig = pipeline.createXLinkIn()

xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")
xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

monoLeft.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(depthai.CameraBoardSocket.LEFT)
monoRight.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(depthai.CameraBoardSocket.RIGHT)

outputDepth = True
outputRectified = False
lrcheck = True
subpixel = False
extended = True

# StereoDepth
stereo.setOutputDepth(outputDepth)
stereo.setOutputRectified(outputRectified)
stereo.setConfidenceThreshold(255)
stereo.setDepthAlign(depthai.CameraBoardSocket.RGB)
stereo.setOutputSize(640, 360)

stereo.setLeftRightCheck(lrcheck)
stereo.setSubpixel(subpixel)

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
stereo.depth.link(spatialLocationCalculator.inputDepth)

topLeft = depthai.Point2f(0.4, 0.4)
bottomRight = depthai.Point2f(0.6, 0.6)

spatialLocationCalculator.setWaitForConfigInput(False)
config = depthai.SpatialLocationCalculatorConfigData()
config.depthThresholds.lowerThreshold = 0
config.depthThresholds.upperThreshold = 10000
config.roi = depthai.Rect(topLeft, bottomRight)
spatialLocationCalculator.initialConfig.addROI(config)
spatialLocationCalculator.out.link(xoutSpatialData.input)
xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)



# NN to demonstrate how to run inference on full FOV frames
nn = pipeline.create(depthai.node.MobileNetDetectionNetwork)
nn.setConfidenceThreshold(0.5)
nn.setBlobPath(str(blobconverter.from_zoo(name="mobilenet-ssd", shaves=6)))
manip.out.link(nn.input)

xoutNn = pipeline.create(depthai.node.XLinkOut)
xoutNn.setStreamName("nn")
nn.out.link(xoutNn.input)

xoutRgb = pipeline.create(depthai.node.XLinkOut)
xoutRgb.setStreamName("rgb")
nn.passthrough.link(xoutRgb.input)


with depthai.Device(pipeline) as device:
    q_rgb = device.getOutputQueue(name='rgb')
    qNn = device.getOutputQueue(name='nn')
    qIsp = device.getOutputQueue(name='isp')

    # Output queue will be used to get the depth frames from the outputs defined above
    depthQueue = device.getOutputQueue(name="depth", maxSize=10, blocking=False)
    spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=10, blocking=False)
    spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig")

    color = (255, 255, 255)
    frame = None

    # client = connect_mqtt()

    # cv2.namedWindow('HSV Tuner', cv2.WINDOW_AUTOSIZE)

    # cv2.createTrackbar('Lower H', "HSV Tuner", 0, 255, on_change)
    # cv2.createTrackbar('Higher H', "HSV Tuner", 0, 255, on_change)
    # cv2.createTrackbar('Lower S', "HSV Tuner", 0, 255, on_change)
    # cv2.createTrackbar('Higher S', "HSV Tuner", 0, 255, on_change)
    # cv2.createTrackbar('Lower V', "HSV Tuner", 0, 255, on_change)
    # cv2.createTrackbar('Higher V', "HSV Tuner", 0, 255, on_change)

    # cv2.setTrackbarPos('Lower H', "HSV Tuner", lowerH)
    # cv2.setTrackbarPos('Higher H', "HSV Tuner", upperH)
    # cv2.setTrackbarPos('Lower S', "HSV Tuner", lowerS)
    # cv2.setTrackbarPos('Higher S', "HSV Tuner", upperS)
    # cv2.setTrackbarPos('Lower V', "HSV Tuner", lowerV)
    # cv2.setTrackbarPos('Higher V', "HSV Tuner", upperV)

    controlQueue = device.getInputQueue('control')
    ctrl = depthai.CameraControl()
    ctrl.setManualExposure(expTime, sensIso)
    # ctrl.setAutoFocusMode(depthai.RawCameraControl.AutoFocusMode.OFF)
    # ctrl.setManualFocus(50)
    controlQueue.send(ctrl)

    cfg = depthai.SpatialLocationCalculatorConfig()

    avgDistance = 0
    avgAngle = 0

    def frameNorm(frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    def displayFrame(name, frame, detections):
        color = (255, 0, 0)
        for detection in detections:
            bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
            cv2.putText(frame, labelMap[detection.label], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        cv2.imshow(name, frame)

    while True:
        if qNn.has():
            dets = qNn.get().detections
            frame = q_rgb.get()
            f = frame.getCvFrame()
            displayFrame("rgb", f, dets)
        if qIsp.has():
            frame = qIsp.get()
            f = frame.getCvFrame()
            cv2.putText(f, str(f.shape), (20, 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255))
            cv2.imshow("isp", f)
        # lowerH = cv2.getTrackbarPos('Lower H', "HSV Tuner")
        # upperH = cv2.getTrackbarPos('Higher H', "HSV Tuner")

        # lowerS = cv2.getTrackbarPos('Lower S', "HSV Tuner")
        # upperS = cv2.getTrackbarPos('Higher S', "HSV Tuner")

        # lowerV = cv2.getTrackbarPos('Lower V', "HSV Tuner")
        # upperV = cv2.getTrackbarPos('Higher V', "HSV Tuner")

        in_rgb = q_rgb.tryGet()
        if in_rgb is not None:
            startTime = time.time()
            frame = in_rgb.getCvFrame()

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lowerThreshold = np.array([lowerH, lowerS, lowerV])
            upperThreshold = np.array([upperH, upperS, upperV])

            #check if color in range
            mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)

            result = cv2.bitwise_and(frame, frame, mask = mask)

            blur = cv2.GaussianBlur(mask, (5, 5), 0)

            edges = cv2.Canny(mask, 75, 150)

            contours, hierarchy = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

            roiList = []

            averageYPixels = 0

            for contour in contours:
                peri = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                rotatedRect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rotatedRect)
                boxArray = np.int0(box)

                cntArea = cv2.contourArea(contour)
                topLeftX = float(boxArray[0][0])
                topLeftY = float(boxArray[0][1])
                bottomRightX = float(boxArray[1][0])
                bottomRightY = float(boxArray[1][1])
                boxColor = (0,0,255)

                if(peri > 35 and cntArea > 50):
                    cv2.drawContours(frame,[boxArray],0,boxColor,2)

                    averageYPixels = averageYPixels + ((topLeftY + bottomRightY)/2)
                    
                    topLeft = depthai.Point2f(-0.005 + (topLeftX/cameraResolutionWidth), -0.005 + (topLeftY/cameraResolutionHeight))
                    bottomRight = depthai.Point2f(0.005 + (bottomRightX/cameraResolutionWidth), 0.005 + (bottomRightY/cameraResolutionHeight))

                    configData = depthai.SpatialLocationCalculatorConfigData()

                    configData.calculationAlgorithm = depthai.SpatialLocationCalculatorAlgorithm.MIN

                    configData.roi = depthai.Rect(topLeft, bottomRight)

                    roiList.append(configData)


            if(len(roiList) > 0):
                cfg.setROIs(roiList)
                averageYPixels = averageYPixels/len(roiList)
                # print(cfg)
                spatialCalcConfigInQueue.send(cfg)
            else:
                jsonString = '{"Distance":' + '-10' + ', "Angle":' + '-100000' + ', "Confidence":' + '0' + ', "Timestamp":' + str(time.time()) +'}'
                print("jsonString")
                # publish(client, jsonString)
                cv2.imshow("frame", frame)
                # cv2.imshow("mask", result)
                continue
            
            inDepthAvg = spatialCalcQueue.get() # blocking call, will wait until a new data has arrived
            spatialData = inDepthAvg.getSpatialLocations()

            zList = []
            xList = []
            yList = []

            pixelYAverage = 0
            pixelXAverage = 0

            pixelAdditionCounter = 0

            for depthData in spatialData:
                roi = depthData.config.roi
                roi = roi.denormalize(width=640, height=400)
                xmin = int(roi.topLeft().x)
                ymin = int(roi.topLeft().y)
                xmax = int(roi.bottomRight().x)
                ymax = int(roi.bottomRight().y)

                zCoordinateCamera = (depthData.spatialCoordinates.z)/25.4
                yCoordinateCamera = (depthData.spatialCoordinates.y)/25.4
                xCoordinateCamera = (depthData.spatialCoordinates.x)/25.4

                # print("BEFORE: " + str(yCoordinateCamera))

                convertedCoordinates = convertCoordinates(xCoordinateCamera, yCoordinateCamera, zCoordinateCamera)

                if(convertedCoordinates[0] != 0):
                    zList.append(convertedCoordinates[2])
                    xList.append(convertedCoordinates[0])
                    yList.append(convertedCoordinates[1])

            if(len(xList) != 0):
                xAverage = np.average(np.array(xList))
                yAverage = np.average(np.array(yList))
                bestGuess = np.array([xAverage + 26, yAverage])
                targetResult = minimize(reduceBestFitError, bestGuess, args = (xList, yList), method = "Nelder-Mead", options = {"maxiter": 10})
                targetCenterX = targetResult.x[0] + 8
                targetCenterY = targetResult.x[1]

                angle = math.atan(targetCenterY/targetCenterX)
                distanceToTargetHypotenuse = math.sqrt((targetCenterX ** 2) + (targetCenterY ** 2))

                targetHeight = 96

                distance = math.sqrt((distanceToTargetHypotenuse ** 2) - (targetHeight ** 2))

                angleToTargetCenterPixels = math.atan((averageYPixels - (cameraResolutionHeight/2)/focalLength))

                angleToGoal = angleToTargetCenterPixels + cameraMountAngleFromUpright

                pixelApproximatedDistance = targetHeightDifference/(math.tan(angleToGoal))

                print(pixelApproximatedDistance)

                jsonString = '{"Distance":' + str(distance) + ', "Angle":' + str(angle) + ', "Confidence":' + str(len(roiList)) + ', "Timestamp":' + str(time.time()) +'}'

                endTime = time.time()

                # print(startTime - endTime)

                # publish(client, jsonString)

                # print("TargetX: " + str(targetCenterX) + " TargetY: " + str(targetCenterY) + " Distance: " + str(distance) + " Angle: " + str(180 * (angle)/pi) + " Confidence: " + str(len(xList)))

            cv2.imshow("frame", frame)
 

        if cv2.waitKey(1) == ord('q'):
            break