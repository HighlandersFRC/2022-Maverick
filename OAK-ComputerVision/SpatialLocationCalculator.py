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

stepSize = 0.05

cameraResolutionWidth = 640.0
cameraResolutionHeight = 480.0

HFOV = 69

# focalLength = cameraResolutionWidth/(2 * tan())

# Camera elevation from horizontal, remember to change
cameraElevation = 40 * pi/180.0

targetHeightDifference = 83

# Start defining a pipeline
pipeline = depthai.Pipeline()

cam_rgb = pipeline.create(depthai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
# cam_rgb.setIspScale(2, 3)

xout_rgb = pipeline.create(depthai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

controlIn = pipeline.create(depthai.node.XLinkIn)
controlIn.setStreamName('control')
controlIn.out.link(cam_rgb.inputControl)

lowerH = 43
upperH = 166

lowerS = 221
upperS = 255

lowerV = 132
upperV = 255

expTime = 1000
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

# MonoCamera
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
stereo.setOutputSize(640, 480)

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

device = depthai.Device(pipeline)
device.startPipeline()

# Output queue will be used to get the depth frames from the outputs defined above
depthQueue = device.getOutputQueue(name="depth", maxSize=1, blocking=False)
spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=1, blocking=False)
spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig")

color = (255, 255, 255)

q_rgb = device.getOutputQueue("rgb")
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
ctrl.setAutoFocusMode(depthai.RawCameraControl.AutoFocusMode.OFF)
ctrl.setManualFocus(0)
controlQueue.send(ctrl)

cfg = depthai.SpatialLocationCalculatorConfig()

avgDistance = 0
avgAngle = 0

while True:
    # # print(device.getInputQueueNames())
    # # lowerH = cv2.getTrackbarPos('Lower H', "HSV Tuner")
    # # upperH = cv2.getTrackbarPos('Higher H', "HSV Tuner")

    # # lowerS = cv2.getTrackbarPos('Lower S', "HSV Tuner")
    # # upperS = cv2.getTrackbarPos('Higher S', "HSV Tuner")

    # # lowerV = cv2.getTrackbarPos('Lower V', "HSV Tuner")
    # # upperV = cv2.getTrackbarPos('Higher V', "HSV Tuner")
    # # getImuAngle()

    # in_rgb = q_rgb.tryGet()

    # if in_rgb is not None:
    #     frame = in_rgb.getCvFrame()
    #     startTime = time.time()
    #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #     lowerThreshold = np.array([lowerH, lowerS, lowerV])
    #     upperThreshold = np.array([upperH, upperS, upperV])

    #     #check if color in range
    #     mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)

    #     result = cv2.bitwise_and(frame, frame, mask = mask)

    #     blur = cv2.GaussianBlur(mask, (5, 5), 0)

    #     edges = cv2.Canny(mask, 75, 150)

    #     contours, hierarchy = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    #     roiList = []

    #     for contour in contours:
    #         peri = cv2.arcLength(contour, True)
    #         approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    #         rotatedRect = cv2.minAreaRect(contour)
    #         box = cv2.boxPoints(rotatedRect)
    #         boxArray = np.int0(box)

    #         cntArea = cv2.contourArea(contour)
    #         topLeftX = float(boxArray[0][0])
    #         topLeftY = float(boxArray[0][1])
    #         bottomRightX = float(boxArray[1][0])
    #         bottomRightY = float(boxArray[1][1])
    #         boxColor = (0,0,255)

    inDepth = depthQueue.get() # blocking call, will wait until a new data has arrived
    inDepthAvg = spatialCalcQueue.get() # blocking call, will wait until a new data has arrived
    
    depthFrame = inDepth.getFrame()
    depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
    depthFrameColor = cv2.equalizeHist(depthFrameColor)
    depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)

    spatialData = inDepthAvg.getSpatialLocations()
    # print(len(spatialData))
    for depthData in spatialData:
        roi = depthData.config.roi
        roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])
        xmin = int(roi.topLeft().x)
        ymin = int(roi.topLeft().y)
        xmax = int(roi.bottomRight().x)
        ymax = int(roi.bottomRight().y)

        fontType = cv2.FONT_HERSHEY_TRIPLEX
        cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)
        cv2.putText(depthFrameColor, f"X: {int(depthData.spatialCoordinates.x)} mm", (xmin + 10, ymin + 20), fontType, 0.5, color)
        cv2.putText(depthFrameColor, f"Y: {int(depthData.spatialCoordinates.y)} mm", (xmin + 10, ymin + 35), fontType, 0.5, color)
        cv2.putText(depthFrameColor, f"Z: {int((depthData.spatialCoordinates.z)/25.4)} in", (xmin + 10, ymin + 50), fontType, 0.5, color)
        print("Z: " + str((spatialData[0].spatialCoordinates.z)/25.4) + " X: " + str((spatialData[0].spatialCoordinates.x)) + " Y: " + str((spatialData[0].spatialCoordinates.y)))



    # cv2.imshow("depth", depthFrameColor)

    newConfig = False
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    elif key == ord('w'):
        if topLeft.y - stepSize >= 0:
            topLeft.y -= stepSize
            bottomRight.y -= stepSize
            newConfig = True
    elif key == ord('a'):
        if topLeft.x - stepSize >= 0:
            topLeft.x -= stepSize
            bottomRight.x -= stepSize
            newConfig = True
    elif key == ord('s'):
        if bottomRight.y + stepSize <= 1:
            topLeft.y += stepSize
            bottomRight.y += stepSize
            newConfig = True
    elif key == ord('d'):
        if bottomRight.x + stepSize <= 1:
            topLeft.x += stepSize
            bottomRight.x += stepSize
            newConfig = True

    if newConfig:
        config.roi = depthai.Rect(topLeft, bottomRight)
        cfg = depthai.SpatialLocationCalculatorConfig()
        cfg.addROI(config)
        spatialCalcConfigInQueue.send(cfg)