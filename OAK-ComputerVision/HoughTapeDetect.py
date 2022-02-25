import cv2
import depthai
import numpy as np  # numpy - manipulate the packet data returned by depthai
import math
from math import cos, sin, tan, pi
from paho.mqtt import client as mqtt_client
from time import sleep
from scipy.optimize import minimize

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

# def getEstimatedTapes(numTapes):
#     xArray = []
#     yArray = []
#     match numTapes:
#         case 1:
#             xArray[0] = 
#             yArray[0] = 
#             xArray[1] = 
#             yArray[1] = 
#         case 2:
#             averageX = 

stepSize = 0.05

cameraResolutionWidth = 1280.0
cameraResolutionHeight = 720.0

HFOV = 69

# focalLength = cameraResolutionWidth/(2 * tan())

# Camera elevation from horizontal, remember to change
cameraElevation = 40 * pi/180.0
# real diff is 61
targetHeightDifference = 83

# Start defining a pipeline
pipeline = depthai.Pipeline()

cam_rgb = pipeline.create(depthai.node.ColorCamera)
cam_rgb.setPreviewSize(1280, 720)
cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setIspScale(2, 3)
# cam_rgb.setPreviewSize(cameraResolutionWidth, cameraResolutionHeight)
cam_rgb.setPreviewKeepAspectRatio(True)

# manipConfig = depthai.ImageManipConfig()
# manipConfig.setCropRect(0.2, 0.2, 0, 0)

# configQueue.send(manipConfig)
# manip = pipeline.create(depthai.node.ImageManip)

# manip.setResizeThumbnail(200,200, 200, 200, 200)

xout_rgb = pipeline.create(depthai.node.XLinkOut)
configIn = pipeline.create(depthai.node.XLinkIn)
configIn.setStreamName('config')
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

controlIn = pipeline.create(depthai.node.XLinkIn)
controlIn.setStreamName('control')
controlIn.out.link(cam_rgb.inputControl)

lowerH = 0
upperH = 94

lowerS = 142
upperS = 255

lowerV = 54
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
# setRectifyEdgeFillColor(-1)
stereo.setOutputSize(1280, 720)

stereo.setLeftRightCheck(lrcheck)
stereo.setSubpixel(subpixel)
# stereo.setExtendedDisparity(extended)

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

# imu = pipeline.create(depthai.node.IMU)
# xlinkOut = pipeline.create(depthai.node.XLinkOut)

# xlinkOut.setStreamName("imu")

# # imu.enableIMUSensor(depthai.IMUSensor.ROTATION_VECTOR, 400)
# imu.enableIMUSensor(depthai.IMUSensor.GAME_ROTATION_VECTOR, 400)

# imu.setBatchReportThreshold(1)

# imu.setMaxBatchReports(10)

# imu.out.link(xlinkOut.input)

# Pipeline defined, now the device is assigned and pipeline is started
device = depthai.Device(pipeline)
device.startPipeline()

# configQueue = device.getInputQueue('config')

# imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking = False)

# Output queue will be used to get the depth frames from the outputs defined above
depthQueue = device.getOutputQueue(name="depth", maxSize=10, blocking=False)
spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=10, blocking=False)
spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig")

color = (255, 255, 255)

q_rgb = device.getOutputQueue("rgb")
frame = None

client = connect_mqtt()

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


configQueue.send(manipConfig)

cfg = depthai.SpatialLocationCalculatorConfig()

avgDistance = 0
avgAngle = 0

while True:
    # print(device.getInputQueueNames())
    # lowerH = cv2.getTrackbarPos('Lower H', "HSV Tuner")
    # upperH = cv2.getTrackbarPos('Higher H', "HSV Tuner")

    # lowerS = cv2.getTrackbarPos('Lower S', "HSV Tuner")
    # upperS = cv2.getTrackbarPos('Higher S', "HSV Tuner")

    # lowerV = cv2.getTrackbarPos('Lower V', "HSV Tuner")
    # upperV = cv2.getTrackbarPos('Higher V', "HSV Tuner")
    # getImuAngle()

    inDepth = depthQueue.get() # blocking call, will wait until a new data has arrived
    
    depthFrame = inDepth.getFrame()
    depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
    depthFrameColor = cv2.equalizeHist(depthFrameColor)
    depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_OCEAN)

    in_rgb = q_rgb.tryGet()
    if in_rgb is not None:
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

        for contour in contours:
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
            rotatedRect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rotatedRect)
            boxArray = np.int0(box)
            topLeftX = float(boxArray[0][0])
            topLeftY = float(boxArray[0][1])
            bottomRightX = float(boxArray[1][0])
            bottomRightY = float(boxArray[1][1])
            boxColor = (0,0,255)

            if(peri > 35 and cv2.contourArea(contour) > 100 and cv2.contourArea(contour) < 1500):
                # cv2.drawContours(frame,[boxArray],0,boxColor,2)

                topLeft = depthai.Point2f(-0.01 + (topLeftX/cameraResolutionWidth), -0.01 + (topLeftY/cameraResolutionHeight))
                bottomRight = depthai.Point2f(0.01 + (bottomRightX/cameraResolutionWidth), 0.01 + (bottomRightY/cameraResolutionHeight))

                configData = depthai.SpatialLocationCalculatorConfigData()

                configData.calculationAlgorithm = depthai.SpatialLocationCalculatorAlgorithm.MIN

                # print(configData.calculationAlgorithm)

                configData.roi = depthai.Rect(topLeft, bottomRight)

                # print("TL: " + str(topLeft.x) + " BR: " + str(bottomRight.x))

                # print("TOP LEFT: " + str(topLeftX) + " BOTTOM RIGHT: " + str(bottomRightX))

                roiList.append(configData)

        # if(len(roiList) == 0):
        #     centerConfigData = depthai.SpatialLocationCalculatorConfigData()
        #     centerConfigData.roi = depthai.Rect(depthai.Point2f(0.49, 0.49), depthai.Point2f(0.51, 0.51))
        #     roiList.append(centerConfigData)

        if(len(roiList) > 0):
            cfg.setROIs(roiList)
            # print(cfg)
            spatialCalcConfigInQueue.send(cfg)

        inDepthAvg = spatialCalcQueue.get() # blocking call, will wait until a new data has arrived
        spatialData = inDepthAvg.getSpatialLocations()

        zList = []
        xList = []
        yList = []

        pixelYAverage = 0
        pixelXAverage = 0

        pixelAdditionCounter = 0

        # spatialCalcQueue.get
        print("====================================")
        for depthData in spatialData:
            roi = depthData.config.roi
            roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])
            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            # xCenterPixels = ((xMin + xMax) * cameraResolutionHeight)/2
            # yCenterPixels = ((yMin + yMax) * cameraResolutionWidth)/2

            # pixelAdditionCounter = pixelAdditionCounter + 1

            zCoordinateCamera = (depthData.spatialCoordinates.z)/25.4
            yCoordinateCamera = (depthData.spatialCoordinates.y)/25.4
            xCoordinateCamera = (depthData.spatialCoordinates.x)/25.4

            # print(depthData.depthAveragePixelCount)

            convertedCoordinates = convertCoordinates(xCoordinateCamera, yCoordinateCamera, zCoordinateCamera)

            # print("XRAW: " + str(xCoordinateCamera) + " Y: " + str(yCoordinateCamera) + " Z: " + str(zCoordinateCamera))

            # print("X: " + str(convertedCoordinates[0]) + " Y: " + str(convertedCoordinates[1]) + " Z: " + str(convertedCoordinates[2]))

            if(convertedCoordinates[0] != 0):
                zList.append(convertedCoordinates[2])
                xList.append(convertedCoordinates[0])
                yList.append(convertedCoordinates[1])

            fontType = cv2.FONT_HERSHEY_TRIPLEX
            cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)
            # cv2.putText(frame, f"X: {int(depthData.spatialCoordinates.x)} mm", (xmin + 10, ymin + 20), fontType, 0.5, color)
            # cv2.putText(frame, f"Y: {int(depthData.spatialCoordinates.y)} mm", (xmin + 10, ymin + 35), fontType, 0.5, color)
            # cv2.putText(frame, f"Z: {int(depthData.spatialCoordinates.z)} mm", (xmin + 10, ymin + 50), fontType, 0.5, color)

        # zMedian = np.median(zList)

        # for i in range(0, len(zList) - 1):
        #     if(abs(zList[i] - zMedian) >= 20):
        #         zList.remove(zList[i])
        #         xList.remove(xList[i])
        #         yList.remove(yList[i])

        # for i in range(0, len(zList)):
        #     factor = targetHeightDifference/zList[i]
        #     zList[i] = zList[i] * factor
        #     # xList[i] = xList[i] * factor
        #     # yList[i] = yList[i] * factor

        if(len(xList) != 0):
            xAverage = np.average(np.array(xList))
            yAverage = np.average(np.array(yList))
            bestGuess = np.array([xAverage + 26, yAverage])
            targetResult = minimize(reduceBestFitError, bestGuess, args = (xList, yList), method = "Nelder-Mead", options = {"maxiter": 10})
            targetCenterX = targetResult.x[0] + 8
            targetCenterY = targetResult.x[1]

            angle = math.atan(targetCenterY/targetCenterX)
            distance = math.sqrt((targetCenterX ** 2) + (targetCenterY ** 2))

            jsonString = '{"Distance":' + str(distance) + ', "Angle":' + str(angle) + ', "Confidence":' + str(len(xList)) + '}'

            publish(client, jsonString)

            print("TargetX: " + str(targetCenterX) + " TargetY: " + str(targetCenterY) + " Distance: " + str(distance) + " Angle: " + str(180 * (angle)/pi))

            # print(xList)
        
        # publish(client, "Hello")

            # if(abs(targetRadiusCheck - 576) > 100):
            #     print("Didn't get correct target")
            # else:
            # print("CenterX: " + str(targetCenterX) + "CenterY: " + str(targetCenterY))

        # print(xList)

        # cv2.imshow("depth", depthFrameColor)
        # cv2.imshow("frame", frame)
        # cv2.imshow("mask", result)
        # cv2.imshow("blur", blur)

        # sleep(0.01)

    # newConfig = False
    key = cv2.waitKey(1)
    if key == ord('q'):
        break