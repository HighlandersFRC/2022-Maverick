package frc.robot.subsystems;

import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.commands.defaults.DriveDefault;
import frc.robot.tools.RobotCoordinateCalculator;
import frc.robot.tools.math.Vector;

public class Drive extends SubsystemBase {
    private final OI OI = new OI();

    // interpolation range
    private double turnTimePercent = 0.7;

    private final double interpolationCorrection = 0;

    private final double cyclePeriod = 1.0/50.0;

    private final double[] velocityArray = new double[3];

    // creating all the falcons
    private final WPI_TalonFX leftForwardMotor = new WPI_TalonFX(3);
    private final WPI_TalonFX leftForwardAngleMotor = new WPI_TalonFX(4);
    private final WPI_TalonFX leftBackMotor = new WPI_TalonFX(5);
    private final WPI_TalonFX leftBackAngleMotor = new WPI_TalonFX(6);
    private final WPI_TalonFX rightForwardMotor = new WPI_TalonFX(1);
    private final WPI_TalonFX rightForwardAngleMotor = new WPI_TalonFX(2);
    private final WPI_TalonFX rightBackMotor = new WPI_TalonFX(7);
    private final WPI_TalonFX rightBackAngleMotor = new WPI_TalonFX(8);

    private Peripherals peripherals;
    private MqttPublish publish;

    private double adjustedX = 0.0;
    private double adjustedY = 0.0;

    private final double moduleXY = ((Constants.ROBOT_WIDTH)/2) - Constants.MODULE_OFFSET;

    // creating all the external encoders
    private CANCoder backRightAbsoluteEncoder = new CANCoder(4);
    private CANCoder frontLeftAbsoluteEncoder = new CANCoder(2);
    private CANCoder frontRightAbsoluteEncoder = new CANCoder(1);
    private CANCoder backLeftAbsoluteEncoder = new CANCoder(3);

    // creating each swerve module with angle and drive motor, module number(relation to robot), and external encoder
    private final SwerveModule leftFront = new SwerveModule(2, leftForwardAngleMotor, leftForwardMotor, 0, frontLeftAbsoluteEncoder);
    private final SwerveModule leftBack = new SwerveModule(3, leftBackAngleMotor, leftBackMotor, 0, backLeftAbsoluteEncoder);
    private final SwerveModule rightFront = new SwerveModule(1, rightForwardAngleMotor, rightForwardMotor, 0, frontRightAbsoluteEncoder);
    private final SwerveModule rightBack = new SwerveModule(4, rightBackAngleMotor, rightBackMotor, 0, backRightAbsoluteEncoder);

    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(moduleXY, moduleXY);
    Translation2d m_frontRightLocation = new Translation2d(moduleXY, -moduleXY);
    Translation2d m_backLeftLocation = new Translation2d(-moduleXY, moduleXY);
    Translation2d m_backRightLocation = new Translation2d(-moduleXY, -moduleXY);

    private double currentX = 0;
    private double currentY = 0;
    private double currentTheta = 0;

    private double estimatedX = 0.0;
    private double estimatedY = 0.0;
    private double estimatedTheta = 0.0;

    private double previousEstimateX = 0.0;
    private double previousEstimateY = 0.0;
    private double previousEstimateTheta = 0.0;

    private double averagedX = 0.0;
    private double averagedY = 0.0;
    private double averagedTheta = 0.0;

    private double initTime;
    private double currentTime;
    private double previousTime;
    private double timeDiff;

    private double previousX = 0;
    private double previousY = 0;
    private double previousTheta = 0;

    private double currentXVelocity = 0;
    private double currentYVelocity = 0;
    private double currentThetaVelocity = 0;

    private double targetCenterX = 8.2296;
    private double targetCenterY = 4.1148;

    private double[] currentFusedOdometry = new double[3];


    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    SwerveDriveOdometry m_odometry; 
    Pose2d m_pose;

    double initAngle;
    double setAngle;
    double diffAngle;
    int pathNum = 1;

    public Drive(Peripherals peripherals, MqttPublish publish) {
        this.peripherals = peripherals;
        this.publish = publish;

        m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(Math.toRadians(peripherals.getNavxAngle())));
    }

    private RobotCoordinateCalculator coordinateCalculator = new RobotCoordinateCalculator(peripherals);

    public void zeroNavxMidMatch() {
        peripherals.zeroNavx();
        m_odometry.resetPosition(new Pose2d(new Translation2d(getFusedOdometryX(), getFusedOdometryY()), new Rotation2d(getFusedOdometryTheta())), new Rotation2d(getFusedOdometryTheta()));
    }

    // get each external encoder
    public double getLeftForwardEncoder() {
        return leftFront.getAbsolutePosition();
    }

    public double getLeftBackEncoder() {
        return leftBack.getAbsolutePosition();
    }

    public double getRightForwardEncoder() {
        return rightFront.getAbsolutePosition();
    }

    public double getRightBackEncoder() {
        return rightBack.getAbsolutePosition();
    }

    // get Joystick adjusted y-value
    public double getAdjustedY(double originalX, double originalY){
        double adjustedY = originalY * Math.sqrt((1-(Math.pow(originalX, 2))/2));
        return adjustedY;
    }

    // get Joystick adjusted x-value
    public double getAdjustedX(double originalX, double originalY){
        double adjustedX = originalX * Math.sqrt((1-(Math.pow(originalY, 2))/2));
        return adjustedX;
    }

    // method run on robot initialization
    public void init() {
        leftFront.init();
        leftBack.init();
        rightBack.init();
        rightFront.init();
        // peripherals.zeroNavx();

        setDefaultCommand(new DriveDefault(this));
    }

    public void autoInit(JSONArray pathPoints) {
        double firstPointAngle = pathPoints.getJSONObject(0).getDouble("angle");
        // System.out.println("FIRST POINT ANGLE: " + firstPointAngle);
        peripherals.setNavxAngle(Math.toDegrees(firstPointAngle));
        // System.out.println("************************ ANGLE SET TO: " + peripherals.getNavxAngle() + " ***************************");
        m_odometry.resetPosition(new Pose2d(new Translation2d(pathPoints.getJSONObject(0).getDouble("x"), pathPoints.getJSONObject(0).getDouble("y")),  new Rotation2d(firstPointAngle)), new Rotation2d(firstPointAngle));
        
        estimatedX = getOdometryX();
        estimatedY = getOdometryY();
        estimatedTheta = getOdometryAngle();

        previousEstimateX = estimatedX;
        previousEstimateY = estimatedY;
        previousEstimateTheta = estimatedTheta;

        currentX = getOdometryX();
        currentY = getOdometryY();
        currentTheta = getOdometryAngle();

        previousX = currentX;
        previousY = currentY;
        previousTheta = currentTheta;

        averagedX = (estimatedX + currentX)/2;
        averagedY = (estimatedY + currentY)/2;
        averagedTheta = (estimatedTheta + currentTheta)/2;

        initTime = Timer.getFPGATimestamp();

        updateOdometryFusedArray();
    }

    public JSONArray getJSONTurnPath() {
        double fusedOdometryX = getFusedOdometryX();
        double fusedOdometryY = getFusedOdometryY();

        JSONObject point1 = new JSONObject();
        point1.put("x", fusedOdometryX);
        point1.put("y", fusedOdometryY);
        point1.put("angle", getFusedOdometryTheta());
        point1.put("time", 0.0);

        double wantedAngleToTarget = Math.atan2((targetCenterY - fusedOdometryY), (targetCenterX - fusedOdometryX));

        JSONObject point2 = new JSONObject();
        point2.put("x", fusedOdometryX);
        point2.put("y", fusedOdometryY);
        point2.put("angle", wantedAngleToTarget);
        point2.put("time", 1);

        JSONArray turnPath = new JSONArray();
        turnPath.put(point1);
        turnPath.put(point2);

        System.out.println("TurnPath (Drive): " + turnPath);

        return turnPath;
    }

    public void updateOdometryFusedArray() {
        double navxOffset = Math.toRadians(peripherals.getNavxAngle());

        double[] cameraDistsToTarget = peripherals.getVisionArray();
        double[] cameraCoordinates = coordinateCalculator.getCameraAdjustedCoordinates(cameraDistsToTarget, navxOffset);

        double cameraXVal = cameraCoordinates[0];
        double cameraYVal = cameraCoordinates[1];
        double cameraConfidence = cameraCoordinates[2];

        // System.out.println("1: " + cameraXVal + " 2: " + cameraYVal);

        m_pose = m_odometry.update(new Rotation2d((navxOffset)), leftFront.getState((navxOffset)), rightFront.getState((navxOffset)), leftBack.getState((navxOffset)), rightBack.getState((navxOffset)));

        currentX = getOdometryX();
        currentY = getOdometryY();
        currentTheta = getOdometryAngle();

        currentTime = Timer.getFPGATimestamp() - initTime;
        timeDiff = currentTime - previousTime;

        // determine current velocities based on current position minus previous position divided by time difference
        currentXVelocity = (currentX - previousX)/timeDiff;
        currentYVelocity = (currentY - previousY)/timeDiff;
        currentThetaVelocity = (currentTheta - previousTheta)/timeDiff;

        // determine estimated position by integrating current velocity by time and adding previous estimated position
        estimatedX = previousEstimateX + (cyclePeriod * currentXVelocity);
        estimatedY = previousEstimateY + (cyclePeriod * currentYVelocity);
        estimatedTheta = previousEstimateTheta + (cyclePeriod * currentThetaVelocity);

        previousX = averagedX;
        previousY = averagedY;
        previousTheta = averagedTheta;
        previousTime = currentTime;
        previousEstimateX = estimatedX;
        previousEstimateY = estimatedY;
        previousEstimateTheta = estimatedTheta;

        cameraConfidence = 0;

        if(cameraConfidence == 0) {
            // System.out.println("Confidence: " + cameraConfidence);
            estimatedX = estimatedX * 0.5;
            estimatedY = estimatedY * 0.5;
            estimatedTheta = estimatedTheta * 0.5;

            currentX = currentX * 0.5;
            currentY = currentY * 0.5;
            currentTheta = currentTheta * 0.5;

            cameraXVal = cameraXVal * 0;
            cameraYVal = cameraYVal * 0;

            averagedX = estimatedX + currentX + cameraXVal;
            averagedY = estimatedY + currentY + cameraYVal;
            averagedTheta = currentTheta + estimatedTheta;

        }
        else if(cameraConfidence < 3) {
            // average the odometry position with estimated position
            estimatedX = estimatedX * 0.45;
            estimatedY = estimatedY * 0.45;
            estimatedTheta = estimatedTheta * 0.5;

            currentX = currentX * 0.45;
            currentY = currentY * 0.45;
            currentTheta = currentTheta * 0.5;

            cameraXVal = cameraXVal * 0.1;
            cameraYVal = cameraYVal * 0.1;

            averagedX = estimatedX + currentX + cameraXVal;
            averagedY = estimatedY + currentY + cameraYVal;
            averagedTheta = currentTheta + estimatedTheta;
        }
        else if(cameraConfidence == 3) {
            estimatedX = estimatedX * 0.425;
            estimatedY = estimatedY * 0.425;
            estimatedTheta = estimatedTheta * 0.5;

            currentX = currentX * 0.425;
            currentY = currentY * 0.425;
            currentTheta = currentTheta * 0.5;

            cameraXVal = cameraXVal * 0.15;
            cameraYVal = cameraYVal * 0.15;

            averagedX = estimatedX + currentX + cameraXVal;
            averagedY = estimatedY + currentY + cameraYVal;
            averagedTheta = currentTheta + estimatedTheta;
        }
        else if(cameraConfidence == 4) {
            estimatedX = estimatedX * 0.35;
            estimatedY = estimatedY * 0.35;
            estimatedTheta = estimatedTheta * 0.5;

            currentX = currentX * 0.35;
            currentY = currentY * 0.35;
            currentTheta = currentTheta * 0.5;

            cameraXVal = cameraXVal * 0.3;
            cameraYVal = cameraYVal * 0.3;

            averagedX = estimatedX + currentX + cameraXVal;
            averagedY = estimatedY + currentY + cameraYVal;
            averagedTheta = currentTheta + estimatedTheta;
        }
        else if(cameraConfidence == 5) {
            estimatedX = estimatedX * 0.3;
            estimatedY = estimatedY * 0.3;
            estimatedTheta = estimatedTheta * 0.5;

            currentX = currentX * 0.3;
            currentY = currentY * 0.3;
            currentTheta = currentTheta * 0.5; 

            cameraXVal = cameraXVal * 0.4;
            cameraYVal = cameraYVal * 0.4;

            averagedX = estimatedX + currentX + cameraXVal;
            averagedY = estimatedY + currentY + cameraYVal;
            averagedTheta = currentTheta + estimatedTheta;
        }
        else if(cameraConfidence > 5) {
            estimatedX = estimatedX * 0.5;
            estimatedY = estimatedY * 0.5;
            estimatedTheta = estimatedTheta * 0.5;

            currentX = currentX * 0.5;
            currentY = currentY * 0.5;
            currentTheta = currentTheta * 0.5;

            cameraXVal = cameraXVal * 0;
            cameraYVal = cameraYVal * 0;

            averagedX = estimatedX + currentX + cameraXVal;
            averagedY = estimatedY + currentY + cameraYVal;
            averagedTheta = currentTheta + estimatedTheta;
        }

        currentFusedOdometry[0] = averagedX;
        currentFusedOdometry[1] = averagedY;
        currentFusedOdometry[2] = averagedTheta;

        // m_odometry.resetPosition(new Pose2d(new Translation2d(averagedX, averagedY),  new Rotation2d(navxOffset)), new Rotation2d(navxOffset));
    }

    public double getFusedOdometryX() {
        return currentFusedOdometry[0];
    }

    public double getFusedOdometryY() {
        return currentFusedOdometry[1];
    }

    public double getFusedOdometryTheta() {
        return currentFusedOdometry[2];
    }

    public double getDistanceToTarget() {
        double xDist = targetCenterX - getFusedOdometryX();
        double yDist = targetCenterY - getFusedOdometryY();

        double distToTarget = Math.sqrt((Math.pow(xDist, 2) + Math.pow(yDist, 2)));

        return distToTarget;
    }

    public double getOdometryX() {
        // return currentFusedOdometry[0];
        return m_odometry.getPoseMeters().getX();
    }

    public double getOdometryY() {
        // return currentFusedOdometry[1];
        return m_odometry.getPoseMeters().getY();
    }

    public double getOdometryAngle() {
        // return currentFusedOdometry[2];
        return m_odometry.getPoseMeters().getRotation().getRadians();
    }

    public double getPeripheralsAngle() {
        return -(peripherals.getNavxAngle());
    }

    public void updateOdometry(double navxOffset) {
        m_pose = m_odometry.update(new Rotation2d(Math.toRadians(-navxOffset)), leftFront.getState(Math.toRadians(-navxOffset)), rightFront.getState(Math.toRadians(-navxOffset)), leftBack.getState(Math.toRadians(-navxOffset)), rightBack.getState(Math.toRadians(-navxOffset)));
    }

    public Pose2d getDriveOdometry() {
        return m_pose;
    }

    public double getShortestAngle(double point1Angle, double point2Angle) {
        double op1 = 0;
        double op2 = 0;
        if(point1Angle >= point2Angle) {
            op2 = ((Math.PI * 2) - (point1Angle - point2Angle));
            op1 = (point1Angle - point2Angle);
        }
        else {
            op1 = ((Math.PI * 2) - (point2Angle - point1Angle));
            op2 = (point2Angle - point1Angle);
        }

        if(op1 <= op2) {
            return -op1;
        }
        else {
            return op2;
        }
    }

    public void driveAutoAligned(double turnRadiansPerSec) {
        updateOdometryFusedArray();

        double originalX = -OI.getDriverLeftY();
        double originalY = -OI.getDriverLeftX();

        if(Math.abs(originalX) < 0.05) {
            originalX = 0;
        }
        if(Math.abs(originalY) < 0.05) {
            originalY = 0;
        }

        double navxOffset = Math.toRadians(peripherals.getNavxAngle());
        double xPower = getAdjustedX(originalX, originalY);
        double yPower = getAdjustedY(originalX, originalY);

        double xSpeed = xPower * Constants.TOP_SPEED;
        double ySpeed = yPower * Constants.TOP_SPEED;

        Vector controllerVector = new Vector(xSpeed, ySpeed);

        leftFront.velocityDrive(controllerVector, turnRadiansPerSec, navxOffset);
        rightFront.velocityDrive(controllerVector, turnRadiansPerSec, navxOffset);
        leftBack.velocityDrive(controllerVector, turnRadiansPerSec, navxOffset);
        rightBack.velocityDrive(controllerVector, turnRadiansPerSec, navxOffset);
    }

    // method to actually run swerve code
    public void teleopDrive() {
        updateOdometryFusedArray();

        // System.out.println("FUSED X: " + getFusedOdometryX() + " FUSED Y: " + getFusedOdometryY() + " Theta: " + getFusedOdometryTheta());

        double turnLimit = 1;
        // this is correct, X is forward in field, so originalX should be the y on the joystick
        double originalX = -OI.getDriverLeftY();
        double originalY = -OI.getDriverLeftX();

        if(Math.abs(originalX) < 0.05) {
            originalX = 0;
        }
        if(Math.abs(originalY) < 0.05) {
            originalY = 0;
        }

        double turn = -OI.getDriverRightX() * (Constants.TOP_SPEED)/(Constants.ROBOT_RADIUS);
        double navxOffset = Math.toRadians(peripherals.getNavxAngle());
        double xPower = getAdjustedX(originalX, originalY);
        double yPower = getAdjustedY(originalX, originalY);

        double xSpeed = xPower * Constants.TOP_SPEED;
        double ySpeed = yPower * Constants.TOP_SPEED;

        Vector controllerVector = new Vector(xSpeed, ySpeed);

        double[] odometryList = new double[3];

        odometryList[0] = getFusedOdometryX();
        odometryList[1] = getFusedOdometryY();
        odometryList[2] = getFusedOdometryTheta();

        // String strOdomList = (odometryList).toString();
        // System.out.println(strOdomList);

        // MqttMessage message = new MqttMessage(strOdomList.getBytes());

        // publish.publish("/pathTool", message);

        // System.out.println("X: " + getOdometryX() + " Y: " + getOdometryY() + " Theta: " + getOdometryAngle());

        leftFront.velocityDrive(controllerVector, turn, navxOffset);
        rightFront.velocityDrive(controllerVector, turn, navxOffset);
        leftBack.velocityDrive(controllerVector, turn, navxOffset);
        rightBack.velocityDrive(controllerVector, turn, navxOffset);

        // leftFront.testDrive();
        // rightFront.testDrive();
        // leftBack.testDrive();
        // rightBack.testDrive();

    }

    public void autoDrive(Vector velocityVector, double turnRadiansPerSec) {
        updateOdometryFusedArray();
        double navxOffset = Math.toRadians(peripherals.getNavxAngle());

        double[] odometryList = new double[3];

        odometryList[0] = getFusedOdometryX();
        odometryList[1] = getFusedOdometryY();
        odometryList[2] = getFusedOdometryTheta();

        // String strOdomList = (odometryList).toString();

        // MqttMessage message = new MqttMessage(strOdomList.getBytes());

        // publish.publish("/pathTool", message);

        // System.out.println("X: " + getFusedOdometryX() + " Y: " + getFusedOdometryY() + " Theta: " + getFusedOdometryTheta());

        // m_pose = m_odometry.update(new Rotation2d(Math.toRadians(-navxOffset)), leftFront.getState(Math.toRadians(-navxOffset)), rightFront.getState(Math.toRadians(-navxOffset)), leftBack.getState(Math.toRadians(-navxOffset)), rightBack.getState(Math.toRadians(-navxOffset)));
        // m_pose = m_odometry.update(new Rotation2d(navxOffset), leftFront.getState(navxOffset), rightFront.getState(navxOffset), leftBack.getState(navxOffset), rightBack.getState(navxOffset));

        leftFront.velocityDrive(velocityVector, turnRadiansPerSec, navxOffset);
        rightFront.velocityDrive(velocityVector, turnRadiansPerSec, navxOffset);
        leftBack.velocityDrive(velocityVector, turnRadiansPerSec, navxOffset);
        rightBack.velocityDrive(velocityVector, turnRadiansPerSec, navxOffset);
    }

    public double safeDivision(double numerator, double denominator) {
        if(Math.abs(denominator) < 0.00001) {
            return 0.0;
        }
        double dividend = numerator/denominator;
        return dividend;
    }

    public double[] constantAccelerationInterpolation(double currentX, double currentY, double currentTheta, double currentXVelocity, double currentYVelocity, double currentThetaVelocity, double time, double timeSinceLastCycle, JSONArray pathPointsJSON) {
        // create the 3 variables for 3 points of interest
        JSONObject currentPoint;
        JSONObject nextPoint;
        JSONObject previousPoint;

        double[] returnArray = new double[3];
        int currentPointIndex = 0;
        double lowestTimeDiff = 0;

        double[] timeDiffArray = new double[pathPointsJSON.length()];

        // search through list of points to find the closest point to us by time
        for(int i = 0; i < pathPointsJSON.length(); i++) {
            timeDiffArray[i] = Math.abs(time - pathPointsJSON.getJSONObject(i).getDouble("time"));
        }
        for(int i = 0; i < timeDiffArray.length; i++) {
            if(i == 0) {
                lowestTimeDiff = timeDiffArray[0];
                currentPointIndex = 0;
            }
            else {
                if(timeDiffArray[i] < lowestTimeDiff) {
                    lowestTimeDiff = timeDiffArray[i];
                    currentPointIndex = i;
                }
            }
        }

        // if the current time is more than the time of the last point, don't move
        if(time > pathPointsJSON.getJSONObject(pathPointsJSON.length() - 1).getDouble("time")) {
            double velocityX = 0;
            double velocityY = 0;
            double thetaChange = 0;

            returnArray[0] = velocityX;
            returnArray[1] = velocityY;
            returnArray[2] = thetaChange;

            // Vector velocityVector = new Vector(velocityX, velocityY);
            // autoDrive(velocityVector, thetaChange);    
            return returnArray;        
        }

        currentPoint = pathPointsJSON.getJSONObject(currentPointIndex);

        double sign = 1;
        double angleDifference = 0;

        for(int i = 0; i < pathPointsJSON.length() - 1; i++) {
            if((pathPointsJSON.getJSONObject(i).getDouble("time") <= time) && (pathPointsJSON.getJSONObject(i + 1).getDouble("time") > time)) {
                double point1Angle = pathPointsJSON.getJSONObject(i).getDouble("angle");
                double point2Angle = pathPointsJSON.getJSONObject(i + 1).getDouble("angle");

                angleDifference = getShortestAngle(currentTheta, point2Angle);
            }
        }

        // System.out.println("DIFFERENCE: " + angleDifference);

        double currentPointTime = currentPoint.getDouble("time");

        double velocityX = 0;
        double velocityY = 0;
        double thetaChange = 0;

        // if the current point is the last point in the path, continue at same velocity towards point
        if(currentPointIndex + 1 >= pathPointsJSON.length()) {
            previousPoint = pathPointsJSON.getJSONObject(currentPointIndex - 1);
            // check if within one cycle of endpoint and set velocities
            // only occurs when interpolation range is 1
            if(Math.abs((currentPointTime - time)) < cyclePeriod) {
                // System.out.println("Less than cycle");
                velocityX = currentXVelocity;
                velocityY = currentYVelocity;
                thetaChange = currentThetaVelocity;
                // System.out.println("INSIDE CYCLE PERIOD BLOCK")
            }
            // otherwise set velocity by checking difference between wanted position and current position divided by time
            else {
                velocityX = (currentPoint.getDouble("x") - currentX)/(currentPointTime - time);
                velocityY = (currentPoint.getDouble("y") - currentY)/(currentPointTime - time);
                thetaChange = (angleDifference)/(currentPointTime - time);
                // System.out.println("POINT THETA: " + currentPoint.getDouble("angle") + " ACTUAL ANGLE: " + currentTheta + " POINT TIME: " + currentPointTime + " TIME: " + time);
            }
            velocityArray[0] = velocityX;
            velocityArray[1] = velocityY;
            velocityArray[2] = thetaChange;
            return velocityArray;

            // System.out.println("Time: " + time + " VelocityX: " + velocityVector.getI() + " VelocityY: " + velocityVector.getJ() + " ThetaChange: " + thetaChange + " CurentX: " + currentX + " CurrentY: " + currentY);
        }
        // if the current point is the first point in the path, continue at same velocity towards point
        else if(currentPointIndex - 1 < 0) {
            nextPoint = pathPointsJSON.getJSONObject(currentPointIndex + 1);
            System.out.println(nextPoint.toString());
            double nextPointTime = nextPoint.getDouble("time");
            // check if within one cycle of endpoint and set velocities
            // only occurs when interpolation range is 1
            if(Math.abs((nextPointTime - time)) < cyclePeriod) {
                velocityX = currentXVelocity;
                velocityY = currentYVelocity;
                thetaChange = currentThetaVelocity;
            }
            // otherwise set velocity by checking difference between next point and current position divided by time
            else {
                velocityX = (nextPoint.getDouble("x") - currentX)/(nextPointTime - time);
                velocityY = (nextPoint.getDouble("y") - currentY)/(nextPointTime - time);
                thetaChange = (angleDifference)/(nextPointTime - time);
                // System.out.println("FIRST P IN PATH" + " POINT THETA: " + currentPoint.getDouble("angle") + " ACTUAL ANGLE: " + currentTheta + " POINT TIME: " + currentPointTime + " TIME: " + time);
            }
            velocityArray[0] = velocityX;
            velocityArray[1] = velocityY;
            velocityArray[2] = thetaChange;
            return velocityArray;
            
            // System.out.println("Time: " + time +  " VelocityX: " + velocityVector.getI() + " VelocityY: " + velocityVector.getJ() + " ThetaChange: " + thetaChange + " CurentX: " + currentX + " CurrentY: " + currentY);
        }

        try{
            turnTimePercent = currentPoint.getDouble("interpolationRange");
        }
        catch(Exception e) {
            turnTimePercent = 0;
        }

        // determine which points are the next point and previous point
        nextPoint = pathPointsJSON.getJSONObject(currentPointIndex + 1);
        previousPoint = pathPointsJSON.getJSONObject(currentPointIndex - 1);

        System.out.println(nextPoint.toString());

        double nextPointTime = nextPoint.getDouble("time");
        double previousPointTime = previousPoint.getDouble("time");

        // calculate difference in times between current point and previous point
        double timeDiffT1 = (currentPointTime - previousPointTime);
        // calculate the difference in times between next point and current point
        double timeDiffT2 = nextPointTime - currentPointTime;

        double minTimeDiff = Math.min(timeDiffT1, timeDiffT2);

        // t1 is the point at which the robot will start a curved trajectory towards the next line segment (based on interpolation range)
        double t1 = currentPointTime - (minTimeDiff * turnTimePercent);

        // System.out.println("T1: " + )

        // t2 is the point at which the robot will end its curved trajectory towards the next line segment (based on interpolation range)
        double t2 = currentPointTime + (minTimeDiff * (turnTimePercent));

        // if on a line segment between previous point and current point
        if(time < t1) {
            // check if within one cycle of endpoint and set velocities
            // only occurs when interpolation range is 1
            if(Math.abs((currentPointTime - time)) < cyclePeriod) {
                velocityX = currentXVelocity;
                velocityY = currentYVelocity;
                thetaChange = currentThetaVelocity;
            }
            // otherwise set velocity by checking difference between current point and current position divided by time
            else {
                // System.out.println("NOT WITHIN CYCLE PERIOD");
                velocityX = (currentPoint.getDouble("x") - currentX)/(currentPointTime - time);
                velocityY = (currentPoint.getDouble("y") - currentY)/(currentPointTime - time);
                thetaChange = (angleDifference)/(currentPointTime - time);
            }
        }
        // if in the interpolation range and curving towards next line segment between current point and next point
        else if(time >= t1 && time < t2) {
            // System.out.println("||||||||||||||||||||||||||");
            // VELOCITIES
            // determine velocities when on line segment going towards t1
            double t1X = (currentPoint.getDouble("x") - previousPoint.getDouble("x"))/timeDiffT1;
            double t1Y = (currentPoint.getDouble("y") - previousPoint.getDouble("y"))/timeDiffT1;
            double t1Theta = (currentPoint.getDouble("angle") - previousPoint.getDouble("angle"))/timeDiffT1;

            // double t1Angle = previousPoint.getDouble("angle") + ((currentPoint.getDouble("angle") - previousPoint.getDouble("angle")) * turnTimePercent);
            double t1Angle = (getShortestAngle(previousPoint.getDouble("angle"), currentPoint.getDouble("angle")) * turnTimePercent) + previousPoint.getDouble("angle");
            // double t2Angle = currentPoint.getDouble("angle") + ((nextPoint.getDouble("angle") - currentPoint.getDouble("angle")) * (1 - turnTimePercent));
            double t2Angle = (getShortestAngle(currentPoint.getDouble("angle"), nextPoint.getDouble("angle")) * turnTimePercent) + currentPoint.getDouble("angle");

            // System.out.println("T1: " + t1Angle + " T2: " + t2Angle);

            // VELOCITIES
            // determine velocities when on line segment after t2 and heading towards next point
            double t2X = (nextPoint.getDouble("x") - currentPoint.getDouble("x"))/timeDiffT2;
            double t2Y = (nextPoint.getDouble("y") - currentPoint.getDouble("y"))/timeDiffT2;
            double t2Theta = (nextPoint.getDouble("angle") - currentPoint.getDouble("angle"))/timeDiffT2;

            angleDifference = getShortestAngle(t1Angle, t2Angle);

            // determine the ideal accelerations on interpolation curve
            double idealAccelX = (t2X - t1X)/(t2 - t1);
            double idealAccelY = (t2Y - t1Y)/(t2- t1);
            double idealAccelTheta = (angleDifference)/(t2 - t1);

            // determine (x,y,theta) position at t1
            double t1XPosition = (t1X * t1) + previousPoint.getDouble("x");
            double t1YPosition = (t1Y * t1) + previousPoint.getDouble("y");
            double t1ThetaPosition = (t1Theta * t1) + previousPoint.getDouble("angle");

            // this is the time of interpolation and is the time one cycle ahead of where we are currently
            double interpTime = time - t1 + cyclePeriod;

            // determine ideal x position at a given time in the interpolation range
            double idealPositionX = (idealAccelX/2) * (Math.pow(interpTime, 2)) + t1X * (interpTime) + t1XPosition;
            double idealPositionY = (idealAccelY/2) * (Math.pow(interpTime, 2)) + t1Y * (interpTime) + t1YPosition;
            double idealPositionTheta = (idealAccelTheta/2) * (Math.pow(interpTime, 2)) + t1Theta * (interpTime) + t1ThetaPosition;

            // determine the ideal velocity at a given time in the interpolation range based on the ideal acceleration at the time
            double idealVelocityX = t1X + (idealAccelX * interpTime);
            double idealVelocityY = t1Y + (idealAccelY * interpTime);

            double idealVelocityTheta = t1Theta + (idealAccelTheta * interpTime);

            // determine the desired velocity of the robot based on difference between our ideal position and current position time a correction coefficient + the ideal velocity
            velocityX = (idealPositionX - currentX) * interpolationCorrection + idealVelocityX;
            velocityY = (idealPositionY - currentY) * interpolationCorrection + idealVelocityY;
            thetaChange = (idealPositionTheta - currentTheta) * interpolationCorrection + idealVelocityTheta;
        }
        // if past the interpolation range and on the line segment towards next point
        else if(time >= t2) {
            // check if within one cycle of endpoint and set velocities
            // only occurs when interpolation range is 1
            if(Math.abs((nextPointTime - time)) < cyclePeriod) {
                velocityX = currentXVelocity;
                velocityY = currentYVelocity;
                thetaChange = currentThetaVelocity;
            }
            // otherwise set velocity by checking difference between next point and current position divided by time
            else {
                velocityX = (nextPoint.getDouble("x") - currentX)/(nextPointTime - time);
                velocityY = (nextPoint.getDouble("y") - currentY)/(nextPointTime - time);
                thetaChange = (angleDifference)/(nextPointTime - time);
            }
        }
        velocityArray[0] = velocityX;
        velocityArray[1] = velocityY;
        velocityArray[2] = thetaChange;
        return velocityArray;         
        // return true;
    }

    private Command DriveDefault() {
        return null;
    }
 
    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}