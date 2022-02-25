package frc.robot.tools;

import frc.robot.sensors.VisionCamera;
import frc.robot.subsystems.MqttSubscribe;
import frc.robot.subsystems.Peripherals;

public class RobotCoordinateCalculator {

    private Peripherals peripherals;

    private double fieldXCenter = 8.2296; // m
    private double fieldYCenter = 4.1148; // m

    private VisionCamera camera = new VisionCamera();
    private MqttSubscribe subscribe;
    
    public RobotCoordinateCalculator(Peripherals peripherals) {
        this.peripherals = peripherals;
        // this.subscribe = subscribe;
    }

    public double[] getCameraAdjustedCoordinates(double[] targetCenterCoordinates, double navxAngle) {
        double[] fieldCoordinateArray = new double[3];
        // double navxAngle = Math.toRadians(peripherals.getNavxAngle());

        double angleFromTarget = (navxAngle + Math.PI/2)%(Math.PI);

        double distToTarget = (targetCenterCoordinates[0]);
        // System.out.println("DISTANCE TO TARGET: " + distToTarget);
        // double yVal = targetCenterCoordinates[1];
        double targetAngleInFrame = targetCenterCoordinates[1];
        // System.out.println("HELLOOOOO");

        double confidenceFactor = targetCenterCoordinates[2];

        double totalAngle = angleFromTarget + targetAngleInFrame;

        // System.out.println("Distance: " + distToTarget + " Angle: " + navxAngle);

        double targetRelYCoordinate = distToTarget * (Math.sin(totalAngle));
        double targetRelXCoordinate = distToTarget * (Math.cos(totalAngle));

        // System.out.println("Angle: " + navxAngle + " X: " + targetRelXCoordinate + " Y: " + targetRelYCoordinate);

        double fieldXCoordinate = fieldXCenter - targetRelXCoordinate;
        double fieldYCoordinate = fieldYCenter - targetRelYCoordinate;

        fieldCoordinateArray[0] = fieldXCoordinate;
        fieldCoordinateArray[1] = fieldYCoordinate;
        fieldCoordinateArray[2] = confidenceFactor;

        // System.out.println("X: " + fieldXCoordinate + " Y: " + fieldYCoordinate);

            // System.out.println("ADFASHDKflasjdf" + fieldCoordinateArray);

            return fieldCoordinateArray;
        // fieldCoordinateArray[0] = -100;
        // fieldCoordinateArray[1] = -100;
        // fieldCoordinateArray[2] = 0;
        // System.out.println("NOT GETTING ANYTHING " + fieldCoordinateArray);
        // return fieldCoordinateArray;
    }

}