package frc.robot.tools;

import frc.robot.Constants;
import frc.robot.subsystems.Peripherals;

public class RobotCoordinateCalculator {

    private double fieldXCenter = 8.2296; // m
    private double fieldYCenter = 4.1148; // m
    
    public RobotCoordinateCalculator(Peripherals peripherals) {
        // this.subscribe = subscribe;
    }

    public double[] getCameraAdjustedCoordinates(double[] targetCenterCoordinates, double navxAngle) {
        double[] fieldCoordinateArray = new double[2];
        // double navxAngle = Math.toRadians(peripherals.getNavxAngle());

        // double angleFromTarget = (navxAngle + Math.PI/2)%(Math.PI);

        double distToTarget = Constants.inchesToMeters(targetCenterCoordinates[0]);

        double totalAngle = targetCenterCoordinates[1];

        // System.out.println("Distance: " + distToTarget + " Angle: " + navxAngle);

        double targetRelYCoordinate = distToTarget * (Math.sin(totalAngle));
        double targetRelXCoordinate = distToTarget * (Math.cos(totalAngle));

        // System.out.println("Angle: " + totalAngle + " X: " + targetRelXCoordinate + " Y: " + targetRelYCoordinate);

        double fieldXCoordinate = fieldXCenter - targetRelXCoordinate;
        double fieldYCoordinate = fieldYCenter - targetRelYCoordinate;

        fieldCoordinateArray[0] = fieldXCoordinate;
        fieldCoordinateArray[1] = fieldYCoordinate;

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