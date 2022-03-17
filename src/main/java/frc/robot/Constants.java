// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double ROBOT_RADIUS = inchesToMeters(16); //m

    public static final double WHEEL_DIAMETER = inchesToMeters(4); //m

    public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER; // m

    // public static final double TOP_SPEED = feetToMeters(16.3); // m/sec

    public static final double TOP_SPEED = 4; // m/sec

    // used to be 6.75 for SDS mk4, 6.37 for Morris
    public static final double GEAR_RATIO = 6.75;

    // Morris 2.0 is 12.8
    public static final double STEER_GEAR_RATIO = 12.8;

    public static final double FALCON_TICS_PER_ROTATION = 2048;

    public static final double ROBOT_WIDTH = inchesToMeters(29); //m

    public static final double MODULE_OFFSET = inchesToMeters(3);

    //{distance(inches), hood angle, RPM}
    public static final double[][] SHOOTING_LOOKUP_TABLE = {
        {58.0, 4.0, 1400.0},
        {63.0, 6.0, 1400.0},
        {67.0, 7.25, 1400.0},
        {73.0, 8.5, 1430.0},
        {79.0, 8.5, 1430.0},
        {84.0, 9.0, 1440.0},
        {88.0, 10.0, 1440.0},
        {93.0, 11.25, 1470.0},
        {98.0, 12, 1470.0},
        {104.0, 12.0, 1470.0},
        {109.0, 12.75, 1500.0},
        {116.0, 14.25, 1530.0},
        {121.0, 15.0, 1530.0},
        {129.0, 17.0, 1530.0},
        {136.0, 18.0, 1530.0},
        {142.0, 19.0, 1530.0},
        {147.0, 19.5, 1530.0},
        {157.0, 21.5, 1560.0},
        {167.0, 24.25, 1560.0}
    };
    
    public static double[] getShooterValues(double distance) {
        SmartDashboard.putNumber("Input dist", distance);
        double minDif = 9999999;
        int index = 0;
        for (int i = 0; i < SHOOTING_LOOKUP_TABLE.length; i ++) {
            if (Math.abs(SHOOTING_LOOKUP_TABLE[i][0] - distance) < minDif) {
                minDif = Math.abs(SHOOTING_LOOKUP_TABLE[i][0] - distance);
                index = i;
                SmartDashboard.putBoolean("INSIDE: " + i, true);
            }
            else {
                SmartDashboard.putBoolean("INSIDE: " + i, false);
            }
            // SmartDashboard.putNumber("MIN DIST" + i, minDif);
            SmartDashboard.putNumber("Index", index);
        }
        
        double[] shootingValues = {SHOOTING_LOOKUP_TABLE[index][1], SHOOTING_LOOKUP_TABLE[index][2]};
        System.out.println(SHOOTING_LOOKUP_TABLE[index][0]);
        return shootingValues;
        /*

        //If the distance is closer than the first setpoint
        if (distance < SHOOTING_LOOKUP_TABLE[0][0]) {
            return {SHOOTING_LOOKUP_TABLE[0][1], SHOOTING_LOOKUP_TABLE[0][2]};
        } else if (distance > SHOOTING_LOOKUP_TABLE[-1][0]) {
            return {SHOOTING_LOOKUP_TABLE[-1][1], SHOOTING_LOOKUP_TABLE[-1][2]};
        } else {
            for (int i = 0; i < SHOOTING_LOOKUP_TABLE.length; i ++) {
                if (distance > SHOOTING_LOOKUP_TABLE[i][0] && distance < SHOOTING_LOOKUP_TABLE[i + 1]) {
                    //Calculate where distance is between setpoints
                    double leftDif = distance - SHOOTING_LOOKUP_TABLE[i][0];
                    double percent = leftDif / (SHOOTING_LOOKUP_TABLE[i + 1][0] - SHOOTING_LOOKUP_TABLE[i][0]);

                    double hood1 = SHOOTING_LOOKUP_TABLE[i][1];
                    double rpm1 = SHOOTING_LOOKUP_TABLE[i][2];
                    double hood2 = SHOOTING_LOOKUP_TABLE[i + 1][1];
                    double rpm2 = SHOOTING_LOOKUP_TABLE[i + 1][2];

                    //Interpolate in-between values for hood angle and shooter rpm
                    double newHood = hood1 + (percent * (hood2 - hood1));
                    double newRPM = rpm1 + (percent * (rpm2 - rpm1));
                    
                    return {newHood, newRPM};
                }
            }
        }

        */
    }

    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public static double feetToMeters(double feet) {
        double inches = feet * 12;
        return inchesToMeters(inches);
    }

    public static int shooterRPMToUnitsPer100MS(double rpm) {
        return (int) (Math.round((rpm / 600.0) * FALCON_TICS_PER_ROTATION));
    }

    public static double unitsPer100MsToRPM(double units) {
        return (units * 600) / (Constants.FALCON_TICS_PER_ROTATION);


    }

}
