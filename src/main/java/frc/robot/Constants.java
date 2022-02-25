// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.dense.block.decomposition.chol.InnerCholesky_DDRB;

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

    public static final double TOP_SPEED = feetToMeters(16.3); // m/sec

    // used to be 6.75 for SDS mk4, 6.37 for Morris
    public static final double GEAR_RATIO = 6.75;

    // Morris 2.0 is 12.8
    public static final double STEER_GEAR_RATIO = 12.8;

    public static final double FALCON_TICS_PER_ROTATION = 2048;

    public static final double ROBOT_WIDTH = inchesToMeters(29); //m

    public static final double MODULE_OFFSET = inchesToMeters(3);

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
