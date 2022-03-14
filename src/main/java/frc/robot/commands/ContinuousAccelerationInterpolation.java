package frc.robot.commands;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.math.Vector;

import org.json.JSONArray;
import org.json.JSONObject;

public class ContinuousAccelerationInterpolation extends CommandBase {
    /** Creates a new ContinuousAccelerationInterpolation. */

    private Drive drive;

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

    private double odometryFusedX = 0;
    private double odometryFusedY = 0;
    private double odometryFusedTheta = 0;

    private JSONArray pathPointsJSON;

    private double[] desiredVelocityArray = new double[3];
    private double desiredThetaChange = 0;
    private boolean generateTurnPath;
    private double turnAngle;

    // private Peripherals peripherals = new Peripherals();

    private double cyclePeriod = 1.0/50.0;

    public ContinuousAccelerationInterpolation(Drive drive, JSONArray path, Boolean generateTurnPath) {
      this.drive = drive;
      this.pathPointsJSON = path;
      this.generateTurnPath = generateTurnPath;
      this.turnAngle = turnAngle;
      addRequirements(this.drive);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      if(generateTurnPath) {
        pathPointsJSON = drive.getJSONTurnPath(0);
      }
      initTime = Timer.getFPGATimestamp();
      // System.out.println("Time: " + currentTime + " Angle: " + drive.getOdometryAngle() + " OdometryX: " + currentX + " PredictedX: " + estimatedX + " OdometryY: " + currentY + " PredictedY: " + estimatedY);

    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Pathing...");
    // get odometry positions
    // currentX = drive.getOdometryX();
    // currentY = drive.getOdometryY();
    // currentTheta = drive.getOdometryAngle();

    odometryFusedX = drive.getFusedOdometryX();
    odometryFusedY = drive.getFusedOdometryY();
    odometryFusedTheta = drive.getFusedOdometryTheta();

    currentTime = Timer.getFPGATimestamp() - initTime;
    timeDiff = currentTime - previousTime;

    // determine current velocities based on current position minus previous position divided by time difference
    currentXVelocity = (currentX - previousX)/timeDiff;
    currentYVelocity = (currentY - previousY)/timeDiff;
    currentThetaVelocity = (currentTheta - previousTheta)/timeDiff;

    // // determine estimated position by integrating current velocity by time and adding previous estimated position
    // estimatedX = previousEstimateX + (cyclePeriod * currentXVelocity);
    // estimatedY = previousEstimateY + (cyclePeriod * currentYVelocity);
    // estimatedTheta = previousEstimateTheta + (cyclePeriod * currentThetaVelocity);

    // // average the odometry position with estimated position
    // averagedX = (estimatedX + currentX)/2;
    // averagedY = (estimatedY + currentY)/2;
    // averagedTheta = (estimatedTheta + currentTheta)/2;

    // System.out.println("Time: " + currentTime + " Angle: " + Math.toDegrees(currentTheta) + " OdometryX: " + currentX + " OdometryY: " + currentY);
    // System.out.println()
    // System.out.println("Time: " + currentTime + " " + drive.getDriveOdometry());

    // call ConstantAccelerationInterpolation function
    desiredVelocityArray = drive.constantAccelerationInterpolation(odometryFusedX, odometryFusedY, odometryFusedTheta, currentXVelocity, currentYVelocity, currentThetaVelocity, currentTime, timeDiff, pathPointsJSON);
    
    // create velocity vector and set desired theta change
    Vector velocityVector = new Vector(desiredVelocityArray[0], desiredVelocityArray[1]);
    desiredThetaChange = desiredVelocityArray[2];

    // System.out.println("@@@@@@@@@@@ " + Math.toDegrees(peripherals.getNavxAngle());

    // call autoDrive function to move the robot
    drive.autoDrive(velocityVector, desiredThetaChange);

    // set all previous variables to current variables to stay up to date
    previousX = currentX;
    previousY = currentY;
    previousTheta = currentTheta;
    previousTime = currentTime;
    // previousEstimateX = estimatedX;
    // previousEstimateY = estimatedY;
    // previousEstimateTheta = estimatedTheta;

    // System.out.println("ODOMETRY ANGLE: " + currentTheta + " THETA CHANGE" + desiredThetaChange);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.publishEndedPath();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(currentTime > pathPointsJSON.getJSONObject(pathPointsJSON.length() - 1).getDouble("time")){
      System.out.println("#####################################       X: " + drive.getFusedOdometryX() + " Y: " + drive.getFusedOdometryY() + " Angle: " + drive.getFusedOdometryTheta());
      return true;
    }
    return false;
  }
}
