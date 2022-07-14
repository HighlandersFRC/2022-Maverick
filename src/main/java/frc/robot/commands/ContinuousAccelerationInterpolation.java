package frc.robot.commands;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.tools.math.Vector;

import org.json.JSONArray;

public class ContinuousAccelerationInterpolation extends CommandBase {
    /** Creates a new ContinuousAccelerationInterpolation. */

    private Drive drive;
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

    private double previousVelocityX = 0;
    private double previousVelocityY = 0;
    private double previousVelocityTheta = 0;

    private double[] desiredVelocityArray = new double[3];
    private double desiredThetaChange = 0;
    private boolean generateTurnPath;

    private FileWriter writer;
    private BufferedWriter out;
    String loc;

    public List<String> pointsList = new ArrayList<String>();

    // private Peripherals peripherals = new Peripherals();

    public ContinuousAccelerationInterpolation(Drive drive, JSONArray path, Boolean generateTurnPath) {
      this.drive = drive;
      this.pathPointsJSON = path;
      this.generateTurnPath = generateTurnPath;
      addRequirements(this.drive);
      loc = "/home/lvuser/deploy/odometryList.csv";
      try {
        writer = new FileWriter(loc, true);
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      
      try{
        out = new BufferedWriter(writer);
      }
        catch(Exception e) {

    }
      if(generateTurnPath) {
        pathPointsJSON = drive.testGetCameraTurn();
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
    currentXVelocity = (odometryFusedX - previousX)/timeDiff;
    currentYVelocity = (odometryFusedY - previousY)/timeDiff;
    currentThetaVelocity = (odometryFusedTheta - previousTheta)/timeDiff;

    currentXVelocity = previousVelocityX;
    currentYVelocity = previousVelocityY;
    currentThetaVelocity = previousVelocityTheta;

    // call ConstantAccelerationInterpolation function
    desiredVelocityArray = drive.constantAccelerationInterpolation(odometryFusedX, odometryFusedY, odometryFusedTheta, currentXVelocity, currentYVelocity, currentThetaVelocity, currentTime, timeDiff, pathPointsJSON);
    
    // create velocity vector and set desired theta change
    Vector velocityVector = new Vector(desiredVelocityArray[0], desiredVelocityArray[1]);
    desiredThetaChange = desiredVelocityArray[2];

    // System.out.println("@@@@@@@@@@@ " + Math.toDegrees(peripherals.getNavxAngle());

    String strOdomList = (odometryFusedX + "," + odometryFusedY + ","+ odometryFusedTheta + "," + currentTime + "," + desiredVelocityArray[0] + "," + desiredVelocityArray[1] + "," + desiredVelocityArray[2] + "\n");

        try {
          pointsList.add(strOdomList);
          // out.write(strOdomList);
          // out.close();
        }
        catch(Exception e) {
 
        }

    System.out.println(strOdomList);

    try {
      out.write(strOdomList);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    // call autoDrive function to move the robot
    drive.autoDrive(velocityVector, desiredThetaChange);

    // set all previous variables to current variables to stay up to date
    previousX = odometryFusedX;
    previousY = odometryFusedY;
    previousTheta = odometryFusedTheta;
    previousTime = currentTime;

    previousVelocityX = desiredVelocityArray[0];
    previousVelocityY = desiredVelocityArray[1];
    previousVelocityTheta = desiredThetaChange;
    // previousEstimateX = estimatedX;
    // previousEstimateY = estimatedY;
    // previousEstimateTheta = estimatedTheta;

    // System.out.println("ODOMETRY ANGLE: " + currentTheta + " THETA CHANGE" + desiredThetaChange);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.autoDrive(new Vector(0, 0), 0);
  //   try {
  //     for(int i = 0; i < pointsList.size(); i++) {
  //       out.write(pointsList.get(i));
  //     }
  //     // out.write(pointsList);
  //     writer.close();
  //     // out.close();
  // } catch (IOException e) {
  //     // TODO Auto-generated catch block
  //     e.printStackTrace();
  // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((Timer.getFPGATimestamp() - initTime) > pathPointsJSON.getJSONObject(pathPointsJSON.length() - 1).getDouble("time")){
      System.out.println("#####################################       X: " + drive.getFusedOdometryX() + " Y: " + drive.getFusedOdometryY() + " Angle: " + drive.getFusedOdometryTheta());
      return true;
    }
    return false;
  }
}
