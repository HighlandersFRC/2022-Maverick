// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class VisionAlignment extends CommandBase {
  /** Creates a new VisionAlignment. */
  private Drive drive;
  private Peripherals peripherals;
  private PID pid = new PID(5, 0, 0);

  private double currentCameraAngle = 0;

  private double[] cameraCoordinates;

  private double cameraConfidence;

  private double maxTurnSpeedRadians = Constants.TOP_SPEED/Constants.ROBOT_RADIUS;

  private int lostCameraConfidence = 0;

  private double pidResult = 0;
  public VisionAlignment(Drive drive, Peripherals peripherals) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.peripherals = peripherals;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetPoint(0);
    pid.setMaxOutput(maxTurnSpeedRadians);
    pid.setMinOutput(-maxTurnSpeedRadians);
    lostCameraConfidence = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cameraCoordinates = peripherals.getVisionArray();
    cameraConfidence = cameraCoordinates[2];
    currentCameraAngle = cameraCoordinates[1];


    System.out.println("CAM ANGLE: " + currentCameraAngle);

    // if(cameraConfidence != 0) {
    //   pid.updatePID(currentCameraAngle);
    //   pidResult = -pid.getResult();
    //   System.out.println("PID RESULT: " + pidResult);

    //   drive.autoDrive(new Vector(0, 0), pidResult);
    // }
    // else {
    //   lostCameraConfidence++;
    // }

    if(cameraConfidence != 0) {
        if(currentCameraAngle > 0.1) {
            drive.autoDrive(new Vector(0, 0), -1.57);
        }
        else if(currentCameraAngle < -0.1) {
            drive.autoDrive(new Vector(0, 0), 1.57);
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(currentCameraAngle) < 0.1 && currentCameraAngle != 0) {
      System.out.println("FINISHED");
      return true;
    }
    return false;
  }
}

