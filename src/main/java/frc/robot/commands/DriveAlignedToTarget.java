// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.math.Vector;

public class DriveAlignedToTarget extends CommandBase {
  private static Drive drive;  
  private static Peripherals peripherals;

  public DriveAlignedToTarget(Drive drive, Peripherals peripherals) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.peripherals = peripherals;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double cameraAngle = peripherals.getVisionArray()[1];

    double confidenceFactor = peripherals.getVisionArray()[2];

    double currentAngle = drive.getOdometryAngle();

    double angleDiff = (currentAngle + cameraAngle)/6;

    // drive.driveAutoAligned(angleDiff);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
