// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class ResetOdometry extends CommandBase {
  private Drive drive;
  private double resetX;
  private double resetY;
  public ResetOdometry(Drive drive, double x, double y) {
    this.drive = drive;
    this.resetX = x;
    this.resetY = y;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drive.setOdometryXYTheta(resetX, resetY);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
