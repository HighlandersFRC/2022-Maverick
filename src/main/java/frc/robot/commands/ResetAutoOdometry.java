// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.json.JSONArray;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class ResetAutoOdometry extends CommandBase {
  /** Creates a new ResetAutoOdometry. */
  private Drive drive;
  private JSONArray pathJSON;
  public ResetAutoOdometry(Drive drive, JSONArray pathJSON) {
    this.drive = drive;
    this.pathJSON = pathJSON;
    addRequirements(this.drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" + pathJSON);
    drive.autoInit(pathJSON);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
