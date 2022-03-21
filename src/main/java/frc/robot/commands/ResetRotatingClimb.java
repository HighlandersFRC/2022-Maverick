// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ResetRotatingClimb extends CommandBase {
  private Climber climber;
  private double percent;
  public ResetRotatingClimb(Climber climber) {
    this.climber = climber;
    this.percent = percent;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SmartDashboard.putNumber("Current", climber.getRotatingClimberCurrent());
    climber.setRotatingMotorPercent(-0.15);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if(climber.getRotatingClimberCurrent() > 1) {
      return true;
    }
    return false;
  }
}
