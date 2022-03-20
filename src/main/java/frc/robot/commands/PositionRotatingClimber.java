// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class PositionRotatingClimber extends CommandBase {
  private Climber climber;
  private double degrees;
  public PositionRotatingClimber(Climber climber, double degrees) {
    this.climber = climber;
    this.degrees = degrees;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setRotatingMotorPosition(degrees);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return (Math.abs(climber.getRotatingMotorPosition() - degrees) < 3);
  }
}
