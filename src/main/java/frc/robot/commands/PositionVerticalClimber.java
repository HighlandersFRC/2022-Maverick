// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class PositionVerticalClimber extends CommandBase {
  private Climber climber;
  private double position;
  public PositionVerticalClimber(Climber climber, double position) {
    this.climber = climber;
    this.position = position;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setClimberFalconsPosition(position);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
