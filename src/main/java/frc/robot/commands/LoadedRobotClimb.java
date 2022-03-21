// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Climber;

public class LoadedRobotClimb extends CommandBase {
  private Carriage climber;
  private double inches;
  public LoadedRobotClimb(Carriage climber, double inches) {
    this.climber = climber;
    this.inches = inches;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // climber.setClimberPercents(-1);
    climber.loadedRobotClimb(inches);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if(Math.abs(climber.getclimberFalcon1Position() - inches) < 1) {
      return true;
    }
    return false;
  }
}
