// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Climber;

public class PositionVerticalClimber extends CommandBase {
  private Carriage climber;
  private double inches;
  public PositionVerticalClimber(Carriage climber, Climber carriage, double inches) {
    this.climber = climber;
    this.inches = inches;
    addRequirements(climber, carriage);
  }

  @Override
  public void initialize() {
    climber.setClimberFalconsPosition(inches);
  }

  @Override
  public void execute() {
    System.out.println("INSIDEEEEEEEEE");
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
