// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.tools.controlloops.PID;

public class RotateClimber extends CommandBase {

  private Climber climber;
  private double rotationPercent;
  private PID pid;
  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;

  public RotateClimber(Climber climber, double rotationPercent) {
    this.climber = climber;
    this.rotationPercent = rotationPercent;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    if (rotationPercent < -0.03) {
      rotationPercent = -0.03;
    } else if (rotationPercent > 0.25) {
      rotationPercent = 0.25;
    }
    pid = new PID(kP, kI, kD);
    pid.setSetPoint(rotationPercent);
    pid.setMinOutput(-0.5);
    pid.setMaxOutput(0.5);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
