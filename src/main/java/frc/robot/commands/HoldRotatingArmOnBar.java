// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Peripherals;

public class HoldRotatingArmOnBar extends CommandBase {
  private Climber climber;
  private double wantedArmAngle;
  private Peripherals peripherals;
  public HoldRotatingArmOnBar(Climber climber, double wantedArmAngle, Peripherals peripherals) {
    this.climber = climber;
    this.wantedArmAngle = wantedArmAngle;
    this.peripherals = peripherals;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double adjustedArmAngle = (peripherals.getNavxPitch() + wantedArmAngle);
    if(adjustedArmAngle < 0) {
      adjustedArmAngle = 0;
    }
    else if(adjustedArmAngle > 90) {
      adjustedArmAngle = 90;
    }
    climber.setRotatingMotorPosition(adjustedArmAngle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
