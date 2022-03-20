// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Climber;

public class CarriageDefault extends CommandBase {
  /** Creates a new CarriageDefault. */
  private Carriage climber;
  public CarriageDefault(Carriage carriage) {
    this.climber = carriage;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    // if (!climber.getClimberLimitSwitch()) {
    //   climber.setClimberPercents(0);
    //   climber.zeroClimberFalcons();
    // } else {
    //   climber.setClimberPercents(-0.1);
    // }
    climber.setClimberPercents(0);
    // climber.setRotatingMotorPercent(0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
