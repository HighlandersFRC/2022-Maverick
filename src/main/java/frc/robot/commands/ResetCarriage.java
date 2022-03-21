// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Climber;

public class ResetCarriage extends CommandBase {

  private Carriage Carriage;
  private double percent;
  public ResetCarriage(Carriage climber) {
    this.Carriage = climber;
    this.percent = percent;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // if(Carriage.getCarriageFalconCurrent() > 30) {
    //   Carriage.setClimberPercents(0);
    // }
    Carriage.setClimberPercents(-0.2);
    SmartDashboard.putNumber("CARRIAGE CURRENT", Carriage.getCarriageFalconCurrent());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if(Carriage.getCarriageFalconCurrent() > 20) {
      return true;
    }
    return false;
  }
}
