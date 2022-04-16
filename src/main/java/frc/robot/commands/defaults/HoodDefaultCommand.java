// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Peripherals;

public class HoodDefaultCommand extends CommandBase {
  /** Creates a new HoodDefaultCommand. */

  private Hood hood;
  private Peripherals peripherals;

  public HoodDefaultCommand(Hood hood, Peripherals peripherals) {
    this.hood = hood;
    this.peripherals = peripherals;
    addRequirements(hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(OI.operatorB.get()) {
      // hood.setHoodPosition(Constants.getShooterValues(peripherals.getLimeLightDistanceToTarget())[0]);
    // } else 
    if (!hood.getLowerLimitSwitch()) {
      hood.setHoodPercent(0);
      hood.zeroHood();
    } else {
      hood.setHoodPercent(-0.1);
    }
    // System.out.println("Position: " + hood.getSensorPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
