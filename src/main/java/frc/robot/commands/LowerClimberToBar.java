// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class LowerClimberToBar extends CommandBase {
  /** Creates a new LowerClimberToBar. */
  private Climber climber;

  private boolean rightDone = false;
  private boolean leftDone = false;

  public LowerClimberToBar(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftDone = false;
    rightDone = false;
    climber.unlockExtendingClimber();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climber.getLeftClimberCurrent() > 60) {
        leftDone = true;
    }
    if(climber.getRightClimberCurrent() > 60) {
        rightDone = true;
    }
    if(rightDone) {
        climber.setClimberPercents(0.3, 0);
    }
    else if(leftDone) {
        climber.setClimberPercents(0, 0.3);
    }
    else {
        climber.setClimberPercents(0.3, 0.3);
    }
    // SmartDashboard.putNumber("Output current", climber.getLeftClimberCurrent());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // climber.setClimber(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(rightDone && leftDone) {
      return true;
    }
    return false;
  }
}