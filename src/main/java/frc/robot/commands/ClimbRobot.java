// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbRobot extends CommandBase {
  /** Creates a new ClimbRobot. */
  private Climber climber;
  private double percent;

  public ClimbRobot(Climber climber, double percent) {
    this.climber = climber;
    this.percent = percent;
    addRequirements(this.climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.unlockExtendingClimber();
    // initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      climber.setClimberPercents(percent, percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      climber.lockExtendingClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
