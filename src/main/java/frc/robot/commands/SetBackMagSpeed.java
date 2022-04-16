// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MagIntake;

public class SetBackMagSpeed extends CommandBase {
  /** Creates a new SetBackMagSpeed. */
  private MagIntake magIntake;
  private double rpm = 0;
  private Boolean ends;
  public SetBackMagSpeed(MagIntake magIntake, double rpm, Boolean doesEnd) {
    this.magIntake = magIntake;
    this.rpm = rpm;
    ends = doesEnd;
    addRequirements(this.magIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    magIntake.setBackMagRPM(rpm);
    magIntake.setFrontMagazine(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(ends && (Math.abs(magIntake.getBackMagRPM() - rpm) < 100)) {
      return true;
    }
    return false;
  }
}
