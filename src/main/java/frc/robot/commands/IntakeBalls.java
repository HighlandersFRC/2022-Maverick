// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Lights.LEDMode;

public class IntakeBalls extends CommandBase {
  private MagIntake magIntake;
  private Lights lights;

  public IntakeBalls(MagIntake magIntake, Lights lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.magIntake = magIntake;
    this.lights = lights;
    addRequirements(this.magIntake, this.lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lights.setMode(LEDMode.YELLOW);
    magIntake.setIntakeDown();
    magIntake.setIntakePercent(-0.5);

    magIntake.moveMagazine();
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
