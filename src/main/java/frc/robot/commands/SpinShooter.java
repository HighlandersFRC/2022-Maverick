// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SpinShooter extends CommandBase {
  /** Creates a new SpinShooter. */
  private Shooter shooter;
  private double rpm;
  private double distance;
  public SpinShooter(Shooter shooter, double rpm, double distance) {
    this.shooter = shooter;
    this.rpm = rpm;
    this.distance = distance;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // rpm = (5.9021 * distance) + 1779.2;
    
    // if(rpm > 4500) {
    //   rpm = 4500;
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("SHOOTING");
    // shooter.setShooterPercent(0.45);
    System.out.println("RPM: " + shooter.getShooterRPM());
    shooter.setShooterRPM(rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shooter.setShooterPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(shooter.getShooterRPM() - rpm) < 100) {
      System.out.println("00000000000000000000000000000000000");
      return true;
    }
    return true;
  }
}
