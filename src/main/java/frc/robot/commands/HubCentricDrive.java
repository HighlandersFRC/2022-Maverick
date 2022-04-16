// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.controlloops.PID;

public class HubCentricDrive extends CommandBase {
  /** Creates a new HubCentricDrive. */
  private Drive drive;
  private Peripherals peripherals;

  private PID pid;

  // private double kP = 6;
  // private double kI = 0.15;
  // private double kD = 0.6;

  private double kP = 15;
  private double kI = 0;
  private double kD = 0;

  public HubCentricDrive(Drive drive, Peripherals peripherals) {
    this.drive = drive;
    this.peripherals = peripherals;
    addRequirements(this.drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PID(kP, kI, kD);
    pid.setSetPoint(0);
    pid.setMinOutput(-4);
    pid.setMaxOutput(4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = peripherals.getLimeLightX();
    pid.updatePID(turn);
    double result = pid.getResult();
    drive.driveAutoAligned(result);
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
