// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Lights.LEDMode;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class TurnDriveTrain extends CommandBase {
  /** Creates a new TurnDriveTrain. */
  private Drive drive;
  private Peripherals peripherals;
  private Lights lights;
  private double turn = 0;

  private PID pid;

  private double kP = 6;
  private double kI = 0.15;
  private double kD = 0.6;

  private int angleSettled = 0;

  private double initTime = 0;
  public TurnDriveTrain(Drive drive, Peripherals peripherals, Lights lights) {
    this.peripherals = peripherals;
    this.drive = drive;
    this.lights = lights;
    addRequirements(this.drive, this.lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PID(kP, kI, kD);
    pid.setSetPoint(0);
    pid.setMinOutput(-4);
    pid.setMaxOutput(4);
    angleSettled = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turn = peripherals.getLimeLightX();
    pid.updatePID(turn);
    double result = pid.getResult();
    System.out.println("RESULT:   " + result);
    SmartDashboard.putNumber("RESULT", result);
    drive.autoDrive(new Vector(0, 0), result);

    if((Math.abs(peripherals.getLimeLightX()) <= 0.03) && Math.abs(peripherals.getLimeLightX()) != 0) {
      angleSettled++;
    }
    else {
      angleSettled = 0;
    }

    if (peripherals.getLimeLightX()!= -1.0) {
      lights.setMode(LEDMode.GREEN);
    } else {
      lights.setMode(LEDMode.REDFLASH);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.autoDrive(new Vector(0, 0), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(angleSettled > 3) {
      return true;
    }
    return false;
  }
}
