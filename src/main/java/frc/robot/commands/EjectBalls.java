// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Lights.LEDMode;

public class EjectBalls extends CommandBase {
  /** Creates a new EjectBalls. */

  private MagIntake magIntake;
  private Lights lights;

  private double backPercent;
  private double frontPercent;
  // private double beltPercent;
  private double time = 0;
  private double initTime;

  private double beamBreakSeenPositive = 0;

  public EjectBalls(MagIntake magIntake, Lights lights, double backPercent, double frontPercent, double time) {
    this.magIntake = magIntake;
    this.backPercent = backPercent;
    this.frontPercent = frontPercent;
    this.lights = lights;
    // this.lowerPercent = lowerPercent;
    // this.beltPercent = beltPercent;
    this.time = time;
    addRequirements(magIntake, lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    beamBreakSeenPositive = 0;
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
    magIntake.setBackMagazine(backPercent);
    magIntake.setFrontMagazine(frontPercent);
    if(magIntake.getUpperBeamBreak() == false) {
      lights.setMode(LEDMode.VIOLET);
      beamBreakSeenPositive++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    magIntake.setBackMagazine(0);
    magIntake.setFrontMagazine(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(beamBreakSeenPositive > 15 && magIntake.getUpperBeamBreak() == true) {
      // System.out.println("=======================================");
      return true;
    }
    if(time != -1) {
      if(Timer.getFPGATimestamp() - initTime > time) {
        return true;
      }
    }
    return false;
  }
}